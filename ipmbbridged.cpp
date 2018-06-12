/* Copyright 2018 Intel
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/timerfd.h>
#include <linux/i2c-dev-user.h>
#include <algorithm>
#include <vector>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/bus/match.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/vtable.hpp>
#include <sdbusplus/server/interface.hpp>
#include <phosphor-logging/log.hpp>

#include "ipmbbridged.hpp"
#include "ipmbdefines.hpp"
#include "ipmbutils.hpp"

/// @brief Ipmb outstanding requests
IpmbOutstandingRequests outstandingRequests;

/// @brief Dbus
const char *IPMB_BUS = "xyz.openbmc_project.Ipmi.Channel.Ipmb";
const char *IPMB_OBJ = "/xyz/openbmc_project/Ipmi/Channel/Ipmb";
const char *HOST_IPMI_INTF = "org.openbmc.HostIpmi";

const char *IPMB_DBUS_INTF = "org.openbmc.Ipmb";
const char *FILTER =
    "type='signal',interface='org.openbmc.Ipmb',member='sendRequest'";

sdbusplus::bus::bus bus = sdbusplus::bus::new_system();

struct pollfd ipmbi2cMasterFd;
struct pollfd ipmbFds[IPMB_TOTAL_FDS];

using namespace phosphor::logging;

/**
 * @brief Ipmb request class methods
 */
IpmbRequest::IpmbRequest()
{
    data.reserve(ipmbMaxDataSize);
    dataLength = 0;
    retriesRemaining = ipmbNumberOfRetries;
}

IpmbRequest::IpmbRequest(uint8_t netFn, uint8_t rsLun, uint8_t rqSA,
                         uint8_t seq, uint8_t rqLun, uint8_t cmd,
                         std::vector<uint8_t> &inputData,
                         const char *serviceSender, const char *objpathSender,
                         uint32_t appSeq) :
    netFn(netFn),
    rsLun(rsLun), rqSA(rqSA), seq(seq), rqLun(rqLun), cmd(cmd),
    service(serviceSender), objpath(objpathSender), appSeq(appSeq)
{
    data.reserve(ipmbMaxDataSize);
    dataLength = inputData.size();
    retriesRemaining = ipmbNumberOfRetries;

    if (dataLength > 0)
    {
        data = std::move(inputData);
    }
}

void IpmbRequest::updateTimeout()
{
    uint64_t currentTime = ipmbCurrentTimeGet();
    timeToRetryUs = currentTime +
                    static_cast<uint64_t>(ipmbMsToUs(ipmbRequestRetryTimeout));
}

void IpmbRequest::incomingMessageHandler()
{
    sdbusplus::message::message mesg =
        bus.new_signal(IPMB_OBJ, HOST_IPMI_INTF, "ReceivedMessage");
    mesg.append(seq, netFn, rsLun, cmd, data);
    mesg.signal_send();
}

void IpmbRequest::i2cToIpmbConstruct(IPMB_HEADER *ipmbBuffer,
                                     size_t bufferLength)
{
    // constructing ipmb request from i2c buffer
    netFn = ipmbNetFnGet(ipmbBuffer->Header.Req.rsNetFnLUN);
    rsLun = ipmbLunFromNetFnLunGet(ipmbBuffer->Header.Req.rsNetFnLUN);
    rqSA = ipmbBuffer->Header.Req.rqSA;
    seq = ipmbSeqGet(ipmbBuffer->Header.Req.rqSeqLUN);
    rqLun = ipmbLunFromSeqLunGet(ipmbBuffer->Header.Req.rqSeqLUN);
    cmd = ipmbBuffer->Header.Req.cmd;

    dataLength =
        bufferLength - (ipmbConnectionHeaderLength +
                        ipmbRequestDataHeaderLength + ipmbChecksumSize);

    if (dataLength > 0)
    {
        data.insert(data.end(), ipmbBuffer->Header.Req.data,
                    &ipmbBuffer->Header.Req.data[dataLength]);
    }
}

size_t IpmbRequest::ipmbToi2cConstruct(
    std::array<uint8_t, ipmbMaxFrameLength> &buffer)
{
    size_t bufferLength = 0;
    static_assert(ipmbMaxFrameLength >= sizeof(IPMB_HEADER));
    IPMB_HEADER *ipmbBuffer = reinterpret_cast<IPMB_HEADER *>(buffer.data());

    // constructing buffer from ipmb request
    ipmbBuffer->Header.Req.address = ipmbRqSlaveAddress;
    ipmbBuffer->Header.Req.rsNetFnLUN = ipmbNetFnLunSet(netFn, rsLun);
    ipmbBuffer->Header.Req.rqSA = rqSA;
    ipmbBuffer->Header.Req.rqSeqLUN = ipmbSeqLunSet(seq, rqLun);
    ipmbBuffer->Header.Req.cmd = cmd;

    ipmbBuffer->Header.Resp.checksum1 = ipmbChecksumCompute(
        buffer.data(), ipmbConnectionHeaderLength - ipmbChecksumSize);

    if (dataLength > 0)
    {
        std::copy(
            data.begin(), data.end(),
            &buffer[ipmbConnectionHeaderLength + ipmbRequestDataHeaderLength]);
    }

    bufferLength = dataLength + ipmbRequestDataHeaderLength +
                   ipmbConnectionHeaderLength + ipmbChecksumSize;

    buffer[bufferLength - ipmbChecksumSize] =
        ipmbChecksumCompute(&buffer[ipmbChecksum2StartOffset],
                            (ipmbRequestDataHeaderLength + dataLength));

    return bufferLength;
}

uint64_t IpmbRequest::retryAbsTimeGet()
{
    return timeToRetryUs;
}

void IpmbRequest::requestTimeoutNotify()
{
    sdbusplus::message::message mesg = bus.new_method_call(
        service.data(), objpath.data(), IPMB_DBUS_INTF, "requestTimeout");

    mesg.append(appSeq, netFn, rqLun, cmd, data);
    sdbusplus::message::message resp = bus.call(mesg);

    if (resp.is_method_error())
    {
        log<level::ERR>("method error: sending requestTimeoutNotify");
    }
}

/**
 * @brief Ipmb response class methods
 */
IpmbResponse::IpmbResponse()
{
    data.reserve(ipmbMaxDataSize);
    dataLength = 0;
}

IpmbResponse::IpmbResponse(uint8_t netFn, uint8_t rqLun, uint8_t rsSA,
                           uint8_t seq, uint8_t rsLun, uint8_t cmd,
                           uint8_t completionCode,
                           std::vector<uint8_t> &inputData) :
    netFn(netFn),
    rqLun(rqLun), rsSA(rsSA), seq(seq), rsLun(rsLun), cmd(cmd),
    completionCode(completionCode)
{
    data.reserve(ipmbMaxDataSize);
    dataLength = inputData.size();

    if (dataLength > 0)
    {
        data = std::move(inputData);
    }
}

void IpmbResponse::incomingMessageHandler()
{
    sdbusplus::message::message mesg = bus.new_method_call(
        service.data(), objpath.data(), IPMB_DBUS_INTF, "returnResponse");
    mesg.append(appSeq, netFn, rsLun, cmd, completionCode, data);
    sdbusplus::message::message resp = bus.call(mesg);

    if (resp.is_method_error())
    {
        log<level::ERR>("method error: sending response to the client app");
    }
}

void IpmbResponse::i2cToIpmbConstruct(IPMB_HEADER *ipmbBuffer,
                                      size_t bufferLength)
{
    netFn = ipmbNetFnGet(ipmbBuffer->Header.Resp.rqNetFnLUN);
    rqLun = ipmbLunFromNetFnLunGet(ipmbBuffer->Header.Resp.rqNetFnLUN);
    rsSA = ipmbBuffer->Header.Resp.rsSA;
    seq = ipmbSeqGet(ipmbBuffer->Header.Resp.rsSeqLUN);
    rsLun = ipmbLunFromSeqLunGet(ipmbBuffer->Header.Resp.rsSeqLUN);
    cmd = ipmbBuffer->Header.Resp.cmd;
    completionCode = ipmbBuffer->Header.Resp.completionCode;

    dataLength =
        bufferLength - (ipmbConnectionHeaderLength +
                        ipmbResponseDataHeaderLength + ipmbChecksumSize);

    if (dataLength > 0)
    {
        data.insert(data.end(), ipmbBuffer->Header.Resp.data,
                    &ipmbBuffer->Header.Resp.data[dataLength]);
    }
}

size_t IpmbResponse::ipmbToi2cConstruct(
    std::array<uint8_t, ipmbMaxFrameLength> &buffer)
{
    size_t bufferLength = 0;
    static_assert(ipmbMaxFrameLength >= sizeof(IPMB_HEADER));
    IPMB_HEADER *ipmbBuffer = reinterpret_cast<IPMB_HEADER *>(buffer.data());

    ipmbBuffer->Header.Resp.address = ipmbRqSlaveAddress;
    ipmbBuffer->Header.Resp.rqNetFnLUN = ipmbNetFnLunSet(netFn, rqLun);
    ipmbBuffer->Header.Resp.rsSA = rsSA;
    ipmbBuffer->Header.Resp.rsSeqLUN = ipmbSeqLunSet(seq, rsLun);
    ipmbBuffer->Header.Resp.cmd = cmd;
    ipmbBuffer->Header.Resp.completionCode = completionCode;

    ipmbBuffer->Header.Resp.checksum1 = ipmbChecksumCompute(
        buffer.data(), ipmbConnectionHeaderLength - ipmbChecksumSize);

    if (dataLength > 0)
    {
        std::copy(
            data.begin(), data.end(),
            &buffer[ipmbConnectionHeaderLength + ipmbResponseDataHeaderLength]);
    }

    bufferLength = dataLength + ipmbResponseDataHeaderLength +
                   ipmbConnectionHeaderLength + ipmbChecksumSize;

    buffer[bufferLength - ipmbChecksumSize] =
        ipmbChecksumCompute(&buffer[ipmbChecksum2StartOffset],
                            (ipmbResponseDataHeaderLength + dataLength));

    return bufferLength;
}

/**
 * @brief Ipmb channel
 */
static int ipmbSend(std::array<uint8_t, ipmbMaxFrameLength> &buffer,
                    size_t bufferSize)
{
    // retries handle rare situations of hw being unable to perform transaction.
    // number of retries is arbitrarly set value
    for (int sendCount = 0; sendCount < ipmbI2cNumberOfRetries; sendCount++)
    {
        if (write(ipmbi2cMasterFd.fd, &buffer[ipmbAddressSize],
                  (bufferSize - ipmbAddressSize)) ==
            (bufferSize - ipmbAddressSize))
        {
            return 0;
        }
    }

    log<level::ERR>("Sent to I2C failed after retries");

    return -1;
}

/**
 * @brief Ipmb Outstanding Requests
 */
int IpmbOutstandingRequests::ipmbOutstandingRequestsInit()
{
    seq = 0;
    ipmbFds[IPMB_TIMER_FD].fd = timerfd_create(CLOCK_MONOTONIC, 0);

    if (ipmbFds[IPMB_TIMER_FD].fd < 0)
    {
        return -1;
    }

    return 0;
}

void IpmbOutstandingRequests::requestAdd(
    sdbusplus::message::message &requestMessage)
{
    size_t bufferLength = 0;
    uint32_t appSeq = 0;
    uint8_t netfn = 0, lun = 0, cmd = 0;
    std::array<uint8_t, ipmbMaxFrameLength> buffer{};
    std::vector<uint8_t> dataReceived;

    dataReceived.reserve(ipmbMaxDataSize);

    // check if we can put another request into the list
    if (outstandingRequests.size() == ipmbMaxOutstandingRequestsCount)
    {
        return;
    }

    requestMessage.read(appSeq, netfn, lun, cmd, dataReceived);

    std::unique_ptr<IpmbRequest> ipmbMessageReceived =
        std::make_unique<IpmbRequest>(
            netfn, lun, ipmbBmcSlaveAddress, seq, lun, cmd, dataReceived,
            requestMessage.get_sender(), requestMessage.get_path(), appSeq);

    // send the request, update the time of next retry, add to the list and bump
    // up sequence number
    bufferLength = ipmbMessageReceived->ipmbToi2cConstruct(buffer);
    ipmbSend(buffer, bufferLength);
    ipmbMessageReceived->updateTimeout();
    outstandingRequests.push_back(std::move(ipmbMessageReceived));
    timerUpdate();
    seqNumSet();
}

int IpmbOutstandingRequests::systemTimerUpdate(uint64_t timeToRetryUs)
{
    struct itimerspec iTimerpec;
    iTimerpec.it_interval.tv_sec = 0;
    iTimerpec.it_interval.tv_nsec = 0;
    iTimerpec.it_value.tv_sec = 0;
    iTimerpec.it_value.tv_nsec = ipmbUsToNs(timeToRetryUs);

    if (timerfd_settime(ipmbFds[IPMB_TIMER_FD].fd, 0, &iTimerpec, nullptr) ==
        -1)
    {
        log<level::ERR>("timerfd_settime() error");
        return -1;
    }

    return 0;
}

void IpmbOutstandingRequests::timerUpdate()
{
    uint64_t timeToRetryUs = 0;

    // sort the list by the expiration time
    std::sort(outstandingRequests.begin(), outstandingRequests.end(),
              [](const std::unique_ptr<IpmbRequest> &requestFirst,
                 const std::unique_ptr<IpmbRequest> &requestSecond) -> bool {
                  return (requestFirst->retryAbsTimeGet() <
                          requestSecond->retryAbsTimeGet());
              });

    // disable timer
    systemTimerUpdate(0);

    uint64_t currentTime = ipmbCurrentTimeGet();

    auto request = outstandingRequests.begin();
    if (request == outstandingRequests.end())
    {
        // list empty, no need to set the timer
        return;
    }
    else
    {
        // next Expiration Time in the past
        if (currentTime >= (*request)->timeToRetryUs)
        {
            // set timer for 1 ms so it expires immediately
            timeToRetryUs = ipmbOneUs;
        }
        else
        {
            // set timer to the neares expiration time
            timeToRetryUs = (*request)->timeToRetryUs - currentTime;
        }
    }

    systemTimerUpdate(timeToRetryUs);
}

void IpmbOutstandingRequests::retryRequest()
{
    std::array<uint8_t, ipmbMaxFrameLength> buffer{};
    size_t dataLength = 0;

    uint64_t currentTime = ipmbCurrentTimeGet();

    auto request = outstandingRequests.begin();
    while (request != outstandingRequests.end())
    {
        // request timed out (no retries left). Erase it
        if ((*request)->retriesRemaining == 0)
        {
            (*request)->requestTimeoutNotify();
            request = outstandingRequests.erase(request);
            continue;
        }
        else
        {
            // Check if request should be retried now
            if ((*request)->timeToRetryUs <= currentTime)
            {
                dataLength = (*request)->ipmbToi2cConstruct(buffer);
                ipmbSend(buffer, dataLength);
                (*request)->retriesRemaining--;
                (*request)->updateTimeout();
            }
            // If current request timeToRetryUs is not in the past, we can leave
            else
            {
                break;
            }
        }
        request++;
    }

    timerUpdate();
}

void IpmbOutstandingRequests::seqNumSet()
{
    seq = ++seq & ipmbSeqMask;
}

std::unique_ptr<IpmbRequest> IpmbOutstandingRequests::responseMatch(
    std::unique_ptr<IpmbResponse> &response)
{
    for (auto request = outstandingRequests.begin();
         request != outstandingRequests.end(); ++request)
    {
        if ((((*request)->netFn |= 1) == (response->netFn)) &&
            (((*request)->rqLun) == (response->rqLun)) &&
            (((*request)->rsLun) == (response->rsLun)) &&
            (((*request)->seq) == (response->seq)) &&
            (((*request)->cmd) == (response->cmd)))
        {
            // match, response is corresponding to previously sent request
            // copy relevant client data
            response->service = std::move((*request)->service);
            response->objpath = std::move((*request)->objpath);
            response->appSeq = (*request)->appSeq;

            auto matchedRequest = std::move(*request);
            outstandingRequests.erase(request);

            return matchedRequest;
        }
    }
    timerUpdate();

    // no match, return null pointer
    return nullptr;
}

/**
 * @brief Dbus methods
 */
static int ipmbSignalCallback(sd_bus_message *bus_msg, void *userdata,
                              sd_bus_error *ret_error)
{
    auto mesg = sdbusplus::message::message(bus_msg);

    if (mesg.is_method_error())
    {
        log<level::ERR>("ipmbSignalCallback() method error");
        return 0;
    }

    outstandingRequests.requestAdd(mesg);

    return 0;
}

static int ipmbMessageSend(sd_bus_message *bus_msg, void *userdata,
                           sd_bus_error *ret_error)
{
    int64_t status = -1;
    size_t bufferLength = 0;
    uint8_t netfn = 0, lun = 0, seq = 0, cmd = 0, cc = 0;
    std::array<uint8_t, ipmbMaxFrameLength> buffer{};
    std::vector<uint8_t> dataReceived;

    dataReceived.reserve(ipmbMaxDataSize);
    auto mesg = sdbusplus::message::message(bus_msg);

    do
    {
        if (mesg.is_method_error())
        {
            return 1;
        }

        mesg.read(seq, netfn, lun, cmd, cc, dataReceived);

        if (dataReceived.size() > ipmbMaxDataSize)
        {
            status = -1;
            break;
        }

        if (netfn & ipmbNetFnResponseMask)
        {
            // response received
            // dataReceived is empty after constructor invocation
            std::unique_ptr<IpmbResponse> ipmbMessageReceived =
                std::make_unique<IpmbResponse>(netfn, lun, ipmbBmcSlaveAddress,
                                               seq, lun, cmd, cc, dataReceived);

            bufferLength = ipmbMessageReceived->ipmbToi2cConstruct(buffer);
            status = ipmbSend(buffer, bufferLength);
        }
        else
        {
            // we are not expecting request here
            log<level::ERR>("ipmbMessageSend got a request");
            status = -1;
        }
    } while (0);

    sdbusplus::message::message reply = mesg.new_method_return();
    reply.append(status);
    reply.method_return();

    return 1;
}

static const sdbusplus::vtable::vtable_t ipmiVtable[] = {
    sdbusplus::vtable::start(),
    sdbusplus::vtable::method("sendMessage", "yyyyyay", "x", ipmbMessageSend),
    sdbusplus::vtable::signal("ReceivedMessage", "yyyyay"),
    sdbusplus::vtable::end()};

static const sdbusplus::vtable::vtable_t ipmbVtable[] = {
    sdbusplus::vtable::start(),
    sdbusplus::vtable::signal("sendRequest", "uyyyay"),
    sdbusplus::vtable::end()};

/**
 * @brief Main
 */
int main(int argc, char *argv[])
{
    int r = -1;
    IPMB_HEADER *ipmbFrame = nullptr;
    std::array<uint8_t, ipmbMaxFrameLength> buffer{};

    const char *ipmbI2cSlave = "/sys/bus/i2c/devices/0-1010/slave-mqueue";
    const char *ipmbI2cMaster = "/dev/i2c-0";

    sdbusplus::server::interface::interface ipmiInterface(
        bus, IPMB_OBJ, HOST_IPMI_INTF, ipmiVtable, nullptr);

    sdbusplus::server::interface::interface ipmbInterface(
        bus, IPMB_OBJ, IPMB_DBUS_INTF, ipmbVtable, nullptr);

    sdbusplus::bus::match::match match(bus, FILTER, ipmbSignalCallback);

    bus.request_name(IPMB_BUS);

    ipmbFds[IPMB_SD_BUS_FD].fd = bus.get_fd();
    if (ipmbFds[IPMB_SD_BUS_FD].fd < 0)
    {
        log<level::ERR>("Error getting fd from bus object");
        return -1;
    }

    // open fd to i2c slave device
    ipmbFds[IPMB_I2C_FD].fd =
        open(ipmbI2cSlave, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (ipmbFds[IPMB_I2C_FD].fd < 0)
    {
        log<level::ERR>("Error opening ipmbI2cSlave");
        return -1;
    }

    // open fd to i2c master device
    ipmbi2cMasterFd.fd = open(ipmbI2cMaster, O_RDWR | O_NONBLOCK);
    if (ipmbi2cMasterFd.fd < 0)
    {
        log<level::ERR>("Error opening ipmbI2cMaster");
        return -1;
    }

    // set slave address of recipient
    if (ioctl(ipmbi2cMasterFd.fd, I2C_SLAVE,
              ipmbAddressTo7BitSet(ipmbRqSlaveAddress)) < 0)
    {
        log<level::ERR>("Error setting ipmbi2cMasterFd slave address");
        return -1;
    }

    // initialize outstanding requests module
    if (outstandingRequests.ipmbOutstandingRequestsInit() < 0)
    {
        log<level::ERR>("Error ipmbOutstandingRequestsInit()");
        return -1;
    }

    // set proper events to poll for
    ipmbFds[IPMB_SD_BUS_FD].events = POLLIN;
    ipmbFds[IPMB_I2C_FD].events = POLLPRI;
    ipmbFds[IPMB_TIMER_FD].events = POLLIN;

    while (1)
    {
        r = poll(ipmbFds, IPMB_TOTAL_FDS, -1);

        if (r < 0)
        {
            continue;
        }

        // outstanding requests timer expired
        if (ipmbFds[IPMB_TIMER_FD].revents & POLLIN)
        {
            outstandingRequests.retryRequest();
        }

        // received dbus event
        if (ipmbFds[IPMB_SD_BUS_FD].revents & POLLIN)
        {
            bus.process_discard();
        }

        // received i2c slave event
        if (ipmbFds[IPMB_I2C_FD].revents & POLLPRI)
        {
            if (r == 0)
                continue;

            lseek(ipmbFds[IPMB_I2C_FD].fd, 0, SEEK_SET);
            r = read(ipmbFds[IPMB_I2C_FD].fd, buffer.data(),
                     ipmbMaxFrameLength);
            if ((r < ipmbMinFrameLength) || (r > ipmbMaxFrameLength))
                continue;

            ipmbFrame = reinterpret_cast<IPMB_HEADER *>(buffer.data());

            // valiate the frame
            if (!isFrameValid(ipmbFrame, r))
            {
                continue;
            }

            // copy frame to ipmib message buffer
            if (ipmbIsResponse(ipmbFrame))
            {
                std::unique_ptr<IpmbResponse> ipmbMessageReceived =
                    std::make_unique<IpmbResponse>();

                ipmbMessageReceived->i2cToIpmbConstruct(ipmbFrame, r);

                // lets match response with outstanding request
                auto ipmbMessageOutstanding =
                    outstandingRequests.responseMatch(ipmbMessageReceived);
                if (ipmbMessageOutstanding)
                {
                    // matching request found - send responce to the client
                    ipmbMessageReceived->incomingMessageHandler();
                }

                // no match - discard the message and proceed
            }
            else
            {
                std::unique_ptr<IpmbRequest> ipmbMessageReceived =
                    std::make_unique<IpmbRequest>();

                ipmbMessageReceived->i2cToIpmbConstruct(ipmbFrame, r);

                // send request to the client
                ipmbMessageReceived->incomingMessageHandler();
            }
        }
    }

    close(ipmbFds[IPMB_I2C_FD].fd);
    close(ipmbFds[IPMB_TIMER_FD].fd);
    close(ipmbFds[IPMB_SD_BUS_FD].fd);
    close(ipmbi2cMasterFd.fd);

    return 0;
}
