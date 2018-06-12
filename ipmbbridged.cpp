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
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <algorithm>
#include <memory>
#include <vector>
#include <sdbusplus/bus.hpp>
#include <sdbusplus/message.hpp>
#include <sdbusplus/vtable.hpp>
#include <sdbusplus/server/interface.hpp>

#include "ipmbbridged.hpp"
#include "ipmbdefines.hpp"
#include "ipmbutils.hpp"

/// @brief Ipmb outstanding requests
IpmbOutstandingRequests outstandingRequests;

/// @brief Dbus
const char *IPMB_BUS = "xyz.openbmc_project.Ipmi.Channel.Ipmb";
const char *IPMB_OBJ = "/xyz/openbmc_project/Ipmi/Channel/Ipmb";
const char *HOST_IPMI_INTF = "org.openbmc.HostIpmi";

sdbusplus::bus::bus *ipmbBus = nullptr;
struct pollfd writeFd;

/// @brief Static methods
static void ipmbOutstandingRequestsTimerCallback(int sig, siginfo_t *si,
                                                 void *uc)
{
    outstandingRequests.retryRequest();
}

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
                         std::vector<uint8_t> &inputData) :
    netFn(netFn),
    rsLun(rsLun), rqSA(rqSA), seq(seq), rqLun(rqLun), cmd(cmd)
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
    uint64_t currentTime = ipmbCurrentTime();
    timeToRetryUs = currentTime + ipmbMsToUs(ipmbRequestRetryTimeout);
}

int IpmbRequest::incomingMessageHandler()
{
    int status = 0;

    auto mesg =
        ipmbBus->new_signal(IPMB_OBJ, HOST_IPMI_INTF, "ReceivedMessage");
    mesg.append(seq, netFn, rsLun, cmd, data);
    mesg.signal_send();

    return status;
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

void IpmbRequest::ipmbToi2cConstruct(uint8_t *buffer, size_t &bufferLength)
{
    IPMB_HEADER *ipmbBuffer = (IPMB_HEADER *)buffer;

    // constructing buffer from ipmb request
    ipmbBuffer->Header.Req.address = ipmbRqSlaveAddress;
    ipmbBuffer->Header.Req.rsNetFnLUN = ipmbNetFnLunSet(netFn, rsLun);
    ipmbBuffer->Header.Req.rqSA = rqSA;
    ipmbBuffer->Header.Req.rqSeqLUN = ipmbSeqLunSet(seq, rqLun);
    ipmbBuffer->Header.Req.cmd = cmd;

    ipmbBuffer->Header.Resp.checksum1 = ipmbChecksumCompute(
        buffer, ipmbConnectionHeaderLength - ipmbChecksumSize);

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
}

uint64_t IpmbRequest::retryAbsTimeGet()
{
    return timeToRetryUs;
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

int IpmbResponse::incomingMessageHandler()
{
    int status = 0;

    // TODO: fill after host IPMI API is ready for receiving responses messages

    return status;
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

void IpmbResponse::ipmbToi2cConstruct(uint8_t *buffer, size_t &bufferLength)
{
    IPMB_HEADER *ipmbBuffer = (IPMB_HEADER *)buffer;

    ipmbBuffer->Header.Resp.address = ipmbRqSlaveAddress;
    ipmbBuffer->Header.Resp.rqNetFnLUN = ipmbNetFnLunSet(netFn, rqLun);
    ipmbBuffer->Header.Resp.rsSA = rsSA;
    ipmbBuffer->Header.Resp.rsSeqLUN = ipmbSeqLunSet(seq, rsLun);
    ipmbBuffer->Header.Resp.cmd = cmd;
    ipmbBuffer->Header.Resp.completionCode = completionCode;

    ipmbBuffer->Header.Resp.checksum1 = ipmbChecksumCompute(
        buffer, ipmbConnectionHeaderLength - ipmbChecksumSize);

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
}

/**
 * @brief Ipmb channel
 */
static int ipmbSend(uint8_t *buffer, size_t bufferSize)
{
    uint8_t sendCount;

    // retries handle rare situations of hw being unable to perform transaction.
    // number of retries is arbitrarly set value
    for (sendCount = 0; sendCount < ipmbI2cNumberOfRetries; sendCount++)
    {
        if (write(writeFd.fd, &buffer[ipmbAddressSize],
                  (bufferSize - ipmbAddressSize)) ==
            (bufferSize - ipmbAddressSize))
        {
            return 0;
        }
    }

    return -1;
}

/**
 * @brief Ipmb Outstanding Requests
 */
int IpmbOutstandingRequests::ipmbOutstandingRequestsInit()
{
    // timer initialization
    sa.sa_flags = SA_SIGINFO;
    sa.sa_sigaction = ipmbOutstandingRequestsTimerCallback;
    sigemptyset(&sa.sa_mask);
    if (sigaction(SIGRTMIN, &sa, nullptr) == -1)
    {
        return -1;
    }

    // signal initialization
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGRTMIN;
    sev.sigev_value.sival_ptr = &timerid;
    if (timer_create(CLOCK_MONOTONIC, &sev, &timerid) == -1)
    {
        return -1;
    }

    return 0;
}

void IpmbOutstandingRequests::requestAdd(std::unique_ptr<IpmbMessage> &request)
{
    auto requestPtr = dynamic_cast<IpmbRequest *>(request.get());

    // add outstanding request to the list and update time when it should be
    // retried
    requestPtr->updateTimeout();
    outstandingRequests.push_back(std::move(request));
    timerUpdate();
}

int IpmbOutstandingRequests::ipmbTimerUpdate(uint64_t timeToRetryUs)
{
    struct itimerspec iTimerpec;
    iTimerpec.it_interval.tv_sec = 0;
    iTimerpec.it_interval.tv_nsec = 0;
    iTimerpec.it_value.tv_sec = 0;
    iTimerpec.it_value.tv_nsec = ipmbUsToNs(timeToRetryUs);

    if (timer_settime(timerid, 0, &iTimerpec, nullptr) == -1)
    {
        return -1;
    }

    return 0;
}

struct requestsByTimestampSort
{
    // operator used for list sorting
    bool operator()(const std::unique_ptr<IpmbMessage> &requestFirst,
                    const std::unique_ptr<IpmbMessage> &requestSecond)
    {
        auto requestFirstPtr = dynamic_cast<IpmbRequest *>(requestFirst.get());
        auto requestSecondPtr =
            dynamic_cast<IpmbRequest *>(requestSecond.get());

        return (requestFirstPtr->retryAbsTimeGet() <
                requestSecondPtr->retryAbsTimeGet());
    }
};

void IpmbOutstandingRequests::timerUpdate()
{
    struct timespec ts;
    uint64_t timeToRetryUs;

    // sort the list by the expiration time
    std::sort(outstandingRequests.begin(), outstandingRequests.end(),
              requestsByTimestampSort());

    // disable timer
    ipmbTimerUpdate(0);

    uint64_t currentTime = ipmbCurrentTime();

    auto request = outstandingRequests.begin();
    if (request == outstandingRequests.end())
    {
        // list empty, no need to set the timer
        return;
    }
    else
    {
        auto requestPtr = dynamic_cast<IpmbRequest *>(request->get());

        // next Expiration Time in the past
        if (currentTime >= requestPtr->timeToRetryUs)
        {
            // set timer for 1 ms so it expires immediately
            timeToRetryUs = ipmbOneUs;
        }
        else
        {
            // set timer to the neares expiration time
            timeToRetryUs = requestPtr->timeToRetryUs - currentTime;
        }
    }

    ipmbTimerUpdate(timeToRetryUs);
}

void IpmbOutstandingRequests::retryRequest()
{
    uint8_t buffer[ipmbMaxFrameLength];
    size_t dataLength;

    uint64_t currentTime = ipmbCurrentTime();

    auto request = outstandingRequests.begin();
    while (request != outstandingRequests.end())
    {
        auto request_result = dynamic_cast<IpmbRequest *>(request->get());

        // request timed out (no retries left). Erase it
        if (request_result->retriesRemaining == 0)
        {
            request = outstandingRequests.erase(request);
            continue;
        }
        else
        {
            // Check if request should be retried now
            if (request_result->timeToRetryUs <= currentTime)
            {
                request_result->ipmbToi2cConstruct(buffer, dataLength);
                ipmbSend(buffer, dataLength);
                request_result->retriesRemaining--;
                request_result->updateTimeout();
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

std::unique_ptr<IpmbMessage> IpmbOutstandingRequests::responseMatch(
    std::unique_ptr<IpmbMessage> &response)
{
    auto responsePtr = dynamic_cast<IpmbResponse *>(response.get());

    for (auto request = outstandingRequests.begin();
         request != outstandingRequests.end(); ++request)
    {
        auto requestPtr = dynamic_cast<IpmbRequest *>(request->get());

        if (((requestPtr->netFn + 1) == (responsePtr->netFn)) &&
            ((requestPtr->rqLun) == (responsePtr->rqLun)) &&
            ((requestPtr->rsLun) == (responsePtr->rsLun)) &&
            ((requestPtr->seq) == (responsePtr->seq)) &&
            ((requestPtr->cmd) == (responsePtr->cmd)))
        {
            // match, responce is corresponding to previously sent request
            auto matchedRequest = std::move(*request);
            request = outstandingRequests.erase(request);
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

static int ipmbMessageSend(sd_bus_message *bus_msg, void *userdata,
                           sd_bus_error *ret_error)
{
    int status;
    size_t bufferLength;
    uint8_t sendBuffer[ipmbMaxFrameLength];
    uint8_t netfn, lun, seq, cmd, cc;
    std::vector<uint8_t> dataReceived;

    dataReceived.reserve(ipmbMaxDataSize);
    auto mesg = sdbusplus::message::message(bus_msg);

    do
    {
        if (mesg.is_method_error())
        {
            return 0;
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
            std::unique_ptr<IpmbMessage> ipmbMessageReceived =
                std::make_unique<IpmbResponse>(netfn, lun, ipmbBmcSlaveAddress,
                                               seq, lun, cmd, cc, dataReceived);
            ipmbMessageReceived->ipmbToi2cConstruct(sendBuffer, bufferLength);
            status = ipmbSend(sendBuffer, bufferLength);
        }
        else
        {
            // request received
            // dataReceived is empty after constructor invocation
            std::unique_ptr<IpmbMessage> ipmbMessageReceived =
                std::make_unique<IpmbRequest>(netfn, lun, ipmbBmcSlaveAddress,
                                              seq, lun, cmd, dataReceived);
            outstandingRequests.requestAdd(ipmbMessageReceived);

            ipmbMessageReceived->ipmbToi2cConstruct(sendBuffer, bufferLength);
            status = ipmbSend(sendBuffer, bufferLength);
        }
    } while (0);

    auto reply = mesg.new_method_return();
    reply.append(status);
    reply.method_return();

    return 0;
}

static const sdbusplus::vtable::vtable_t ipmb_vtable[] = {
    sdbusplus::vtable::start(),
    sdbusplus::vtable::method("sendMessage", "yyyyyay", "x", ipmbMessageSend,
                              SD_BUS_VTABLE_UNPRIVILEGED),
    sdbusplus::vtable::signal("ReceivedMessage", "yyyyay"),
    sdbusplus::vtable::end()};

/**
 * @brief Main
 */

int main(int argc, char *argv[])
{
    int r;
    IPMB_HEADER *ipmbFrame;
    uint8_t data[ipmbMaxFrameLength];
    struct pollfd ipmbFds[IPMB_TOTAL_FDS];

    const char *ipmbI2cSlave = "/sys/bus/i2c/devices/0-1010/slave-mqueue";
    const char *ipmbI2cMaster = "/dev/i2c-0";

    // open fd to i2c slave device
    ipmbFds[IPMB_I2C_FD].fd =
        open(ipmbI2cSlave, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (ipmbFds[IPMB_I2C_FD].fd < 0)
    {
        return -1;
    }

    // open fd to i2c master device
    writeFd.fd = open(ipmbI2cMaster, O_RDWR | O_NONBLOCK);
    if (writeFd.fd < 0)
    {
        return -1;
    }

    // set slave address of recipient
    if (ioctl(writeFd.fd, 0x0703, ipmbAddressTo7BitSet(ipmbRqSlaveAddress)) < 0)
    {
        return -1;
    }

    // configure dbus
    auto bus = sdbusplus::bus::new_system();
    ipmbBus = &bus;

    auto interface = sdbusplus::server::interface::interface(
        bus, IPMB_OBJ, HOST_IPMI_INTF, ipmb_vtable, nullptr);

    bus.request_name(IPMB_BUS);

    ipmbFds[IPMB_SD_BUS_FD].fd = bus.get_fd();
    if (ipmbFds[IPMB_SD_BUS_FD].fd < 0)
    {
        return -1;
    }

    // set proper events to poll for
    ipmbFds[IPMB_SD_BUS_FD].events = POLLIN;
    ipmbFds[IPMB_I2C_FD].events = POLLPRI;

    // initialize outstanding requests module
    outstandingRequests.ipmbOutstandingRequestsInit();

    while (1)
    {
        r = poll(ipmbFds, IPMB_TOTAL_FDS, -1);

        if (r < 0)
        {
            continue;
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
            r = read(ipmbFds[IPMB_I2C_FD].fd, data, sizeof(data));
            if ((r < ipmbMinFrameLength) || (r > ipmbMaxFrameLength))
                continue;

            ipmbFrame = (IPMB_HEADER *)data;

            // 1. valiate the frame
            if (!isFrameValid(ipmbFrame, r))
            {
                continue;
            }

            // 2. copy frame to ipmib message buffer
            if (ipmbIsResponse(ipmbFrame))
            {
                std::unique_ptr<IpmbMessage> ipmbMessageReceived =
                    std::make_unique<IpmbResponse>();
                ipmbMessageReceived->i2cToIpmbConstruct(ipmbFrame, r);

                // lets match response with outstanding request
                auto ipmbMessageOutstanding =
                    outstandingRequests.responseMatch(ipmbMessageReceived);
                if (ipmbMessageOutstanding)
                {
                    // matching request found - send responce to the client
                    ipmbMessageOutstanding->incomingMessageHandler();
                }

                // no match - discard the message and proceed
            }
            else
            {
                std::unique_ptr<IpmbMessage> ipmbMessageReceived =
                    std::make_unique<IpmbRequest>();
                ipmbMessageReceived->i2cToIpmbConstruct(ipmbFrame, r);

                // send request to the client
                ipmbMessageReceived->incomingMessageHandler();
            }
        }
    }

    close(ipmbFds[IPMB_I2C_FD].fd);
    close(ipmbFds[IPMB_SD_BUS_FD].fd);
    close(writeFd.fd);

    return 0;
}
