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

#include "ipmbbridged.hpp"

#include "ipmbdefines.hpp"
#include "ipmbutils.hpp"

#include <linux/i2c-dev-user.h>

#include <boost/asio.hpp>
#include <phosphor-logging/log.hpp>
#include <sdbusplus/asio/object_server.hpp>
#include <vector>

/**
 * @brief Dbus
 */
static constexpr const char *ipmbBus = "xyz.openbmc_project.Ipmi.Channel.Ipmb";
static constexpr const char *ipmbObj = "/xyz/openbmc_project/Ipmi/Channel/Ipmb";
static constexpr const char *hostIpmiIntf = "org.openbmc.HostIpmi";

static constexpr const char *ipmbDbusIntf = "org.openbmc.Ipmb";
static constexpr const char *filter =
    "type='signal',interface='org.openbmc.Ipmb',member='sendRequest'";

boost::asio::io_service io;
auto conn = std::make_shared<sdbusplus::asio::connection>(io);

/**
 * @brief Channel configuration table
 * TODO : move to user configuration as JSON file
 */
static const std::vector<IpmbChannelConfig> ipmbChannelsConfig = {
    // ME channel
    {ipmbChannelType::me, "/sys/bus/i2c/devices/5-1010/slave-mqueue",
     "/dev/i2c-5", 0x20, 0x2C}, // 8 bit addresses
    // IPMB header channel
    {ipmbChannelType::ipmb, "/sys/bus/i2c/devices/0-1010/slave-mqueue",
     "/dev/i2c-0", 0x20, 0x58}}; // 8 bit addresses

static std::list<IpmbChannel> ipmbChannels;

/**
 * @brief Ipmb request class methods
 */
IpmbRequest::IpmbRequest()
{
    data.reserve(ipmbMaxDataSize);
    dataLength = 0;
    retriesRemaining = ipmbNumberOfRetries;
}

IpmbRequest::IpmbRequest(uint8_t address, uint8_t netFn, uint8_t rsLun,
                         uint8_t rqSA, uint8_t seq, uint8_t rqLun, uint8_t cmd,
                         std::vector<uint8_t> &inputData,
                         const char *serviceSender, const char *objpathSender,
                         uint32_t appSeq) :
    address(address),
    netFn(netFn), rsLun(rsLun), rqSA(rqSA), seq(seq), rqLun(rqLun), cmd(cmd),
    service(serviceSender), objpath(objpathSender), appSeq(appSeq)
{
    data.reserve(ipmbMaxDataSize);
    dataLength = inputData.size();
    retriesRemaining = ipmbNumberOfRetries;
    state = ipmbRequestState::invalid;

    if (dataLength > 0)
    {
        data = std::move(inputData);
    }
}

void IpmbRequest::incomingMessageHandler()
{
    sdbusplus::message::message mesg =
        conn->new_signal(ipmbObj, hostIpmiIntf, "ReceivedMessage");
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

int IpmbRequest::ipmbToi2cConstruct(std::vector<uint8_t> &buffer)
{
    size_t bufferLength = dataLength + ipmbRequestDataHeaderLength +
                          ipmbConnectionHeaderLength + ipmbChecksumSize;

    if (bufferLength > ipmbMaxFrameLength)
    {
        return -1;
    }

    buffer.resize(bufferLength);
    static_assert(ipmbMaxFrameLength >= sizeof(IPMB_HEADER));
    auto ipmbBuffer = reinterpret_cast<IPMB_HEADER *>(buffer.data());

    // constructing buffer from ipmb request
    ipmbBuffer->Header.Req.address = address;
    ipmbBuffer->Header.Req.rsNetFnLUN = ipmbNetFnLunSet(netFn, rsLun);
    ipmbBuffer->Header.Req.rqSA = rqSA;
    ipmbBuffer->Header.Req.rqSeqLUN = ipmbSeqLunSet(seq, rqLun);
    ipmbBuffer->Header.Req.cmd = cmd;

    ipmbBuffer->Header.Resp.checksum1 = ipmbChecksumCompute(
        buffer.data(), ipmbConnectionHeaderLength - ipmbChecksumSize);

    if (dataLength > 0)
    {
        std::copy(data.begin(), data.end(), ipmbBuffer->Header.Req.data);
    }

    buffer[bufferLength - ipmbChecksumSize] =
        ipmbChecksumCompute(buffer.data() + ipmbChecksum2StartOffset,
                            (ipmbRequestDataHeaderLength + dataLength));

    return 0;
}

void IpmbRequest::requestSendFailedNotify()
{
    conn->async_method_call(
        [](boost::system::error_code &ec, int64_t status) {
            if (ec)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Error requestSendFailedNotify, async_method_call() ec");
            }

            if (status != 0)
            {
                phosphor::logging::log<phosphor::logging::level::DEBUG>(
                    "Error requestSendFailedNotify, status != 0");
            }
        },
        service, objpath, ipmbDbusIntf, "requestSendFailed", appSeq);
}

// TODO w/a to differentiate channel origin of incoming IPMI response: saving
// channel number at two oldest unused bits of seq
void IpmbRequest::addChannelToSeq(const ipmbChannelType &channelType)
{
    uint8_t newSeq = (seq | ((static_cast<uint8_t>(channelType) & 0x3) << 6));
    seq = newSeq;
}

/**
 * @brief Ipmb response class methods
 */
IpmbResponse::IpmbResponse()
{
    data.reserve(ipmbMaxDataSize);
    dataLength = 0;
}

IpmbResponse::IpmbResponse(uint8_t address, uint8_t netFn, uint8_t rqLun,
                           uint8_t rsSA, uint8_t seq, uint8_t rsLun,
                           uint8_t cmd, uint8_t completionCode,
                           std::vector<uint8_t> &inputData) :
    address(address),
    netFn(netFn), rqLun(rqLun), rsSA(rsSA), seq(seq), rsLun(rsLun), cmd(cmd),
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
    conn->async_method_call(
        [](boost::system::error_code &ec, int64_t status) {
            if (ec)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Error incomingMessageHandler, async_method_call() ec");
            }

            if (status != 0)
            {
                phosphor::logging::log<phosphor::logging::level::DEBUG>(
                    "Error incomingMessageHandler, status != 0");
            }
        },
        service, objpath, ipmbDbusIntf, "returnResponse", appSeq, netFn, rsLun,
        cmd, completionCode, data);
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

int IpmbResponse::ipmbToi2cConstruct(std::vector<uint8_t> &buffer)
{
    size_t bufferLength = dataLength + ipmbResponseDataHeaderLength +
                          ipmbConnectionHeaderLength + ipmbChecksumSize;

    if (bufferLength > ipmbMaxFrameLength)
    {
        return -1;
    }

    buffer.resize(bufferLength);
    auto ipmbBuffer = reinterpret_cast<IPMB_HEADER *>(buffer.data());

    ipmbBuffer->Header.Resp.address = address;
    ipmbBuffer->Header.Resp.rqNetFnLUN = ipmbNetFnLunSet(netFn, rqLun);
    ipmbBuffer->Header.Resp.rsSA = rsSA;
    ipmbBuffer->Header.Resp.rsSeqLUN = ipmbSeqLunSet(seq, rsLun);
    ipmbBuffer->Header.Resp.cmd = cmd;
    ipmbBuffer->Header.Resp.completionCode = completionCode;

    ipmbBuffer->Header.Resp.checksum1 = ipmbChecksumCompute(
        buffer.data(), ipmbConnectionHeaderLength - ipmbChecksumSize);

    if (dataLength > 0)
    {
        std::copy(data.begin(), data.end(), ipmbBuffer->Header.Resp.data);
    }

    buffer[bufferLength - ipmbChecksumSize] =
        ipmbChecksumCompute(buffer.data() + ipmbChecksum2StartOffset,
                            (ipmbResponseDataHeaderLength + dataLength));

    return 0;
}

/**
 * @brief Ipmb channel
 */
void IpmbChannel::ipmbResponseSend(std::shared_ptr<std::vector<uint8_t>> buffer,
                                   size_t retriesAttempted = 0)
{
    boost::asio::async_write(
        i2cMasterSocket,
        boost::asio::buffer(buffer->data() + ipmbAddressSize,
                            buffer->size() - ipmbAddressSize),
        [this, buffer, retriesAttempted](const boost::system::error_code ec,
                                         size_t bytesSent) {
            if (ec)
            {
                size_t currentRetryCnt = retriesAttempted;

                if (currentRetryCnt > ipmbI2cNumberOfRetries)
                {
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "Sent to I2C failed after retries");
                    return;
                }
                currentRetryCnt++;
                ipmbResponseSend(buffer, currentRetryCnt);
            }
        });
}

void IpmbChannel::ipmbRequestSend(std::shared_ptr<IpmbRequest> request,
                                  std::shared_ptr<std::vector<uint8_t>> buffer,
                                  size_t retriesAttempted = 0)
{
    boost::asio::async_write(
        i2cMasterSocket,
        boost::asio::buffer(buffer->data() + ipmbAddressSize,
                            buffer->size() - ipmbAddressSize),
        [this, request, buffer, retriesAttempted](
            const boost::system::error_code ec, size_t bytesSent) {
            if (ec)
            {
                size_t currentRetryCnt = retriesAttempted;

                if (currentRetryCnt > ipmbI2cNumberOfRetries)
                {
                    // transaction failed to be sent on I2C after retries - arm
                    // a timer and try again later
                    requestTimerCallback(request, buffer);
                    phosphor::logging::log<phosphor::logging::level::ERR>(
                        "Sent to I2C failed after retries");
                    return;
                }
                currentRetryCnt++;
                ipmbRequestSend(request, buffer, currentRetryCnt);
                return;
            }

            // transaction sent on I2C - arm a timer
            requestTimerCallback(request, buffer);
        });
}

/**
 * @brief Ipmb Outstanding Requests
 */
void IpmbChannel::makeRequestInvalid(std::shared_ptr<IpmbRequest> request)
{
    // change request state to invalid and remove it from outstanding requests
    // list
    request->state = ipmbRequestState::invalid;
    outstandingRequests[request->seq] = nullptr;
}

void IpmbChannel::makeRequestValid(std::shared_ptr<IpmbRequest> request)
{
    // change request state to valid and add it to outstanding requests list
    request->state = ipmbRequestState::valid;
    outstandingRequests[request->seq] = request;
}

bool IpmbChannel::seqNumGet(uint8_t &seq)
{
    static uint8_t seqNum = 0;

    for (int i = 0; i < ipmbMaxOutstandingRequestsCount; i++)
    {
        seqNum = ++seqNum & ipmbSeqMask;
        if (seqNum == ipmbMaxOutstandingRequestsCount)
        {
            seqNum = 0;
        }

        if (outstandingRequests[seqNum] == nullptr)
        {
            seq = seqNum;
            return true;
        }
    }

    return false;
}

void IpmbChannel::requestTimerCallback(
    std::shared_ptr<IpmbRequest> request,
    std::shared_ptr<std::vector<uint8_t>> buffer)
{
    request->timer->expires_after(
        std::chrono::milliseconds(ipmbRequestRetryTimeout));
    request->timer->async_wait(
        [this, request, buffer](const boost::system::error_code &ec) {
            // no response for sent request - retry
            if (ec)
            {
                if (ec == boost::asio::error::operation_aborted)
                {
                    return;
                }

                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Error requestTimerCallback()");
                request->requestSendFailedNotify();
                makeRequestInvalid(request);
                return;
            }

            // check if request is still valid
            if (request->state != ipmbRequestState::valid)
            {
                // request has been invalidated, nothing to do here
                return;
            }

            // check if any retries left
            if (request->retriesRemaining > 0)
            {
                // below will retry request, decrease retries remaining number
                // and reschedule a timer
                ipmbRequestSend(request, buffer);
                request->retriesRemaining--;
                return;
            }
            else
            {
                // no retries left, invalidate request
                request->requestSendFailedNotify();
                makeRequestInvalid(request);
            }
        });
}

void IpmbChannel::requestAdd(std::shared_ptr<IpmbRequest> request)
{
    // create timer and buffer used for initial sending and retrying requests
    request->timer = move(std::make_unique<boost::asio::steady_timer>(io));
    if (request->timer == nullptr)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error timer == null, requestAdd()");
        return;
    }
    std::shared_ptr<std::vector<uint8_t>> buffer =
        std::make_shared<std::vector<uint8_t>>();

    int status = request->ipmbToi2cConstruct(*buffer);
    if (status != 0)
    {
        return;
    }

    makeRequestValid(request);
    ipmbRequestSend(request, buffer);
}

std::shared_ptr<IpmbRequest>
    IpmbChannel::responseMatch(std::unique_ptr<IpmbResponse> &response)
{
    std::shared_ptr<IpmbRequest> request = outstandingRequests[response->seq];

    if (request != nullptr)
    {
        if (((ipmbRespNetFn(request->netFn)) == (response->netFn)) &&
            ((request->rqLun) == (response->rqLun)) &&
            ((request->rsLun) == (response->rsLun)) &&
            ((request->cmd) == (response->cmd)))
        {
            // match, response is corresponding to previously sent request
            // copy relevant client data
            response->service = std::move(request->service);
            response->objpath = std::move(request->objpath);
            response->appSeq = request->appSeq;

            makeRequestInvalid(request);
            request->timer->cancel();
            return request;
        }
    }

    return nullptr;
}

void IpmbChannel::processI2cEvent()
{
    do
    {
        std::array<uint8_t, ipmbMaxFrameLength> buffer{};

        lseek(ipmbi2cSlaveFd, 0, SEEK_SET);
        int r = read(ipmbi2cSlaveFd, buffer.data(), ipmbMaxFrameLength);
        if ((r < ipmbMinFrameLength) || (r > ipmbMaxFrameLength))
            break;

        auto ipmbFrame = reinterpret_cast<IPMB_HEADER *>(buffer.data());

        // valiate the frame
        if (!isFrameValid(ipmbFrame, r))
        {
            break;
        }

        // copy frame to ipmib message buffer
        if (ipmbIsResponse(ipmbFrame))
        {
            std::unique_ptr<IpmbResponse> ipmbMessageReceived =
                std::make_unique<IpmbResponse>();

            ipmbMessageReceived->i2cToIpmbConstruct(ipmbFrame, r);

            // lets match response with outstanding request
            auto ipmbMessageOutstanding = responseMatch(ipmbMessageReceived);
            if (ipmbMessageOutstanding)
            {
                // matching request found - send response to the client
                ipmbMessageReceived->incomingMessageHandler();
            }

            // no match - discard the message and proceed
        }
        else
        {
            std::unique_ptr<IpmbRequest> ipmbMessageReceived =
                std::make_unique<IpmbRequest>();

            ipmbMessageReceived->i2cToIpmbConstruct(ipmbFrame, r);

            // TODO w/a to differentiate channel origin of incoming IPMI
            // response: extracting channel number from seq
            ipmbMessageReceived->addChannelToSeq(getChannelType());

            // send request to the client
            ipmbMessageReceived->incomingMessageHandler();
        }
    } while (0);
    i2cSlaveSocket.async_wait(
        boost::asio::ip::tcp::socket::wait_error,
        [this](const boost::system::error_code &ec) {
            if (ec)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Error processI2cEvent()");
                return;
            }

            processI2cEvent();
        });
}

IpmbChannel::IpmbChannel(boost::asio::io_service &io,
                         uint8_t ipmbBmcSlaveAddress,
                         uint8_t ipmbRqSlaveAddress, ipmbChannelType type) :
    i2cSlaveSocket(io),
    i2cMasterSocket(io), ipmbBmcSlaveAddress(ipmbBmcSlaveAddress),
    ipmbRqSlaveAddress(ipmbRqSlaveAddress), type(type)
{
}

int IpmbChannel::ipmbChannelInit(const char *ipmbI2cSlave,
                                 const char *ipmbI2cMaster)
{
    // open fd to i2c slave device
    ipmbi2cSlaveFd = open(ipmbI2cSlave, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (ipmbi2cSlaveFd < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error opening ipmbI2cSlave");
        return -1;
    }

    // open fd to i2c master device
    ipmbi2cMasterFd = open(ipmbI2cMaster, O_RDWR | O_NONBLOCK);
    if (ipmbi2cMasterFd < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error opening ipmbI2cMaster");
        close(ipmbi2cSlaveFd);
        return -1;
    }

    // set slave address of recipient
    if (ioctl(ipmbi2cMasterFd, I2C_SLAVE,
              ipmbAddressTo7BitSet(ipmbRqSlaveAddress)) < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error setting ipmbi2cMasterFd slave address");
        close(ipmbi2cSlaveFd);
        close(ipmbi2cMasterFd);
        return -1;
    }

    i2cMasterSocket.assign(ipmbi2cMasterFd);
    i2cSlaveSocket.assign(boost::asio::ip::tcp::v4(), ipmbi2cSlaveFd);
    i2cSlaveSocket.async_wait(
        boost::asio::ip::tcp::socket::wait_error,
        [this](const boost::system::error_code &ec) {
            if (ec)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Error processI2cEvent()");
                return;
            }

            processI2cEvent();
        });

    return 0;
}

uint8_t IpmbChannel::getBmcSlaveAddress()
{
    return ipmbBmcSlaveAddress;
}

uint8_t IpmbChannel::getRqSlaveAddress()
{
    return ipmbRqSlaveAddress;
}

ipmbChannelType IpmbChannel::getChannelType()
{
    return type;
}

static IpmbChannel *getChannel(ipmbChannelType channelType)
{
    auto channel =
        std::find_if(ipmbChannels.begin(), ipmbChannels.end(),
                     [channelType](IpmbChannel &channel) {
                         return channel.getChannelType() == channelType;
                     });
    if (channel != ipmbChannels.end())
    {
        return &(*channel);
    }

    return nullptr;
}

static int initializeChannels()
{
    for (auto &channelConfig : ipmbChannelsConfig)
    {
        auto channel = ipmbChannels.emplace(
            ipmbChannels.end(), io, channelConfig.ipmbBmcSlaveAddress,
            channelConfig.ipmbRqSlaveAddress, channelConfig.type);

        if (channel->ipmbChannelInit(channelConfig.ipmbI2cSlave,
                                     channelConfig.ipmbI2cMaster) < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "initializeChannels: channel initialization failed");
            return -1;
        }
    }

    return 0;
}

/**
 * @brief Dbus callbacks
 */
auto ipmbSendMessage = [](uint8_t seq, uint8_t netfn, uint8_t lun, uint8_t cmd,
                          uint8_t cc, std::vector<uint8_t> &dataReceived) {
    int64_t status = -1;
    std::shared_ptr<std::vector<uint8_t>> buffer =
        std::make_shared<std::vector<uint8_t>>();

    do
    {
        if (dataReceived.size() > ipmbMaxDataSize)
        {
            status = -1;
            break;
        }

        if (netfn & ipmbNetFnResponseMask)
        {
            IpmbChannel *channel = getChannel(getChannelFromSeq(seq));
            if (channel == nullptr)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Error channel == nullptr ipmbSendMessage");
                break;
            }

            uint8_t rqSlaveAddress = channel->getRqSlaveAddress();
            uint8_t bmcSlaveAddress = channel->getBmcSlaveAddress();

            // response received
            // dataReceived is empty after constructor invocation
            std::unique_ptr<IpmbResponse> ipmbMessageReceived =
                std::make_unique<IpmbResponse>(rqSlaveAddress, netfn, lun,
                                               bmcSlaveAddress, seq, lun, cmd,
                                               cc, dataReceived);

            status = ipmbMessageReceived->ipmbToi2cConstruct(*buffer);
            if (status != 0)
            {
                break;
            }

            channel->ipmbResponseSend(buffer);
            status = 0;
        }
        else
        {
            // we are not expecting request here
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "ipmbMessageSend got a request");
            status = -1;
        }
    } while (0);

    return status;
};

void signalCallback(sdbusplus::message::message &requestMessage)
{
    uint32_t appSeq = 0;
    uint8_t netfn = 0, lun = 0, cmd = 0;
    uint8_t seqNum = 0;

    std::vector<uint8_t> dataReceived;
    dataReceived.reserve(ipmbMaxDataSize);

    requestMessage.read(appSeq, netfn, lun, cmd, dataReceived);

    if (dataReceived.size() > ipmbMaxDataSize)
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "signalCallback: received data size above maximum value");
        return;
    }

    // check outstanding request list for valid sequence number
    // default channel for received requests is ME
    IpmbChannel *channel = getChannel(ipmbChannelType::me);
    if (channel == nullptr)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "signalCallback: requested channel does not exist");
        return;
    }

    bool seqValid = channel->seqNumGet(seqNum);

    uint8_t bmcSlaveAddress = channel->getBmcSlaveAddress();
    uint8_t rqSlaveAddress = channel->getRqSlaveAddress();

    // construct the request to either add it to outstanding request list or
    // notify client app that it has not been sent
    std::shared_ptr<IpmbRequest> ipmbMessageReceived =
        std::make_shared<IpmbRequest>(rqSlaveAddress, netfn, ipmbRsLun,
                                      bmcSlaveAddress, seqNum, lun, cmd,
                                      dataReceived, requestMessage.get_sender(),
                                      requestMessage.get_path(), appSeq);

    if (seqValid)
    {
        channel->requestAdd(ipmbMessageReceived);
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::WARNING>(
            "cannot add more requests to the list");
        ipmbMessageReceived->requestSendFailedNotify();
    }
}

/**
 * @brief Main
 */
int main(int argc, char *argv[])
{
    conn->request_name(ipmbBus);

    sdbusplus::bus::match::match match(
        static_cast<sdbusplus::bus::bus &>(*conn), filter, signalCallback);

    auto server = sdbusplus::asio::object_server(conn);

    std::shared_ptr<sdbusplus::asio::dbus_interface> iface =
        server.add_interface(ipmbObj, hostIpmiIntf);

    iface->register_method("sendMessage", std::move(ipmbSendMessage));
    iface->initialize();

    if (initializeChannels() < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error initializeChannels");
        return -1;
    }

    io.run();

    return 0;
}
