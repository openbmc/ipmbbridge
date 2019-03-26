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

#include <fstream>
#include <nlohmann/json.hpp>
#include <phosphor-logging/log.hpp>
#include <tuple>

extern "C" {
#include <i2c/smbus.h>
#include <linux/i2c-dev.h>
}

/**
 * @brief Dbus
 */
static constexpr const char *ipmbBus = "xyz.openbmc_project.Ipmi.Channel.Ipmb";
static constexpr const char *ipmbObj = "/xyz/openbmc_project/Ipmi/Channel/Ipmb";
static constexpr const char *hostIpmiIntf = "org.openbmc.HostIpmi";
static constexpr const char *ipmbDbusIntf = "org.openbmc.Ipmb";

boost::asio::io_service io;
auto conn = std::make_shared<sdbusplus::asio::connection>(io);

static std::list<IpmbChannel> ipmbChannels;

/**
 * @brief Ipmb request class methods
 */
IpmbRequest::IpmbRequest()
{
    data.reserve(ipmbMaxDataSize);
}

IpmbRequest::IpmbRequest(uint8_t address, uint8_t netFn, uint8_t rsLun,
                         uint8_t rqSA, uint8_t seq, uint8_t rqLun, uint8_t cmd,
                         std::vector<uint8_t> &inputData) :
    address(address),
    netFn(netFn), rsLun(rsLun), rqSA(rqSA), seq(seq), rqLun(rqLun), cmd(cmd),
    timer(io)
{
    data.reserve(ipmbMaxDataSize);
    state = ipmbRequestState::invalid;

    if (inputData.size() > 0)
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

    size_t dataLength =
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
    size_t bufferLength = data.size() + ipmbRequestDataHeaderLength +
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

    if (data.size() > 0)
    {
        std::copy(data.begin(), data.end(), ipmbBuffer->Header.Req.data);
    }

    buffer[bufferLength - ipmbChecksumSize] =
        ipmbChecksumCompute(buffer.data() + ipmbChecksum2StartOffset,
                            (ipmbRequestDataHeaderLength + data.size()));

    return 0;
}

std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>
    IpmbRequest::returnMatchedResponse()
{
    return std::make_tuple(
        static_cast<int>(ipmbResponseStatus::success), matchedResponse->netFn,
        matchedResponse->rsLun, matchedResponse->cmd,
        matchedResponse->completionCode, matchedResponse->data);
}

static std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>
    returnStatus(ipmbResponseStatus status)
{
    // we only want to send status here, other fields are not relevant
    return std::make_tuple(static_cast<int>(status), 0, 0, 0, 0,
                           std::vector<uint8_t>(0));
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

    if (inputData.size() > 0)
    {
        data = std::move(inputData);
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

    size_t dataLength =
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
    size_t bufferLength = data.size() + ipmbResponseDataHeaderLength +
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

    if (data.size() > 0)
    {
        std::copy(data.begin(), data.end(), ipmbBuffer->Header.Resp.data);
    }

    buffer[bufferLength - ipmbChecksumSize] =
        ipmbChecksumCompute(buffer.data() + ipmbChecksum2StartOffset,
                            (ipmbResponseDataHeaderLength + data.size()));

    return 0;
}

bool IpmbCommandFilter::isBlocked(const uint8_t reqNetFn, const uint8_t cmd)
{
    auto blockedCmd = unhandledCommands.find({reqNetFn, cmd});

    if (blockedCmd != unhandledCommands.end())
    {
        return true;
    }

    return false;
}

void IpmbCommandFilter::addFilter(const uint8_t reqNetFn, const uint8_t cmd)
{
    if (unhandledCommands.insert({reqNetFn, cmd}).second)
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "addFilter: added command to filter",
            phosphor::logging::entry("netFn = %d", reqNetFn),
            phosphor::logging::entry("cmd = %d", cmd));
    }
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
                        "ipmbResponseSend: sent to I2C failed after retries");
                    return;
                }
                currentRetryCnt++;
                ipmbResponseSend(buffer, currentRetryCnt);
            }
        });
}

/**
 * @brief Ipmb Outstanding Requests
 */
void IpmbChannel::makeRequestInvalid(IpmbRequest &request)
{
    // change request state to invalid and remove it from outstanding requests
    // list
    request.state = ipmbRequestState::invalid;
    outstandingRequests[request.seq] = nullptr;
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

void IpmbChannel::responseMatch(std::unique_ptr<IpmbResponse> &response)
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
            request->state = ipmbRequestState::matched;
            request->timer->cancel();
            request->matchedResponse = std::move(response);
        }
    }
}

void IpmbChannel::processI2cEvent()
{
    std::array<uint8_t, ipmbMaxFrameLength> buffer{};
    auto ipmbFrame = reinterpret_cast<IPMB_HEADER *>(buffer.data());

    lseek(ipmbi2cSlaveFd, 0, SEEK_SET);
    int r = read(ipmbi2cSlaveFd, buffer.data(), ipmbMaxFrameLength);
    if ((r < ipmbMinFrameLength) || (r > ipmbMaxFrameLength))
    {
        goto end;
    }

    // valiate the frame
    if (!isFrameValid(ipmbFrame, r))
    {
        goto end;
    }

    // copy frame to ipmib message buffer
    if (ipmbIsResponse(ipmbFrame))
    {
        std::unique_ptr<IpmbResponse> ipmbMessageReceived =
            std::make_unique<IpmbResponse>();

        ipmbMessageReceived->i2cToIpmbConstruct(ipmbFrame, r);

        // try to match response with outstanding request
        responseMatch(ipmbMessageReceived);
    }
    else
    {
        // if command is blocked - respond with 'invalid command'
        // completion code
        if (commandFilter)
        {
            uint8_t netFn = ipmbNetFnGet(ipmbFrame->Header.Req.rsNetFnLUN);
            uint8_t cmd = ipmbFrame->Header.Req.cmd;

            if (commandFilter->isBlocked(netFn, cmd))
            {
                uint8_t seq = ipmbSeqGet(ipmbFrame->Header.Req.rqSeqLUN);
                uint8_t lun =
                    ipmbLunFromSeqLunGet(ipmbFrame->Header.Req.rqSeqLUN);
                std::vector<uint8_t> data;

                // prepare generic response
                auto ipmbResponse =
                    IpmbResponse(ipmbRqSlaveAddress, ipmbRespNetFn(netFn), lun,
                                 ipmbBmcSlaveAddress, seq, ipmbRsLun, cmd,
                                 ipmbIpmiInvalidCommand, data);

                std::shared_ptr<std::vector<uint8_t>> buffer =
                    std::make_shared<std::vector<uint8_t>>();

                if (ipmbResponse.ipmbToi2cConstruct(*buffer) == 0)
                {
                    ipmbResponseSend(buffer);
                }

                goto end;
            }
        }

        auto ipmbMessageReceived = IpmbRequest();

        ipmbMessageReceived.i2cToIpmbConstruct(ipmbFrame, r);

        // TODO w/a to differentiate channel origin of incoming IPMI
        // response: extracting channel number from seq
        ipmbMessageReceived.addChannelToSeq(getChannelType());

        // send request to the client
        ipmbMessageReceived.incomingMessageHandler();
    }

end:
    i2cSlaveSocket.async_wait(
        boost::asio::ip::tcp::socket::wait_error,
        [this](const boost::system::error_code &ec) {
            if (ec)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "Error: processI2cEvent()");
                return;
            }

            processI2cEvent();
        });
}

IpmbChannel::IpmbChannel(boost::asio::io_service &io,
                         uint8_t ipmbBmcSlaveAddress,
                         uint8_t ipmbRqSlaveAddress, ipmbChannelType type,
                         std::shared_ptr<IpmbCommandFilter> commandFilter) :
    i2cSlaveSocket(io),
    i2cMasterSocket(io), ipmbBmcSlaveAddress(ipmbBmcSlaveAddress),
    ipmbRqSlaveAddress(ipmbRqSlaveAddress), type(type),
    commandFilter(commandFilter)
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
            "ipmbChannelInit: error opening ipmbI2cSlave");
        return -1;
    }

    // open fd to i2c master device
    ipmbi2cMasterFd = open(ipmbI2cMaster, O_RDWR | O_NONBLOCK);
    if (ipmbi2cMasterFd < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "ipmbChannelInit: error opening ipmbI2cMaster");
        close(ipmbi2cSlaveFd);
        return -1;
    }

    // set slave address of recipient
    if (ioctl(ipmbi2cMasterFd, I2C_SLAVE,
              ipmbAddressTo7BitSet(ipmbRqSlaveAddress)) < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "ipmbChannelInit: error setting ipmbi2cMasterFd slave address");
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
                    "Error: processI2cEvent()");
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

void IpmbChannel::addFilter(const uint8_t respNetFn, const uint8_t cmd)
{
    if (commandFilter)
    {
        commandFilter->addFilter(respNetFn, cmd);
    }
}

std::tuple<int, uint8_t, uint8_t, uint8_t, uint8_t, std::vector<uint8_t>>
    IpmbChannel::requestAdd(boost::asio::yield_context &yield,
                            std::shared_ptr<IpmbRequest> request)
{
    makeRequestValid(request);

    std::vector<uint8_t> buffer(0);
    if (request->ipmbToi2cConstruct(buffer) != 0)
    {
        return returnStatus(ipmbResponseStatus::error);
    }

    for (int i = 0; i < ipmbNumberOfTries; i++)
    {
        boost::system::error_code ec;
        int i2cRetryCnt = 0;

        for (; i2cRetryCnt < ipmbI2cNumberOfRetries; i2cRetryCnt++)
        {
            boost::asio::async_write(
                i2cMasterSocket,
                boost::asio::buffer(buffer.data() + ipmbAddressSize,
                                    buffer.size() - ipmbAddressSize),
                yield[ec]);

            if (ec)
            {
                continue; // retry
            }
            break;
        }

        if (i2cRetryCnt == ipmbI2cNumberOfRetries)
        {
            phosphor::logging::log<phosphor::logging::level::INFO>(
                "requestAdd: Sent to I2C failed after retries");
        }

        request->timer->expires_after(
            std::chrono::milliseconds(ipmbRequestRetryTimeout));
        request->timer->async_wait(yield[ec]);

        if (ec && ec != boost::asio::error::operation_aborted)
        {
            // unexpected error - invalidate request and return generic error
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "requestAdd: async_wait error");
            makeRequestInvalid(*request);
            return returnStatus(ipmbResponseStatus::error);
        }

        if (request->state == ipmbRequestState::matched)
        {
            // matched response, send it to client application
            makeRequestInvalid(*request);
            return request->returnMatchedResponse();
        }
    }

    makeRequestInvalid(*request);
    return returnStatus(ipmbResponseStatus::timeout);
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
    std::shared_ptr<IpmbCommandFilter> commandFilter =
        std::make_shared<IpmbCommandFilter>();

    const char *config_file_path = "/usr/share/ipmb-channels.json";
    std::ifstream config_file(config_file_path);
    if (!config_file.is_open())
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "initializeChannels: Cannot open config path");
        return -1;
    }
    try
    {
        auto data = nlohmann::json::parse(config_file, nullptr);
        for (auto channelConfig : data["channels"])
        {
            std::string typeConfig = channelConfig["type"];
            std::string masterPath = channelConfig["master-path"];
            std::string slavePath = channelConfig["slave-path"];
            uint8_t bmcAddr = channelConfig["bmc-addr"];
            uint8_t reqAddr = channelConfig["remote-addr"];
            ipmbChannelType type = typeConfig == "me" ? ipmbChannelType::me
                                                      : ipmbChannelType::ipmb;
            auto channel = ipmbChannels.emplace(ipmbChannels.end(), io, bmcAddr,
                                                reqAddr, type, commandFilter);
            if (channel->ipmbChannelInit(slavePath.c_str(),
                                         masterPath.c_str()) < 0)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>(
                    "initializeChannels: channel initialization failed");
                return -1;
            }
        }
    }
    catch (...)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "initializeChannels: Error parsing config file");
        return -1;
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

    if (dataReceived.size() > ipmbMaxDataSize)
    {
        return status;
    }

    if (netfn & ipmbNetFnResponseMask)
    {
        IpmbChannel *channel = getChannel(getChannelFromSeq(seq));
        if (channel == nullptr)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "ipmbSendMessage: channel does not exist");
            return status;
        }

        // if command is not supported, add it to filter
        if (cc == ipmbIpmiInvalidCommand)
        {
            channel->addFilter(ipmbReqNetFnFromRespNetFn(netfn), cmd);
        }

        uint8_t rqSlaveAddress = channel->getRqSlaveAddress();
        uint8_t bmcSlaveAddress = channel->getBmcSlaveAddress();

        // response received
        // dataReceived is empty after constructor invocation
        std::unique_ptr<IpmbResponse> ipmbMessageReceived =
            std::make_unique<IpmbResponse>(rqSlaveAddress, netfn, lun,
                                           bmcSlaveAddress, seq, lun, cmd, cc,
                                           dataReceived);

        status = ipmbMessageReceived->ipmbToi2cConstruct(*buffer);
        if (status != 0)
        {
            return status;
        }

        channel->ipmbResponseSend(buffer);
        return status;
    }

    // we are not expecting request here
    phosphor::logging::log<phosphor::logging::level::ERR>(
        "ipmbSendMessage: got a request");
    return status;
};

auto ipmbHandleRequest = [](boost::asio::yield_context yield,
                            uint8_t reqChannel, uint8_t netfn, uint8_t lun,
                            uint8_t cmd, std::vector<uint8_t> dataReceived) {
    IpmbChannel *channel = getChannel(static_cast<ipmbChannelType>(reqChannel));
    if (channel == nullptr)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "ipmbHandleRequest: requested channel does not exist");
        return returnStatus(ipmbResponseStatus::invalid_param);
    }

    // check outstanding request list for valid sequence number
    uint8_t seqNum = 0;
    bool seqValid = channel->seqNumGet(seqNum);
    if (!seqValid)
    {
        phosphor::logging::log<phosphor::logging::level::WARNING>(
            "ipmbHandleRequest: cannot add more requests to the list");
        return returnStatus(ipmbResponseStatus::busy);
    }

    uint8_t bmcSlaveAddress = channel->getBmcSlaveAddress();
    uint8_t rqSlaveAddress = channel->getRqSlaveAddress();

    // construct the request to add it to outstanding request list
    std::shared_ptr<IpmbRequest> request = std::make_shared<IpmbRequest>(
        rqSlaveAddress, netfn, ipmbRsLun, bmcSlaveAddress, seqNum, lun, cmd,
        dataReceived);

    if (!request->timer)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "ipmbHandleRequest: timer object does not exist");
        return returnStatus(ipmbResponseStatus::error);
    }

    return channel->requestAdd(yield, request);
};

/**
 * @brief Main
 */
int main(int argc, char *argv[])
{
    conn->request_name(ipmbBus);

    auto server = sdbusplus::asio::object_server(conn);

    std::shared_ptr<sdbusplus::asio::dbus_interface> ipmiIface =
        server.add_interface(ipmbObj, hostIpmiIntf);
    std::shared_ptr<sdbusplus::asio::dbus_interface> ipmbIface =
        server.add_interface(ipmbObj, ipmbDbusIntf);

    ipmiIface->register_method("sendMessage", std::move(ipmbSendMessage));
    ipmbIface->register_method("sendRequest", std::move(ipmbHandleRequest));
    ipmiIface->initialize();
    ipmbIface->initialize();

    if (initializeChannels() < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error initializeChannels");
        return -1;
    }

    io.run();
    return 0;
}
