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

#include "ipmbdefines.hpp"

#include <boost/asio.hpp>
#include <sdbusplus/message.hpp>
#include <vector>

#ifndef IPMBBRIDGED_HPP
#define IPMBBRIDGED_HPP

/**
 * @brief Ipmb outstanding requests defines
 */
constexpr int ipmbMaxOutstandingRequestsCount = 64;
constexpr int ipmbNumberOfRetries = 5;
constexpr uint64_t ipmbRequestRetryTimeout = 250; // ms

/**
 * @brief Ipmb I2C communication
 */
constexpr uint8_t ipmbI2cNumberOfRetries = 5;

/**
 * @brief Ipmb defines
 */
constexpr size_t ipmbMaxDataSize = 256;
constexpr size_t ipmbConnectionHeaderLength = 3;
constexpr size_t ipmbResponseDataHeaderLength = 4;
constexpr size_t ipmbRequestDataHeaderLength = 3;
constexpr size_t ipmbAddressSize = 1;
constexpr size_t ipmbChecksumSize = 1;
constexpr size_t ipmbChecksum2StartOffset = 3;
constexpr size_t ipmbMinFrameLength = 7;
constexpr size_t ipmbMaxFrameLength = ipmbConnectionHeaderLength +
                                      ipmbResponseDataHeaderLength +
                                      ipmbChecksumSize + ipmbMaxDataSize;

/**
 * @brief Ipmb misc
 */
constexpr uint8_t ipmbNetFnResponseMask = 0x01;
constexpr uint8_t ipmbLunMask = 0x03;
constexpr uint8_t ipmbSeqMask = 0x3F;
constexpr uint8_t ipmbRsLun = 0x0;

/**
 * @brief Ipmb setters
 */
constexpr uint8_t ipmbNetFnLunSet(uint8_t netFn, uint8_t lun)
{
    return ((netFn << 2) | (lun & ipmbLunMask));
}

constexpr uint8_t ipmbSeqLunSet(uint8_t seq, uint8_t lun)
{
    return ((seq << 2) | (lun & ipmbLunMask));
}

constexpr uint8_t ipmbAddressTo7BitSet(uint8_t address)
{
    return address >> 1;
}

constexpr uint8_t ipmbRespNetFn(uint8_t netFn)
{
    return netFn |= 1;
}

/**
 * @brief Ipmb getters
 */
constexpr uint8_t ipmbNetFnGet(uint8_t netFnLun)
{
    return netFnLun >> 2;
}

constexpr uint8_t ipmbLunFromNetFnLunGet(uint8_t netFnLun)
{
    return netFnLun & ipmbLunMask;
}

constexpr uint8_t ipmbSeqGet(uint8_t seqNumLun)
{
    return seqNumLun >> 2;
}

constexpr uint8_t ipmbLunFromSeqLunGet(uint8_t seqNumLun)
{
    return seqNumLun & ipmbLunMask;
}

/**
 * @brief Ipmb checkers
 */
constexpr bool ipmbIsResponse(IPMB_HEADER *ipmbHeader)
{
    return ipmbNetFnGet(ipmbHeader->Header.Resp.rqNetFnLUN) &
           ipmbNetFnResponseMask;
}

enum class ipmbRequestState
{
    invalid,
    valid,
};

/**
 * @brief Channel types
 */
enum class ipmbChannelType
{
    ipmb = 0,
    me = 1
};

/**
 * @brief Channel configuration structure
 */
struct IpmbChannelConfig
{
    ipmbChannelType type;
    const char *ipmbI2cSlave;
    const char *ipmbI2cMaster;
    uint8_t ipmbBmcSlaveAddress;
    uint8_t ipmbRqSlaveAddress;
};

// TODO w/a to differentiate channel origin of incoming IPMI response:
// extracting channel number from 2 oldest bits of seq
constexpr ipmbChannelType getChannelFromSeq(const uint8_t &seq)
{
    return static_cast<ipmbChannelType>((seq & 0xC0) >> 6);
}

/**
 * @brief IpmbRequest declaration
 */
struct IpmbRequest
{
    uint8_t address;
    uint8_t netFn;
    uint8_t rsLun;
    uint8_t rqSA;
    uint8_t seq;
    uint8_t rqLun;
    uint8_t cmd;
    std::vector<uint8_t> data;

    size_t dataLength;
    uint8_t retriesRemaining;
    ipmbRequestState state;
    std::unique_ptr<boost::asio::steady_timer> timer;

    // client data
    std::string service;
    std::string objpath;
    uint32_t appSeq;

    IpmbRequest();

    IpmbRequest(uint8_t address, uint8_t netFn, uint8_t rsLun, uint8_t rqSA,
                uint8_t seq, uint8_t rqLun, uint8_t cmd,
                std::vector<uint8_t> &inputData, const char *serviceSender,
                const char *objpathSender, uint32_t appSeq);

    void incomingMessageHandler();

    void requestSendFailedNotify();

    void i2cToIpmbConstruct(IPMB_HEADER *ipmbBuffer, size_t bufferLength);

    int ipmbToi2cConstruct(std::vector<uint8_t> &buffer);

    // TODO w/a to differentiate channel origin of incoming IPMI response:
    // saving channel number at two oldest unused bits of seq
    void addChannelToSeq(const ipmbChannelType &channelType);
};

/**
 * @brief IpmbResponse declaration
 */
struct IpmbResponse
{
    uint8_t address;
    uint8_t netFn;
    uint8_t rqLun;
    uint8_t rsSA;
    uint8_t seq;
    uint8_t rsLun;
    uint8_t cmd;
    uint8_t completionCode;
    std::vector<uint8_t> data;

    size_t dataLength;

    // client data
    std::string service;
    std::string objpath;
    uint32_t appSeq;

    IpmbResponse();

    IpmbResponse(uint8_t address, uint8_t netFn, uint8_t rqLun, uint8_t rsSA,
                 uint8_t seq, uint8_t rsLun, uint8_t cmd,
                 uint8_t completionCode, std::vector<uint8_t> &inputData);

    void incomingMessageHandler();

    void i2cToIpmbConstruct(IPMB_HEADER *ipmbBuffer, size_t bufferLength);

    int ipmbToi2cConstruct(std::vector<uint8_t> &buffer);
};

/**
 * @brief IpmbChannel class declaration
 */
class IpmbChannel
{
  private:
    boost::asio::ip::tcp::socket i2cSlaveSocket;
    boost::asio::posix::stream_descriptor i2cMasterSocket;

    int ipmbi2cMasterFd;
    int ipmbi2cSlaveFd;

    uint8_t ipmbBmcSlaveAddress;
    uint8_t ipmbRqSlaveAddress;

    ipmbChannelType type;

    uint8_t initialized;

    // list storing outstanding requests
    std::array<std::shared_ptr<IpmbRequest>, ipmbMaxOutstandingRequestsCount>
        outstandingRequests;

    void makeRequestInvalid(std::shared_ptr<IpmbRequest> request);

    void makeRequestValid(std::shared_ptr<IpmbRequest> request);

    void requestTimerCallback(std::shared_ptr<IpmbRequest> request,
                              std::shared_ptr<std::vector<uint8_t>> buffer);

    std::shared_ptr<IpmbRequest>
        responseMatch(std::unique_ptr<IpmbResponse> &response);

  public:
    void requestAdd(std::shared_ptr<IpmbRequest> requestToSend);

    bool seqNumGet(uint8_t &seq);

    ipmbChannelType getChannelType();

    uint8_t getBmcSlaveAddress();

    uint8_t getRqSlaveAddress();

    void processI2cEvent();

    void ipmbResponseSend(std::shared_ptr<std::vector<uint8_t>> buffer,
                          size_t retriesAttempted);

    void ipmbRequestSend(std::shared_ptr<IpmbRequest> request,
                         std::shared_ptr<std::vector<uint8_t>> buffer,
                         size_t retriesAttempted);

    IpmbChannel(boost::asio::io_service &io, uint8_t ipmbBmcSlaveAddress,
                uint8_t ipmbRqSlaveAddress, ipmbChannelType type);

    int ipmbChannelInit(const char *ipmbI2cSlave, const char *ipmbI2cMaster);
};

#endif
