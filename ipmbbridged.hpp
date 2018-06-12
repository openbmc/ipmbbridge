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

#include <vector>
#include <memory>
#include <signal.h>
#include <time.h>

#include "ipmbdefines.hpp"

#ifndef IPMBBRIDGED_HPP
#define IPMBBRIDGED_HPP

/**
 * @brief Ipmb poll descriptors
 */
enum
{
    IPMB_SD_BUS_FD = 0,
    IPMB_I2C_FD,
    IPMB_TOTAL_FDS
};

/**
 * @brief Ipmb addresses (8bit)
 */
const uint8_t ipmbBmcSlaveAddress = 0x20;
const uint8_t ipmbRqSlaveAddress = 0x58;

/**
 * @brief Ipmb defines
 */
const size_t ipmbMaxDataSize = 256;
const size_t ipmbConnectionHeaderLength = 3;
const size_t ipmbResponseDataHeaderLength = 4;
const size_t ipmbRequestDataHeaderLength = 3;
const size_t ipmbAddressSize = 1;
const size_t ipmbChecksumSize = 1;
const size_t ipmbChecksum2StartOffset = 3;
const size_t ipmbMinFrameLength = 7;
const size_t ipmbMaxFrameLength = ipmbConnectionHeaderLength +
                                  ipmbResponseDataHeaderLength +
                                  ipmbChecksumSize + ipmbMaxDataSize;

/**
 * @brief Ipmb outstanding requests defines
 */
const uint8_t ipmbNumberOfRetries = 5;
const uint64_t ipmbRequestRetryTimeout = 250;
const int ipmbOneUs = 1;

/**
 * @brief Ipmb misc
 */
const uint8_t ipmbI2cNumberOfRetries = 5;
const uint8_t ipmbNetFnResponseMask = 0x01;
const uint8_t ipmbLunMask = 0x03;

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
 * @brief Ipmb time
 */
constexpr uint64_t ipmbMsToUs(uint64_t timeMs)
{
    return timeMs * 1000;
}

constexpr uint64_t ipmbMsToNs(uint64_t timeMs)
{
    return timeMs * 1000000;
}

constexpr uint64_t ipmbUsToNs(uint64_t timeUs)
{
    return timeUs * 1000;
}

/**
 * @brief Ipmb checkers
 */
constexpr bool ipmbIsResponse(IPMB_HEADER *ipmbHeader)
{
    return ipmbNetFnGet(ipmbHeader->Header.Resp.rqNetFnLUN) &
           ipmbNetFnResponseMask;
}

inline uint64_t ipmbCurrentTime()
{
    struct timespec timespec;
    clock_gettime(CLOCK_MONOTONIC, &timespec);
    return ((timespec.tv_sec) * 1000000 + (timespec.tv_nsec) / 1000);
}

/**
 * @brief IpmbMessage class declaration
 */
class IpmbMessage
{
  public:
    // virtual function for handling incoming messages (i2c -> client)
    virtual int incomingMessageHandler() = 0;

    // virtual function for constructing ipmb message from i2c buffer
    virtual void i2cToIpmbConstruct(IPMB_HEADER *ipmbBuffer,
                                    size_t dataLength) = 0;

    // virtual function for constructing the i2c buffer
    virtual void ipmbToi2cConstruct(uint8_t *buffer, size_t &bufferLength) = 0;

    virtual ~IpmbMessage(){};

  protected:
    IpmbMessage(){};
};

/**
 * @brief IpmbRequest class declaration
 */
class IpmbRequest : public IpmbMessage
{
  public:
    uint8_t netFn;
    uint8_t rsLun;
    uint8_t rqSA;
    uint8_t seq;
    uint8_t rqLun;
    uint8_t cmd;
    std::vector<uint8_t> data;

    size_t dataLength;
    uint64_t timeToRetryUs;
    uint8_t retriesRemaining;

    IpmbRequest();

    IpmbRequest(uint8_t netFn, uint8_t rsLun, uint8_t rqSA, uint8_t seq,
                uint8_t rqLun, uint8_t cmd, std::vector<uint8_t> &inputData);

    void updateTimeout();

    int incomingMessageHandler();

    void i2cToIpmbConstruct(IPMB_HEADER *ipmbBuffer, size_t bufferLength);

    void ipmbToi2cConstruct(uint8_t *buffer, size_t &bufferLength);

    uint64_t retryAbsTimeGet();
};

/**
 * @brief IpmbResponse class declaration
 */
class IpmbResponse : public IpmbMessage
{
  public:
    uint8_t netFn;
    uint8_t rqLun;
    uint8_t rsSA;
    uint8_t seq;
    uint8_t rsLun;
    uint8_t cmd;
    uint8_t completionCode;
    std::vector<uint8_t> data;

    size_t dataLength;

    IpmbResponse();

    IpmbResponse(uint8_t netFn, uint8_t rqLun, uint8_t rsSA, uint8_t seq,
                 uint8_t rsLun, uint8_t cmd, uint8_t completionCode,
                 std::vector<uint8_t> &inputData);

    int incomingMessageHandler();

    void i2cToIpmbConstruct(IPMB_HEADER *ipmbBuffer, size_t bufferLength);

    void ipmbToi2cConstruct(uint8_t *buffer, size_t &bufferLength);
};

/**
 * @brief IpmbOutstandingRequests class declaration
 */
class IpmbOutstandingRequests
{
  private:
    // list storing outstanding requests
    std::vector<std::unique_ptr<IpmbMessage>> outstandingRequests;

    // structures used for timer
    timer_t timerid;
    sigset_t mask;
    struct sigevent sev;
    struct sigaction sa;

  public:
    int ipmbOutstandingRequestsInit();

    void requestAdd(std::unique_ptr<IpmbMessage> &request);

    int ipmbTimerUpdate(uint64_t timeToRetryUs);

    void timerUpdate();

    void retryRequest();

    std::unique_ptr<IpmbMessage>
        responseMatch(std::unique_ptr<IpmbMessage> &response);
};

#endif
