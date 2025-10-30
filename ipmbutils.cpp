// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright 2018 Intel

#include "ipmbbridged.hpp"
#include "ipmbdefines.hpp"

#include <inttypes.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/**
 * @brief Ipmb utils for checksum
 */
bool ipmbChecksumValidate(uint8_t* data, uint8_t length)
{
    uint8_t checksum = 0;

    // compute checksum.
    for (uint8_t idx = 0; idx < length; idx++)
    {
        checksum += data[idx];
    }

    // check if checksum is valid.
    if (0 == checksum)
    {
        // checksum valid.
        return true;
    }
    else
    {
        // checksum invalid.
        return false;
    }
}

uint8_t ipmbChecksumCompute(uint8_t* data, uint8_t length)
{
    uint8_t checksum = 0;

    // compute checksum.
    for (uint8_t idx = 0; idx < length; idx++)
    {
        checksum += data[idx];
    }

    checksum = (~checksum) + 1;

    // return computed checksum value.
    return checksum;
}

inline bool ipmbConnectionHeaderChecksumValidate(IPMB_HEADER* ipmbHeader)
{
    return ipmbChecksumValidate(reinterpret_cast<uint8_t*>(ipmbHeader),
                                ipmbConnectionHeaderLength);
}

inline bool ipmbDataChecksumValidate(IPMB_HEADER* ipmbHeader, uint8_t length)
{
    return ipmbChecksumValidate(
        (reinterpret_cast<uint8_t*>(ipmbHeader) + ipmbConnectionHeaderLength),
        (length - ipmbConnectionHeaderLength));
}

bool isFrameValid(IPMB_HEADER* frame, uint8_t length)
{
    bool frameValid = ipmbConnectionHeaderChecksumValidate(frame);
    if (false == frameValid)
    {
        // invalid connection header checksum.
        return false;
    }

    // data checksum validation.
    return ipmbDataChecksumValidate(frame, length);
}
