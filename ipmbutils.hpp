// SPDX-License-Identifier: Apache-2.0
// SPDX-FileCopyrightText: Copyright 2018 Intel

#ifndef IPMBUTILS_HPP
#define IPMBUTILS_HPP

/**
 * @brief function performing IPMB header and data checksum validation
 */
bool isFrameValid(IPMB_HEADER* frame, uint8_t length);

/**
 * @brief function computing IPMB layer checksum
 */
uint8_t ipmbChecksumCompute(uint8_t* data, uint8_t length);

#endif
