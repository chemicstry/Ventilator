/* Copyright 2020-2021, RespiraWorks

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#pragma once

#include <cstdint>

// The polynomial 0x741B8CD7 has Hamming distance 6 up to 16360 bits
// and Hamming distance 4 up to 114663 bits.
//[Philip Koopman, 32-Bit Cyclic Redundancy Codes for Internet Applications
// 2002.] https://users.ece.cmu.edu/~koopman/crc/
constexpr uint32_t Crc32Polynomial{0x741B8CD7};

// Calculates CRC32 of a given byte array.
// NOTE: each byte separately is passed to CRC calculation, i.e. bytes are not
// packed to uint32_t value, but extended with zeroes in upper bits. I.e. 0x42
// will be interpreted as 0x00000042. This is to have simpler code as we are
// dealing with bytes when calculating CRC.
// @param data - data on which to calculate CRC32
// @param length - length of the data
// @returns CRC32 if length > 0, 0 otherwise
uint32_t soft_crc32(const uint8_t *data, uint32_t length);

// Performs a single CRC32 calculation.
// @param crc - CRC value pre-computed in earlier iteration, or initial CRC
// value
// @param data - byte on which to calculate the CRC
// @returns CRC32
uint32_t crc32_single(uint32_t crc, uint8_t data);

// Assumes last 4 bytes in buf are CRC32, calculates CRC for buf[0:len-4] and
// checks if they match.
// @returns true if CRC match, false if not match or length < 5
bool crc_ok(const uint8_t *buf, uint32_t len);
