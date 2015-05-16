/*
MB85RC04 driver code is placed under the BSD license.
Written by Egor Fedorov (egor.fedorov@emlid.com)
Copyright (c) 2014, Emlid Limited
Copyright (c) 2014-2015, CAVOK IT, B. Rexwinkel (development@cavok-it.nl)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Emlid Limited nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL EMLID LIMITED BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "MB85RC04.h"

MB85RC04::MB85RC04(const char* i2cDev, uint8_t address)
{
    this->i2cDev = std::string(i2cDev);
    this->device_address = address;
}

bool MB85RC04::testConnection()
{
    // Try to read 16 bytes from a random location
    const uint8_t length = 16;
    uint16_t reg_address = rand() % (512 - length);
    uint8_t data[length];

    if (readBytes(reg_address, length, data))
        return true;

    return false;
}

int8_t MB85RC04::readByte(uint16_t register_address, uint8_t* data)
{
    return readBytes(register_address, 1, data);
}

int8_t MB85RC04::readBytes(uint16_t register_address, uint8_t length, uint8_t* data)
{
    bool ninth_bit = register_address & 0x100;
    uint8_t dev_address = device_address | ninth_bit;
    register_address = (uint8_t)(register_address & 0x00FF);

    return I2Cdev::readBytes(i2cDev.c_str(), dev_address, register_address, length, data);
}

bool MB85RC04::writeByte(uint16_t register_address, uint8_t* data)
{
    return writeBytes(register_address, 1, data);
}

bool MB85RC04::writeBytes(uint16_t register_address, uint8_t length, uint8_t* data)
{
    bool ninth_bit = register_address & 0x100;
    uint8_t dev_address = device_address | ninth_bit;
    register_address = (uint8_t)(register_address & 0x00FF);

    return I2Cdev::writeBytes(i2cDev.c_str(), dev_address, register_address, length, data);
}
