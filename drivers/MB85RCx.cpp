/* ======================================================================================================== */
/* Based on (BSD-licensed) MB85RC04 driver code written by Egor Fedorov (egor.fedorov@emlid.com)            */
/* Copyright (c) 2014, Emlid Limited                                                                        */
/* Copyright (c) 2014-2015, CAVOK IT, B. Rexwinkel (development@cavok-it.nl)                                */
/* All rights reserved.                                                                                     */
/*                                                                                                          */
/* Redistribution and use in source and binary forms, with or                                               */
/* without modification, are permitted provided that the following                                          */
/* conditions are met:                                                                                      */
/*                                                                                                          */
/* 1. Redistributions of source code must retain the above copyright                                        */
/*    notice, this list of conditions and the following disclaimer.                                         */
/*                                                                                                          */
/* 2. Redistributions in binary form must reproduce the above                                               */
/*    copyright notice, this list of conditions and the following                                           */
/*    disclaimer in the documentation and/or other materials provided                                       */
/*    with the distribution.                                                                                */
/*                                                                                                          */
/* 3. Neither the name of the copyright holder nor the names of its                                         */
/*    contributors may be used to endorse or promote products derived                                       */
/*    from this software without specific prior written permission.                                         */
/*                                                                                                          */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED   */
/* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A   */
/* PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE      */
/* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT   */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS      */
/* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR   */
/* TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF     */
/* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                               */
/* ======================================================================================================== */

#include "MB85RCx.h"


MB85RC04::MB85RC04(const char* i2cDev, uint8_t address)
{
    _i2cDev = std::string(i2cDev);
    _device_address = address;
}

MB85RC04::~MB85RC04()
{

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
    uint8_t dev_address = _device_address | ninth_bit;
    register_address = (uint8_t)(register_address & 0x00FF);

    return I2Cdev::readBytes(_i2cDev.c_str(), dev_address, register_address, length, data);
}

bool MB85RC04::writeByte(uint16_t register_address, uint8_t* data)
{
    return writeBytes(register_address, 1, data);
}

bool MB85RC04::writeBytes(uint16_t register_address, uint8_t length, uint8_t* data)
{
    bool ninth_bit = register_address & 0x100;
    uint8_t dev_address = _device_address | ninth_bit;
    register_address = (uint8_t)(register_address & 0x00FF);

    return I2Cdev::writeBytes(_i2cDev.c_str(), dev_address, register_address, length, data);
}


MB85RC256::MB85RC256(const char* i2cDev, uint8_t address)
{
    _i2cDev = std::string(i2cDev);
    _device_address = address;
}

MB85RC256::~MB85RC256()
{

}

bool MB85RC256::testConnection()
{
    // Try to read 16 bytes from a random location
    const uint8_t length = 16;
    uint16_t reg_address = rand() % (32768 - length);
    uint8_t data[length];

    if (readBytes(reg_address, length, data))
        return true;

    return false;
}

int8_t MB85RC256::readByte(uint16_t register_address, uint8_t* data)
{
    return readBytes(register_address, 1, data);
}

int8_t MB85RC256::readBytes(uint16_t register_address, uint8_t length, uint8_t* data)
{
    uint8_t register_address_low = register_address & 0x00FF;
    uint8_t register_address_high = (register_address >> 8) & 0x007F;   // address is 15-bits wide

    I2Cdev::writeByte(_i2cDev.c_str(), _device_address, register_address_high, register_address_low);   // This sets the address to read from
    return I2Cdev::readBytes(_i2cDev.c_str(), _device_address, length, data);
}

bool MB85RC256::writeByte(uint16_t register_address, uint8_t* data)
{
    return writeBytes(register_address, 1, data);
}

bool MB85RC256::writeBytes(uint16_t register_address, uint8_t length, uint8_t* data)
{
    uint8_t register_address_low = register_address & 0x00FF;
    uint8_t register_address_high = (register_address >> 8) & 0x007F;    // address is 15-bits wide

    uint8_t data_buff[length + 1];
    memcpy(data_buff+1, data, (length * sizeof(*data)));
    data_buff[0] = register_address_low;

    return I2Cdev::writeBytes(_i2cDev.c_str(), _device_address, register_address_high, length+1, data_buff);
}
