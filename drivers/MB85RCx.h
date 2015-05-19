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

#ifndef MB85RCx_H_
#define MB85RCx_H_

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <cstring>
#include "I2Cdev.h"


class MB85RCx
{
  public:
    virtual ~MB85RCx() {}

    virtual bool testConnection() = 0;
    virtual int8_t readByte(uint16_t, uint8_t*) = 0;
    virtual int8_t readBytes(uint16_t, uint8_t, uint8_t*) = 0;
    virtual bool writeByte(uint16_t, uint8_t*) = 0;
    virtual bool writeBytes(uint16_t, uint8_t, uint8_t*) = 0;

    virtual const uint16_t getSize() { return 0; }

  protected:
    std::string _i2cDev;
    uint8_t _device_address;
};

class MB85RC04 : public MB85RCx
{
  public:
    MB85RC04(const char *i2cDev, uint8_t address = 0b1010000);
    ~MB85RC04();

    bool testConnection();
    int8_t readByte(uint16_t register_address, uint8_t* data);
    int8_t readBytes(uint16_t register_address, uint8_t length, uint8_t* data);
    bool writeByte(uint16_t register_address, uint8_t* data);
    bool writeBytes(uint16_t register_address, uint8_t length, uint8_t* data);

    virtual const uint16_t getSize() { return 512; }
};

class MB85RC256 : public MB85RCx
{
  public:
    MB85RC256(const char *i2cDev, uint8_t address = 0b1010000);
    ~MB85RC256();

    bool testConnection();
    int8_t readByte(uint16_t register_address, uint8_t* data);
    int8_t readBytes(uint16_t register_address, uint8_t length, uint8_t* data);
    bool writeByte(uint16_t register_address, uint8_t* data);
    bool writeBytes(uint16_t register_address, uint8_t length, uint8_t* data);

    virtual const uint16_t getSize() { return 32768; }
};

#endif //MB85RCx_H_
