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

#ifndef MB85RC04_HPP
#define MB85RC04_HPP

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include "I2Cdev.h"

class MB85RC04
{
  std::string i2cDev;
  uint8_t device_address;

public:
  MB85RC04(const char *i2cDev, uint8_t address = 0b1010000);

  bool testConnection();

  int8_t readByte(uint16_t register_address, uint8_t* data);
  int8_t readBytes(uint16_t register_address, uint8_t length, uint8_t* data);
  bool writeByte(uint16_t register_address, uint8_t* data);
  bool writeBytes(uint16_t register_address, uint8_t length, uint8_t* data);
};

#endif // MB85RC04_HPP
