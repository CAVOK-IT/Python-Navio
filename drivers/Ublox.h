/* ======================================================================================================== */
/* Copyright (c) 2014-2015, B. Rexwinkel (CAVOK IT)                                                         */
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

#ifndef UBLOX_H_
#define UBLOX_H_

#include <cstddef>
#include <cstdio>
#include <cstring>
#include <stdint.h>
#include <cstdio>
#include <cmath>
#include <string>
#include <iostream>
#include <sys/time.h>

#include "SPIdev.h"
#include "Ublox_messages.h"


const uint16_t MSG_RECV_BUFF_LENGTH = 2048;
const uint16_t MSG_SEND_BUFF_LENGTH = 1024;

class UBXscanner
{
  public:
    UBXscanner();

    void reset();
    int8_t update(uint8_t data);

  public:
    enum State {
        Sync1,
        Sync2,
        Class,
        ID,
        Length1,
        Length2,
        Payload,
        CK_A,
        CK_B,
        Done
    };

    uint8_t message_recv_buffer[MSG_RECV_BUFF_LENGTH];   // Buffer for incoming UBX messages
    // uint8_t message_send_buffer[MSG_SEND_BUFF_LENGTH];
    size_t message_recv_length;                          // Length of the received message
    // size_t message_send_length;
    uint16_t payload_length;    // Length of current message payload

  protected:
    uint16_t position;          // Holds the current buffer offset
    State state;                // Current scanner state
};

namespace UBXparser
{
    bool encodeMessage(ubx_message_t message_type, uint8_t* message_send_buff, size_t length);
    bool decodeMessage(uint8_t* message, size_t length);
    uint16_t getMessageClsID(uint8_t* message);
    bool checkMessageValid(uint8_t* message, size_t length);
    void calculateChecksum(uint8_t* message, size_t length, uint8_t* checksum);
}

class Ublox
{
  public:
    Ublox(std::string spi_dev = "/dev/spidev0.0");
    Ublox(std::string spi_dev, UBXscanner* scan);
    ~Ublox();

    bool testConnection();
    int16_t getMessage(ubx_message_t message_type, uint8_t* parse_buffer, uint16_t ms_timeout=200);
    bool sendMessage();

  protected:
    bool pollMessage(ubx_message_t message_type);

  protected:
    std::string spi_device_name;
    UBXscanner* scanner;
    //UBXparser* parser;
};

#endif // UBLOX_H_
