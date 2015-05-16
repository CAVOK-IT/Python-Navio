/* ======================================================================================================== */
/* Written by Egor Fedorov (egor.fedorov@emlid.com), modified by B. Rexwinkel                               */
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

#include "Ublox.h"
#include "Ublox_messages.h"


/** Default constructor.
*/
UBXscanner::UBXscanner()
{
    reset();
}

/** Resets the scanner. Call this to be able to scan for
    new messages.
 * @return void
*/
void UBXscanner::reset()
{
    message_recv_length = 0;
    position = 0;
    state = Sync1;
}

/** Updates the message buffer with new data.
 * @param data Single byte of received data to add to the buffer.
 * @return The current state of the scanner. Returns Done when a
           full message has been received.
*/
int8_t UBXscanner::update(uint8_t data)
{
    if (state != Done)
        message_recv_buffer[position++] = data;

    switch (state) {
        case Sync1:
            if (data == 0xb5)
                state = Sync2;
            else
                reset();
            break;

        case Sync2:
            if (data == 0x62)
                state = Class;
            else
                reset();
            break;

        case Class:
            state = ID;
            break;

        case ID:
            state = Length1;
            break;

        case Length1:
            payload_length = data;
            state = Length2;
            break;

        case Length2:
            payload_length += data << 8;
            state = Payload;
            break;

        case Payload:
            if (position >= (MSG_RECV_BUFF_LENGTH - 2))   // message_buffer is about to overflow
                reset();
            else if (position == payload_length + 6)
                state = CK_A;
            break;

        case CK_A:
            state = CK_B;
            break;

        case CK_B:
            message_recv_length = position;
            state = Done;
            break;

        case Done:
        default:
            break;
    }

    return state;
}


// UBX parser
namespace UBXparser
{
    bool decodeMessage(uint8_t* message, size_t length)
    {
        bool result = false;
        ubx_message_t msg_class_id;

        if (checkMessageValid(message, length)) {
            msg_class_id = (ubx_message_t)getMessageClsID(message);

            switch (msg_class_id) {
                case ACK_ACK:
                    UBX_MSG::AckAck msg_block;
                    memcpy(&msg_block, message, length);
                    printf("Acknowledged!\n");
                    break;
                case ACK_NAK:
                    printf("Not acknowledged!\n");
                    break;
                default:
                    printf("*other message*\n");
            }
        }

        return result;
    }

    bool encodeMessage(ubx_message_t message_type, uint8_t* message_send_buff, size_t length)
    {
        message_send_buff[0] = UBX_MSG::SYNC_BYTE_1;
        message_send_buff[1] = UBX_MSG::SYNC_BYTE_2;

        return true;
    }

    uint16_t getMessageClsID(uint8_t* message)
    {
        return (message[2] << 8) | message[3];
    }

    /** Check if the message in the current buffer is valid (i.e. has
        a valid header and checksum).
     * @param[in] message Pointer to the message
     * @param[in] length Length of the message
     * @return Message is valid (true) or not (false)
     */
    bool checkMessageValid(uint8_t* message, size_t length)
    {
        bool result = false;
        uint8_t checksum[2];

        if (length >= 8) {
            if (message[0] == UBX_MSG::SYNC_BYTE_1 && message[1] == UBX_MSG::SYNC_BYTE_2) {
                calculateChecksum(message, length, checksum);
                if (message[length-2] == checksum[0] && message[length-1] == checksum[1]) {
                    result = true;
                }
            }
        }

        return result;
    }

    /** Calculate the checksum over a message.
     * @param[in] message Pointer to the message
     * @param[in] length Length of the message
     * @param[out] checksum Pointer to checksum variable (uint8_t[2])
     * @return void
     */
    void calculateChecksum(uint8_t* message, size_t length, uint8_t* checksum)
    {
        uint8_t ck_a = 0;
        uint8_t ck_b = 0;

        for (size_t i=2; i<(length-2); i++) {
            ck_a += message[i];
            ck_b += ck_a;
        }

        checksum[0] = ck_a;
        checksum[1] = ck_b;
    }
}


Ublox::Ublox(std::string spi_dev)
{
    spi_device_name = spi_dev;
    scanner = new UBXscanner();
}

Ublox::Ublox(std::string spi_dev, UBXscanner* scan)
{
    spi_device_name = spi_dev;
    scanner = scan;
}

Ublox::~Ublox()
{
    delete scanner;
}

bool Ublox::testConnection()
{
    scanner->message_recv_buffer[0] = UBX_MSG::SYNC_BYTE_1;
    scanner->message_recv_buffer[1] = UBX_MSG::SYNC_BYTE_2;
    scanner->message_recv_buffer[2] = UBX_MSG::CLASS_ACK;
    scanner->message_recv_buffer[3] = UBX_MSG::ACK_ACK;
    scanner->message_recv_buffer[4] = 0x02;
    scanner->message_recv_buffer[5] = 0x00;
    scanner->message_recv_buffer[6] = UBX_MSG::CLASS_CFG;
    scanner->message_recv_buffer[7] = UBX_MSG::CFG_RATE;
    scanner->message_recv_buffer[8] = 0;
    scanner->message_recv_buffer[9] = 0;

    UBXparser::calculateChecksum(scanner->message_recv_buffer, 10, &scanner->message_recv_buffer[8]);

    scanner->message_recv_length = 10;

    return true;
}

bool Ublox::pollMessage(ubx_message_t message_type)
{
    uint8_t poll_message[8] = { UBX_MSG::SYNC_BYTE_1, UBX_MSG::SYNC_BYTE_2, (uint8_t)(message_type >> 8), (uint8_t)(message_type & 0x00FF) };
    UBXparser::calculateChecksum(poll_message, 8, &poll_message[6]);
    uint8_t response[8];

    if (SPIdev::transfer(spi_device_name.c_str(), (unsigned char *)&poll_message, (unsigned char *)&response, 8, 200000) < 0) {
        return false;
    }

    return true;
}

int16_t Ublox::getMessage(ubx_message_t message_type, uint8_t* parse_buffer, uint16_t ms_timeout)
{
    int16_t result = 0;
    uint8_t out_data = 0, in_data = 0;
    int8_t scanner_status;
    uint8_t* scanner_buffer = scanner->message_recv_buffer;

    timeval tm_start, tm_curr;
    double starttime;
    uint16_t ms_dt = 0;

    if (pollMessage(message_type)) {
        gettimeofday(&tm_start, NULL);
        starttime = (tm_start.tv_sec + (tm_start.tv_usec / 1000000.0));

        do {
            if (SPIdev::transfer(spi_device_name.c_str(), &out_data, &in_data, 1, 200000) < 0) {
                result = -1;
                break;
            }

            scanner_status = scanner->update(in_data);

            if (scanner_status == UBXscanner::Done) {
                if (UBXparser::getMessageClsID(scanner_buffer) == message_type && UBXparser::checkMessageValid(scanner_buffer, scanner->message_recv_length)) {
                    memcpy(parse_buffer, scanner_buffer, scanner->message_recv_length);
                    result = scanner->message_recv_length;
                    break;
                }

                scanner->reset();
            }

            //usleep(200);
            gettimeofday(&tm_curr, NULL);
            ms_dt = floor(((tm_curr.tv_sec + (tm_curr.tv_usec / 1000000.0)) - starttime) * 1000.0);
        } while (ms_dt < ms_timeout);
    }

    scanner->reset();
    return result;
}

bool Ublox::sendMessage()
{

    return true;
}
