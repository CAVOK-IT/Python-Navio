/* ======================================================================================================== */
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

#ifndef RCIN_H_
#define RCIN_H_

#include <algorithm>
#include <functional>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <pthread.h>
#include <cstdlib>

#include "pigpio.h"


#define SCANNER_TYPE_PPM    1
#define SCANNER_TYPE_SBUS   2

// see http://graphics.stanford.edu/~seander/bithacks.html#ParityLookupTable
static const bool ParityTable256[256] =
{
#define P2(n) n, n^1, n^1, n
#define P4(n) P2(n), P2(n^1), P2(n^1), P2(n)
#define P6(n) P4(n), P4(n^1), P4(n^1), P4(n)
    P6(0), P6(1), P6(1), P6(0)
};

typedef void (*cb_func)(uint16_t*);

class BaseScanner
{
  public:
    virtual ~BaseScanner() {}

    virtual void reset() = 0;
    virtual int8_t update(int, uint32_t) = 0;
    virtual void parseFrame(uint16_t*) = 0;
    virtual void getRawFrame(uint16_t*) = 0;

    static const uint8_t    OUT_BUFF_SIZE   =  25;
    uint8_t channels_in_frame;
    enum State {
        Sync,
        Frame,
        Done,
        Error
    };

  protected:
    uint16_t _buffer[OUT_BUFF_SIZE];
};

class PPMscanner : public BaseScanner
{
  public:
    PPMscanner();
    ~PPMscanner();

    void reset();
    int8_t update(int level, uint32_t tick);
    void parseFrame(uint16_t* out_data);
    void getRawFrame(uint16_t* out_data);

    static const uint8_t    MAX_CHAN          =    8;
    State state;

  protected:
    static const uint16_t   SYNC_PERIOD       = 2700;
    static const uint16_t   MIN_PULSE_WIDTH   = 1000;
    static const uint16_t   MAX_PULSE_WIDTH   = 2000;

    uint32_t _previousTick;
    uint8_t _chan_num;              // channel number in PPM frame
};

namespace PPMparser
{
    bool parseFrame(uint16_t* in_data, uint16_t* out_data, uint8_t number_of_channels);
}

class SBUSscanner : public BaseScanner
{
  public:
    SBUSscanner();
    ~SBUSscanner();

    void reset();
    int8_t update(int level, uint32_t tick);
    void parseFrame(uint16_t* out_data);
    void getRawFrame(uint16_t* out_data);

    State state;

  protected:
    static const uint16_t   SYNC_PERIOD              = 3300;
    static const uint8_t    PULSE_SYNC_GRACE_PERIOD  =    1;
    static const uint8_t    FRAME_LENGTH             =   25;
    static const uint8_t    BYTE_LENGTH              =   12;

    uint32_t _previousTick;
    uint16_t _frame[FRAME_LENGTH]; // raw bytes, including start-, parity- and stop-bits
    uint8_t _byte_num;             // byte number in SBUS frame
    uint8_t _bit_num;              // bit in SBUS byte
};


namespace SBUSparser
{
    bool parseFrame(uint16_t* in_data, uint16_t* out_data);
}


class RCin
{
  public:
    RCin(int scanner_type = SCANNER_TYPE_PPM, bool output_raw_data = false, uint8_t gpio_in = 4, uint8_t samplerate = 5);
    ~RCin();

    bool initialize();
    static void feed(int gpio, int level, uint32_t tick, void* self);
    void _feed(int level, uint32_t tick);
    bool enable(uint8_t number_of_channels = 0);
    bool disable();
    void registerCallback(cb_func callback);
    void readChannels(uint16_t* channel_data);

    static const uint8_t OUTDATA_BUFFER_SIZE = 24;
    bool is_initialized;

  protected:
    BaseScanner* _scanner;
    cb_func _cb;
    pthread_mutex_t outdata_lock;
    uint8_t _input_gpio;
    uint8_t _sample_rate;
    bool _raw_data_output;
    uint16_t _outdata_buff[OUTDATA_BUFFER_SIZE];
};

#endif //RCIN_H_
