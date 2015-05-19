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

#include "RCin.h"


namespace PPMparser
{
    bool parseFrame(uint16_t* in_data, uint16_t* out_data, uint8_t number_of_channels)
    {
        for (size_t i=0; i<number_of_channels; i++) {
            out_data[i] = in_data[i];
        }

        return true;
    }
}


namespace SBUSparser
{
    bool parseFrame(uint16_t* in_data, uint16_t* out_data)
    {
        // Check start-byte
        if(in_data[0] != 0xF0) {
            return false;
        }
        // Note: I found the end-byte may differ (not sure what data it may contain), so
        //       we're not checking it conforms to anything.

        // Let's shift all this data into place.
        // Proportional channels:
        out_data[0]  = ( in_data[1]        | (in_data[2]  << 8)) & 0x07FF;
        out_data[1]  = ((in_data[2]  >> 3) | (in_data[3]  << 5)) & 0x07FF;
        out_data[2]  = ((in_data[3]  >> 6) | (in_data[4]  << 2)  | (in_data[5]  << 10)) & 0x07FF;
        out_data[3]  = ((in_data[5]  >> 1) | (in_data[6]  << 7)) & 0x07FF;
        out_data[4]  = ((in_data[6]  >> 4) | (in_data[7]  << 4)) & 0x07FF;
        out_data[5]  = ((in_data[7]  >> 7) | (in_data[8]  << 1)  | (in_data[9]  <<  9)) & 0x07FF;
        out_data[6]  = ((in_data[9]  >> 2) | (in_data[10] << 6)) & 0x07FF;
        out_data[7]  = ((in_data[10] >> 5) | (in_data[11] << 3)) & 0x07FF;
        out_data[8]  = ( in_data[12]       | (in_data[13] << 8)) & 0x07FF;
        out_data[9]  = ((in_data[13] >> 3) | (in_data[14] << 5)) & 0x07FF;
        out_data[10] = ((in_data[14] >> 6) | (in_data[15] << 2)  | (in_data[16] << 10)) & 0x07FF;
        out_data[11] = ((in_data[16] >> 1) | (in_data[17] << 7)) & 0x07FF;
        out_data[12] = ((in_data[17] >> 4) | (in_data[18] << 4)) & 0x07FF;
        out_data[13] = ((in_data[18] >> 7) | (in_data[19] << 1)  | (in_data[20] <<  9)) & 0x07FF;
        out_data[14] = ((in_data[20] >> 2) | (in_data[21] << 6)) & 0x07FF;
        out_data[15] = ((in_data[21] >> 5) | (in_data[22] << 3)) & 0x07FF;

        // Digital channels:
        out_data[16] = (in_data[23] & 0x01) ? 1 : 0;
        out_data[17] = (in_data[23] & 0x02) ? 1 : 0;

        // Battery F/S and frame-drop flags:
        out_data[18] = (in_data[23] & 0x04) ? 1 : 0;    // frame drop
        out_data[19] = (in_data[23] & 0x08) ? 1 : 0;    // battery f/s

        return true;
    }
}


PPMscanner::PPMscanner()
{
    _previousTick = 0;
    reset();
}

PPMscanner::~PPMscanner()
{

}

void PPMscanner::reset()
{
    if (channels_in_frame == 0) {
        channels_in_frame = MAX_CHAN;
    }
    else {
        channels_in_frame = std::min(MAX_CHAN, channels_in_frame);
    }
    std::fill_n(_buffer, OUT_BUFF_SIZE, 0); // reset output buffer to 0
    _chan_num = 0;
    state = Sync;
}

int8_t PPMscanner::update(int level, uint32_t tick)
{
    if(level < 0) {
        // some kind of error occurred
        reset();
    }

    uint32_t deltaTime = tick - _previousTick;
    _previousTick = tick;

    switch (state) {
        case Sync:
            if(level == 1 && deltaTime >= SYNC_PERIOD) {   // expect start of new frame
                state = Frame;
            }
            else {
                reset();
            }
            break;

        case Frame:
            if (level == 1) {
                if (deltaTime < MIN_PULSE_WIDTH || deltaTime > MAX_PULSE_WIDTH) {
                    // lost sync
                    reset();
                }
                else {
                    _buffer[_chan_num] = (uint16_t)deltaTime;
                    _chan_num++;
                    if (_chan_num == channels_in_frame) {
                        state = Done;
                    }
                }
            }
            break;

        case Done:
            break;

        case Error:
        default:
            reset();
            break;
    }

    return state;
}

void PPMscanner::parseFrame(uint16_t* out_data)
{
    if (!state == Done) {
        // Shouldn't be here
        return;
    }

    PPMparser::parseFrame(_buffer, out_data, channels_in_frame);
}

void PPMscanner::getRawFrame(uint16_t* out_data)
{
    if (state != Done) {
        // Shouldn't be here
        return;
    }

    memcpy(out_data, _buffer, (channels_in_frame * sizeof(*_buffer)));
}


SBUSscanner::SBUSscanner()
{
    _previousTick = 0;
    reset();
}

SBUSscanner::~SBUSscanner()
{

}

void SBUSscanner::reset()
{
    std::fill_n(_frame, FRAME_LENGTH, 0);   // reset framebuffer to 0
    std::fill_n(_buffer, OUT_BUFF_SIZE, 0); // reset output buffer to 0
    _byte_num = 0;
    _bit_num = 0;
    state = Sync;
}

int8_t SBUSscanner::update(int level, uint32_t tick)
{
    if(level < 0) {
        // some kind of error occurred
        reset();
    }

    uint32_t deltaTime = tick - _previousTick;
    _previousTick = tick;
    uint16_t bits;

    switch (state) {
        case Sync:
            if(level == 1 && deltaTime >= SYNC_PERIOD) {   // expect start of new frame
                state = Frame;
            }
            else {
                reset();
            }
            break;

        case Frame:
            bits = (deltaTime + PULSE_SYNC_GRACE_PERIOD) / 10;

            if(bits > (BYTE_LENGTH - _bit_num)) {   // lost sync
                reset();
            }

            if(level == 0) {
                for(size_t bitnum = _bit_num; bitnum < _bit_num + bits; bitnum++) {
                    _frame[_byte_num] |= (1U << bitnum);
                }
            }

            _bit_num += bits;

            if(_bit_num == BYTE_LENGTH) {     // assume we have received a full byte
                _byte_num++;
                _bit_num = 0;
            }

            if(_byte_num == FRAME_LENGTH - 1 && _bit_num >= BYTE_LENGTH - 3) {
                // Assume we have received a full frame here. We take the remaining parity- and two stop-bits for granted, because
                // we will not receive a notification again until the next frame starts.

                for(size_t i=0; i<FRAME_LENGTH; i++) {
                    // check start-bit
                    if(!(_frame[i] & 1)) {
                        reset();
                    }
                    // check stop-bits
                    if(_frame[i] & 0x0C00) {
                        reset();
                    }

                    _buffer[i] = ((_frame[i] >> 1) & 0x00FF);

                    // check parity-bit (even)
                    bool parity = ParityTable256[_buffer[i]];
                    if((bool)(_frame[i] & 0x0200) == parity) {
                        reset();
                    }
                }

                state = Done;
            }
            break;

        case Done:
            break;

        case Error:
        default:
            reset();
            break;
    }

    return state;
}

void SBUSscanner::parseFrame(uint16_t* out_data)
{
    if (state != Done) {
        // Shouldn't be here
        return;
    }

    SBUSparser::parseFrame(_buffer, out_data);
}

void SBUSscanner::getRawFrame(uint16_t* out_data)
{
    if (state != Done) {
        // Shouldn't be here
        return;
    }

    memcpy(out_data, _buffer, (FRAME_LENGTH * sizeof(*_buffer)));
}


RCin::RCin(int scanner_type, bool raw_output, uint8_t gpio_in, uint8_t samplerate)
{
    switch (scanner_type) {
        case SCANNER_TYPE_PPM:
            _scanner = new PPMscanner();
            break;

        case SCANNER_TYPE_SBUS:
            _scanner = new SBUSscanner();
            break;

        default:
            //Error
        break;
    }

    _raw_data_output = raw_output;
    _input_gpio = gpio_in;
    _sample_rate = samplerate;

    is_initialized = false;
    _cb = NULL;
    pthread_mutex_init(&outdata_lock, NULL);
}

RCin::~RCin()
{
    gpioTerminate();
    delete _scanner;
    pthread_mutex_destroy(&outdata_lock);
}

bool RCin::initialize()
{
    if (!is_initialized && gpioCfgClock(_sample_rate, PI_DEFAULT_CLK_PERIPHERAL, 0) == 0) {
        if (gpioInitialise() >= 0) {
            if (gpioSetMode(_input_gpio, PI_INPUT) == 0) {
                if (gpioSetPullUpDown(_input_gpio, PI_PUD_DOWN) == 0) {
                    _scanner->reset();
                    is_initialized = true;
                }
            }
        }
    }

    return is_initialized;
}

extern "C" void RCin::feed(int gpio, int level, uint32_t tick, void* self)
{
    /*
        We need this static(!) wrapper function to be able to use _feed as a callback
        from the C-library, as C doesn't support calling C++ member functions directly.
    */
    RCin* s = static_cast<RCin*>(self);
    s->_feed(level, tick);
}

void RCin::_feed(int level, uint32_t tick)
{
    int8_t scanner_state = _scanner->update(level, tick);
    if (scanner_state == BaseScanner::Done) {
        pthread_mutex_lock(&outdata_lock);
            // clear the buffer first
            memset(_outdata_buff, 0, (OUTDATA_BUFFER_SIZE * sizeof(*_outdata_buff)));
            if (_raw_data_output) {
                _scanner->getRawFrame(_outdata_buff);
            }
            else {
                _scanner->parseFrame(_outdata_buff);
            }

            // Call callback if set
            if (_cb) {
                _cb(_outdata_buff);
            }
        pthread_mutex_unlock(&outdata_lock);

        _scanner->reset();
    }
}

bool RCin::enable(uint8_t number_of_channels)
{
    _scanner->channels_in_frame = number_of_channels;
    _scanner->reset();
    return !gpioSetAlertFuncEx(_input_gpio, feed, this);
}

bool RCin::disable()
{
    return !gpioSetAlertFunc(_input_gpio, NULL);
}

void RCin::registerCallback(cb_func callback)
{
    _cb = callback;
}

void RCin::readChannels(uint16_t* channel_data)
{
    pthread_mutex_lock(&outdata_lock);
        memcpy(channel_data, _outdata_buff, (OUTDATA_BUFFER_SIZE * sizeof(*_outdata_buff)));
    pthread_mutex_unlock(&outdata_lock);
}
