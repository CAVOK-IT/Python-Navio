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

#ifndef NAVIO_H_
#define NAVIO_H_

#ifndef PY_SSIZE_T_CLEAN
#define PY_SSIZE_T_CLEAN
#endif

#include <Python.h>
#include <structmember.h>
#include <cstdio>
#include <stdint.h>
#include <string>
#include <map>
#include <cmath>
#include <stdexcept>

#include "drivers/Ublox.h"          /* GPS  */
#include "drivers/Ublox_messages.h"
#include "drivers/MPU9250.h"        /* IMU  */
#include "drivers/MS5611.h"         /* baro */
#include "drivers/PCA9685.h"        /* PWM  */
#include "drivers/ADS1115.h"        /* ADC  */
#include "drivers/MB85RCx.h"        /* FRAM */
#include "drivers/RCin.h"           /* PPM input */


#define RPI_MODEL_A                   10
#define RPI_MODEL_A_PLUS              11
#define RPI_MODEL_B                   20
#define RPI_MODEL_B_PLUS              21
#define RPI_MODEL_B_2                 22    // default
#define BANANA_PI                     30

#define NAVIO                         10
#define NAVIO_RAW                     11
#define NAVIO_PLUS                    20    // default

#define BIT_COMPONENTS_GPS             1
#define BIT_COMPONENTS_IMU             2
#define BIT_COMPONENTS_BARO            4
#define BIT_COMPONENTS_PWM             8
#define BIT_COMPONENTS_ADC            16
#define BIT_COMPONENTS_FRAM           32
#define BIT_COMPONENTS_PPM            64
#define BIT_COMPONENTS_ALL           127    // default

#define RC_MODE_PPM                    1    // default
#define RC_MODE_PPM_RAW                2
#define RC_MODE_SBUS                   3

#define UBX_MSG_RECV_BUFF_LENGTH    2048

#define IMU_MODE_6DOF                  1
#define IMU_MODE_9DOF                  2    // default

#define ADC_GAIN_0                  0x00
#define ADC_GAIN_1                  0x01
#define ADC_GAIN_2                  0x02    // default
#define ADC_GAIN_4                  0x03
#define ADC_GAIN_8                  0x04
#define ADC_GAIN_16                 0x05
#define ADC_MUX_AIN0_AIN1           0x00    // default
#define ADC_MUX_AIN0_AIN3           0x01
#define ADC_MUX_AIN1_AIN3           0x02
#define ADC_MUX_AIN2_AIN3           0x03
#define ADC_MUX_AIN0_GND            0x04
#define ADC_MUX_AIN1_GND            0x05
#define ADC_MUX_AIN2_GND            0x06
#define ADC_MUX_AIN3_GND            0x07
#define ADC_MODE_CONT               0x00
#define ADC_MODE_SINGLE             0x01    // default
#define ADC_RATE_8                  0x00
#define ADC_RATE_16                 0x01
#define ADC_RATE_32                 0x02
#define ADC_RATE_64                 0x03
#define ADC_RATE_128                0x04    // default
#define ADC_RATE_250                0x05
#define ADC_RATE_475                0x06
#define ADC_RATE_860                0x07


const std::map<uint8_t, uint16_t>
SPS_ADS1115 = {
    {ADC_RATE_8,     8},
    {ADC_RATE_16,   16},
    {ADC_RATE_32,   32},
    {ADC_RATE_64,   64},
    {ADC_RATE_128, 128},
    {ADC_RATE_250, 250},
    {ADC_RATE_475, 475},
    {ADC_RATE_860, 860},
};

// Python struct format definitions for Ublox messages. (https://docs.python.org/3.2/library/struct.html#format-characters)
// Need to keep this in sync with structures defined in 'drivers/Ublox_messages.h'
const std::map<std::string, std::string[2]>
PYSTRUCTFMT_UBX_MSG_FIXEDFIELDS = {
    {"BYTEORDER_SIZE_ALIGNMENT", {"<", ""}},                                // Byte Order: little-endian, Size: standard, Alignment: none
    {"HEADER", {"BBBBH", "Sync1, Sync2, clsID, msgID, payloadLength"}},
    {"FOOTER", {"BB", "CK_A, CK_B"}},
};

const std::map<uint16_t, std::string[2]>
PYSTRUCTFMT_UBX_MSG = {
    {(uint16_t)ACK_ACK, {"BB", "clsID, msgID"}},                                                                            // ANSWER
    {(uint16_t)ACK_NAK, {"BB", "clsID, msgID"}},                                                                            // ANSWER

    {(uint16_t)AID_ALM, {"", ""}},                                                                                          // TODO
    {(uint16_t)AID_ALPSRV, {"", ""}},                                                                                       // TODO
    {(uint16_t)AID_ALP, {"", ""}},                                                                                          // TODO
    {(uint16_t)AID_AOP, {"", ""}},                                                                                          // TODO
    {(uint16_t)AID_DATA, {"", ""}},                                                                                         // TODO
    {(uint16_t)AID_EPH, {"", ""}},                                                                                          // TODO
    {(uint16_t)AID_HUI, {"", ""}},                                                                                          // TODO
    {(uint16_t)AID_INI, {"", ""}},                                                                                          // TODO
    {(uint16_t)AID_REQ, {"", ""}},                                                                                          // TODO

    {(uint16_t)CFG_ANT, {"HH", "flags, pins"}},                                                                             // GET / SET
    {(uint16_t)CFG_CFG, {"III", "clearMask, saveMask, loadMask"}},                                                          // SET
    {(uint16_t)CFG_DAT, {"H", "datumNum"}},                                                                                 // SET
    // TODO: add support for user defined Datums
    // TODO: add support for CFG_DAT *GET*
    {(uint16_t)CFG_INF, {"", ""}},                                                                                          // TODO
    {(uint16_t)CFG_ITFM, {"II", "config, config2"}},                                                                        // SET
    {(uint16_t)CFG_MSG, {"BBB", "clsID, msgID, rate"}},                                                                     // GET / SET
    // TODO: add support for getting/setting on other IO targets ??
    {(uint16_t)CFG_NAV5, {"HBBiIbBHHHHBBIII", "mask, dynModel, fixMode, fixedAlt, fixedAltVar, minElev, drLimit, pDop, tDop, pAcc, tAcc, staticHoldThresh, dgpsTimeOut, reserved2, reserved3, reserved4"}}, // GET / SET
    {(uint16_t)CFG_NAVX5, {"HHIBBBBBBBBBBHIBBBBBBHII", "mask1, reserved0, reserved1, reserved2, minSVs, maxSVs, minCNO, reserved5, iniFix3D, reserved6, reserved7, reserved8, wknRollover, reserved9, reserved10, reserved11, usePPP, useAOP, reserved12, reserved13, aopOrbMaxErr, reserved3, reserved4"}},    // GET / SET
    {(uint16_t)CFG_NMEA, {"BBBB", "filter, version, numSV, flags"}},                                                        // GET / SET
    {(uint16_t)CFG_NVS, {"IIIB", "clearMask, saveMask, loadMask, deviceMask"}},                                             // SET
    {(uint16_t)CFG_PM2, {"BBBBIIIIHHHHIIBBHI", "version, reserved1, reserved2, reserved3, flags, updatePeriod, searchPeriod, gridOffset, onTime, minAcqTime, reserved4, reserved5, reserved6, reserved7, reserved8, reserved9, reserved10, reserved11"}},   // GET / SET
    {(uint16_t)CFG_PRT, {"BBHIIHHHH", "portID, reserved0, txReady, mode, reserved3, inProtoMask, outProtoMask, reserved4, reserved5"}}, // GET / SET **SPI only**
    // TODO: adjust poll message to specify the requested portID
    // TODO: CFG_PRT for other ports ??
    {(uint16_t)CFG_RATE, {"HHH", "measRate, navRate, timeRef"}},                                                            // GET / SET
    {(uint16_t)CFG_RINV, {"", ""}},                                                                                         // TODO
    {(uint16_t)CFG_RST, {"HBB", "navBbrMask, resetMode, reserved1"}},                                                       // SET
    {(uint16_t)CFG_RXM, {"BB", "reserved1, lpMode"}},                                                                       // GET / SET
    {(uint16_t)CFG_SBAS, {"BBBBI", "mode, usage, maxSBAS, scanmode2, scanmode1"}},                                          // SET
    {(uint16_t)CFG_TMODE2, {"BBHiiiIII", "timeMode, reserved1, flags, ecefXOrLat, ecefYOrLon, ecefZOrAlt, fixedPosAcc, svMinDur, svinAccLimit"}},   // GET / SET **only for timing receivers**
    {(uint16_t)CFG_TP, {"IIbBBBhhi", "interval, length, status, timeRef, flags, reserved1, antCableDelay, rfGroupDelay, userDelay"}},   // GET / SET
    {(uint16_t)CFG_TP5, {"BBHhhIIIIiI", "tpIdx, reserved0, reserved1, antCableDelay, rfGroupDelay, freqPeriod, freqPeriodLock, pulseLenRatio, pulseLenRatioLock, userConfigDelay, flags"}},  // GET / SET **only Timepulse 0**
    // TODO: adjust poll message to specify the Timepulse number
    {(uint16_t)CFG_USB, {"HHHHHH32s32s32s", "vendorID, productID, reserved1, reserved2, powerConsumption, flags, vendorString, productString, serialNumber"}},  // GET / SET

    {(uint16_t)INF_DEBUG, {"", ""}},                                                                                        // TODO
    {(uint16_t)INF_ERROR, {"", ""}},                                                                                        // TODO
    {(uint16_t)INF_NOTICE, {"", ""}},                                                                                       // TODO
    {(uint16_t)INF_TEST, {"", ""}},                                                                                         // TODO
    {(uint16_t)INF_WARNING, {"", ""}},                                                                                      // TODO

    {(uint16_t)MON_HW, {"IIIIHHBBBBI25BBHIII", "pinSel, pinBank, pinDir, pinVal, noisePerMS, agcCnt, aStatus, aPower, flags, reserved1, usedMask, VP_0, VP_1, VP_2, VP_3, VP_4, VP_5, VP_6, VP_7, VP_8, VP_9, VP_10, VP_11, VP_12, VP_13, VP_14, VP_15, VP_16, VP_17, VP_18, VP_19, VP_20, VP_21, VP_22, VP_23, VP_24, jamInd, reserved3, pinIrq, pullH, pullL"}},   // PERIODIC / POLLED
    {(uint16_t)MON_HW2, {"bBbBB3BI2III", "ofsI, magI, ofsQ, magQ, cfgSource, reserved0_0, reserved0_1, reserved0_2, lowLevCfg, reserved1_0, reserved1_1, postStatus, reserved2"}},      // PERIODIC / POLLED
    {(uint16_t)MON_IO, {"", ""}},                                                                                           // TODO
    {(uint16_t)MON_MSGPP, {"8H8H8H8H8H8H6I", "msg1_0, msg1_1, msg1_2, msg1_3, msg1_4, msg1_5, msg1_6, msg1_7, msg2_0, msg2_1, msg2_2, msg2_3, msg2_4, msg2_5, msg2_6, msg2_7, msg3_0, msg3_1, msg3_2, msg3_3, msg3_4, msg3_5, msg3_6, msg3_7, msg4_0, msg4_1, msg4_2, msg4_3, msg4_4, msg4_5, msg4_6, msg4_7, msg5_0, msg5_1, msg5_2, msg5_3, msg5_4, msg5_5, msg5_6, msg5_7, msg6_0, msg6_1, msg6_2, msg6_3, msg6_4, msg6_5, msg6_6, msg6_7, skipped_0, skipped_1, skipped_2, skipped_3, skipped_4, skipped_5"}},  // PERIODIC / POLLED
    {(uint16_t)MON_RXBUF, {"6H6B6B", "pending_0, pending_1, pending_2, pending_3, pending_4, pending_5, usage_0, usage_1, usage_2, usage_3, usage_4, usage_5, peakUsage_0, peakUsage_1, peakUsage_2, peakUsage_3, peakUsage_4, peakUsage_5"}},  // PERIODIC / POLLED
    {(uint16_t)MON_RXR, {"B", "flags"}},                                                                                    // GET
    {(uint16_t)MON_TXBUF, {"6H6B6BBBBB", "pending_0, pending_1, pending_2, pending_3, pending_4, pending_5, usage_0, usage_1, usage_2, usage_3, usage_4, usage_5, peakUsage_0, peakUsage_1, peakUsage_2, peakUsage_3, peakUsage_4, peakUsage_5, tUsage, tPeakusage, errors, reserved1"}},   // PERIODIC / POLLED
    {(uint16_t)MON_VER, {"", ""}},                                                                                          // TODO

    {(uint16_t)NAV_AOPSTATUS, {"IBBBBIII", "iTOW, config, status, reserved0, reserved1, avail, reserved2, reserved3"}},     // PERIODIC / POLLED
    {(uint16_t)NAV_CLOCK, {"IiiII", "iTOW, clkB, clkD, tAcc, fAcc"}},                                                       // PERIODIC / POLLED
    {(uint16_t)NAV_DGPS, {"IihhBBH", "iTOW, age, baseId, baseHealth, numCh, status, reserved1"}},                           // PERIODIC / POLLED
    {(uint16_t)NAV_DOP, {"IHHHHHHH", "iTOW, gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP"}},                                    // PERIODIC / POLLED
    {(uint16_t)NAV_POSECEF, {"IiiiI", "iTOW, ecefX, ecefY, ecefZ, pAcc"}},                                                  // PERIODIC / POLLED
    {(uint16_t)NAV_POSLLH, {"IiiiiII", "iTOW, lon, lat, height, hMSL, hAcc, vAcc"}},                                        // PERIODIC / POLLED
    {(uint16_t)NAV_SBAS, {"IBBbBB3B", "iTOW, geo, mode, sys, service, cnt, reserved0_0, reserved0_1, reserved0_2"}},        // PERIODIC / POLLED
    {(uint16_t)NAV_SOL, {"IihBBiiiIiiiIHBBI", "iTOW, fTOW, week, gpsFix, flags, ecefX, ecefY, ecefZ, pAcc, ecefVX, ecefVY, ecefVZ, sAcc, pDOP, reserved1, numSV, reserved2"}},  // PERIODIC / POLLED
    {(uint16_t)NAV_STATUS, {"IBBBBII", "iTOW, gpsFix, flags, fixStat, flags2, ttff, msss"}},                                // PERIODIC / POLLED
    {(uint16_t)NAV_SVINFO, {"IBBH", "iTOW, numCh, globalFlags, reserved2"}},                                                // PERIODIC / POLLED
    {(uint16_t)NAV_TIMEGPS, {"IihbBI", "iTOW, fTOW, week, leapS, valid, tAcc"}},                                            // PERIODIC / POLLED
    {(uint16_t)NAV_TIMEUTC, {"IIiHBBBBBB", "iTOW, tAcc, nano, year, month, day, hour, min, sec, valid"}},                   // PERIODIC / POLLED
    {(uint16_t)NAV_VELECEF, {"IiiiI", "iTOW, ecefVX, ecefVY, ecefVZ, sAcc"}},                                               // PERIODIC / POLLED
    {(uint16_t)NAV_VELNED, {"IiiiIIiII", "iTOW, velN, velE, velD, speed, gSpeed, heading, sAcc, cAcc"}},                    // PERIODIC / POLLED

    {(uint16_t)RXM_PMREQ, {"", ""}},                                                                                        // TODO
    {(uint16_t)RXM_RAW, {"ihBB", "iTOW, week, numSV, reserved1"}},                                                          // PERIODIC / POLLED **only for RAW receivers**
    {(uint16_t)RXM_SFRB, {"BB10I", "chn, svid, dwrd_0, dwrd_1, dwrd_2, dwrd_3, dwrd_4, dwrd_5, dwrd_6, dwrd_7, dwrd_8, dwrd_9"}},   // PERIODIC **only for RAW receivers**
    {(uint16_t)RXM_SVSI, {"ihBB", "iTow, week, numVis, numSV"}},                                                            // PERIODIC / POLLED

    {(uint16_t)TIM_SVIN, {"IiiiIIBBH", "dur, meanX, meanY, meanZ, meanV, obs, valid, active, reserved1"}},                  // PERIODIC / POLLED **only for Timing receivers**
    {(uint16_t)TIM_TM2, {"", ""}},                                                                                          // TODO
    {(uint16_t)TIM_TP, {"", ""}},                                                                                           // TODO
    {(uint16_t)TIM_VRFY, {"", ""}},                                                                                         // TODO
};

const std::map<uint16_t, std::string[2]>
PYSTRUCTFMT_UBX_SUBMSG = {
    {(uint16_t)NAV_SBAS, {"BBBBBBhHh", "svid, flags, udre, svSys, svService, reserved1, prc, reserved2, ic"}},
    {(uint16_t)NAV_SVINFO, {"BBBBBbhi", "chn, svid, flags, quality, cno, elev, azim, prRes"}},

    {(uint16_t)RXM_RAW, {"ddfBbbB", "cpMes, prMes, doMes, sv, mesQI, cno, lli"}},
    {(uint16_t)RXM_SVSI, {"BBhbB", "svid, svFlag, azim, elev, age"}},
};


#endif // NAVIO_H_
