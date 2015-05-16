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

#ifndef UBLOXMESSAGES_H_
#define UBLOXMESSAGES_H_


#include <stdint.h>


namespace UBX_MSG
{
    const uint8_t SYNC_BYTE_1       = 0xB5;
    const uint8_t SYNC_BYTE_2       = 0x62;

    // UBX message class ACK
    const uint8_t CLASS_ACK         = 0x05;
        const uint8_t ACK_ACK       = 0x01;
        const uint8_t ACK_NAK       = 0x00;

    // UBX message class AID
    const uint8_t CLASS_AID         = 0x0B;
        const uint8_t AID_ALM       = 0x30;
        const uint8_t AID_ALPSRV    = 0x32;
        const uint8_t AID_ALP       = 0x50;
        const uint8_t AID_AOP       = 0x33;
        const uint8_t AID_DATA      = 0x10;
        const uint8_t AID_EPH       = 0x31;
        const uint8_t AID_HUI       = 0x02;
        const uint8_t AID_INI       = 0x01;
        const uint8_t AID_REQ       = 0x00;

    // UBX message class CFG
    const uint8_t CLASS_CFG         = 0x06;
        const uint8_t CFG_ANT       = 0x13;
        const uint8_t CFG_CFG       = 0x09;
        const uint8_t CFG_DAT       = 0x06;
        const uint8_t CFG_EKF       = 0x12;
        const uint8_t CFG_ESFGWT    = 0x29;
        const uint8_t CFG_FXN       = 0x0E;
        const uint8_t CFG_INF       = 0x02;
        const uint8_t CFG_ITFM      = 0x39;
        const uint8_t CFG_MSG       = 0x01;
        const uint8_t CFG_NAV5      = 0x24;
        const uint8_t CFG_NAVX5     = 0x23;
        const uint8_t CFG_NMEA      = 0x17;
        const uint8_t CFG_NVS       = 0x22;
        const uint8_t CFG_PM2       = 0x3B;
        const uint8_t CFG_PM        = 0x32;
        const uint8_t CFG_PRT       = 0x00;
        const uint8_t CFG_RATE      = 0x08;
        const uint8_t CFG_RINV      = 0x34;
        const uint8_t CFG_RST       = 0x04;
        const uint8_t CFG_RXM       = 0x11;
        const uint8_t CFG_SBAS      = 0x16;
        const uint8_t CFG_TMODE2    = 0x3D;
        const uint8_t CFG_TMODE     = 0x1D;
        const uint8_t CFG_TP5       = 0x31;
        const uint8_t CFG_TP        = 0x07;
        const uint8_t CFG_USB       = 0x1B;

    // UBX message class ESF
    const uint8_t CLASS_ESF         = 0x10;
        const uint8_t ESF_MEAS      = 0x12;
        const uint8_t ESF_STATUS    = 0x10;

    // UBX message class INF
    const uint8_t CLASS_INF         = 0x04;
        const uint8_t INF_DEBUG     = 0x04;
        const uint8_t INF_ERROR     = 0x00;
        const uint8_t INF_NOTICE    = 0x02;
        const uint8_t INF_TEST      = 0x03;
        const uint8_t INF_WARNING   = 0x01;

    // UBX message class MON
    const uint8_t CLASS_MON         = 0x0A;
        const uint8_t MON_HW2       = 0x0B;
        const uint8_t MON_HW        = 0x09;
        const uint8_t MON_IO        = 0x02;
        const uint8_t MON_MSGPP     = 0x06;
        const uint8_t MON_RXBUF     = 0x07;
        const uint8_t MON_RXR       = 0x21;
        const uint8_t MON_TXBUF     = 0x08;
        const uint8_t MON_VER       = 0x04;

    // UBX message class NAV
    const uint8_t CLASS_NAV         = 0x01;
        const uint8_t NAV_AOPSTATUS = 0x60;    // AssistNow autonomous status
        const uint8_t NAV_CLOCK     = 0x22;    // Clock solution
        const uint8_t NAV_DGPS      = 0x31;    // DGPS data used for NAV
        const uint8_t NAV_DOP       = 0x04;    // Dilution Of Precision
        const uint8_t NAV_EKFSTATUS = 0x40;    // Dead-Reckoning software status *DEPRICATED*
        const uint8_t NAV_POSECEF   = 0x01;    // Position solution in ECEF
        const uint8_t NAV_POSLLH    = 0x02;    // Geodetic Position Solution
        const uint8_t NAV_SBAS      = 0x32;    // SBAS status data
        const uint8_t NAV_SOL       = 0x06;    // Navigation solution information
        const uint8_t NAV_STATUS    = 0x03;    // Receiver navigation status
        const uint8_t NAV_SVINFO    = 0x30;    // Space Vehicle information
        const uint8_t NAV_TIMEGPS   = 0x20;    // GPS time solution
        const uint8_t NAV_TIMEUTC   = 0x21;    // UTC time solution
        const uint8_t NAV_VELECEF   = 0x11;    // Velocity solution in ECEF
        const uint8_t NAV_VELNED    = 0x12;    // Velocity solution in NED

    // UBX message class RXM
    const uint8_t CLASS_RXM         = 0x02;
        const uint8_t RXM_ALM       = 0x30;
        const uint8_t RXM_EPH       = 0x31;
        const uint8_t RXM_PMREQ     = 0x41;
        const uint8_t RXM_RAW       = 0x10;
        const uint8_t RXM_SFRB      = 0x11;
        const uint8_t RXM_SVSI      = 0x20;

    // UBX message class TIM
    const uint8_t CLASS_TIM         = 0x0D;
        const uint8_t TIM_SVIN      = 0x04;
        const uint8_t TIM_TM2       = 0x03;
        const uint8_t TIM_TP        = 0x01;
        const uint8_t TIM_VRFY      = 0x06;


// Set alignment to 1, push current alignment on the stack. See also https://gcc.gnu.org/onlinedocs/gcc/Structure-Packing-Pragmas.html
// According to the u-blox manual, this shouldn't be a problem, but do it anyway to be safe...
#pragma pack(push, 1)

    // UBX protocol message structures.
    struct Header
    {
        uint8_t Sync1;          // UBX sync byte 1
        uint8_t Sync2;          // UBX sync byte 2
        uint8_t clsID;          // Message Class
        uint8_t msgID;          // Message ID
        uint16_t payloadLength; // Length of the payload
    };

    struct Footer
    {
        uint8_t CK_A;           // Checksum A
        uint8_t CK_B;           // Checksum B
    };

    struct AckAck
    {
        Header header;
        uint8_t clsID;          // Message Class this NAK is a reply to
        uint8_t msgID;          // Message ID this NAK is a reply to
        Footer footer;
    };

    struct AckNak
    {
        Header header;
        uint8_t clsID;          // Message Class this NAK is a reply to
        uint8_t msgID;          // Message ID this NAK is a reply to
        Footer footer;
    };

    struct NavAOPStatus
    {
        Header header;
        uint32_t iTOW;          // GPS Time of Week
        uint8_t config;         // AssistNow Autonomous enabled
        uint8_t status;         // AssistNow Autonomous subsystem running
        uint8_t reserved0;      // Reserved
        uint8_t reserved1;      // Reserved
        uint32_t avail;         // Data availability for GPS SVs (PRN 1-32)
        uint32_t reserved2;     // Reserved
        uint32_t reserved3;     // Reserved
        Footer footer;
    };

    struct NavClock
    {
        Header header;
        uint32_t iTOW;          // GPS Time of Week
        int32_t clkB;           // Clock bias in ns
        int32_t clkD;           // Clock drift in ns/s
        uint32_t tAcc;          // Time accuracy estimate
        uint32_t fAcc;          // Frequency accuracy estimate
        Footer footer;
    };

    const uint8_t MAX_NAVDGPS_R = 255;   //?? TODO: check max
    struct NavDGPS_R
    {
        uint8_t svid;           // Satellite ID
        uint8_t flags;          // Bitmask / channel number
        uint16_t ageC;          // Age of latest correction data
        float prc;              // Pseudo Range correction
        float prrc;             // Pseudo Range Rate correction
    };
    struct NavDGPS
    {
        Header header;
        uint32_t iTOW;          // GPS Time of Week
        int32_t age;            // Age of newest correction data
        int16_t baseId;         // DGPS base station ID
        int16_t baseHealth;     // DGPS base station health status
        uint8_t numCh;          // Number of channels for which correction data is following
        uint8_t status;         // DGPS correction type status
        uint16_t reserved1;     // Reserved
        NavDGPS_R rept[MAX_NAVDGPS_R];
        Footer footer;
    };

    struct NavDOP
    {
        Header header;
        uint32_t iTOW;          // GPS millisecond Time of Week
        uint16_t gDOP;          // Geometric DOP
        uint16_t pDOP;          // Position DOP
        uint16_t tDOP;          // Time DOP
        uint16_t vDOP;          // Vertical DOP
        uint16_t hDOP;          // Horizontal DOP
        uint16_t nDOP;          // Northing DOP
        uint16_t eDOP;          // Easting DOP
        Footer footer;
    };

    struct NavPosECEF
    {
        Header header;
        uint32_t iTOW;          // GPS millisecond Time of Week
        int32_t ecefX;          // ECEF X coordinate
        int32_t ecefY;          // ECEF Y coordinate
        int32_t ecefZ;          // ECEF Z coordinate
        uint32_t pAcc;          // Position accuracy estimate
        Footer footer;
    };

    struct NavPosLLH
    {
        Header header;
        uint32_t iTOW;          // GPS millisecond Time of Week
        int32_t lon;            // Longitude
        int32_t lat;            // Latitude
        int32_t height;         // Height above ellipsoid
        int32_t hMSL;           // Height above MSL
        uint32_t hAcc;          // Horizontal accuracy estimate
        uint32_t vAcc;          // Vertical accuracy estimate
        Footer footer;
    };

    const uint8_t MAX_NAV_SBAS_R = 255; //?? TODO: check max
    struct NavSBAS_R
    {
        uint8_t svid;           // SV ID
        uint8_t flags;          // Flags for this SV
        uint8_t udre;           // Monitoring status
        uint8_t svSys;          // SBAS system (see 'sys')
        uint8_t svService;      // Services available (see 'service')
        uint8_t reserved1;      // Reserved
        int16_t prc;            // Pseudo Range correction
        uint16_t reserved2;     // Reserved
        int16_t ic;             // Ionosphere correction
    };
    struct NavSBAS
    {
        Header header;
        uint32_t iTOW;          // GPS Time of Week
        uint8_t geo;            // PRN Number of the GEO where correction and integrity data is used from
        uint8_t mode;           // SBAS mode ()
        int8_t sys;             // SBAS system (WAAS/EGNOS/etc.)
        uint8_t service;        // SBAS services available [BITFIELD]
        uint8_t cnt;            // Number of SV data following
        uint8_t reserved0[3];   // Reserved
        NavSBAS_R rept[MAX_NAV_SBAS_R];
        Footer footer;
    };
    const uint8_t NAV_SBAS_BITFIELD_SERVICE_RANGING         = 0b00000001;
    const uint8_t NAV_SBAS_BITFIELD_SERVICE_CORRECTIONS     = 0b00000010;
    const uint8_t NAV_SBAS_BITFIELD_SERVICE_INTEGRITY       = 0b00000100;
    const uint8_t NAV_SBAS_BITFIELD_SERVICE_TESTMODE        = 0b00001000;

    struct NavSOL
    {
        uint32_t iTOW;          // GPS Time of Week
        int32_t fTOW;           // Fractional ns remainder of rounded iTOW ms
        int16_t week;           // GPS week (GPS time)
        uint8_t gpsFix;         // GPS fix type
        uint8_t flags;          // Fix status flags [BITFIELD]
        int32_t ecefX;          // ECEF X coordinate
        int32_t ecefY;          // ECEF Y coordinate
        int32_t ecefZ;          // ECEF Z coordinate
        uint32_t pAcc;          // 3D position accuracy estimate
        int32_t ecefVX;         // ECEF X velocity
        int32_t ecefVY;         // ECEF Y velocity
        int32_t ecefVZ;         // ECEF Z velocity
        uint32_t sAcc;          // Velocity accuracy estimate
        uint16_t pDOP;          // Position DOP
        uint8_t reserved1;      // Reserved
        uint8_t numSV;          // Number of SVs used in nav solution
        uint32_t reserved2;     // Reserved
    };
    const uint8_t NAV_SOL_BITFIELD_FLAGS_GPSFIXOK           = 0b00000001;
    const uint8_t NAV_SOL_BITFIELD_FLAGS_DIFFSOLN           = 0b00000010;
    const uint8_t NAV_SOL_BITFIELD_FLAGS_WKNSET             = 0b00000100;
    const uint8_t NAV_SOL_BITFIELD_FLAGS_TOWSET             = 0b00001000;

    struct NavStatus
    {
        Header header;
        uint32_t iTOW;          // GPS millisecond Time of Week
        uint8_t gpsFix;         // GPSfix type
        uint8_t flags;          // Navigation status flags [BITFIELD]
        uint8_t fixStat;        // Fix status information [BITFIELD]
        uint8_t flags2;         // Further information about navigation output [BITFIELD]
        uint32_t ttff;          // Time to First Fix (ms time tag)
        uint32_t msss;          // Milliseconds since startup/reset
        Footer footer;
    };
    const uint8_t NAV_STATUS_BITFIELD_FLAGS_GPSFIXOK        = 0b00000001;
    const uint8_t NAV_STATUS_BITFIELD_FLAGS_DIFFSOLN        = 0b00000010;
    const uint8_t NAV_STATUS_BITFIELD_FLAGS_WKNSET          = 0b00000100;
    const uint8_t NAV_STATUS_BITFIELD_FLAGS_TOWSET          = 0b00001000;
    const uint8_t NAV_STATUS_BITFIELD_FIXSTAT_DGPSISTAT     = 0b00000001;
    const uint8_t NAV_STATUS_BITFIELD_FIXSTAT_MAPMATCHING   = 0b11000000;
    const uint8_t NAV_STATUS_BITFIELD_FLAGS2_PSMSTATE       = 0b00000011;

    const uint8_t MAX_NAV_SVINFO_R = 255; //?? TODO: check max
    struct NavSVInfo_R
    {
        uint8_t chn;            // Channel number
        uint8_t svid;           // SV ID
        uint8_t flags;          // SV information [BITFIELD]
        uint8_t quality;        // Signal quality information [BITFIELD]
        uint8_t cno;            // Carrier to Noise Ratio (signal strength)
        int8_t elev;            // SV elevation
        int16_t azim;           // SV azimuth
        int32_t prRes;          // Pseudo Range Residual
    };
    struct NavSVInfo
    {
        Header header;
        uint32_t iTOW;          // GPS Time of Week
        uint8_t numCh;          // Number of channels
        uint8_t globalFlags;    // Chip hardware info [BITFIELD]
        uint16_t reserved2;     // Reserved
        NavSVInfo_R rept[MAX_NAV_SVINFO_R];
        Footer footer;
    };
    const uint8_t NAV_SVINFO_BITFIELD_GLOBALFLAGS_CHIPGEN   = 0b00000111;
    const uint8_t NAV_SVINFO_BITFIELD_FLAGS_SVUSED          = 0b00000001;
    const uint8_t NAV_SVINFO_BITFIELD_FLAGS_DIFFCORR        = 0b00000010;
    const uint8_t NAV_SVINFO_BITFIELD_FLAGS_ORBITAVAIL      = 0b00000100;
    const uint8_t NAV_SVINFO_BITFIELD_FLAGS_ORBITEPH        = 0b00001000;
    const uint8_t NAV_SVINFO_BITFIELD_FLAGS_UNHEALTHY       = 0b00010000;
    const uint8_t NAV_SVINFO_BITFIELD_FLAGS_ORBITALM        = 0b00100000;
    const uint8_t NAV_SVINFO_BITFIELD_FLAGS_ORBITAOP        = 0b01000000;
    const uint8_t NAV_SVINFO_BITFIELD_FLAGS_SMOOTHED        = 0b10000000;
    const uint8_t NAV_SVINFO_BITFIELD_QUALITY_QUALITYIND    = 0b00001111;

    struct NavTimeGPS
    {
        uint32_t iTOW;          // GPS Time of Week
        int32_t fTOW;           // Fractional ns remainder of rounded iTOW ms
        int16_t week;           // GPS week (GPS time)
        int8_t leapS;           // Leap Seconds (GPS_UTC)
        uint8_t valid;          // Validity flags [BITFIELD]
        uint32_t tAcc;          // Time accuracy estimate
    };
    const uint8_t NAV_TIMEGPS_BITFIELD_VALID_TOW            = 0b00000001;
    const uint8_t NAV_TIMEGPS_BITFIELD_VALID_WEEK           = 0b00000010;
    const uint8_t NAV_TIMEGPS_BITFIELD_VALID_UTC            = 0b00000100;

    struct NavTimeUTC
    {
        uint32_t iTOW;          // GPS Time of Week
        uint32_t tAcc;          // Time accuracy estimate
        int32_t nano;           // ns of second
        uint16_t year;          // Year
        uint8_t month;          // Month
        uint8_t day;            // Day
        uint8_t hour;           // Hour
        uint8_t min;            // Minute
        uint8_t sec;            // Second
        uint8_t valid;          // Validity flags [BITFIELD]
    };
    const uint8_t NAV_TIMEUTC_BITFIELD_VALID_TOW            = 0b00000001;
    const uint8_t NAV_TIMEUTC_BITFIELD_VALID_WKN            = 0b00000010;
    const uint8_t NAV_TIMEUTC_BITFIELD_VALID_UTC            = 0b00000100;

    struct NavVelECEF
    {
        uint32_t iTOW;          // GPS Time of Week
        int32_t ecefVX;         // ECEF X velocity
        int32_t ecefVY;         // ECEF Y velocity
        int32_t ecefVZ;         // ECEF Z velocity
        uint32_t sAcc;          // Speed accuracy estimate
    };

    struct NavVelNED
    {
        uint32_t iTOW;          // GPS Time of Week
        int32_t velN;           // NED North velocity
        int32_t velE;           // NED East velocity
        int32_t velD;           // NED down velocity
        uint32_t speed;         // Speed (3D)
        uint32_t gSpeed;        // Ground Speed (2D)
        int32_t heading;        // Heading of motion (2D)
        uint32_t sAcc;          // Speed accuracy estimate
        uint32_t cAcc;          // Course / heading accuracy estimate
    };

// Restore original alignment from stack
#pragma pack(pop)

}

enum ubx_message_t
{
    // UBX message class ACK
    ACK_ACK                 = (UBX_MSG::CLASS_ACK << 8) + UBX_MSG::ACK_ACK,
    ACK_NAK                 = (UBX_MSG::CLASS_ACK << 8) + UBX_MSG::ACK_NAK,

    // UBX message class AID
    AID_ALM                 = (UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_ALM,
    AID_ALPSRV              = (UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_ALPSRV,
    AID_ALP                 = (UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_ALP,
    AID_AOP                 = (UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_AOP,
    AID_DATA                = (UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_DATA,
    AID_EPH                 = (UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_EPH,
    AID_HUI                 = (UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_HUI,
    AID_INI                 = (UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_INI,
    AID_REQ                 = (UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_REQ,

    // UBX message class CFG
    CFG_ANT                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_ANT,
    CFG_CFG                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_CFG,
    CFG_DAT                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_DAT,
    CFG_EKF                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_EKF,         // ADR receiver only ***DEPRICATED***
    CFG_ESFGWT              = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_ESFGWT,      // ADR receiver only
    //CFG_ESFDWT ???
    CFG_FXN                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_FXN,         // ***DEPRICATED***
    CFG_INF                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_INF,
    CFG_ITFM                = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_ITFM,
    CFG_MSG                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_MSG,
    CFG_NAV5                = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_NAV5,
    CFG_NAVX5               = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_NAVX5,
    CFG_NMEA                = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_NMEA,
    CFG_NVS                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_NVS,
    CFG_PM                  = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_PM,          // ***DEPRICATED***
    CFG_PM2                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_PM2,
    CFG_PRT                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_PRT,
    CFG_RATE                = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_RATE,
    CFG_RINV                = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_RINV,
    CFG_RST                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_RST,
    CFG_RXM                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_RXM,
    CFG_SBAS                = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_SBAS,
    CFG_TMODE               = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_TMODE,       // ***DEPRICATED***
    CFG_TMODE2              = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_TMODE2,      // Timing receiver only
    CFG_TP5                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_TP5,
    CFG_TP                  = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_TP,
    CFG_USB                 = (UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_USB,

    // UBX message class ESF
    ESF_MEAS                = (UBX_MSG::CLASS_ESF << 8) + UBX_MSG::ESF_MEAS,        // ADR receiver only
    ESF_STATUS              = (UBX_MSG::CLASS_ESF << 8) + UBX_MSG::ESF_STATUS,      // ADR receiver only

    // UBX message class INF
    INF_DEBUG               = (UBX_MSG::CLASS_INF << 8) + UBX_MSG::INF_DEBUG,
    INF_ERROR               = (UBX_MSG::CLASS_INF << 8) + UBX_MSG::INF_ERROR,
    INF_NOTICE              = (UBX_MSG::CLASS_INF << 8) + UBX_MSG::INF_NOTICE,
    INF_TEST                = (UBX_MSG::CLASS_INF << 8) + UBX_MSG::INF_TEST,
    INF_WARNING             = (UBX_MSG::CLASS_INF << 8) + UBX_MSG::INF_WARNING,

    // UBX message class MON
    MON_HW                  = (UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_HW,
    MON_HW2                 = (UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_HW2,
    MON_IO                  = (UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_IO,
    MON_MSGPP               = (UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_MSGPP,
    MON_RXBUF               = (UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_RXBUF,
    MON_RXR                 = (UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_RXR,
    MON_TXBUF               = (UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_TXBUF,
    MON_VER                 = (UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_VER,

    // UBX message class NAV
    NAV_AOPSTATUS           = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_AOPSTATUS,
    NAV_CLOCK               = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_CLOCK,
    NAV_DGPS                = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_DGPS,
    NAV_DOP                 = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_DOP,
    NAV_EKFSTATUS           = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_EKFSTATUS,   // ADR receiver only ***DEPRICATED***
    NAV_POSECEF             = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_POSECEF,
    NAV_POSLLH              = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_POSLLH,
    NAV_SBAS                = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_SBAS,
    NAV_SOL                 = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_SOL,
    NAV_STATUS              = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_STATUS,
    NAV_SVINFO              = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_SVINFO,
    NAV_TIMEGPS             = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_TIMEGPS,
    NAV_TIMEUTC             = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_TIMEUTC,
    NAV_VELECEF             = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_VELECEF,
    NAV_VELNED              = (UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_VELNED,

    // UBX message class RXM
    RXM_ALM                 = (UBX_MSG::CLASS_RXM << 8) + UBX_MSG::RXM_ALM,         // RAW-data receiver only ***DEPRICATED***
    RXM_EPH                 = (UBX_MSG::CLASS_RXM << 8) + UBX_MSG::RXM_EPH,         // RAW-data receiver only ***DEPRICATED***
    RXM_PMREQ               = (UBX_MSG::CLASS_RXM << 8) + UBX_MSG::RXM_PMREQ,
    RXM_RAW                 = (UBX_MSG::CLASS_RXM << 8) + UBX_MSG::RXM_RAW,         // RAW-data receiver only
    RXM_SFRB                = (UBX_MSG::CLASS_RXM << 8) + UBX_MSG::RXM_SFRB,        // RAW-data receiver only
    RXM_SVSI                = (UBX_MSG::CLASS_RXM << 8) + UBX_MSG::RXM_SVSI,

    // UBX message class TIM
    TIM_SVIN                = (UBX_MSG::CLASS_TIM << 8) + UBX_MSG::TIM_SVIN,        // Timing receiver only
    TIM_TM2                 = (UBX_MSG::CLASS_TIM << 8) + UBX_MSG::TIM_TM2,
    TIM_TP                  = (UBX_MSG::CLASS_TIM << 8) + UBX_MSG::TIM_TP,
    TIM_VRFY                = (UBX_MSG::CLASS_TIM << 8) + UBX_MSG::TIM_VRFY,
};

#endif  // UBLOXMESSAGES_H_
