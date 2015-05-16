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


#ifndef MPU9250_H_
#define MPU9250_H_

#include <cmath>
#include <cstring>
#include <cstdint>
#include <string>
#include <cstdio>
#include <sys/time.h>

#include "SPIdev.h"


class MPU9250
{
  public:
    static const uint8_t MODE_6DOF          = 0x00;
    static const uint8_t MODE_9DOF          = 0x01;

  protected:
    static const uint8_t SPI_READ_FLAG      = 0x80;
    static const uint8_t I2C_READ_FLAG      = 0x80;

    static const uint8_t AK8963_I2C_ADDR    = 0x0C;

    std::string spi_device_name;
    uint8_t imu_mode;

    bool MPU6500_calibrated;
    bool AK8963_calibrated;
    bool AK8963_measurement_enabled;

    uint8_t internal_sample_rate;

    double accel_hwu_g; //= 0.0;
    double gyro_hwu_dps; //= 0.0;
    double mag_sens_adj[3]; //= {1.0, 1.0, 1.0};

    struct settings_t {
        uint8_t smplrt_div;
        uint8_t config;
        uint8_t gyro_config;
        uint8_t accel_config;
        uint8_t accel_config2;
        uint8_t fifo_en;
        uint8_t i2c_mst_ctrl;
        uint8_t i2c_mst_delay_ctrl;
        uint8_t user_ctrl;
        uint8_t pwr_mgmt_1;
        uint8_t pwr_mgmt_2;
        uint8_t i2c_slv4_ctrl;
    } settings;

  public:
    MPU9250(uint8_t mode=MODE_9DOF, std::string spi_dev="/dev/spidev0.1");
    ~MPU9250();

    void initialize();
    bool testConnection();
    uint8_t selfTest();
    void calibrate();
    void getMotion6(float* accel, float* gyro);
    void getMotion9(float* accel, float* gyro, float* mag);
    uint16_t getSampleRate();
    uint16_t setSampleRate(uint16_t rate);
    uint16_t getDLPF();
    uint16_t setDLPF(uint16_t dlpf);
    uint16_t getAccelDLPF();
    uint16_t setAccelDLPF(uint16_t accel_dlpf);
    uint16_t getGyroFSR();
    uint16_t setGyroFSR(uint16_t gyro_fsr);
    uint16_t getAccelFSR();
    uint16_t setAccelFSR(uint16_t accel_fsr);

    // TODO: remove these
    uint16_t AK8963_getSampleRate();
    uint16_t AK8963_setSampleRate(uint16_t rate);

  protected:
    bool readByte(uint8_t reg_address, uint8_t* rx_data);
    bool readBytes(uint8_t reg_address, size_t length, uint8_t* rx_data);
    bool writeByte(uint8_t reg_address, uint8_t tx_data);
    bool readI2CByte(uint8_t dev_address, uint8_t reg_address, uint8_t* rx_data);
    bool readI2CBytes(uint8_t dev_address, uint8_t reg_address, uint8_t length, uint8_t* rx_data);
    bool writeI2CByte(uint8_t dev_address, uint8_t reg_address, uint8_t data);

    void initI2Cmaster();
    void readConfig();
    void writeConfig();
    void getBiases(double* gyro, double* accel, size_t nr_of_samples);

    void AK8963_initialize();
    bool AK8963_testConnection();
    uint8_t AK8963_selfTest();
    void AK8963_reset();        // remove?
    bool AK8963_setMode(uint8_t mode);
    void AK8963_enableMeasurement();
    void AK8963_disableMeasurement();
    // uint16_t AK8963_getSampleRate();
    // uint16_t AK8963_setSampleRate(uint16_t rate);
    bool AK8963_read(double* mag_data);
    void AK8963_calibrate();
};


namespace MPU9250_REG_ADDR
{
    // Read / write
    const uint8_t SELF_TEST_X_GYRO      = 0x00;
    const uint8_t SELF_TEST_Y_GYRO      = 0x01;
    const uint8_t SELF_TEST_Z_GYRO      = 0x02;
    const uint8_t SELF_TEST_X_ACCEL     = 0x0D;
    const uint8_t SELF_TEST_Y_ACCEL     = 0x0E;
    const uint8_t SELF_TEST_Z_ACCEL     = 0x0F;
    const uint8_t XG_OFFSET_H           = 0x13;
    const uint8_t XG_OFFSET_L           = 0x14;
    const uint8_t YG_OFFSET_H           = 0x15;
    const uint8_t YG_OFFSET_L           = 0x16;
    const uint8_t ZG_OFFSET_H           = 0x17;
    const uint8_t ZG_OFFSET_L           = 0x18;
    const uint8_t SMPLRT_DIV            = 0x19;
    const uint8_t CONFIG                = 0x1A;
    const uint8_t GYRO_CONFIG           = 0x1B;
    const uint8_t ACCEL_CONFIG          = 0x1C;
    const uint8_t ACCEL_CONFIG2         = 0x1D;
    const uint8_t LP_ACCEL_ODR          = 0x1E;
    const uint8_t WOM_THR               = 0x1F;
    const uint8_t FIFO_EN               = 0x23;
    const uint8_t I2C_MST_CTRL          = 0x24;
    const uint8_t I2C_SLV0_ADDR         = 0x25;
    const uint8_t I2C_SLV0_REG          = 0x26;
    const uint8_t I2C_SLV0_CTRL         = 0x27;
    const uint8_t I2C_SLV1_ADDR         = 0x28;
    const uint8_t I2C_SLV1_REG          = 0x29;
    const uint8_t I2C_SLV1_CTRL         = 0x2A;
    const uint8_t I2C_SLV2_ADDR         = 0x2B;
    const uint8_t I2C_SLV2_REG          = 0x2C;
    const uint8_t I2C_SLV2_CTRL         = 0x2D;
    const uint8_t I2C_SLV3_ADDR         = 0x2E;
    const uint8_t I2C_SLV3_REG          = 0x2F;
    const uint8_t I2C_SLV3_CTRL         = 0x30;
    const uint8_t I2C_SLV4_ADDR         = 0x31;
    const uint8_t I2C_SLV4_REG          = 0x32;
    const uint8_t I2C_SLV4_DO           = 0x33;
    const uint8_t I2C_SLV4_CTRL         = 0x34;
    // Read only
    const uint8_t I2C_SLV4_DI           = 0x35;
    const uint8_t I2C_MST_STATUS        = 0x36;
    // Read / write
    const uint8_t INT_PIN_CFG           = 0x37;
    const uint8_t INT_ENABLE            = 0x38;
    // Read only
    const uint8_t INT_STATUS            = 0x3A;
    const uint8_t ACCEL_XOUT_H          = 0x3B;
    const uint8_t ACCEL_XOUT_L          = 0x3C;
    const uint8_t ACCEL_YOUT_H          = 0x3D;
    const uint8_t ACCEL_YOUT_L          = 0x3E;
    const uint8_t ACCEL_ZOUT_H          = 0x3F;
    const uint8_t ACCEL_ZOUT_L          = 0x40;
    const uint8_t TEMP_OUT_L            = 0x41;
    const uint8_t TEMP_OUT_H            = 0x42;
    const uint8_t GYRO_XOUT_H           = 0x43;
    const uint8_t GYRO_XOUT_L           = 0x44;
    const uint8_t GYRO_YOUT_H           = 0x45;
    const uint8_t GYRO_YOUT_L           = 0x46;
    const uint8_t GYRO_ZOUT_H           = 0x47;
    const uint8_t GYRO_ZOUT_L           = 0x48;
    const uint8_t EXT_SENS_DATA_00      = 0x49;
    const uint8_t EXT_SENS_DATA_01      = 0x4A;
    const uint8_t EXT_SENS_DATA_02      = 0x4B;
    const uint8_t EXT_SENS_DATA_03      = 0x4C;
    const uint8_t EXT_SENS_DATA_04      = 0x4D;
    const uint8_t EXT_SENS_DATA_05      = 0x4E;
    const uint8_t EXT_SENS_DATA_06      = 0x4F;
    const uint8_t EXT_SENS_DATA_07      = 0x50;
    const uint8_t EXT_SENS_DATA_08      = 0x51;
    const uint8_t EXT_SENS_DATA_09      = 0x52;
    const uint8_t EXT_SENS_DATA_10      = 0x53;
    const uint8_t EXT_SENS_DATA_11      = 0x54;
    const uint8_t EXT_SENS_DATA_12      = 0x55;
    const uint8_t EXT_SENS_DATA_13      = 0x56;
    const uint8_t EXT_SENS_DATA_14      = 0x57;
    const uint8_t EXT_SENS_DATA_15      = 0x58;
    const uint8_t EXT_SENS_DATA_16      = 0x59;
    const uint8_t EXT_SENS_DATA_17      = 0x5A;
    const uint8_t EXT_SENS_DATA_18      = 0x5B;
    const uint8_t EXT_SENS_DATA_19      = 0x5C;
    const uint8_t EXT_SENS_DATA_20      = 0x5D;
    const uint8_t EXT_SENS_DATA_21      = 0x5E;
    const uint8_t EXT_SENS_DATA_22      = 0x5F;
    const uint8_t EXT_SENS_DATA_23      = 0x60;
    // Read / write
    const uint8_t I2C_SLV0_DO           = 0x63;
    const uint8_t I2C_SLV1_DO           = 0x64;
    const uint8_t I2C_SLV2_DO           = 0x65;
    const uint8_t I2C_SLV3_DO           = 0x66;
    const uint8_t I2C_MST_DELAY_CTRL    = 0x67;
    const uint8_t SIGNAL_PATH_RESET     = 0x68;
    const uint8_t MOT_DETECT_CTRL       = 0x69;
    const uint8_t USER_CTRL             = 0x6A;
    const uint8_t PWR_MGMT_1            = 0x6B;
    const uint8_t PWR_MGMT_2            = 0x6C;
    const uint8_t FIFO_COUNTH           = 0x72;
    const uint8_t FIFO_COUNTL           = 0x73;
    const uint8_t FIFO_R_W              = 0x74;
    const uint8_t WHO_AM_I              = 0x75;
    const uint8_t XA_OFFSET_H           = 0x77;
    const uint8_t XA_OFFSET_L           = 0x78;
    const uint8_t YA_OFFSET_H           = 0x7A;
    const uint8_t YA_OFFSET_L           = 0x7B;
    const uint8_t ZA_OFFSET_H           = 0x7D;
    const uint8_t ZA_OFFSET_L           = 0x7E;
}

namespace MPU9250_SETTINGS
{
    // Generic
    const uint16_t SMPLRT_MIN           =    4;
    const uint16_t SMPLRT_MAX           = 1000;
    const uint16_t COMPASS_SMPLRT_MAX   =  100;

    // CONFIG
    const uint8_t FIFO_MODE_BLOCK       = 0x40;
    const uint8_t _MASK__GYRO_DLPF      = 0x07;
    const uint8_t GYRO_DLPF_250         = 0x00;
    const uint8_t GYRO_DLPF_184         = 0x01;
    const uint8_t GYRO_DLPF_92          = 0x02;
    const uint8_t GYRO_DLPF_41          = 0x03;
    const uint8_t GYRO_DLPF_20          = 0x04;
    const uint8_t GYRO_DLPF_10          = 0x05;
    const uint8_t GYRO_DLPF_5           = 0x06;
    const uint8_t GYRO_DLPF_3600        = 0x07;

    // GYRO_CONFIG
    const uint8_t GYRO_ST_EN            = 0xE0;
    const uint8_t _MASK__GYRO_FSR       = 0x18;
    const uint8_t GYRO_FSR_250          = 0x00;
    const uint8_t GYRO_FSR_500          = 0x08;
    const uint8_t GYRO_FSR_1000         = 0x10;
    const uint8_t GYRO_FSR_2000         = 0x18;

    // ACCEL_CONFIG
    const uint8_t ACCEL_ST_EN           = 0xE0;
    const uint8_t _MASK__ACCEL_FSR      = 0x18;
    const uint8_t ACCEL_FSR_2           = 0x00;
    const uint8_t ACCEL_FSR_4           = 0x08;
    const uint8_t ACCEL_FSR_8           = 0x10;
    const uint8_t ACCEL_FSR_16          = 0x18;

    // ACCEL_CONFIG2
    const uint8_t _MASK__ACCEL_DLPF     = 0x07;
    const uint8_t ACCEL_DLPF_460        = 0x00;
    const uint8_t ACCEL_DLPF_184        = 0x01;
    const uint8_t ACCEL_DLPF_92         = 0x02;
    const uint8_t ACCEL_DLPF_41         = 0x03;
    const uint8_t ACCEL_DLPF_20         = 0x04;
    const uint8_t ACCEL_DLPF_10         = 0x05;
    const uint8_t ACCEL_DLPF_5          = 0x06;

    // FIFO_EN
    const uint8_t FIFO_TEMP_OUT_EN      = 0x80;
    const uint8_t FIFO_GYRO_EN          = 0x70;
    const uint8_t FIFO_ACCEL_EN         = 0x08;
    const uint8_t FIFO_SLV2_EN          = 0x04;
    const uint8_t FIFO_SLV1_EN          = 0x02;
    const uint8_t FIFO_SLV0_EN          = 0x01;

    // I2C_MST_CTRL
    const uint8_t I2C_MULT_MST_EN       = 0x80;
    const uint8_t I2C_MST_WAIT_FOR_ES   = 0x40;
    const uint8_t SLV_3_FIFO_EN         = 0x20;
    const uint8_t I2C_MST_P_NSR         = 0x10;
    const uint8_t _MASK__I2C_MST_CLK    = 0x0F;
    const uint8_t I2C_MST_CLK_348       = 0x00;
    const uint8_t I2C_MST_CLK_333       = 0x01;
    const uint8_t I2C_MST_CLK_320       = 0x02;
    const uint8_t I2C_MST_CLK_308       = 0x03;
    const uint8_t I2C_MST_CLK_296       = 0x04;
    const uint8_t I2C_MST_CLK_286       = 0x05;
    const uint8_t I2C_MST_CLK_276       = 0x06;
    const uint8_t I2C_MST_CLK_267       = 0x07;
    const uint8_t I2C_MST_CLK_258       = 0x08;
    const uint8_t I2C_MST_CLK_500       = 0x09;
    const uint8_t I2C_MST_CLK_471       = 0x0A;
    const uint8_t I2C_MST_CLK_444       = 0x0B;
    const uint8_t I2C_MST_CLK_421       = 0x0C;
    const uint8_t I2C_MST_CLK_400       = 0x0D;
    const uint8_t I2C_MST_CLK_381       = 0x0E;
    const uint8_t I2C_MST_CLK_364       = 0x0F;

    // I2C_SLV4_CTRL
    const uint8_t _MASK__I2C_SLV4_CTRL  = 0x5F;

    // I2C_MST_DELAY_CTRL
    const uint8_t I2C_DELAY_ES_SHADOW   = 0x80;
    const uint8_t I2C_SLV4_DLY_EN       = 0x10;
    const uint8_t I2C_SLV3_DLY_EN       = 0x08;
    const uint8_t I2C_SLV2_DLY_EN       = 0x04;
    const uint8_t I2C_SLV1_DLY_EN       = 0x02;
    const uint8_t I2C_SLV0_DLY_EN       = 0x01;

    // USER_CTRL
    const uint8_t FIFO_ENABLE           = 0x40; // officially named FIFO_EN, but modified because it could be ambiguous with the register address named FIFO_EN
    const uint8_t I2C_MST_EN            = 0x20;
    const uint8_t I2C_IF_DIS            = 0x10;
    const uint8_t FIFO_RST              = 0x04;
    const uint8_t I2C_MST_RST           = 0x02;
    const uint8_t SIG_COND_RST          = 0x01;

    // PWR_MGMT_1
    const uint8_t H_RESET               = 0x80;
    const uint8_t SLEEP                 = 0x40;
    const uint8_t CYCLE                 = 0x20;
    const uint8_t GYRO_STANDBY          = 0x10;
    const uint8_t PD_PTAT               = 0x08;
    const uint8_t CLKSEL_INT            = 0x00;
    const uint8_t CLKSEL_AUTO           = 0x01;
    const uint8_t CLKSEL_STOP           = 0x07;

    // PWR_MGMT_2
    const uint8_t ALL_SENS_EN           = 0x00;
    const uint8_t ACCEL_X_DIS           = 0x20;
    const uint8_t ACCEL_Y_DIS           = 0x10;
    const uint8_t ACCEL_Z_DIS           = 0x08;
    const uint8_t GYRO_X_DIS            = 0x04;
    const uint8_t GYRO_Y_DIS            = 0x02;
    const uint8_t GYRO_Z_DIS            = 0x01;
}

namespace AK8963_REG_ADDR
{
    // Read only
    const uint8_t WIA                   = 0x00; // Device ID
    const uint8_t INFO                  = 0x01; // Information
    const uint8_t ST1                   = 0x02; // Status 1
    const uint8_t HXL                   = 0x03; // Measurement data X-axis
    const uint8_t HXH                   = 0x04; // Measurement data X-axis
    const uint8_t HYL                   = 0x05; // Measurement data Y-axis
    const uint8_t HYH                   = 0x06; // Measurement data Y-axis
    const uint8_t HZL                   = 0x07; // Measurement data Z-axis
    const uint8_t HZH                   = 0x08; // Measurement data Z-axis
    const uint8_t ST2                   = 0x09; // Status 2
    // Read / write
    const uint8_t CNTL1                 = 0x0A; // Control
    const uint8_t CNTL2                 = 0x0B; // Control 2
    const uint8_t ASTC                  = 0x0C; // Self-test
    const uint8_t I2CDIS                = 0x0F; // I2C disable
    // Read only
    const uint8_t ASAX                  = 0x10; // Fuse ROM X-axis sensitivity adjustment value
    const uint8_t ASAY                  = 0x11; // Fuse ROM Y-axis sensitivity adjustment value
    const uint8_t ASAZ                  = 0x12; // Fuse ROM Z-axis sensitivity adjustment value
}

namespace AK8963_SETTINGS
{
    // Generic
    const uint32_t MODE_CHG_DELAY_US    =  100; // Minimum time between mode changes (in µs)
    const uint16_t MAX_SAMPLE_RATE      =  100; // Maximum compass sample rate                      TODO: change this to something sensible

    // CNTL1
    const uint8_t _MASK__MODE           = 0x0F;
    const uint8_t MODE_POWERDOWN        = 0x00;
    const uint8_t MODE_SINGLE           = 0x01;
    const uint8_t MODE_CONT_1           = 0x02;
    const uint8_t MODE_CONT_2           = 0x06;
    const uint8_t MODE_EXT_TRIGGER      = 0x04;
    const uint8_t MODE_SELFTEST         = 0x08;
    const uint8_t MODE_FUSEROM          = 0x0F;
    const uint8_t _MASK__RESOLUTION     = 0x10;
    const uint8_t RESOLUTION_14BIT      = 0x00;
    const uint8_t RESOLUTION_16BIT      = 0x10;

    // CNTL2
    const uint8_t SRST                  = 0x01;

    // ASTC
    const uint8_t ST_EN                 = 0x40;
}

namespace MPU9250_TEST
{
    const uint8_t SAMPLE_RATE_DIV       = 0;                                    // Sample rate divider 1kHz     [MPU9250_REG_ADDR::SMPLRT_DIV   ]
    const uint8_t ACCEL_FSR             = MPU9250_SETTINGS::ACCEL_FSR_2;        // Accelerometer FSR +-2G       [MPU9250_REG_ADDR::ACCEL_CONFIG ]
    const uint8_t ACCEL_DLPF            = MPU9250_SETTINGS::ACCEL_DLPF_92;      // Accelerometer LPF 92kHz      [MPU9250_REG_ADDR::ACCEL_CONFIG2]
    const uint8_t GYRO_FSR              = MPU9250_SETTINGS::GYRO_FSR_250;       // Gyroscope FSR +-250DPS       [MPU9250_REG_ADDR::GYRO_CONFIG  ]
    const uint8_t GYRO_DLPF             = MPU9250_SETTINGS::GYRO_DLPF_92;       // Gyroscope LPF 92kHz          [MPU9250_REG_ADDR::CONFIG       ]

    const uint32_t STAB_TIME_US         = 200000;   // Stabilization time
    const size_t NR_OF_SAMPLES          = 200;      // Number of samples to collect

    const double ACCEL_HWU_G           = 16384.0;   // Accelerometer hardware-units per G (at FSR of +-2G)
    const double GYRO_HWU_DPS          = 131.0;     // Gyroscope hardware-units per DPS (at FSR of +-250)

    const float CRITERIA_A_GYRO         = 0.5;      // [(self-test bias - normal bias) / factory-trim] ratio must exceed(!) 50%
    const float CRITERIA_B_GYRO         = 60.0;     // [self-test bias - normal bias] must not exceed 60 DPS. This seems to be used when factory-trim values are unavailable.
    const float CRITERIA_C_GYRO         = 20.0;     // [normal bias] must not exceed 20 DPS.

    const float CRITERIA_A_ACCEL        = 0.5;      // [(self-test bias - normal bias) / factory-trim] ratio must not(!) exceed 50%
    const float CRITERIA_B_ACCEL_MIN    = 0.225;    // [self-test bias - normal bias] must exceed 225mg. This seems to be used when factory-trim values are unavailable.
    const float CRITERIA_B_ACCEL_MAX    = 0.675;    // [self-test bias - normal bias] must not exceed 675mg. This seems to be used when factory-trim values are unavailable.
    const float CRITERIA_C_ACCEL        = 0.5;      // [normal bias] must not exceed 500mg.

    const uint16_t ST_TABLE[256] = {
        2620,2646,2672,2699,2726,2753,2781,2808,            //   7
        2837,2865,2894,2923,2952,2981,3011,3041,            //  15
        3072,3102,3133,3165,3196,3228,3261,3293,            //  23
        3326,3359,3393,3427,3461,3496,3531,3566,            //  31
        3602,3638,3674,3711,3748,3786,3823,3862,            //  39
        3900,3939,3979,4019,4059,4099,4140,4182,            //  47
        4224,4266,4308,4352,4395,4439,4483,4528,            //  55
        4574,4619,4665,4712,4759,4807,4855,4903,            //  63
        4953,5002,5052,5103,5154,5205,5257,5310,            //  71
        5363,5417,5471,5525,5581,5636,5693,5750,            //  79
        5807,5865,5924,5983,6043,6104,6165,6226,            //  87
        6289,6351,6415,6479,6544,6609,6675,6742,            //  95
        6810,6878,6946,7016,7086,7157,7229,7301,            // 103
        7374,7448,7522,7597,7673,7750,7828,7906,            // 111
        7985,8065,8145,8227,8309,8392,8476,8561,            // 119
        8647,8733,8820,8909,8998,9088,9178,9270,            // 127
        9363,9457,9551,9647,9743,9841,9939,10038,           // 135
        10139,10240,10343,10446,10550,10656,10763,10870,    // 143
        10979,11089,11200,11312,11425,11539,11654,11771,    // 151
        11889,12008,12128,12249,12371,12495,12620,12746,    // 159
        12874,13002,13132,13264,13396,13530,13666,13802,    // 167
        13940,14080,14221,14363,14506,14652,14798,14946,    // 175
        15096,15247,15399,15553,15709,15866,16024,16184,    // 183
        16346,16510,16675,16842,17010,17180,17352,17526,    // 191
        17701,17878,18057,18237,18420,18604,18790,18978,    // 199
        19167,19359,19553,19748,19946,20145,20347,20550,    // 207
        20756,20963,21173,21385,21598,21814,22033,22253,    // 215
        22475,22700,22927,23156,23388,23622,23858,24097,    // 223
        24338,24581,24827,25075,25326,25579,25835,26093,    // 231
        26354,26618,26884,27153,27424,27699,27976,28255,    // 239
        28538,28823,29112,29403,29697,29994,30294,30597,    // 247
        30903,31212,31524,31839,32157,32479,32804,33132     // 255
    };
}

namespace AK8963_TEST
{
    const uint16_t DATA_READY_TIMEOUT_MS    = 500;

    // 16-bit
    const int16_t CRITERIA_HX_MIN           = -200;
    const int16_t CRITERIA_HX_MAX           = 200;
    const int16_t CRITERIA_HY_MIN           = -200;
    const int16_t CRITERIA_HY_MAX           = 200;
    const int16_t CRITERIA_HZ_MIN           = -3200;
    const int16_t CRITERIA_HZ_MAX           = -800;
}

#endif // MPU9250_H_
