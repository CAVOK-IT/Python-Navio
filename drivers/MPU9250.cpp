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

#include "MPU9250.h"

using namespace MPU9250_REG_ADDR;
using namespace MPU9250_SETTINGS;


MPU9250::MPU9250(uint8_t mode, std::string spi_dev)
{
    imu_mode = mode;
    spi_device_name = spi_dev;

    MPU6500_calibrated = false;
    AK8963_calibrated = false;
    AK8963_measurement_enabled = false;

    internal_sample_rate = 8;   // kHz

    accel_hwu_g = 0.0;
    gyro_hwu_dps = 0.0;
    mag_sens_adj[0] = mag_sens_adj[1] = mag_sens_adj[2] = 1.0;
}

MPU9250::~MPU9250()
{
}

bool MPU9250::readByte(uint8_t reg_address, uint8_t* rx_data)
{
    return readBytes(reg_address, 1, rx_data);
}

bool MPU9250::readBytes(uint8_t reg_address, size_t length, uint8_t* rx_data)
{
    uint8_t tx[256] = {(uint8_t)(reg_address | SPI_READ_FLAG)};
    uint8_t rx[256] = {0};

    if (SPIdev::transfer(spi_device_name.c_str(), (unsigned char *)tx, (unsigned char *)rx, length+1, 100000) < 0) {
        return false;
    }

    memmove(rx_data, rx+1, length);
    return true;
}

bool MPU9250::writeByte(uint8_t reg_address, uint8_t tx_data)
{
    uint8_t tx[2] = {reg_address, tx_data};
    uint8_t rx[2] = {0};

    if (SPIdev::transfer(spi_device_name.c_str(), (unsigned char *)tx, (unsigned char *)rx, 2, 100000) < 0) {
        return false;
    }

    return true;
}

bool MPU9250::readI2CByte(uint8_t dev_address, uint8_t reg_address, uint8_t* rx_data)
{
    return readI2CBytes(dev_address, reg_address, 1, rx_data);
}

bool MPU9250::readI2CBytes(uint8_t dev_address, uint8_t reg_address, uint8_t length, uint8_t* rx_data)
{
    // The MPU9250 contains only 24 EXT_SENS_DATA registers.
    if ((EXT_SENS_DATA_00 + length) > EXT_SENS_DATA_23) {
        return false;
    }

    // Only least-significant nibble is used for read length
    uint8_t slv_ctrl_mask = (length & 0x0F) | I2C_READ_FLAG;

    // Initiate read from device at dev_address
    if (writeByte(I2C_SLV0_ADDR, dev_address|I2C_READ_FLAG)) {
        // Select register to start the read from
        if (writeByte(I2C_SLV0_REG, reg_address)) {
            // Read length byte(s) into EXT_SENS_DATA_xx registers
            if (writeByte(I2C_SLV0_CTRL, slv_ctrl_mask)) {
                // Wait for data to be written to registers
                usleep(10000);   // TODO: check rates to set a matching sleep time

                return readBytes(EXT_SENS_DATA_00, length, rx_data);
            }
        }
    }

    return false;
}

bool MPU9250::writeI2CByte(uint8_t dev_address, uint8_t reg_address, uint8_t data)
{
    if (writeByte(I2C_SLV0_ADDR, dev_address)) {      // Initiate write to device at dev_address
        if (writeByte(I2C_SLV0_REG, reg_address)) {   // Select register at reg_address)
            return writeByte(I2C_SLV0_DO, data);      // Write data to slave data-out register
        }
    }

    return false;
}

void MPU9250::initialize()
{
    uint8_t response;

    testConnection();

    // Reset device
    writeByte(PWR_MGMT_1, H_RESET);
    usleep(250000);
    // Enable I2C master module and put serial interface in SPI-only mode (!important to prevent it from reverting to I2C mode)
    writeByte(USER_CTRL, (I2C_MST_EN | I2C_IF_DIS));
    usleep(10000);

    // Set clock source to auto-select
    writeByte(PWR_MGMT_1, CLKSEL_AUTO);
    usleep(10000);
    // Enable accelerometers and gyroscopes
    writeByte(PWR_MGMT_2, ALL_SENS_EN);
    usleep(10000);
    initI2Cmaster();

    // Set sample rates
    setSampleRate(200);
    AK8963_setSampleRate(50);

    // Set DLPF's and internal sample rate
    setDLPF(92);
    setAccelDLPF(92);

    // Set full-scale ranges of sensors
    setGyroFSR(2000);
    setAccelFSR(4);


    AK8963_enableMeasurement();
    //AK8963_disableMeasurement

    // mpu_set_gyro_fsr(2000)
    // mpu_set_accel_fsr(2)
    // mpu_set_lpf(42)
    // mpu_set_sample_rate(50)  <- seems to also set lpf to 1/2 sample rate
    // mpu_configure_fifo(0)
    // setup_compass();
    // mpu_set_compass_sample_rate(10)

    // Perform self-test.
    //self_test();
    // Perform initial calibrations
    //calibrate();

    // Initialize (and calibrate) AK8963
    //AK8963_initialize();
}

void MPU9250::initI2Cmaster()
{
    // Set I2C master clock to 400kHz
    writeByte(I2C_MST_CTRL, I2C_MST_CLK_400);
    usleep(10000);

    /*
    /  The AK8963 requires some specific setup to be able to work as an I2C slave of the MPU9250.
    /  SLV0 is used to continuously read from the AK8963's measurement registers (ST1-ST2).
    /  SLV1 is used to continuously trigger a single measurement.
    /  The actual rate can then be set using the I2C_MST_DELAY bits in the I2C_SLV4_CTRL register.
    /  We'll be using the SLV4 interface to configure the AK8963, because it'll only perform a single
    /  I2C transfer.
    */

    // First, we set SLV0 as an interface to read data from the AK8963.
    writeByte(I2C_SLV0_ADDR, AK8963_I2C_ADDR|I2C_READ_FLAG);    // set slave address and prepare for READ
    usleep(10000);
    writeByte(I2C_SLV0_REG, AK8963_REG_ADDR::ST1);              // select ST1 as the register to start reading from
    usleep(10000);
    // TODO: setup byte swapping

    // Then, we set SLV1 as an interface to write to the AK8963
    writeByte(I2C_SLV1_ADDR, AK8963_I2C_ADDR);                  // set slave address and prepare for WRITE
    usleep(10000);
    writeByte(I2C_SLV1_REG, AK8963_REG_ADDR::CNTL1);            // select CNTL1 as the register to write to
    usleep(10000);
    writeByte(I2C_SLV1_DO, AK8963_SETTINGS::MODE_SINGLE);       // set DO to trigger single-measurement mode
    usleep(10000);

    // Enable SLV0 and SLV1 delays as set in I2C_SLV4_CTRL register (I2C_MST_DLY bits)
    writeByte(I2C_MST_DELAY_CTRL, (I2C_SLV1_DLY_EN | I2C_SLV0_DLY_EN));
}

bool MPU9250::testConnection()
{
    uint8_t response = 0;

    //readByte(WHO_AM_I, &response);
    for (size_t i=0; i<10; i++) {
        usleep(10000);
        readByte(WHO_AM_I, &response);
        if (response != 0) {
            break;
        }
    }

#ifdef NAVIO_DEBUG
    printf("MPU9250 WAI response: %#x\n", response);
#endif

    if (response != 0x71) {
        //return false;
    }

    if (imu_mode == MODE_9DOF) {
        //AK8963_initialize();
        if (!AK8963_testConnection()) {
            //return false;
        }
    }

    return true;
}

void MPU9250::readConfig()
{
    readByte(SMPLRT_DIV, &settings.smplrt_div);
    readByte(CONFIG, &settings.config);
    readByte(GYRO_CONFIG, &settings.gyro_config);
    readByte(ACCEL_CONFIG, &settings.accel_config);
    readByte(ACCEL_CONFIG2, &settings.accel_config2);
    readByte(FIFO_EN, &settings.fifo_en);
    readByte(I2C_MST_CTRL, &settings.i2c_mst_ctrl);
    readByte(I2C_MST_DELAY_CTRL, &settings.i2c_mst_delay_ctrl);
    readByte(USER_CTRL, &settings.user_ctrl);
    readByte(PWR_MGMT_1, &settings.pwr_mgmt_1);
    readByte(PWR_MGMT_2, &settings.pwr_mgmt_2);
    readByte(I2C_SLV4_CTRL, &settings.i2c_slv4_ctrl);
    settings.i2c_slv4_ctrl &= _MASK__I2C_SLV4_CTRL; // Dont't want to preserve the I2C_SLV4_EN and I2C_SLV4_REG_DIS bits.

    // Also update some instance variables
    getSampleRate();
    getDLPF();
    getGyroFSR();
    getAccelFSR();
}

void MPU9250::writeConfig()
{
    writeByte(SMPLRT_DIV, settings.smplrt_div);
    writeByte(CONFIG, settings.config);
    writeByte(GYRO_CONFIG, settings.gyro_config);
    writeByte(ACCEL_CONFIG, settings.accel_config);
    writeByte(ACCEL_CONFIG2, settings.accel_config2);
    writeByte(FIFO_EN, settings.fifo_en);
    writeByte(I2C_MST_CTRL, settings.i2c_mst_ctrl);
    writeByte(I2C_MST_DELAY_CTRL, settings.i2c_mst_delay_ctrl);
    writeByte(USER_CTRL, settings.user_ctrl);
    writeByte(PWR_MGMT_1, settings.pwr_mgmt_1);
    writeByte(PWR_MGMT_2, settings.pwr_mgmt_2);
    writeByte(I2C_SLV4_CTRL, settings.i2c_slv4_ctrl);

    // Also update some instance variables
    getSampleRate();
    getDLPF();
    getGyroFSR();
    getAccelFSR();
}

uint8_t MPU9250::selfTest()
{
    // result is a 8-bit mask representing each sensors pass(=0)/fail(=1) status:
    //     0b00000000   => all pass
    //     bit 0:2      => gyro pass/fail   (x y z)
    //     bit 3:5      => accel pass/fail  (x y z)
    //     bit 6:7      => mag pass/fail    (00 = OK, all other values indicate failure)
    // f.i. 0b00100000 represents z-axis accelerometer failed.
    uint8_t result = 0;

    // Store original config in settings, to be restored after testing.
    readConfig();

    /* Setup */
    // Set sample rate divider to 1kHz
    writeByte(SMPLRT_DIV, 0x00);
    // Set gyro LPF to 92kHz
    writeByte(CONFIG, GYRO_DLPF_92);
    // Set gyro FSR to 250dps
    writeByte(GYRO_CONFIG, MPU9250_TEST::GYRO_FSR);
    gyro_hwu_dps = 131.072;   // temporarily change this until restoring from settings
    // Set acc FSR to 2g
    writeByte(ACCEL_CONFIG, MPU9250_TEST::ACCEL_FSR);
    accel_hwu_g = 16384.0;  // temporarily change this until restoring from settings
    // Set acc LPF to 92kHz
    writeByte(ACCEL_CONFIG2, ACCEL_DLPF_92);

    // Stabilize
    usleep(MPU9250_TEST::STAB_TIME_US);

    double gyro_bias[3] = {};   // In deg/s
    double accel_bias[3] = {};  // In g
    getBiases(gyro_bias, accel_bias, MPU9250_TEST::NR_OF_SAMPLES);

    // Set gyro FSR to 250dps and enable test-mode
    writeByte(GYRO_CONFIG, (MPU9250_TEST::GYRO_FSR | GYRO_ST_EN));
    // gyro_hwu_dps = 131.0;
    // Set acc FSR to 2g and enable test-mode
    writeByte(ACCEL_CONFIG, (MPU9250_TEST::ACCEL_FSR | ACCEL_ST_EN));
    // accel_hwu_g = 16384.0;

    // Stabilize
    usleep(MPU9250_TEST::STAB_TIME_US);

    double gyro_st_bias[3] = {};    // In deg/s
    double accel_st_bias[3] = {};   // In g
    getBiases(gyro_st_bias, accel_st_bias, MPU9250_TEST::NR_OF_SAMPLES);

    double gyro_st_response[3];
    double accel_st_response[3];

    for (size_t i=0; i<3; i++) {
        gyro_st_response[i] = gyro_st_bias[i] - gyro_bias[i];
        accel_st_response[i] = accel_st_bias[i] - accel_bias[i];
    }

    // Read factory self-test data and calculate ratio bewteen those and the biases
    // just calculated during testing.
    // The factory values read are just indexes for the look-up table that contains the
    // actual test values.
    double gyro_ftrim[6] = {};
    double accel_ftrim[6] = {};
    double gyro_st_shift_ratio[3] = {};
    double accel_st_shift_ratio[3] = {};

    uint8_t st_data[6] = {};
    readBytes(SELF_TEST_X_GYRO, 3, st_data);
    readBytes(SELF_TEST_X_ACCEL, 3, st_data+3);
    for (size_t ii=0; ii<3; ii++) {
        if (st_data[ii] != 0) {
            gyro_ftrim[ii] = MPU9250_TEST::ST_TABLE[st_data[ii] - 1] / gyro_hwu_dps;
            gyro_st_shift_ratio[ii] = gyro_st_response[ii] / gyro_ftrim[ii];
        }
        if (st_data[ii+3] != 0) {
            accel_ftrim[ii] = MPU9250_TEST::ST_TABLE[st_data[ii+3] - 1] / accel_hwu_g;
            accel_st_shift_ratio[ii] = accel_st_response[ii] / accel_ftrim[ii] - 1.0;   // important! -1.0
        }
    }

    // Check against criteria
    for (size_t iii=0; iii<3; iii++) {
        // Gyro
        if (gyro_st_shift_ratio[iii] != 0) {
            // Criteria A
            if (fabs(gyro_st_shift_ratio[iii]) < MPU9250_TEST::CRITERIA_A_GYRO) {
                // FAILED!
                result |= 1 << iii;
            }
        }
        else {
            // Criteria B (factory offset value not available)
            if (gyro_st_response[iii] < MPU9250_TEST::CRITERIA_B_GYRO) {
                // FAILED!
                result |= 1 << iii;
            }
        }
        // Criteria C
        if (fabs(gyro_bias[iii]) > MPU9250_TEST::CRITERIA_C_GYRO) {
            // FAILED!
            result |= 1 << iii;
        }

        // Accel
        if (accel_st_shift_ratio[iii] != 0) {
            // Criteria A
            if (fabs(accel_st_shift_ratio[iii]) > MPU9250_TEST::CRITERIA_A_ACCEL) {
                // FAILED!
                result |= 1 << (iii + 3);
            }
        }
        else {
            // Criteria B (factory offset value not available)
            if (accel_st_response[iii] < MPU9250_TEST::CRITERIA_B_ACCEL_MIN || accel_st_response[iii] > MPU9250_TEST::CRITERIA_B_ACCEL_MAX) {
                // FAILED!
                result |= 1 << (iii + 3);
            }
        }
        // Criteria C
        if (fabs(accel_bias[iii]) > MPU9250_TEST::CRITERIA_C_ACCEL) {
            // FAILED!
            result |= 1 << (iii + 3);
        }
    }

    // Magnetometer self-test
    if (imu_mode == MODE_9DOF) {
        //AK8963_self_test();
    }

#ifdef NAVIO_DEBUG
    if (result == 0) {
        printf("MPU9250 self-test passed!\n\n");
    }
    else {
        printf("MPU9250 self-test failed!\n\n");
    }
    printf("+-------------------------------------------------+\n");
    printf("|     |     Gyro     |     Accel    |     Mag     |\n");
    printf("|-------------------------------------------------|\n");
    printf("|  X  |     %s     |     %s     |     %s    |\n", ((1 & result)?"FAIL":"PASS"), ((8 & result)?"FAIL":"PASS"), ((0)?"FAIL":"PASS"));
    printf("|  Y  |     %s     |     %s     |     %s    |\n", ((2 & result)?"FAIL":"PASS"), ((16 & result)?"FAIL":"PASS"), ((0)?"FAIL":"PASS"));
    printf("|  Z  |     %s     |     %s     |     %s    |\n", ((4 & result)?"FAIL":"PASS"), ((32 & result)?"FAIL":"PASS"), ((0)?"FAIL":"PASS"));
    printf("+-------------------------------------------------+\n");

    printf("\nNormal biases:\n");
    printf("\tGyro:\t%10f, %10f, %10f (deg/s)\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    printf("\tAccel:\t%10f, %10f, %10f (g)\n", accel_bias[0], accel_bias[1], accel_bias[2]);
    printf("Self-test biases:\n");
    printf("\tGyro:\t%10f, %10f, %10f (deg/s)\n", gyro_st_bias[0], gyro_st_bias[1], gyro_st_bias[2]);
    printf("\tAccel:\t%10f, %10f, %10f (g)\n", accel_st_bias[0], accel_st_bias[1], accel_st_bias[2]);
    printf("Self-test responses (self-test bias - normal bias):\n");
    printf("\tGyro:\t%10f, %10f, %10f (deg/s)\n", gyro_st_response[0], gyro_st_response[1], gyro_st_response[2]);
    printf("\tAccel:\t%10f, %10f, %10f (g)\n", accel_st_response[0], accel_st_response[1], accel_st_response[2]);
    printf("Factory-trim values:\n");
    printf("\tGyro:\t%10f, %10f, %10f (deg/s)\n", gyro_ftrim[0], gyro_ftrim[1], gyro_ftrim[2]);
    printf("\tAccel:\t%10f, %10f, %10f (g)\n", accel_ftrim[0], accel_ftrim[1], accel_ftrim[2]);
    printf("Self-test shift ratios:\n");
    printf("\tGyro:\t%10f, %10f, %10f (%%)\n", gyro_st_shift_ratio[0], gyro_st_shift_ratio[1], gyro_st_shift_ratio[2]);
    printf("\tAccel:\t%10f, %10f, %10f (%%)\n", accel_st_shift_ratio[0], accel_st_shift_ratio[1], accel_st_shift_ratio[2]);
#endif // NAVIO_DEBUG

    /* Restore original config */
    // The settings should still contain the original values, so write them to the config registers.
    writeConfig();

    return result;
}

void MPU9250::calibrate()
{

}

void MPU9250::getMotion6(float* accel, float* gyro)
{

}


void MPU9250::getMotion9(float* accel, float* gyro, float* mag)
{
    uint8_t raw_data[22] = {};

    readBytes(ACCEL_XOUT_H, 22, raw_data);
    for (size_t i=0; i<3; i++) {
        accel[i] = (float)((int16_t)(((int16_t)raw_data[2*i] << 8) | raw_data[2*i+1]) / accel_hwu_g);
        gyro[i] = (float)((int16_t)(((int16_t)raw_data[2*i+8] << 8) | raw_data[2*i+9]) / gyro_hwu_dps);
        mag[i] = (float)((int16_t)(((int16_t)raw_data[2*i+16] << 8) | raw_data[2*i+15])); // AK8963 bytes are little-endian
    }

#ifdef NAVIO_DEBUG
    printf("ST1: %#x, ST2: %#x\n", raw_data[14], raw_data[21]);
#endif // NAVIO_DEBUG

}

void MPU9250::getBiases(double* gyro, double* accel, size_t nr_of_samples)
{
    uint8_t raw_data[14] = {};
    int32_t accel_accumulated[3] = {};
    int32_t gyro_accumulated[3] = {};

    // Get samples.
    for (size_t i=0; i<nr_of_samples; i++) {
        readBytes(ACCEL_XOUT_H, 14, raw_data);

        accel_accumulated[0] += (int16_t)(((int16_t)raw_data[0] << 8) | raw_data[1]);
        accel_accumulated[1] += (int16_t)(((int16_t)raw_data[2] << 8) | raw_data[3]);
        accel_accumulated[2] += (int16_t)(((int16_t)raw_data[4] << 8) | raw_data[5]);

        gyro_accumulated[0] += (int16_t)(((int16_t)raw_data[8] << 8) | raw_data[9]);
        gyro_accumulated[1] += (int16_t)(((int16_t)raw_data[10] << 8) | raw_data[11]);
        gyro_accumulated[2] += (int16_t)(((int16_t)raw_data[12] << 8) | raw_data[13]);
    }

    // Reduce to averages.
    for (size_t ii=0; ii<3; ii++) {
        accel[ii] = (double)(accel_accumulated[ii] / (double)nr_of_samples) / accel_hwu_g;
        gyro[ii] = (double)(gyro_accumulated[ii] / (double)nr_of_samples) / gyro_hwu_dps;
    }

    // Remove gravity.
    if (accel[2] > 0.0) {
        accel[2] -= 1.0;
    }
    else {
        accel[2] += 1.0;
    }

#ifdef NAVIO_DEBUG
    printf("Measured accelerometer biases:\tx: %10f, y: %10f, z: %10f (g)\n", accel[0], accel[1], accel[2]);
    printf("Measured gyroscope biases:\tx: %10f, y: %10f, z: %10f (deg/s)\n", gyro[0], gyro[1], gyro[2]);
#endif // NAVIO_DEBUG
}

uint16_t MPU9250::getSampleRate()
{
    // Sample-rate divider is only used when internal sample-rate is 1kHz
    if (internal_sample_rate != 1) {
        return internal_sample_rate * 1000;
    }

    return 1000 / (1 + settings.smplrt_div);
}

uint16_t MPU9250::setSampleRate(uint16_t rate)
{
    uint8_t divider;

    if (rate < SMPLRT_MIN) {
        rate = SMPLRT_MIN;
    }
    if (rate > SMPLRT_MAX) {
        rate = SMPLRT_MAX;
    }

    divider = 1000 / rate - 1; // Samplerate divider is only used at 1kHz
    if (divider != settings.smplrt_div) {
        if (writeByte(SMPLRT_DIV, divider)) {
            readByte(SMPLRT_DIV, &settings.smplrt_div);
        }
    }

    // ??? TODO: set dlpf to half sample-rate ???

    return getSampleRate();
}

uint16_t MPU9250::getDLPF()
{
    uint16_t result = 0;

    switch (settings.config & _MASK__GYRO_DLPF) {
        case GYRO_DLPF_3600:
            result = 3600;
            internal_sample_rate = 8;
            break;
        case GYRO_DLPF_250:
            result = 250;
            internal_sample_rate = 8;
            break;
        case GYRO_DLPF_184:
            result = 184;
            internal_sample_rate = 1;
            break;
        case GYRO_DLPF_92:
            result = 92;
            internal_sample_rate = 1;
            break;
        case GYRO_DLPF_41:
            result = 41;
            internal_sample_rate = 1;
            break;
        case GYRO_DLPF_20:
            result = 20;
            internal_sample_rate = 1;
            break;
        case GYRO_DLPF_10:
            result = 10;
            internal_sample_rate = 1;
            break;
        case GYRO_DLPF_5:
            result = 5;
            internal_sample_rate = 1;
            break;
    }

    return result;
}

uint16_t MPU9250::setDLPF(uint16_t dlpf)
{
    // Note: this also sets the internal sample rate!

    uint8_t dlpf_config = (settings.config & ~_MASK__GYRO_DLPF); //| (dlpf & _MASK__GYRO_DLPF);

    if (dlpf >= 3600) {
        dlpf_config |= GYRO_DLPF_3600;
    }
    else if (dlpf >= 250) {
        dlpf_config |= GYRO_DLPF_250;
    }
    else if (dlpf >= 184) {
        dlpf_config |= GYRO_DLPF_184;
    }
    else if (dlpf >= 92) {
        dlpf_config |= GYRO_DLPF_92;
    }
    else if (dlpf >= 41) {
        dlpf_config |= GYRO_DLPF_41;
    }
    else if (dlpf >= 20) {
        dlpf_config |= GYRO_DLPF_20;
    }
    else if (dlpf >= 10) {
        dlpf_config |= GYRO_DLPF_10;
    }
    else if (dlpf >= 5) {
        dlpf_config |= GYRO_DLPF_5;
    }

    if (dlpf_config != settings.config) {
        if (writeByte(CONFIG, dlpf_config)) {
            readByte(CONFIG, &settings.config);
        }
    }

    return getDLPF();  // get_dlpf() also updates internal_sample_rate
}

uint16_t MPU9250::getAccelDLPF()
{
    uint16_t result = 0;

    switch (settings.accel_config2 & _MASK__ACCEL_DLPF) {
        case ACCEL_DLPF_460:
            result = 460;
            break;
        case ACCEL_DLPF_184:
            result = 184;
            break;
        case ACCEL_DLPF_92:
            result = 92;
            break;
        case ACCEL_DLPF_41:
            result = 41;
            break;
        case ACCEL_DLPF_20:
            result = 20;
            break;
        case ACCEL_DLPF_10:
            result = 10;
            break;
        case ACCEL_DLPF_5:
            result = 5;
            break;
    }

    return result;
}

uint16_t MPU9250::setAccelDLPF(uint16_t accel_dlpf)
{
    uint8_t accel_dlpf_config = (settings.accel_config2 & ~_MASK__ACCEL_DLPF); //| (accel_dlpf & _MASK__ACCEL_DLPF);

    if (accel_dlpf >= 460) {
        accel_dlpf_config |= ACCEL_DLPF_460;
    }
    else if (accel_dlpf >= 184) {
        accel_dlpf_config |= ACCEL_DLPF_184;
    }
    else if (accel_dlpf >= 92) {
        accel_dlpf_config |= ACCEL_DLPF_92;
    }
    else if (accel_dlpf >= 41) {
        accel_dlpf_config |= ACCEL_DLPF_41;
    }
    else if (accel_dlpf >= 20) {
        accel_dlpf_config |= ACCEL_DLPF_20;
    }
    else if (accel_dlpf >= 10) {
        accel_dlpf_config |= ACCEL_DLPF_10;
    }
    else if (accel_dlpf >= 5) {
        accel_dlpf_config |= ACCEL_DLPF_5;
    }

    if (accel_dlpf_config != settings.accel_config2) {
        if (writeByte(CONFIG, accel_dlpf_config)) {
            readByte(CONFIG, &settings.accel_config2);
        }
    }

    return getAccelDLPF();
}

uint16_t MPU9250::getGyroFSR()
{
    uint16_t result = 0;

    switch (settings.gyro_config & _MASK__GYRO_FSR) {
        case GYRO_FSR_250:
            result = 250;
            gyro_hwu_dps = 131.072;
            break;
        case GYRO_FSR_500:
            result = 500;
            gyro_hwu_dps = 65.5;
            break;
        case GYRO_FSR_1000:
            result = 1000;
            gyro_hwu_dps = 32.8;
            break;
        case GYRO_FSR_2000:
            result = 2000;
            gyro_hwu_dps = 16.4;
            break;
    }

    return result;
}

uint16_t MPU9250::setGyroFSR(uint16_t gyro_fsr)
{
    uint8_t gyro_config = (settings.gyro_config & ~_MASK__GYRO_FSR); //| (gyro_fsr & _MASK__GYRO_FSR);

    if (gyro_fsr >= 2000) {
        gyro_config |= GYRO_FSR_2000;
    }
    else if (gyro_fsr >= 1000) {
        gyro_config |= GYRO_FSR_1000;
    }
    else if (gyro_fsr >= 500) {
        gyro_config |= GYRO_FSR_500;
    }
    else if (gyro_fsr >= 250) {
        gyro_config |= GYRO_FSR_250;
    }

    if (gyro_config != settings.gyro_config) {
        if (writeByte(GYRO_CONFIG, gyro_config)) {
            readByte(GYRO_CONFIG, &settings.gyro_config);
        }
    }

    return getGyroFSR();
}

uint16_t MPU9250::getAccelFSR()
{
    uint16_t result = 0;

    switch (settings.accel_config & _MASK__ACCEL_FSR) {
        case ACCEL_FSR_16:
            result = 16;
            accel_hwu_g = 2048.0;
            break;
        case ACCEL_FSR_8:
            result = 8;
            accel_hwu_g = 4096.0;
            break;
        case ACCEL_FSR_4:
            result = 4;
            accel_hwu_g = 8192.0;
            break;
        case ACCEL_FSR_2:
            result = 2;
            accel_hwu_g = 16384.0;
            break;
    }

    return result;
}

uint16_t MPU9250::setAccelFSR(uint16_t accel_fsr)
{
    uint8_t accel_config = (settings.accel_config & ~_MASK__ACCEL_FSR); //| (accel_fsr & _MASK__ACCEL_FSR);

    if (accel_fsr >= 16) {
        accel_config |= ACCEL_FSR_16;
    }
    else if (accel_fsr >= 8) {
        accel_config |= ACCEL_FSR_8;
    }
    else if (accel_fsr >= 4) {
        accel_config |= ACCEL_FSR_4;
    }
    else if (accel_fsr >= 2) {
        accel_config |= ACCEL_FSR_2;
    }

    if (accel_config != settings.accel_config) {
        if (writeByte(ACCEL_CONFIG, accel_config)) {
            readByte(ACCEL_CONFIG, &settings.accel_config);
        }
    }

    return getAccelFSR();
}

void MPU9250::AK8963_initialize()
{
    // RESET
    //AK8963_reset()

    // CALIBRATE
    //AK8963_calibrate()

    // SELF-TEST

    // ? RESET ?





    //usleep(100000);
    //double mag_data[3];
    // for (size_t i=0; i<25; i++) {
    //     usleep(100000);
         //AK8963_read(mag_data);
    // }


    // // Do a soft-reset of the AK8963
    // writeByte(I2C_SLV1_REG, AK8963_REG_ADDR::CNTL1);            // select CNTL1 register address to write to
    // printf("Resetting AK8963.\n");
    // writeByte(I2C_SLV1_DO, AK8963_SETTINGS::SRST);              // write reset to SLV1
    // usleep(10000);
    // printf("Going into power-down mode.\n");
    // writeByte(I2C_SLV1_DO, AK8963_SETTINGS::MODE_POWERDOWN);     Make sure we transit to power-down mode (not sure
    //                                                                if this is needed. Is the DO register written to
    //                                                                the slave only once or not??).
    // usleep(100000);
    // // The AK8963 should be in power-down mode now

    // // Start "calibration" routine
    // //AK8963_calibrate(); // DOESN'T WORK HERE!?!?!
    // // We should have returned to power-down mode after calibration.





    // writeByte(I2C_SLV1_REG, AK8963_REG_ADDR::CNTL1);            // select register address to write to
    // writeByte(I2C_SLV1_DO, AK8963_SETTINGS::MODE_POWERDOWN);    // write this to slv1
    // usleep(100000);

    // Setup slave 0 to


    // Reset device




    //writeI2CByte(AK8963_I2C_ADDR, AK8963_REG_ADDR::CNTL2, AK8963_SETTINGS::SRST);
    //usleep(100000);

    // Perform initial calibrations
    //AK8963_calibrate();

    // TODO
}

bool MPU9250::AK8963_testConnection()
{
    uint8_t response;

    writeByte(I2C_SLV4_ADDR, (AK8963_I2C_ADDR | I2C_READ_FLAG));
    usleep(10000);
    writeByte(I2C_SLV4_REG, AK8963_REG_ADDR::WIA);
    usleep(10000);
    writeByte(I2C_SLV4_CTRL, I2C_READ_FLAG);

    for (size_t i=0; i<10; i++) {
        usleep(10000);
        readByte(I2C_SLV4_DI, &response);
        if (response != 0) {
            break;
        }
    }

#ifdef NAVIO_DEBUG
    printf("AK8963 WIA response: %#x\n", response);
#endif // NAVIO_DEBUG

    if (response != 0x48) {
        return false;
    }

    return true;
}

void MPU9250::AK8963_reset()
{

}

bool MPU9250::AK8963_setMode(uint8_t mode)
{
    uint8_t tmp_ctrl;

    if (writeByte(I2C_SLV4_REG, AK8963_REG_ADDR::CNTL1)) {
        usleep(10000);
        if (writeByte(I2C_SLV4_DO, mode)) {
            usleep(10000);
            if (readByte(I2C_SLV4_CTRL, &tmp_ctrl)) {
                usleep(10000);
                if (writeByte(I2C_SLV4_CTRL, (tmp_ctrl | 0x80))) {
                    // Wait min. time between switching modes (just-in-case the mode is changed directly after calling this function).
                    usleep(AK8963_SETTINGS::MODE_CHG_DELAY_US);
                    return true;
                }
            }
        }
    }

    return false;
}

void MPU9250::AK8963_enableMeasurement()
{
    if (AK8963_measurement_enabled) {
        return;
    }

    uint8_t tmp_ctrl;

    readByte(I2C_SLV1_CTRL, &tmp_ctrl);
    writeByte(I2C_SLV1_CTRL, ((tmp_ctrl & 0xF0) | 0x81));  // Enable 1-byte writes to SLV1 (0x01 of 0x81)
    usleep(10000);
    readByte(I2C_SLV0_CTRL, &tmp_ctrl);
    writeByte(I2C_SLV0_CTRL, ((tmp_ctrl & 0xF0) | 0x88));  // Enable 8-byte reads from SLV0

    AK8963_measurement_enabled = true;
}

void MPU9250::AK8963_disableMeasurement()
{
    if (!AK8963_measurement_enabled) {
        return;
    }

    uint8_t tmp_ctrl;

    readByte(I2C_SLV1_CTRL, &tmp_ctrl);
    writeByte(I2C_SLV1_CTRL, (tmp_ctrl & ~I2C_READ_FLAG));  // Disable write to SLV1
    usleep(10000);
    readByte(I2C_SLV0_CTRL, &tmp_ctrl);
    writeByte(I2C_SLV0_CTRL, (tmp_ctrl & ~I2C_READ_FLAG));  // Disable read from SLV0

    AK8963_measurement_enabled = false;
}

uint16_t MPU9250::AK8963_getSampleRate()
{
    uint16_t sample_rate = getSampleRate();

    return sample_rate / (1 + (settings.i2c_slv4_ctrl & 0x1F));
}

uint16_t MPU9250::AK8963_setSampleRate(uint16_t rate)
{
    uint16_t sample_rate = getSampleRate();

    if (rate > sample_rate) {
        rate = sample_rate;
    }
    if (rate < 1) {
        rate = 1;
    }
    if (rate > AK8963_SETTINGS::MAX_SAMPLE_RATE) {
        rate = AK8963_SETTINGS::MAX_SAMPLE_RATE;
    }

    uint16_t divider = sample_rate / rate - 1;       // Samplerate is relative to the internal sample-rate with the sample-rate divider applied.
    // Divider is max 31 (5 bits)!
    if (divider > 31) {
        divider = 31;
    }
    if (writeByte(I2C_SLV4_CTRL, (uint8_t)divider)) {
        usleep(10000);
        readByte(I2C_SLV4_CTRL, &settings.i2c_slv4_ctrl);
        settings.i2c_slv4_ctrl &= _MASK__I2C_SLV4_CTRL;
    }

    return AK8963_getSampleRate();
}

bool MPU9250::AK8963_read(double* mag_data)
{
    bool result = false;
    double mag_hwu_ut = (4912.0 / 32760);
    uint8_t data[8] = {};
    int16_t mag_sens_adj[3] = {1, 1, 1};


    // timeval tm_start, tm_curr;
    // double starttime;
    // uint32_t ms_dt = 0;

    // uint8_t length = 1;
    // uint8_t slv_ctrl_mask = (length & 0x0F) | I2C_READ_FLAG;

    // writeByte(I2C_SLV0_ADDR, AK8963_I2C_ADDR|I2C_READ_FLAG);
    // writeByte(I2C_SLV0_REG, AK8963_REG_ADDR::ST1);
    // writeByte(I2C_SLV0_CTRL, (1 | I2C_READ_FLAG));
    // usleep(200000);

    // // Enable 1 byte reads:
    // slv_ctrl_mask = (1 & 0x0F) | I2C_READ_FLAG;
    // writeByte(I2C_SLV0_CTRL, slv_ctrl_mask);

    // gettimeofday(&tm_start, NULL);
    // starttime = (tm_start.tv_sec + (tm_start.tv_usec / 1000000.0));
    // do {
    //     usleep(10000);  // TODO: remove this or change to something appropriate!
        readBytes(EXT_SENS_DATA_00, 8, data);

        // Check if ST1 != 0
        if (data[0] != 0) {
            for (size_t i=0; i<3; i++) {
                mag_data[i] = (int16_t)((int16_t)data[i*2+2] << 8 | data[i*2+1]) * mag_sens_adj[i] / mag_hwu_ut;
            }
            printf("mag data read:\t\tMX: %f, MY: %f, MZ: %f\n", mag_data[0], mag_data[1], mag_data[2]);
            printf("raw data read: ST1: %#x, MX: %#x %#x, MY: %#x %#x, MZ: %#x %#x, ST2: %#x\n", data[0], data[2], data[1], data[4], data[3], data[6], data[5], data[7]);
            result = true;
            //break;
        }

        // gettimeofday(&tm_curr, NULL);
        // ms_dt = floor(((tm_curr.tv_sec + (tm_curr.tv_usec / 1000000.0)) - starttime) * 1000.0);
    // } while (ms_dt < 5000);


    // Note: data[0] == 1 if data is ready (ST1)
    // if (readI2CByte(AK8963_I2C_ADDR, AK8963_REG_ADDR::ST1, data)) {
    //     printf("Status ready: %u\n", data[0]);
    //     if (data[0] != 0 && readI2CBytes(AK8963_I2C_ADDR, AK8963_REG_ADDR::ST1, 8, data)) {
    //         for (size_t i=0; i<3; i++) {
    //             mag_data[i] = (int16_t)((int16_t)data[i*2+2] << 8 | data[i*2+1]) * mag_sens_adj[i] / mag_hwu_ut;
    //         }
    //         printf("data read: %#x %#x %#x %#x %#x %#x %#x %#x\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
    //         result = true;
    //     }

    // }

    return result;
}

uint8_t MPU9250::AK8963_selfTest()
{
    // Set power-down mode
    // Enable self-test in ASTC register
    // Set self-test mode
    // Check data-ready (ST1 reg)
    // Read data
    // Disable self-test in ASTC register
    // Set power-down mode

    uint8_t result = 0;
    uint8_t data[8] = {};
    double mag_data[3] = {};

    // WARNING: AK8963 needs to be "calibrated" before self-test!
    if (!AK8963_calibrated) {
        AK8963_calibrate();
    }

    //uint8_t length = 1;
    //uint8_t slv_ctrl_mask = (length & 0x0F) | I2C_READ_FLAG;

    //printf("Going to power-down mode.\n");
    //writeByte(I2C_SLV1_ADDR, AK8963_I2C_ADDR);                  // set slave address and prepare for WRITE
    writeByte(I2C_SLV1_REG, AK8963_REG_ADDR::CNTL1);            // select register address to write to
    writeByte(I2C_SLV1_DO, AK8963_SETTINGS::MODE_POWERDOWN);    // write this to slv1
    usleep(100000);
    //printf("Enabling self-test.\n");
    writeByte(I2C_SLV1_REG, AK8963_REG_ADDR::ASTC);             // select register address to write to
    writeByte(I2C_SLV1_DO, AK8963_SETTINGS::ST_EN);             // write this to slv1
    usleep(100000);
    //printf("Going to self-test mode.\n");
    writeByte(I2C_SLV1_REG, AK8963_REG_ADDR::CNTL1);            // select register address to write to
    writeByte(I2C_SLV1_DO, (AK8963_SETTINGS::MODE_SELFTEST | AK8963_SETTINGS::RESOLUTION_16BIT));     // write this to slv1
    usleep(100000);
    //printf("Enabling reading of ST1 register.\n");
    //writeByte(I2C_SLV0_ADDR, (AK8963_I2C_ADDR | I2C_READ_FLAG));    // set slave address and prepare for READ
    writeByte(I2C_SLV0_REG, AK8963_REG_ADDR::ST1);              // select register address to read from
    writeByte(I2C_SLV0_CTRL, (1 | I2C_READ_FLAG));                       // enable 1-byte reads
    usleep(100000);

    timeval tm_start, tm_curr;
    double starttime;
    uint16_t ms_dt = 0;

    gettimeofday(&tm_start, NULL);
    starttime = (tm_start.tv_sec + (tm_start.tv_usec / 1000000.0));
    //printf("Reading EXT_SENS_DATA_00.\n");
    do {
        usleep(1000);
        readByte(EXT_SENS_DATA_00, data);
        if (data[0] != 0) {
            //printf("ST1: %#x\n", data[0]);
            //printf("Enabling reading of ST1 - ST2 registers.\n");
            writeByte(I2C_SLV0_CTRL, (0x08 | I2C_READ_FLAG));                       // enable 8-byte reads
            usleep(200000);
            //printf("Reading ST1 - ST2 registers.\n");
            readBytes(EXT_SENS_DATA_00, 8, data);
            for (size_t i=0; i<3; i++) {
                mag_data[i] = (int16_t)((int16_t)data[i*2+2] << 8 | data[i*2+1]) * mag_sens_adj[i];
            }

            printf("raw data: %#x (%#x %#x) (%#x %#x) (%#x %#x) %#x\n", data[0], data[2], data[1], data[4], data[3], data[6], data[5], data[7]);
            printf("mag_data: %f %f %f\n", mag_data[0], mag_data[1], mag_data[2]);
            break;
        }

        gettimeofday(&tm_curr, NULL);
        ms_dt = floor(((tm_curr.tv_sec + (tm_curr.tv_usec / 1000000.0)) - starttime) * 1000.0);
    } while (ms_dt < AK8963_TEST::DATA_READY_TIMEOUT_MS);

    if (mag_data[0] < AK8963_TEST::CRITERIA_HX_MIN || mag_data[0] > AK8963_TEST::CRITERIA_HX_MAX) {
        // FAILED!
        result |= 1;
    }
    if (mag_data[1] < AK8963_TEST::CRITERIA_HY_MIN || mag_data[1] > AK8963_TEST::CRITERIA_HY_MAX) {
        // FAILED!
        result |= 1 << 1;
    }
    if (mag_data[2] < AK8963_TEST::CRITERIA_HZ_MIN || mag_data[2] > AK8963_TEST::CRITERIA_HZ_MAX) {
        // FAILED!
        result |= 1 << 2;
    }

    //printf("Disabling self-test.\n");
    //writeByte(I2C_SLV0_ADDR, AK8963_I2C_ADDR);                  // set slave address and prepare for WRITE
    writeByte(I2C_SLV1_REG, AK8963_REG_ADDR::ASTC);             // select register address to write to
    writeByte(I2C_SLV1_DO, 0x00);                               // write this to slv1
    usleep(100000);
    //printf("Going to power-down mode.\n");
    writeByte(I2C_SLV1_REG, AK8963_REG_ADDR::CNTL1);            // select register address to write to
    writeByte(I2C_SLV1_DO, AK8963_SETTINGS::MODE_POWERDOWN);    // write this to slv1
    //usleep(100000);


//     // Make sure we are in power-down mode
//     AK8963_set_mode(AK8963_SETTINGS::MODE_POWERDOWN);
//     // Enable self-test
//     writeI2CByte(AK8963_I2C_ADDR, AK8963_REG_ADDR::ASTC, AK8963_SETTINGS::ST_EN);
//     // Set self-test mode and 16-bit resolution
//     writeI2CByte(AK8963_I2C_ADDR, AK8963_REG_ADDR::CNTL1, AK8963_SETTINGS::MODE_SELFTEST | AK8963_SETTINGS::RESOLUTION_16BIT);

//     // Poll data-ready (ST1) and read values if available within timeout.
//     double mag_data[3] = {};

//     // timeval tm_start, tm_curr;
//     // double starttime;
//     // uint16_t ms_dt = 0;
//     // gettimeofday(&tm_start, NULL);
//     // starttime = (tm_start.tv_sec + (tm_start.tv_usec / 1000000.0));

//     AK8963_read(mag_data);
//     // do {
//     //     if (AK8963_read(mag_data)) {
//     //         break;
//     //     }

//     //     gettimeofday(&tm_curr, NULL);
//     //     ms_dt = floor(((tm_curr.tv_sec + (tm_curr.tv_usec / 1000000.0)) - starttime) * 1000.0);
//     // } while (ms_dt < AK8963_TEST::DATA_READY_TIMEOUT_MS);

//     if (mag_data[0] < AK8963_TEST::CRITERIA_HX_MIN || mag_data[0] > AK8963_TEST::CRITERIA_HX_MAX) {
//         // FAILED!
//         result |= 1;
//     }
//     if (mag_data[1] < AK8963_TEST::CRITERIA_HY_MIN || mag_data[1] > AK8963_TEST::CRITERIA_HY_MAX) {
//         // FAILED!
//         result |= 1 << 1;
//     }
//     if (mag_data[2] < AK8963_TEST::CRITERIA_HZ_MIN || mag_data[2] > AK8963_TEST::CRITERIA_HZ_MAX) {
//         // FAILED!
//         result |= 1 << 2;
//     }

//     // Disable self-test
//     writeI2CByte(AK8963_I2C_ADDR, AK8963_REG_ADDR::ASTC, 0x00);
//     // Return to power-down mode
//     AK8963_set_mode(AK8963_SETTINGS::MODE_POWERDOWN);

// #ifdef NAVIO_DEBUG
//     printf("Magnetometer data read in self-test mode:\n");
//     printf("\tX: %f, Y: %f, Z: %f\n", mag_data[0], mag_data[1], mag_data[2]);
// #endif // NAVIO_DEBUG

    return result;
}

void MPU9250::AK8963_calibrate()
{
    // Set power-down mode
    AK8963_setMode(AK8963_SETTINGS::MODE_POWERDOWN);
    // Set Fuse-ROM access mode
    AK8963_setMode(AK8963_SETTINGS::MODE_FUSEROM);
    // Read Fuse-ROM and apply factor
    uint8_t asa_data[3] = {};
    writeByte(I2C_SLV0_ADDR, AK8963_REG_ADDR::ASAX);
    writeByte(I2C_SLV0_CTRL, (3 | I2C_READ_FLAG));
    usleep(100000);
    if (readBytes(EXT_SENS_DATA_00, 3, asa_data)) {
        for (size_t i=0; i<3; i++) {
            mag_sens_adj[i] = (asa_data[i] - 128.0) / 256.0 + 1.0;
        }

        AK8963_calibrated = true;
    }
    // Return to power-down mode
    AK8963_setMode(AK8963_SETTINGS::MODE_POWERDOWN);

#ifdef NAVIO_DEBUG
    printf("Magenetometer calibration data: X: %f, Y: %f, Z: %f\n", mag_sens_adj[0], mag_sens_adj[1], mag_sens_adj[2]);
#endif // NAVIO_DEBUG
}
