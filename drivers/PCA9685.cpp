/*
PCA9685 driver code is placed under the BSD license.
Written by Mikhail Avkhimenia (mikhail.avkhimenia@emlid.com)
Copyright (c) 2014, Emlid Limited, www.emlid.com
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

#include "PCA9685.h"

/** PCA9685 constructor.
 * @param address I2C address
 * @see PCA9685_DEFAULT_ADDRESS
 */
PCA9685::PCA9685(const char *i2cDev, uint8_t address) {
    this->i2cDev = std::string(i2cDev);
    this->devAddr = address;
}

/** Power on and prepare for general usage.
 * This method reads prescale value stored in PCA9685 and calculate frequency based on it.
 * Then it enables auto-increment of register address to allow for faster writes.
 * And finally the restart is performed to enable clocking.
 */
void PCA9685::initialize() {
    this->frequency = getFrequency();
    I2Cdev::writeBit(i2cDev.c_str(), devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_AI_BIT, 1);
    restart();
}

/** Verify the I2C connection.
 * @return True if connection is valid, false otherwise
 */
bool PCA9685::testConnection() {
    uint8_t data;
    int8_t status = I2Cdev::readByte(i2cDev.c_str(), devAddr, PCA9685_RA_PRE_SCALE, &data);
    if (status > 0)
        return true;
    else
        return false;
}

/** Put PCA9685 to sleep mode thus turning off the outputs.
 * @see PCA9685_MODE1_SLEEP_BIT
 */
void PCA9685::sleep() {
    I2Cdev::writeBit(i2cDev.c_str(), devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
}

/** Disable sleep mode and start the outputs.
 * @see PCA9685_MODE1_SLEEP_BIT
 * @see PCA9685_MODE1_RESTART_BIT
 */
void PCA9685::restart() {
    I2Cdev::writeBit(i2cDev.c_str(), devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
    usleep(500);    // Allow oscillator to stabilize
    I2Cdev::writeBit(i2cDev.c_str(), devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

/** Calculate prescale value based on the specified frequency and write it to the device.
 * @return Frequency in Hz
 * @see PCA9685_RA_PRE_SCALE
 */
float PCA9685::getFrequency() {
    uint8_t data;
    I2Cdev::readByte(i2cDev.c_str(), devAddr, PCA9685_RA_PRE_SCALE, &data);
    return 25000000.f / 4096.f / (data + 1);
}


/** Calculate prescale value based on the specified frequency and write it to the device.
 * @param Frequency in Hz
 * @see PCA9685_RA_PRE_SCALE
 */
void PCA9685::setFrequency(float frequency) {
    uint8_t prescale = roundf(25000000.f / 4096.f / frequency)  - 1;
    I2Cdev::writeBit(i2cDev.c_str(), devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
    I2Cdev::writeByte(i2cDev.c_str(), devAddr, PCA9685_RA_PRE_SCALE, prescale);
    usleep(10000);
    this->frequency = getFrequency();
    restart();
}

/** Set channel start offset of the pulse and it's length
 * @param Channel number (0-15)
 * @param Offset (0-4095)
 * @param Length (0-4095)
 * @see PCA9685_RA_LED0_ON_L
 */
void PCA9685::setPWM(uint8_t channel, uint16_t offset, uint16_t length) {
    uint16_t led_on = offset;
    uint16_t led_off = offset + length;

    if (length == 0) {
        led_off = 4096; // sets always OFF bit (0x1000)
    }
    else if (led_off >= 4096) {
        led_off = led_off - 4096;
    }

    if (led_off == 0) {
        led_on = 4096;  // sets always ON bit (0x1000)
    }

    uint8_t data[4] = {(uint8_t)(led_on & 0x00FF), (uint8_t)(led_on >> 8), (uint8_t)(led_off & 0x00FF), (uint8_t)(led_off >> 8)};
#ifdef NAVIO_DEBUG
    printf("LED_ON_L %x, LED_ON_H %x, LED_OFF_L %x, LED_OFF_H %x\n", data[0], data[1], data[2], data[3]);
#endif
    I2Cdev::writeBytes(i2cDev.c_str(), devAddr, PCA9685_RA_LED0_ON_L + 4 * channel, 4, data);
}

/** Set channel's pulse length
 * @param Channel number (0-15)
 * @param Length (0-4095)
 * @see PCA9685_RA_LED0_ON_L
 */
void PCA9685::setPWM(uint8_t channel, uint16_t length) {
    setPWM(channel, 0, length);
}

/** Set channel's pulse length in milliseconds
 * @param Channel number (0-15)
 * @param Length in milliseconds
 * @see PCA9685_RA_LED0_ON_L
 */
void PCA9685::setPWMmS(uint8_t channel, float length_mS) {
    setPWM(channel, round((length_mS * 4096.f) / (1000.f / frequency) - 1));
}

/** Set channel's pulse length in microseconds
 * @param Channel number (0-15)
 * @param Length in microseconds
 * @see PCA9685_RA_LED0_ON_L
 */
void PCA9685::setPWMuS(uint8_t channel, float length_uS) {
    setPWM(channel, round((length_uS * 4096.f) / (1000000.f / frequency) - 1));
}

/** Set start offset of the pulse and it's length for all channels
 * @param Offset (0-4095)
 * @param Length (0-4095)
 * @see PCA9685_RA_ALL_LED_ON_L
 */
void PCA9685::setAllPWM(uint16_t offset, uint16_t length) {
    uint8_t data[4] = {(uint8_t)(offset & 0x00FF), (uint8_t)(offset >> 8), (uint8_t)(length & 0x00FF), (uint8_t)(length >> 8)};
    I2Cdev::writeBytes(i2cDev.c_str(), devAddr, PCA9685_RA_ALL_LED_ON_L, 4, data);
}

/** Set pulse length for all channels
 * @param Length (0-4095)
 * @see PCA9685_RA_ALL_LED_ON_L
 */
void PCA9685::setAllPWM(uint16_t length) {
    setAllPWM(0, length);
}

/** Set pulse length in milliseconds for all channels
 * @param Length in milliseconds
 * @see PCA9685_RA_ALL_LED_ON_L
 */
void PCA9685::setAllPWMmS(float length_mS) {
    setAllPWM(round((length_mS * 4096.f) / (1000.f / frequency) - 1));
}

/** Set pulse length in microseconds for all channels
 * @param Length in microseconds
 * @see PCA9685_RA_ALL_LED_ON_L
 */
void PCA9685::setAllPWMuS(float length_uS) {
    setAllPWM(round((length_uS * 4096.f) / (1000000.f / frequency) - 1));
}
