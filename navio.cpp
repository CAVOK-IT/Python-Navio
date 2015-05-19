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

#include "navio.h"

// Exceptions
static PyObject* InitializationException;
static PyObject* ConnectionException;

// Callback function pointers
static PyObject* PPM_callback = 0;
static PyObject* GPS_callback = 0;

// Typedef
typedef struct {
    PyObject_HEAD

    int rpi_model;
    int navio_model;
    int enabled_components;
    int rc_input_signal;

    bool is_initialized;

    Ublox* gps;     /* GPS  */
    MPU9250* imu;   /* IMU  */
    MS5611* baro;   /* baro */
    PCA9685* pwm;   /* PWM  */
    ADS1115* adc;   /* ADC  */
    MB85RCx* fram;  /* FRAM */
    RCin* ppm;      /* PPM  */
} Navio;

// BEGIN GPS
static std::string
GPS_get_structformat(uint16_t message_type)
{
    std::string structformat;

    try {
        structformat = PYSTRUCTFMT_UBX_MSG_FIXEDFIELDS.at("BYTEORDER_SIZE_ALIGNMENT")[0];
        structformat+= PYSTRUCTFMT_UBX_MSG_FIXEDFIELDS.at("HEADER")[0];
        structformat+= PYSTRUCTFMT_UBX_MSG.at(message_type)[0];
        if (PYSTRUCTFMT_UBX_SUBMSG.count(message_type) > 0) {
            structformat+= "|" + PYSTRUCTFMT_UBX_SUBMSG.at(message_type)[0] + "|";
        }
        structformat+= PYSTRUCTFMT_UBX_MSG_FIXEDFIELDS.at("FOOTER")[0];
    }
    catch (const std::out_of_range& e) {
        PyErr_SetString(PyExc_ValueError, "Invalid message type specified.");
        structformat.clear();
    }

    return structformat;
}

static std::string
GPS_get_structfields(uint16_t message_type)
{
    std::string structfields;

    try {
        structfields = PYSTRUCTFMT_UBX_MSG_FIXEDFIELDS.at("HEADER")[1] + ", ";
        structfields+= PYSTRUCTFMT_UBX_MSG.at(message_type)[1] + ", ";
        if (PYSTRUCTFMT_UBX_SUBMSG.count(message_type) > 0) {
            structfields+= "|" + PYSTRUCTFMT_UBX_SUBMSG.at(message_type)[1] + ", |";
        }
        structfields+= PYSTRUCTFMT_UBX_MSG_FIXEDFIELDS.at("FOOTER")[1];
    }
    catch (const std::out_of_range& e) {
        PyErr_SetString(PyExc_ValueError, "Invalid message type specified.");
        structfields.clear();
    }

    return structfields;
}

static PyObject *
Navio_GPS_get_messageformat(Navio* self, PyObject *args, PyObject *kwds)
{
    uint16_t message_type;
    static char *kwlist[] = {(char *)"message_type", NULL};
    if(!PyArg_ParseTupleAndKeywords(args, kwds, "H", kwlist, &message_type)) {
        return NULL;
    }

    std::string structformat = GPS_get_structformat(message_type);

    if (structformat.empty()) {
        // Exeption should already have been set by GPS_get_structformat.
        return NULL;
    }

    return Py_BuildValue("s", structformat.c_str());
}

static PyObject *
Navio_GPS_get_messagefields(Navio* self, PyObject *args, PyObject *kwds)
{
    uint16_t message_type;
    static char *kwlist[] = {(char *)"message_type", NULL};
    if(!PyArg_ParseTupleAndKeywords(args, kwds, "H", kwlist, &message_type)) {
        return NULL;
    }

    std::string structfields = GPS_get_structfields(message_type);

    if (structfields.empty()) {
        // Exeption should already have been set by GPS_get_structfields.
        return NULL;
    }

    return Py_BuildValue("s", structfields.c_str());
}

static PyObject *
Navio_GPS_get_message(Navio* self, PyObject *args, PyObject *kwds)
{
    if (!(self->is_initialized && self->gps)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or GPS unavailable.");
        return NULL;
    }

    uint16_t message_type;
    uint16_t timeout = 1000;

    static char *kwlist[] = {(char *)"message_type", (char *)"timeout", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "H|i", kwlist, &message_type, &timeout)) {
        return NULL;
    }

    uint8_t message_recv_buff[UBX_MSG_RECV_BUFF_LENGTH];
    int16_t message_recv_length = self->gps->getMessage((ubx_message_t)message_type, message_recv_buff, timeout);

    if (message_recv_length > 0) {
        return Py_BuildValue("y#", (char *)message_recv_buff, message_recv_length);
    }

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
Navio_GPS_set_config(Navio* self, PyObject *args, PyObject *kwds)
{
    if (!(self->is_initialized && self->gps)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or GPS unavailable.");
        return NULL;
    }

    uint16_t message_type;
    uint16_t timeout = 1000;

    Py_INCREF(Py_None);
    return Py_None;
}

// END GPS

// BEGIN IMU
/*
    bool initialize(int sample_rate_div = 1, uint8_t low_pass_filter = 0x01);
    bool testConnection();
    unsigned int set_gyro_scale(int scale);
    unsigned int set_acc_scale(int scale);
    void calib_acc();
    void calib_mag();
    void read_temp();
    void read_acc();
    void read_gyro();
    void read_mag();
    void read_all();
    void getMotion9(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz);
    void getMotion6(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

    float acc_divider;
    float gyro_divider;
    int calib_data[3];
    float magnetometer_ASA[3];
    float temperature;
    float accelerometer_data[3];
    float gyroscope_data[3];
    float magnetometer_data[3];
*/
// static PyObject *
// Navio_IMU_get_motion6(Navio* self)
// {

// }

// static PyObject *
// Navio_IMU_get_motion6(Navio* self)
// {

// }

// static PyObject *
// Navio_IMU_get_motion6(Navio* self)
// {

// }

// static PyObject *
// Navio_IMU_get_motion6(Navio* self)
// {

// }

// static PyObject *
// Navio_IMU_get_motion6(Navio* self)
// {

// }

static PyObject *
Navio_IMU_reset(Navio* self)
{
    self->imu->initialize();

    Py_INCREF(Py_None);
    return Py_None;
}

// TODO: remove this!
static PyObject *
Navio_IMU_test(Navio* self)
{
    //self->imu->read_mag();

    //uint8_t whoami = self->imu->AK8963_whoami();
    bool whoami = self->imu->testConnection();

    if (whoami) {
        self->imu->selfTest();
    }

    return Py_BuildValue("I", whoami);
}

// static PyObject *
// Navio_IMU_get_samplerate(Navio* self)
// {

// }

// static PyObject *
// Navio_IMU_set_samplerate(Navio* self, PyObject *args)
// {

// }

// static PyObject *
// Navio_IMU_get_compass_samplerate(Navio* self)
// {

// }

// static PyObject *
// Navio_IMU_set_compass_samplerate(Navio* self, PyObject *args)
// {

// }


static PyObject *
Navio_IMU_get_motion6(Navio* self)
{
    if (!(self->is_initialized && self->gps)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or IMU unavailable.");
        return NULL;
    }

    float accel[3];
    float gyro[3];

    self->imu->getMotion6(accel, gyro);

    return Py_BuildValue("(fff)(fff)", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
}

static PyObject *
Navio_IMU_get_motion9(Navio* self)
{
    if (!(self->is_initialized && self->gps)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or IMU unavailable.");
        return NULL;
    }

    float accel[3];
    float gyro[3];
    float mag[3];

    self->imu->getMotion9(accel, gyro, mag);

    return Py_BuildValue("(fff)(fff)(fff)", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], mag[0], mag[1], mag[2]);
}
// END IMU

// BEGIN baro
static PyObject *
Navio_BARO_get_temp_and_press(Navio* self)
{
    if (!(self->is_initialized && self->baro)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or BARO unavailable.");
        return NULL;
    }

    self->baro->refreshPressure();
    usleep(10000);
    self->baro->readPressure();

    self->baro->refreshTemperature();
    usleep(10000);
    self->baro->readTemperature();

    self->baro->calculatePressureAndTemperature();

    return Py_BuildValue("{s:f,s:f}", "temp", self->baro->getTemperature(), "press", self->baro->getPressure());
}
// END baro

// BEGIN PWM
static PyObject *
Navio_PWM_get_frequency(Navio* self, PyObject *args)
{
    if (!(self->is_initialized && self->pwm)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or PWM unavailable.");
        return NULL;
    }

    return Py_BuildValue("f", self->pwm->getFrequency());
}

static PyObject *
Navio_PWM_set_frequency(Navio* self, PyObject *args)
{
    if (!(self->is_initialized && self->pwm)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or PWM unavailable.");
        return NULL;
    }

    float frequency;

    if (!PyArg_ParseTuple(args, "f", &frequency)) {
        return NULL;
    }
    if (frequency < 24.0 || frequency > 1526.0) {
        PyErr_SetString(PyExc_ValueError, "Frequency out of range.");
        return NULL;
    }

    self->pwm->setFrequency(frequency);

    return Py_BuildValue("f", self->pwm->getFrequency());
}

static PyObject *
Navio_PWM_set_led(Navio* self, PyObject *args, PyObject *kwds)
{
    if (!(self->is_initialized && self->pwm)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or PWM unavailable.");
        return NULL;
    }

    uint8_t r = 0, g = 0, b = 0;
    uint16_t pwm_val_r, pwm_val_g, pwm_val_b;

    static char *kwlist[] = {(char *)"R", (char *)"G", (char *)"B", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|bbb", kwlist, &r, &g, &b)) {
        return NULL;
    }

    // Convert the 8-bit RGB values to inversed 12-bit
    pwm_val_r = floor(4096.5 - (r / 255.0) * 4096.0);
    pwm_val_g = floor(4096.5 - (g / 255.0) * 4096.0);
    pwm_val_b = floor(4096.5 - (b / 255.0) * 4096.0);

    self->pwm->setPWM(0, pwm_val_b);
    self->pwm->setPWM(1, pwm_val_g);
    self->pwm->setPWM(2, pwm_val_r);

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
Navio_PWM_set_pulse(Navio* self, PyObject *args, PyObject *kwds)
{
    if (!(self->is_initialized && self->pwm)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or PWM unavailable.");
        return NULL;
    }

    uint8_t chan;
    uint16_t val, offset = 0;

    static char *kwlist[] = {(char *)"channel", (char *)"length", (char *)"offset", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "bH|H", kwlist, &chan, &val, &offset)) {
        return NULL;
    }

    // Available channel numbers is 3 to 15 (channels 0-2 are used for the on-board LED)
    chan = chan + 2;
    if (chan < 3 || chan > 15) {
        PyErr_SetString(PyExc_ValueError, "PWM channel out of range");
        return NULL;
    }
    // Available values are 0 - 4095
    if (val < 0 || val > 4095) {
        PyErr_SetString(PyExc_ValueError, "PWM value out of range");
        return NULL;
    }

    self->pwm->setPWM(chan, offset, val);

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
Navio_PWM_set_pulse_ms(Navio* self, PyObject *args, PyObject *kwds)
{
    if (!(self->is_initialized && self->pwm)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or PWM unavailable.");
        return NULL;
    }

    uint8_t chan;
    float length_ms;

    static char *kwlist[] = {(char *)"channel", (char *)"ms", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "bf", kwlist, &chan, &length_ms)) {
        return NULL;
    }

    // Available channel numbers is 3 to 15 (channels 0-2 are used for the on-board LED)
    chan = chan + 2;
    if (chan < 3 || chan > 15) {
        PyErr_SetString(PyExc_ValueError, "PWM channel out of range");
        return NULL;
    }

    self->pwm->setPWMmS(chan, length_ms);

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
Navio_PWM_set_pulse_us(Navio* self, PyObject *args, PyObject *kwds)
{
    if (!(self->is_initialized && self->pwm)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or PWM unavailable.");
        return NULL;
    }

    uint8_t chan;
    float length_us;

    static char *kwlist[] = {(char *)"channel", (char *)"us", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "bf", kwlist, &chan, &length_us)) {
        return NULL;
    }

    // Available channel numbers is 3 to 15 (channels 0-2 are used for the on-board LED)
    chan = chan + 2;
    if (chan < 3 || chan > 15) {
        PyErr_SetString(PyExc_ValueError, "PWM channel out of range");
        return NULL;
    }

    self->pwm->setPWMuS(chan, length_us);

    Py_INCREF(Py_None);
    return Py_None;
}
// END PWM

// BEGIN ADC
/*
    bool testConnection();
    uint16_t getMultiplexer();
    void setMultiplexer(uint16_t mux);
    uint16_t getGain();
    void setGain(uint16_t gain);
    uint16_t getMode();
    void setMode(uint16_t mode);
    uint16_t getRate();
    void setRate(uint16_t rate);
    float getMilliVolts();
    void setComparatorMode(uint16_t comparatorMode);
    void setComparatorPolarity(uint16_t polarit);
    void setComparatorLatchEnabled(uint16_t latchStatus);
    void setComparatorQueueMode(uint16_t queueMode);
*/
static PyObject *
Navio_ADC_get_mux_mode(Navio* self)
{
    if (!(self->is_initialized && self->adc)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or ADC unavailable.");
        return NULL;
    }

    return Py_BuildValue("B", (uint8_t)(self->adc->getMultiplexer() >> ADS1115_MUX_SHIFT));
}

static PyObject *
Navio_ADC_set_mux_mode(Navio* self, PyObject *args)
{
    if (!(self->is_initialized && self->adc)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or ADC unavailable.");
        return NULL;
    }

    uint16_t mode;

    if (!PyArg_ParseTuple(args, "B", &mode)) {
        return NULL;
    }

    switch(mode) {
        case ADC_MUX_AIN0_AIN1:
        case ADC_MUX_AIN0_AIN3:
        case ADC_MUX_AIN1_AIN3:
        case ADC_MUX_AIN2_AIN3:
        case ADC_MUX_AIN0_GND:
        case ADC_MUX_AIN1_GND:
        case ADC_MUX_AIN2_GND:
        case ADC_MUX_AIN3_GND:
            self->adc->setMultiplexer((mode << ADS1115_MUX_SHIFT));
            break;

        default:
            PyErr_SetString(PyExc_ValueError, "Invalid ADC MUX mode specified.");
            return NULL;
    }

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
Navio_ADC_get_gain(Navio* self)
{
    if (!(self->is_initialized && self->adc)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or ADC unavailable.");
        return NULL;
    }

    return Py_BuildValue("B", (uint8_t)(self->adc->getGain() >> ADS1115_PGA_SHIFT));
}

static PyObject *
Navio_ADC_set_gain(Navio* self, PyObject *args)
{
    if (!(self->is_initialized && self->adc)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or ADC unavailable.");
        return NULL;
    }

    uint16_t gain;

    if (!PyArg_ParseTuple(args, "B", &gain)) {
        return NULL;
    }

    switch (gain) {
        case ADC_GAIN_0:
        case ADC_GAIN_1:
        case ADC_GAIN_2:
        case ADC_GAIN_4:
        case ADC_GAIN_8:
        case ADC_GAIN_16:
            self->adc->setGain((gain << ADS1115_PGA_SHIFT));
            break;

        default:
            PyErr_SetString(PyExc_ValueError, "Invalid ADC gain specified.");
            return NULL;
    }

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
Navio_ADC_get_mode(Navio* self)
{
    if (!(self->is_initialized && self->adc)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or ADC unavailable.");
        return NULL;
    }

    return Py_BuildValue("B", (uint8_t)(self->adc->getMode() >> ADS1115_MODE_SHIFT));
}

static PyObject *
Navio_ADC_set_mode(Navio* self, PyObject *args)
{
    if (!(self->is_initialized && self->adc)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or ADC unavailable.");
        return NULL;
    }

    uint16_t mode;

    if (!PyArg_ParseTuple(args, "B", &mode)) {
        return NULL;
    }

    switch (mode) {
        case ADC_MODE_CONT:
        case ADC_MODE_SINGLE:
            self->adc->setMode((mode << ADS1115_MODE_SHIFT));
            break;

        default:
            PyErr_SetString(PyExc_ValueError, "Invalid ADC mode specified.");
            return NULL;
    }

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
Navio_ADC_get_rate(Navio* self)
{
    if (!(self->is_initialized && self->adc)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or ADC unavailable.");
        return NULL;
    }

    return Py_BuildValue("B", (uint8_t)(self->adc->getRate() >> ADS1115_RATE_SHIFT));
}

static PyObject *
Navio_ADC_set_rate(Navio* self, PyObject *args)
{
    if (!(self->is_initialized && self->adc)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or ADC unavailable.");
        return NULL;
    }

    uint16_t rate;

    if (!PyArg_ParseTuple(args, "B", &rate)) {
        return NULL;
    }

    switch (rate) {
        case ADC_RATE_8:
        case ADC_RATE_16:
        case ADC_RATE_32:
        case ADC_RATE_64:
        case ADC_RATE_128:
        case ADC_RATE_250:
        case ADC_RATE_475:
        case ADC_RATE_860:
            self->adc->setRate((rate << ADS1115_RATE_SHIFT));
            break;

        default:
            PyErr_SetString(PyExc_ValueError, "Invalid ADC rate specified.");
            return NULL;
    }

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
Navio_ADC_read(Navio* self, PyObject *args)
{
    if (!(self->is_initialized && self->adc)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or ADC unavailable.");
        return NULL;
    }

    return Py_BuildValue("f", self->adc->getMilliVolts());
}

static PyObject *
Navio_ADC_read_all(Navio* self)
{
    if (!(self->is_initialized && self->adc)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or ADC unavailable.");
        return NULL;
    }

    uint16_t mux_mode = ADC_MUX_AIN0_GND;
    float readings[4] = {};
    uint16_t sps = SPS_ADS1115.at((uint8_t)(self->adc->getRate() >> ADS1115_RATE_SHIFT));
    uint16_t delay;

    for (size_t i=0; i<4; i++) {
        self->adc->setMultiplexer((mux_mode << ADS1115_MUX_SHIFT));

        // Wait for conversion to complete
        delay = ceil((1.0 / sps) * 1000000.0);
        usleep(delay);

        readings[i] = self->adc->getMilliVolts();

        mux_mode++;
    }

    return Py_BuildValue("(ffff)", readings[0], readings[1], readings[2], readings[3]);
}
// END ADC

// BEGIN FRAM
static PyObject *
Navio_FRAM_read(Navio* self, PyObject *args, PyObject *kwds)
{
    if (!(self->is_initialized && self->fram)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or FRAM unavailable.");
        return NULL;
    }

    uint16_t reg_address;
    uint8_t length;
    char *data_buffer;
    PyObject *result;

    static char *kwlist[] = {(char *)"address", (char *)"length", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "Hb", kwlist, &reg_address, &length)) {
        return NULL;
    }

    if (length > 127) {
        PyErr_SetString(PyExc_ValueError, "You can read no more than 127 bytes at a time.");
        return NULL;
    }
    if ((reg_address + length) > self->fram->getSize()) {
        PyErr_SetString(PyExc_ValueError, "Trying to read beyond FRAM boundary.");
        return NULL;
    }

    try {
        data_buffer = new char[length];
    }
    catch (std::bad_alloc& ba) {
        return PyErr_NoMemory();
    }

    if (self->fram->readBytes(reg_address, length, (uint8_t*)data_buffer)) {
        result = Py_BuildValue("y#", data_buffer, length);
    }
    else {
        PyErr_SetString(PyExc_IOError, "Error reading from FRAM.");
        result = NULL;
    }
    delete[] data_buffer;
    return result;
}

static PyObject *
Navio_FRAM_write(Navio* self, PyObject *args, PyObject *kwds)
{
    if (!(self->is_initialized && self->fram)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or FRAM unavailable.");
        return NULL;
    }

    uint16_t reg_address;
    char *data;
    Py_ssize_t length;

    static char *kwlist[] = {(char *)"address", (char *)"data", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "Hy#", kwlist, &reg_address, &data, &length)) {
        return NULL;
    }

    if (length > 127) {
        PyErr_SetString(PyExc_ValueError, "You can write no more than 127 bytes at a time");
        return NULL;
    }
    if ((reg_address + length) > self->fram->getSize()) {
        PyErr_SetString(PyExc_ValueError, "Data extends beyond FRAM boundary");
        return NULL;
    }

    if (!self->fram->writeBytes(reg_address, length, (uint8_t *)data)) {
        PyErr_SetString(PyExc_IOError, "Error writing to FRAM. Possible data corruption occurred!");
        return NULL;
    }

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
Navio_FRAM_clear(Navio* self)
{
    if (!(self->is_initialized && self->fram)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or FRAM unavailable.");
        return NULL;
    }

    uint8_t data[64] = {0};
    for (size_t i=1; i<self->fram->getSize(); i+=64) {
        if (!self->fram->writeBytes(i-1, 64, data)) {
            PyErr_SetString(PyExc_IOError, "Error clearing FRAM. Possible data corruption occurred!");
            return NULL;
        }
    }

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
Navio_FRAM_test(Navio* self)
{
    if (!(self->is_initialized && self->fram)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or FRAM unavailable.");
        return NULL;
    }

    bool result = false;

    const int length = 9;   // no more than 127!
    uint16_t reg_address = rand() % (self->fram->getSize() - length); // try this at a random memory location
    uint8_t test_data[length] = {0x4E, 0x6F, 0x75, 0x64, 0x26, 0x4C, 0x61, 0x72, 0x73};
    uint8_t read_data[length] = {0};
    uint8_t tmp[length] = {0};

    if (self->fram->readBytes(reg_address, length, tmp)) {      // temporarily store the original data
        if (self->fram->writeBytes(reg_address, length, test_data)) {
            if (self->fram->readBytes(reg_address, length, read_data)) {
                for (int i=0; i<length; i++) {
                    if (!(read_data[i] == test_data[i])) {
                        result = false;
                        break;
                    }
                    result = true;
                }
                if (!self->fram->writeBytes(reg_address, length, tmp)) {    // restore the original data
                    result = false;
                }
            }
        }
    }

    return Py_BuildValue("i", result);
}
// END FRAM

// BEGIN PPM
static PyObject *
Navio_PPM_enable(Navio* self, PyObject *args, PyObject *kwds)
{
    if (!(self->is_initialized && self->ppm)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or PPM unavailable.");
        return NULL;
    }

    uint8_t number_of_channels = 0;

    static char *kwlist[] = {(char *)"number_of_channels", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|B", kwlist, &number_of_channels)) {
        return NULL;
    }

    self->ppm->enable(number_of_channels);

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
Navio_PPM_disable(Navio* self)
{
    if (!(self->is_initialized && self->ppm)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or PPM unavailable.");
        return NULL;
    }

    self->ppm->disable();

    Py_INCREF(Py_None);
    return Py_None;
}

static PyObject *
Navio_PPM_read(Navio* self)
{
    if (!(self->is_initialized && self->ppm)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or PPM unavailable.");
        return NULL;
    }

    PyObject* channel_data = PyList_New(RCin::OUTDATA_BUFFER_SIZE);
    if (channel_data) {
        uint16_t channel_buffer[RCin::OUTDATA_BUFFER_SIZE] = {0};

        self->ppm->readChannels(channel_buffer);

        for (ssize_t i=0; i<RCin::OUTDATA_BUFFER_SIZE; i++) {
            PyList_SET_ITEM(channel_data, i, PyLong_FromUnsignedLong(channel_buffer[i]));
        }
    }

    return channel_data;
}

// PPM_callback wrapper
static void
PPM_callback_wrapper(uint16_t* channel_buffer)
{
    if (!PPM_callback) {
        return;
    }

    // Acquire the GIL
    PyGILState_STATE gstate = PyGILState_Ensure();
        PyObject* channel_data = PyList_New(RCin::OUTDATA_BUFFER_SIZE);
        if (channel_data) {
            for (ssize_t i=0; i<RCin::OUTDATA_BUFFER_SIZE; i++) {
                PyList_SET_ITEM(channel_data, i, PyLong_FromUnsignedLong(channel_buffer[i]));
            }
            PyObject* args = Py_BuildValue("(N)", channel_data);    // PyObject_CallObject actually needs a tuple as *args
            if (args) {
                PyObject* result = PyObject_CallObject(PPM_callback, args);
                Py_DECREF(args);
                Py_XDECREF(result);
            }

            Py_DECREF(channel_data);
        }
    PyGILState_Release(gstate);
    // GIL released
}

static PyObject *
Navio_PPM_register_callback(Navio* self, PyObject* args)
{
    if (!(self->is_initialized && self->ppm)) {
        PyErr_SetString(PyExc_IOError, "Navio not initialized and/or PPM unavailable.");
        return NULL;
    }

    PyObject* temp = 0;

    if (!PyArg_ParseTuple(args, "|O:PPM_register_callback", &temp)) {
        return NULL;
    }

    // Disable PPM reading while we set this up.
    self->ppm->disable();

    if (!temp || temp == Py_None) {
        self->ppm->registerCallback(NULL);
        Py_XDECREF(PPM_callback);   /* Dispose of previous callback */
        PPM_callback = 0;
    }
    else {
        if (!PyCallable_Check(temp)) {
            PyErr_SetString(PyExc_TypeError, "Parameter must be callable.");
            return NULL;
        }
        Py_XINCREF(temp);           /* Add a reference to new callback */
        Py_XDECREF(PPM_callback);   /* Dispose of previous callback */
        PPM_callback = temp;        /* Remember new callback */

        self->ppm->registerCallback(PPM_callback_wrapper);
    }

    Py_INCREF(Py_None);
    return Py_None;
}
// END PPM

static PyMethodDef Navio_methods[] = {
    {"GPS_get_messageformat", (PyCFunction)Navio_GPS_get_messageformat, METH_VARARGS|METH_KEYWORDS,
     "Returns the data format for the specified UBX message."
    },
    {"GPS_get_messagefields", (PyCFunction)Navio_GPS_get_messagefields, METH_VARARGS|METH_KEYWORDS,
     "Returns the fields for the specified UBX message."
    },
    {"GPS_get_message", (PyCFunction)Navio_GPS_get_message, METH_VARARGS|METH_KEYWORDS,
     "Returns requested GPS (UBX protocol) message."
    },


    {"IMU_reset", (PyCFunction)Navio_IMU_reset, METH_NOARGS,
     "Reset IMU."
    },
    {"IMU_test", (PyCFunction)Navio_IMU_test, METH_NOARGS,
     "Test IMU."
    },
    // {"IMU_get_motion6", (PyCFunction)Navio_IMU_get_motion6, METH_NOARGS,
    //  "Returns IMU gyro and accelerometer data."
    // },
    // {"IMU_get_motion9", (PyCFunction)Navio_IMU_get_motion9, METH_NOARGS,
    //  "Returns IMU gyro, accelerometer and magnetometer data."
    // },

    {"BARO_get_temp_and_press", (PyCFunction)Navio_BARO_get_temp_and_press, METH_NOARGS,
     "Returns calculated temperature (in degrees Celsius) and pressure (in mbar)."
    },

    {"PWM_get_frequency", (PyCFunction)Navio_PWM_get_frequency, METH_NOARGS,
     "Get the current PWM operating frequency."
    },
    {"PWM_set_frequency", (PyCFunction)Navio_PWM_set_frequency, METH_VARARGS|METH_KEYWORDS,
     "Set the PWM operating frequency (24.0-1526.0 Hz)."
    },
    {"PWM_set_led", (PyCFunction)Navio_PWM_set_led, METH_VARARGS|METH_KEYWORDS,
     "Set the RGB values (0-255) of the on-board LED."
    },
    {"PWM_set_pulse", (PyCFunction)Navio_PWM_set_pulse, METH_VARARGS|METH_KEYWORDS,
     "Set the PWM pulse length (0-4095) of the specified channel."
    },
    {"PWM_set_pulse_ms", (PyCFunction)Navio_PWM_set_pulse_ms, METH_VARARGS|METH_KEYWORDS,
     "Set the PWM pulse length in ms of the specified channel."
    },
    {"PWM_set_pulse_us", (PyCFunction)Navio_PWM_set_pulse_us, METH_VARARGS|METH_KEYWORDS,
     "Set the PWM pulse length in us of the specified channel."
    },

    {"ADC_get_mux_mode", (PyCFunction)Navio_ADC_get_mux_mode, METH_NOARGS,
     "Get the current ADC input multiplexer mode."
    },
    {"ADC_set_mux_mode", (PyCFunction)Navio_ADC_set_mux_mode, METH_VARARGS,
     "Set the ADC input multiplexer mode."
    },
    {"ADC_get_gain", (PyCFunction)Navio_ADC_get_gain, METH_NOARGS,
     "Get the current ADC gain."
    },
    {"ADC_set_gain", (PyCFunction)Navio_ADC_set_gain, METH_VARARGS,
     "Set the ADC gain."
    },
    {"ADC_get_mode", (PyCFunction)Navio_ADC_get_mode, METH_NOARGS,
     "Get the current ADC operating mode."
    },
    {"ADC_set_mode", (PyCFunction)Navio_ADC_set_mode, METH_VARARGS,
     "Set the ADC operating mode."
    },
    {"ADC_get_rate", (PyCFunction)Navio_ADC_get_rate, METH_NOARGS,
     "Get the current ADC measurement rate."
    },
    {"ADC_set_rate", (PyCFunction)Navio_ADC_set_rate, METH_VARARGS,
     "Set the ADC measurement rate."
    },
    {"ADC_read", (PyCFunction)Navio_ADC_read, METH_NOARGS,
     "Get a reading (in mV) from the currently selected ADC input(s)."
    },
    {"ADC_read_all", (PyCFunction)Navio_ADC_read_all, METH_NOARGS,
     "Get an end-point reading (in mV) from all four ADC inputs."
    },

    {"FRAM_read", (PyCFunction)Navio_FRAM_read, METH_VARARGS|METH_KEYWORDS,
     "Read data from FRAM."
    },
    {"FRAM_write", (PyCFunction)Navio_FRAM_write, METH_VARARGS|METH_KEYWORDS,
     "Write data to FRAM."
    },
    {"FRAM_clear", (PyCFunction)Navio_FRAM_clear, METH_NOARGS,
     "Clear FRAM."
    },
    {"FRAM_test", (PyCFunction)Navio_FRAM_test, METH_NOARGS,
     "Perform FRAM read/write test."
    },

    {"PPM_enable", (PyCFunction)Navio_PPM_enable, METH_VARARGS|METH_KEYWORDS,
     "Enable reading from PPM input."
    },
    {"PPM_disable", (PyCFunction)Navio_PPM_disable, METH_NOARGS,
     "Disable reading from PPM input."
    },
    {"PPM_register_callback", (PyCFunction)Navio_PPM_register_callback, METH_VARARGS,
     "Register callback to be called on received PPM data."
    },
    {"PPM_read", (PyCFunction)Navio_PPM_read, METH_NOARGS,
     "Read PPM channel data."
    },

    {NULL}  /* Sentinel */
};

static void cleanup(Navio *self)
{
    if (self->gps)
        delete self->gps;
    if (self->imu)
        delete self->imu;
    if (self->baro)
        delete self->baro;
    if (self->pwm)
        delete self->pwm;
    if (self->adc)
        delete self->adc;
    if (self->fram)
        delete self->fram;
    if (self->ppm)
        delete self->ppm;
}

static PyObject *
Navio_new(PyTypeObject *type, PyObject *args, PyObject *kwds)
{
    static Navio *self = 0;  /* static to create a Singleton */

    if (!self) {
        self = (Navio *)type->tp_alloc(type, 0);

        self->gps   = 0;
        self->imu   = 0;
        self->baro  = 0;
        self->pwm   = 0;
        self->adc   = 0;
        self->fram  = 0;
        self->ppm   = 0;

        self->is_initialized = false;
    }
    else {
        Py_XINCREF(self);
    }

    return (PyObject *)self;
}

static int
Navio_init(Navio *self, PyObject *args, PyObject *kwds)
{
    const char *i2c_addr;

    self->rpi_model = RPI_MODEL_B_2;
    self->navio_model = NAVIO_PLUS;
    self->enabled_components = BIT_COMPONENTS_ALL;
    self->rc_input_signal = RC_MODE_PPM;

    static char *kwlist[] = {(char *)"rpi_model", (char *)"navio_model", (char *)"enabled_components", (char *)"rc_input_signal", NULL};

    if (self && !self->is_initialized) {
        if (!PyArg_ParseTupleAndKeywords(args, kwds, "|iiii", kwlist, &self->rpi_model, &self->navio_model, &self->enabled_components, &self->rc_input_signal)) {
            return -1;
        }

        switch (self->rpi_model)
        {
            case RPI_MODEL_A:
            case RPI_MODEL_A_PLUS:  // don't know yet, so assume the same as model A
                i2c_addr = RASPBERRY_PI_MODEL_A_I2C;
                break;
            case RPI_MODEL_B:
            case RPI_MODEL_B_PLUS:
                i2c_addr = RASPBERRY_PI_MODEL_B_I2C;
                break;
            case RPI_MODEL_B_2:     // this correct?
                i2c_addr = RASPBERRY_PI_2_MODEL_B_I2C;
                break;
            case BANANA_PI:
                i2c_addr = BANANA_PI_I2C;
                break;

            default:
                i2c_addr = RASPBERRY_PI_2_MODEL_B_I2C;    // assume Pi 2 model B (should also work on Pi 1 B/B+)
        }

        if ((self->enabled_components & BIT_COMPONENTS_GPS) != 0) {
            try {
                self->gps = new Ublox();
            }
            catch (std::bad_alloc) {
                goto AllocErr;
            }
            if (!self->gps->testConnection()) {
                PyErr_SetString(InitializationException, "Unable to communicate with Ublox (GPS).");
                goto InitErr;
            }
        }
        if ((self->enabled_components & BIT_COMPONENTS_IMU) != 0) {
            try {
                self->imu = new MPU9250();
            }
            catch (std::bad_alloc) {
                goto AllocErr;
            }
            self->imu->initialize();
            if (!self->imu->testConnection()) {
                PyErr_SetString(InitializationException, "Unable to communicate with MPU9250 (IMU).");
                goto InitErr;
            }
        }
        if ((self->enabled_components & BIT_COMPONENTS_BARO) != 0) {
            try {
                self->baro = new MS5611(i2c_addr);
            }
            catch (std::bad_alloc) {
                goto AllocErr;
            }
            self->baro->initialize();
            if (!self->baro->testConnection()) {
                PyErr_SetString(InitializationException, "Unable to communicate with MS5611 (baro).");
                goto InitErr;
            }
        }
        if ((self->enabled_components & BIT_COMPONENTS_PWM) != 0) {
            try {
                self->pwm = new PCA9685(i2c_addr);
            }
            catch (std::bad_alloc) {
                goto AllocErr;
            }
            self->pwm->initialize();
            if (!self->pwm->testConnection()) {
                PyErr_SetString(InitializationException, "Unable to communicate with PCA9685 (PWM).");
                goto InitErr;
            }
        }
        if ((self->enabled_components & BIT_COMPONENTS_ADC) != 0) {
            try {
                self->adc = new ADS1115(i2c_addr);
            }
            catch (std::bad_alloc) {
                goto AllocErr;
            }
            if (!self->adc->testConnection()) {
                PyErr_SetString(InitializationException, "Unable to communicate with ADS1115 (ADC).");
                goto InitErr;
            }
        }
        if ((self->enabled_components & BIT_COMPONENTS_FRAM) != 0) {
            try {
                switch (self->navio_model) {
                    case NAVIO:
                    case NAVIO_RAW:
                        self->fram = new MB85RC04(i2c_addr);
                        break;
                    case NAVIO_PLUS:
                        self->fram = new MB85RC256(i2c_addr);
                        break;
                    default:
                        PyErr_SetString(PyExc_ValueError, "Invalid Navio model specified.");
                        goto InitErr;
                }
            }
            catch (std::bad_alloc) {
                goto AllocErr;
            }
            if (!self->fram->testConnection()) {
                PyErr_SetString(InitializationException, "Unable to communicate with FRAM.");
                goto InitErr;
            }
        }
        if ((self->enabled_components & BIT_COMPONENTS_PPM) != 0) {
            try {
                bool rawdata = (self->rc_input_signal == RC_MODE_PPM_RAW);
                switch (self->rc_input_signal) {
                    case RC_MODE_PPM:
                    case RC_MODE_PPM_RAW:
                        self->ppm = new RCin(SCANNER_TYPE_PPM, rawdata);
                        break;
                    case RC_MODE_SBUS:
                        self->ppm = new RCin(SCANNER_TYPE_SBUS);
                        break;
                    default:
                        PyErr_SetString(PyExc_ValueError, "Invalid signal type specified.");
                        goto InitErr;
                }
            }
            catch (std::bad_alloc) {
                goto AllocErr;
            }

            if (!self->ppm->initialize()) {
                PyErr_SetString(InitializationException, "Unable to initialize GPIO for PPM input.");
                goto InitErr;
            }
        }

        // Make sure the GIL has been created since we need to acquire it in our
        // callbacks to safely call into the python interpreter.
        if (!PyEval_ThreadsInitialized()) {
            PyEval_InitThreads();
        }

        self->is_initialized = true;
    }

    if (self && self->is_initialized) {
        return 0;
    }
    else {
        goto InitErr;
    }

InitErr:
    cleanup(self);
    return -1;
AllocErr:
    cleanup(self);
    PyErr_NoMemory();
    return -1;
}

static void
Navio_dealloc(Navio *self)
{
    cleanup(self);
    Py_TYPE(self)->tp_free((PyObject*)self);

    //self = NULL;
}

static PyTypeObject NavioType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "navio.Navio",                                      /* tp_name */
    sizeof(Navio),                                      /* tp_basicsize */
    0,                                                  /* tp_itemsize */
    (destructor)Navio_dealloc,                          /* tp_dealloc */
    0,                                                  /* tp_print */
    0,                                                  /* tp_getattr */
    0,                                                  /* tp_setattr */
    0,                                                  /* tp_reserved */
    0,                                                  /* tp_repr */
    0,                                                  /* tp_as_number */
    0,                                                  /* tp_as_sequence */
    0,                                                  /* tp_as_mapping */
    0,                                                  /* tp_hash  */
    0,                                                  /* tp_call */
    0,                                                  /* tp_str */
    0,                                                  /* tp_getattro */
    0,                                                  /* tp_setattro */
    0,                                                  /* tp_as_buffer */
    Py_TPFLAGS_DEFAULT,                                 /* tp_flags */
    "Navio",                                            /* tp_doc */
    0,                                                  /* tp_traverse */
    0,                                                  /* tp_clear */
    0,                                                  /* tp_richcompare */
    0,                                                  /* tp_weaklistoffset */
    0,                                                  /* tp_iter */
    0,                                                  /* tp_iternext */
    Navio_methods,                                      /* tp_methods */
    0,                                                  /* tp_members */
    0,                                                  /* tp_getset */
    0,                                                  /* tp_base */
    0,                                                  /* tp_dict */
    0,                                                  /* tp_descr_get */
    0,                                                  /* tp_descr_set */
    0,                                                  /* tp_dictoffset */
    (initproc)Navio_init,                               /* tp_init */
    0,                                                  /* tp_alloc */
    Navio_new,                                          /* tp_new */
};

static PyModuleDef navio = {
    PyModuleDef_HEAD_INIT,
    "navio",
    "Python extension for the Navio shield for Raspberry Pi",
    -1,
    NULL, NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC
PyInit_navio(void)
{
    PyObject* m;

    if (PyType_Ready(&NavioType) < 0)
        return NULL;

    m = PyModule_Create(&navio);
    if (m == NULL)
        return NULL;

    // Add the Navio-type to the module
    Py_INCREF(&NavioType);
    PyModule_AddObject(m, "Navio", (PyObject *)&NavioType);

    // Add exceptions
    InitializationException = PyErr_NewException("navio.InitializationException", NULL, NULL);
    Py_INCREF(InitializationException);
    PyModule_AddObject(m, "InitializationException", InitializationException);
    ConnectionException = PyErr_NewException("navio.ConnectionException", NULL, NULL);
    Py_INCREF(ConnectionException);
    PyModule_AddObject(m, "ConnectionException", ConnectionException);

    // Add module constants
    // RPi models
    PyModule_AddIntMacro(m, RPI_MODEL_A);
    PyModule_AddIntMacro(m, RPI_MODEL_A_PLUS);
    PyModule_AddIntMacro(m, RPI_MODEL_B);
    PyModule_AddIntMacro(m, RPI_MODEL_B_PLUS);
    PyModule_AddIntMacro(m, RPI_MODEL_B_2);
    PyModule_AddIntMacro(m, BANANA_PI);

    // Navio board types
    PyModule_AddIntMacro(m, NAVIO);
    PyModule_AddIntMacro(m, NAVIO_RAW);
    PyModule_AddIntMacro(m, NAVIO_PLUS);

    // Navio(+) component bits
    PyModule_AddIntMacro(m, BIT_COMPONENTS_GPS);
    PyModule_AddIntMacro(m, BIT_COMPONENTS_IMU);
    PyModule_AddIntMacro(m, BIT_COMPONENTS_BARO);
    PyModule_AddIntMacro(m, BIT_COMPONENTS_PWM);
    PyModule_AddIntMacro(m, BIT_COMPONENTS_ADC);
    PyModule_AddIntMacro(m, BIT_COMPONENTS_FRAM);
    PyModule_AddIntMacro(m, BIT_COMPONENTS_PPM);
    PyModule_AddIntMacro(m, BIT_COMPONENTS_ALL);

    // RC input
    PyModule_AddIntMacro(m, RC_MODE_PPM);
    PyModule_AddIntMacro(m, RC_MODE_PPM_RAW);
    PyModule_AddIntMacro(m, RC_MODE_SBUS);

    // ADC config
    PyModule_AddIntMacro(m, ADC_GAIN_0);
    PyModule_AddIntMacro(m, ADC_GAIN_1);
    PyModule_AddIntMacro(m, ADC_GAIN_2);
    PyModule_AddIntMacro(m, ADC_GAIN_4);
    PyModule_AddIntMacro(m, ADC_GAIN_8);
    PyModule_AddIntMacro(m, ADC_GAIN_16);
    PyModule_AddIntMacro(m, ADC_MUX_AIN0_AIN1);
    PyModule_AddIntMacro(m, ADC_MUX_AIN0_AIN3);
    PyModule_AddIntMacro(m, ADC_MUX_AIN1_AIN3);
    PyModule_AddIntMacro(m, ADC_MUX_AIN2_AIN3);
    PyModule_AddIntMacro(m, ADC_MUX_AIN0_GND);
    PyModule_AddIntMacro(m, ADC_MUX_AIN1_GND);
    PyModule_AddIntMacro(m, ADC_MUX_AIN2_GND);
    PyModule_AddIntMacro(m, ADC_MUX_AIN3_GND);
    PyModule_AddIntMacro(m, ADC_MODE_CONT);
    PyModule_AddIntMacro(m, ADC_MODE_SINGLE);
    PyModule_AddIntMacro(m, ADC_RATE_8);
    PyModule_AddIntMacro(m, ADC_RATE_16);
    PyModule_AddIntMacro(m, ADC_RATE_32);
    PyModule_AddIntMacro(m, ADC_RATE_64);
    PyModule_AddIntMacro(m, ADC_RATE_128);
    PyModule_AddIntMacro(m, ADC_RATE_250);
    PyModule_AddIntMacro(m, ADC_RATE_475);
    PyModule_AddIntMacro(m, ADC_RATE_860);

    // UBX messages
    // UBX message class ACK
    PyModule_AddIntConstant(m, "UBX_MSG_ACK_ACK",       ((UBX_MSG::CLASS_ACK << 8) + UBX_MSG::ACK_ACK));
    PyModule_AddIntConstant(m, "UBX_MSG_ACK_NAK",       ((UBX_MSG::CLASS_ACK << 8) + UBX_MSG::ACK_NAK));

    // UBX message class AID
    PyModule_AddIntConstant(m, "UBX_MSG_AID_ALM",       ((UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_ALM));
    PyModule_AddIntConstant(m, "UBX_MSG_AID_ALPSRV",    ((UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_ALPSRV));
    PyModule_AddIntConstant(m, "UBX_MSG_AID_ALP",       ((UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_ALP));
    PyModule_AddIntConstant(m, "UBX_MSG_AID_AOP",       ((UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_AOP));
    PyModule_AddIntConstant(m, "UBX_MSG_AID_DATA",      ((UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_DATA));
    PyModule_AddIntConstant(m, "UBX_MSG_AID_EPH",       ((UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_EPH));
    PyModule_AddIntConstant(m, "UBX_MSG_AID_HUI",       ((UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_HUI));
    PyModule_AddIntConstant(m, "UBX_MSG_AID_INI",       ((UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_INI));
    PyModule_AddIntConstant(m, "UBX_MSG_AID_REQ",       ((UBX_MSG::CLASS_AID << 8) + UBX_MSG::AID_REQ));

    // UBX message class CFG
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_ANT",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_ANT));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_CFG",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_CFG));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_DAT",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_DAT));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_EKF",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_EKF));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_ESFGWT",    ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_ESFGWT));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_FXN",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_FXN));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_INF",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_INF));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_ITFM",      ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_ITFM));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_MSG",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_MSG));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_NAV5",      ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_NAV5));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_NAVX5",     ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_NAVX5));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_NMEA",      ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_NMEA));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_NVS",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_NVS));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_PM2",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_PM2));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_PM",        ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_PM));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_PRT",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_PRT));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_RATE",      ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_RATE));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_RINV",      ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_RINV));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_RST",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_RST));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_RXM",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_RXM));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_SBAS",      ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_SBAS));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_TMODE2",    ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_TMODE2));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_TMODE",     ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_TMODE));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_TP5",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_TP5));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_TP",        ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_TP));
    PyModule_AddIntConstant(m, "UBX_MSG_CFG_USB",       ((UBX_MSG::CLASS_CFG << 8) + UBX_MSG::CFG_USB));

    // UBX message class ESF
    PyModule_AddIntConstant(m, "UBX_MSG_ESF_MEAS",      ((UBX_MSG::CLASS_ESF << 8) + UBX_MSG::ESF_MEAS));
    PyModule_AddIntConstant(m, "UBX_MSG_ESF_STATUS",    ((UBX_MSG::CLASS_ESF << 8) + UBX_MSG::ESF_STATUS));

    // UBX message class INF
    PyModule_AddIntConstant(m, "UBX_MSG_INF_DEBUG",     ((UBX_MSG::CLASS_INF << 8) + UBX_MSG::INF_DEBUG));
    PyModule_AddIntConstant(m, "UBX_MSG_INF_ERROR",     ((UBX_MSG::CLASS_INF << 8) + UBX_MSG::INF_ERROR));
    PyModule_AddIntConstant(m, "UBX_MSG_INF_NOTICE",    ((UBX_MSG::CLASS_INF << 8) + UBX_MSG::INF_NOTICE));
    PyModule_AddIntConstant(m, "UBX_MSG_INF_TEST",      ((UBX_MSG::CLASS_INF << 8) + UBX_MSG::INF_TEST));
    PyModule_AddIntConstant(m, "UBX_MSG_INF_WARNING",   ((UBX_MSG::CLASS_INF << 8) + UBX_MSG::INF_WARNING));

    // UBX message class MON
    PyModule_AddIntConstant(m, "UBX_MSG_MON_HW2",       ((UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_HW2));
    PyModule_AddIntConstant(m, "UBX_MSG_MON_HW",        ((UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_HW));
    PyModule_AddIntConstant(m, "UBX_MSG_MON_IO",        ((UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_IO));
    PyModule_AddIntConstant(m, "UBX_MSG_MON_MSGPP",     ((UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_MSGPP));
    PyModule_AddIntConstant(m, "UBX_MSG_MON_RXBUF",     ((UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_RXBUF));
    PyModule_AddIntConstant(m, "UBX_MSG_MON_RXR",       ((UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_RXR));
    PyModule_AddIntConstant(m, "UBX_MSG_MON_TXBUF",     ((UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_TXBUF));
    PyModule_AddIntConstant(m, "UBX_MSG_MON_VER",       ((UBX_MSG::CLASS_MON << 8) + UBX_MSG::MON_VER));

    // UBX message class NAV
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_AOPSTATUS", ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_AOPSTATUS));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_CLOCK",     ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_CLOCK));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_DGPS",      ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_DGPS));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_DOP",       ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_DOP));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_EKFSTATUS", ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_EKFSTATUS));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_POSECEF",   ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_POSECEF));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_POSLLH",    ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_POSLLH));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_SBAS",      ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_SBAS));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_SOL",       ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_SOL));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_STATUS",    ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_STATUS));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_SVINFO",    ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_SVINFO));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_TIMEGPS",   ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_TIMEGPS));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_TIMEUTC",   ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_TIMEUTC));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_VELECEF",   ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_VELECEF));
    PyModule_AddIntConstant(m, "UBX_MSG_NAV_VELNED",    ((UBX_MSG::CLASS_NAV << 8) + UBX_MSG::NAV_VELNED));

    // UBX message class RXM
    PyModule_AddIntConstant(m, "UBX_MSG_RXM_ALM",       ((UBX_MSG::CLASS_RXM << 8) + UBX_MSG::RXM_ALM));
    PyModule_AddIntConstant(m, "UBX_MSG_RXM_EPH",       ((UBX_MSG::CLASS_RXM << 8) + UBX_MSG::RXM_EPH));
    PyModule_AddIntConstant(m, "UBX_MSG_RXM_PMREQ",     ((UBX_MSG::CLASS_RXM << 8) + UBX_MSG::RXM_PMREQ));
    PyModule_AddIntConstant(m, "UBX_MSG_RXM_RAW",       ((UBX_MSG::CLASS_RXM << 8) + UBX_MSG::RXM_RAW));
    PyModule_AddIntConstant(m, "UBX_MSG_RXM_SFRB",      ((UBX_MSG::CLASS_RXM << 8) + UBX_MSG::RXM_SFRB));
    PyModule_AddIntConstant(m, "UBX_MSG_RXM_SVSI",      ((UBX_MSG::CLASS_RXM << 8) + UBX_MSG::RXM_SVSI));

    // UBX message class TIM
    PyModule_AddIntConstant(m, "UBX_MSG_TIM_SVIN",      ((UBX_MSG::CLASS_TIM << 8) + UBX_MSG::TIM_SVIN));
    PyModule_AddIntConstant(m, "UBX_MSG_TIM_TM2",       ((UBX_MSG::CLASS_TIM << 8) + UBX_MSG::TIM_TM2));
    PyModule_AddIntConstant(m, "UBX_MSG_TIM_TP",        ((UBX_MSG::CLASS_TIM << 8) + UBX_MSG::TIM_TP));
    PyModule_AddIntConstant(m, "UBX_MSG_TIM_VRFY",      ((UBX_MSG::CLASS_TIM << 8) + UBX_MSG::TIM_VRFY));

    return m;
}


/*

    Navio.getMotion6()
    Navio.getMotion9()

    Navio.getTempAndPressure()

    Navio.setLED(r, g, b)
    Navio.setPWM(chan, val)
    Navio.setPWMmS(chan, mS)
    Navio.setPWMuS(chan, uS)

*/
