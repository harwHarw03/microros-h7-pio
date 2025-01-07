#pragma once
#include <Arduino.h>
#include "SimpleI2C.h"

#define HMC5883_ID (0x0D)
#define HMC5883_ADDRESS_MAG (0x3C >> 1) // 0011110x

#define HMC5883_REGISTER_MAG_CRA_REG_M (0x00)
#define HMC5883_REGISTER_MAG_CRB_REG_M (0x01)
#define HMC5883_REGISTER_MAG_MR_REG_M (0x02)
#define HMC5883_REGISTER_MAG_OUT_X_H_M (0x03)
#define HMC5883_REGISTER_MAG_OUT_X_L_M (0x04)
#define HMC5883_REGISTER_MAG_OUT_Z_H_M (0x05)
#define HMC5883_REGISTER_MAG_OUT_Z_L_M (0x06)
#define HMC5883_REGISTER_MAG_OUT_Y_H_M (0x07)
#define HMC5883_REGISTER_MAG_OUT_Y_L_M (0x08)
#define HMC5883_REGISTER_MAG_SR_REG_Mg (0x09)
#define HMC5883_REGISTER_MAG_IRA_REG_M (0x0A)
#define HMC5883_REGISTER_MAG_IRB_REG_M (0x0B)
#define HMC5883_REGISTER_MAG_IRC_REG_M (0x0C)
#define HMC5883_REGISTER_MAG_TEMP_OUT_H_M (0x31)
#define HMC5883_REGISTER_MAG_TEMP_OUT_L_M (0x32)

#define HMC5883_MAGGAIN_1_3 (0x20) // +/- 1.3
// _hmc5883_Gauss_LSB_XY = 1100;
// _hmc5883_Gauss_LSB_Z = 980;
#define HMC5883_MAGGAIN_1_9 (0x40) // +/- 1.9
// _hmc5883_Gauss_LSB_XY = 855;
// _hmc5883_Gauss_LSB_Z = 760;
#define HMC5883_MAGGAIN_2_5 (0x60) // +/- 2.5
// _hmc5883_Gauss_LSB_XY = 670;
// _hmc5883_Gauss_LSB_Z = 600;
#define HMC5883_MAGGAIN_4_0 (0x80) // +/- 4.0
// _hmc5883_Gauss_LSB_XY = 450;
// _hmc5883_Gauss_LSB_Z = 400;
#define HMC5883_MAGGAIN_4_7 (0xA0) // +/- 4.7
// _hmc5883_Gauss_LSB_XY = 400;
// _hmc5883_Gauss_LSB_Z = 255;
#define HMC5883_MAGGAIN_5_6 (0xC0) // +/- 5.6
// _hmc5883_Gauss_LSB_XY = 330;
// _hmc5883_Gauss_LSB_Z = 295;
#define HMC5883_MAGGAIN_8_1 (0xE0) // +/- 8.1
// _hmc5883_Gauss_LSB_XY = 230;
// _hmc5883_Gauss_LSB_Z = 205;

static float _hmc5883_Gauss_LSB_XY = 1100.0F; // Varies with gain
static float _hmc5883_Gauss_LSB_Z = 980.0F;   // Varies with gain

bool calibrate = true;
float declinationAngle = 3.44; // yogyakarta : +0.70° (1°42')
float magx, magy, magz, mx, my, mz, heading, true_heading;
float xMax, yMax, xMin, yMin = 0.0;

void hmc5883l_calibrate()
{
    if (xMax == 0.0) {
        xMax = mx;
    }

    if (yMax == 0.0) {
        yMax = my;
    }

    if (calibrate) {
        xMax = max(xMax, mx);
        yMax = max(yMax, my);
        xMin = min(xMin, mx);
        yMin = min(yMin, my);
    }
}

void hmc5883l_GetData()
{
    uint8_t data[6];
    _I2C2.readBytes(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_OUT_X_H_M, 6, &data[0]);
    magx = (int16_t)(data[1] | ((int16_t)data[0] << 8));
    magz = (int16_t)(data[3] | ((int16_t)data[2] << 8));
    magy = (int16_t)(data[5] | ((int16_t)data[4] << 8));

    mx = magx / _hmc5883_Gauss_LSB_XY * 100;
    my = magy / _hmc5883_Gauss_LSB_XY * 100;
    mz = _hmc5883_Gauss_LSB_Z * 100;
    // 100 = sensor gauss to microtesla  | source : adafruit_sensor.h
}

void hmc5883l_init()
{
    _I2C2.init();
    _I2C2.writeByte(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_MR_REG_M, 0x00);
    _I2C2.writeByte(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_CRB_REG_M, (byte)HMC5883_MAGGAIN_1_3);
    _hmc5883_Gauss_LSB_XY = 1100;
    _hmc5883_Gauss_LSB_Z = 980;
    hmc5883l_GetData();
    hmc5883l_calibrate();
    // Wire.begin();
}