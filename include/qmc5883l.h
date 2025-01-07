#pragma once

#include <Arduino.h>
#include "SimpleI2C.h"

// ————————————————————————————————————————————————————————————————
// QMC5883L I²C address and register definitions
// ————————————————————————————————————————————————————————————————
#define QMC5883L_ADDRESS       0x0D

// control registers
#define QMC5883L_REG_CONTROL_1 0x09
#define QMC5883L_REG_CONTROL_2 0x0A  // reset / set‑reset period
#define QMC5883L_REG_SET_RESET  0x0B  // set‑reset enable
#define QMC5883L_REG_OUT_X_L    0x00  // read starts here, 6 bytes X,Y,Z

// CONTROL_1 bit fields
#define QMC_MODE_STANDBY        0b00000000
#define QMC_MODE_CONTINUOUS     0b00000001

#define QMC_ODR_10HZ            0b00000000
#define QMC_ODR_50HZ            0b00000100
#define QMC_ODR_100HZ           0b00001000
#define QMC_ODR_200HZ           0b00001100

#define QMC_RANGE_2G            0b00000000
#define QMC_RANGE_8G            0b00010000

#define QMC_OSR_512             0b00000000
#define QMC_OSR_256             0b01000000
#define QMC_OSR_128             0b10000000
#define QMC_OSR_64              0b11000000

// ————————————————————————————————————————————————————————————————
// Calibration & conversion constants
// ————————————————————————————————————————————————————————————————
static constexpr float QMC5883_LSB_PER_uT = 40.96f;  
    // ±8 G → ±800 µT full‑scale, 16‑bit → 65536/1600 ≈ 40.96 LSB per µT

// your original globals (renamed for QMC)
bool    calibrate      = true;
float   declinationDeg = 3.44f;    // Yogyakarta +0.70° → 0.70 × (180/π)=3.44 rad?
float   rawX, rawY, rawZ;
float   mx, my, mz;
float   heading, trueHeading;
float   xMax=0, xMin=0, yMax=0, yMin=0;



////calibration data 
// hard‑iron offsets (µT)
static constexpr float hard_iron_bias_x =  41.16886643f;
static constexpr float hard_iron_bias_y = -89.87465738f;
static constexpr float hard_iron_bias_z = 569.66392911f;

// soft‑iron correction matrix (3×3)
//   [ Sxx  Sxy  Sxz ]
//   [ Syx  Syy  Syz ]
//   [ Szx  Szy  Szz ]
static constexpr float Sxx =  5.265898315001022f;
static constexpr float Sxy =  0.054028985259799656f;
static constexpr float Sxz = -0.22103196000979536f;

static constexpr float Syx =  0.05402898525979976f;
static constexpr float Syy =  5.341521822820932f;
static constexpr float Syz =  0.057716910125935224f;

static constexpr float Szx = -0.22103196000979475f;
static constexpr float Szy =  0.05771691012593512f;
static constexpr float Szz =  6.520325842726514f;

// ————————————————————————————————————————————————————————————————
// Low‑level I²C helpers
// ————————————————————————————————————————————————————————————————
static void writeRegister(uint8_t reg, uint8_t value) {
    _I2C2.writeByte(QMC5883L_ADDRESS, reg, value);
}


// ————————————————————————————————————————————————————————————————
// Public API
// ————————————————————————————————————————————————————————————————



/// Read raw X,Y,Z; convert to µT
void hmc5883l_GetData() {
    uint8_t buf[6];
    _I2C2.readBytes(QMC5883L_ADDRESS, QMC5883L_REG_OUT_X_L, 6, &buf[0]);

    // QMC5883L outputs little‑endian: LSB first
    rawX = int16_t(buf[0] | (buf[1] << 8));
    rawY = int16_t(buf[2] | (buf[3] << 8));
    rawZ = int16_t(buf[4] | (buf[5] << 8));

    // convert to micro‑tesla
    mx = rawX / QMC5883_LSB_PER_uT;
    my = rawY / QMC5883_LSB_PER_uT;
    mz = rawZ / QMC5883_LSB_PER_uT;
}

void hmc5883l_GetData_calibed()
{
    uint8_t buf[6];
   _I2C2.readBytes(QMC5883L_ADDRESS, QMC5883L_REG_OUT_X_L, 6, &buf[0]);

    // raw counts → µT
    rawX = int16_t(buf[0] | (buf[1] << 8));
    rawY = int16_t(buf[2] | (buf[3] << 8));
    rawZ = int16_t(buf[4] | (buf[5] << 8));
    mx   = rawX / QMC5883_LSB_PER_uT;
    my   = rawY / QMC5883_LSB_PER_uT;
    mz   = rawZ / QMC5883_LSB_PER_uT;

    // — apply hard‑iron correction —
    float vx = mx - hard_iron_bias_x;
    float vy = my - hard_iron_bias_y;
    float vz = mz - hard_iron_bias_z;

    // — apply soft‑iron correction matrix —
    float cx = Sxx * vx + Sxy * vy + Sxz * vz;
    float cy = Syx * vx + Syy * vy + Syz * vz;
    float cz = Szx * vx + Szy * vy + Szz * vz;

    // overwrite mx,my,mz with fully calibrated values
    mx = cx;
    my = cy;
    mz = cz;
}


/// Dynamic two‑axis min/max calibration
void hmc5883l_calibrate() {
    if (xMax == 0 && xMin == 0) {
        xMax = xMin = mx;
        yMax = yMin = my;
    }
    if (calibrate) {
        xMax = max(xMax, mx);
        xMin = min(xMin, mx);
        yMax = max(yMax, my);
        yMin = min(yMin, my);
    }
}

/// Compute heading (°) with applied declination
float hmc5883l_computeHeading() {
    // offset compensation
    float x = mx - (xMax + xMin) / 2.0f;
    float y = my - (yMax + yMin) / 2.0f;

    heading = atan2(y, x) * 180.0f / PI;
    // normalize
    if (heading < 0) heading += 360.0f;
    // apply declination
    trueHeading = heading + declinationDeg;
    if (trueHeading >= 360.0f) trueHeading -= 360.0f;
    return trueHeading;
}

void hmc5883l_init() {
    // Wire2.begin();
    // soft‑reset / set‑reset period
    _I2C2.init();
    writeRegister(QMC5883L_REG_CONTROL_2, 0x01);
    // enable set‑reset pulses
    writeRegister(QMC5883L_REG_SET_RESET, 0x01);
    // continuous mode, ODR=200 Hz, range=±8 G, OSR=512
    writeRegister(QMC5883L_REG_CONTROL_1,
        QMC_MODE_CONTINUOUS |
        QMC_ODR_200HZ    |
        QMC_RANGE_8G     |
        QMC_OSR_512
    );

    // prime first reading and calibration
    hmc5883l_GetData();
    hmc5883l_calibrate();
}