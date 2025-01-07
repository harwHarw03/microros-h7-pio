/* reference :
  mostly imitating https://blog.naver.com/ysahn2k/221410891895
  original paper and source code is at https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
*/

#pragma once
#include <Arduino.h>
#include "SimpleI2C.h"
#include "mpu6050_register.h"

// accel & gyro data
//acx: 157  acy: 105  acz: 134  gyx: -76  gyy: 1  gyz: 1 
#define MPU6050_AXOFFSET 157
#define MPU6050_AYOFFSET 105
#define MPU6050_AZOFFSET 134// AcX = 242 | AcY = -55 | AcZ = -669 | 
#define MPU6050_AXGAIN 4096.0 // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
#define MPU6050_AYGAIN 4096.0 // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
#define MPU6050_AZGAIN 4096.0 // AFS_SEL = 2, +/-8g, MPU6050_ACCEL_FS_8
#define MPU6050_GXOFFSET -76
#define MPU6050_GYOFFSET 1
#define MPU6050_GZOFFSET 1  //GyX = -31 | GyY = -19 | GyZ = -19
#define MPU6050_GXGAIN 16.384 // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000
#define MPU6050_GYGAIN 16.384 // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000
#define MPU6050_GZGAIN 16.384 // FS_SEL = 3, +/-2000degree/s, MPU6050_GYRO_FS_2000

// I2C _I2C(Wire);

// values
volatile int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float axg, ayg, azg, gxrs, gyrs, gzrs;
int32_t cal_acx, cal_acy, cal_acz, cal_tmp, cal_gyx, cal_gyy, cal_gyz;
int32_t imu_offset[6];

void mpu6050_get_data(bool calibrating = false)
{
  uint8_t data_raw[14];
  _I2C.readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14, &data_raw[0]);
  AcX = data_raw[0] << 8 | data_raw[1];
  AcY = data_raw[2] << 8 | data_raw[3];
  if (calibrating) {
    AcZ = (data_raw[4] << 8 | data_raw[5]) - 4096;
  } else {
    AcZ = data_raw[4] << 8 | data_raw[5];
  }
  Tmp = data_raw[6] << 8 | data_raw[7];
  GyX = data_raw[8] << 8 | data_raw[9];
  GyY = data_raw[10] << 8 | data_raw[11];
  GyZ = data_raw[12] << 8 | data_raw[13];
}

void mpu6050_calibrate() {
  int16_t count = 2000;
  for (int16_t i = 0; i < count; i++) {
    if (i % 200 == 0) Serial.println("Calibrating.....");
    mpu6050_get_data(true);
    delay(10);
    // Sum data
    cal_acx += AcX;
    cal_acy += AcY;
    cal_acz += AcZ;
    cal_gyx += GyX;
    cal_gyy += GyY;
    cal_gyz += GyZ;
  }

  cal_acx /= count;
  cal_acy /= count;
  cal_acz /= count;
  cal_gyx /= count;
  cal_gyy /= count;
  cal_gyz /= count;

  int32_t input_offset[6] = {cal_acx, cal_acy, cal_acz, cal_gyx, cal_gyy, cal_gyz};



  while (1) {
    Serial.print("acx: ");
    Serial.print(cal_acx);
    Serial.print("  acy: ");
    Serial.print(cal_acy);
    Serial.print("  acz: ");
    Serial.print(cal_acz);
    Serial.print("  gyx: ");
    Serial.print(cal_gyx);
    Serial.print("  gyy: ");
    Serial.print(cal_gyy);
    Serial.print("  gyz: ");
    Serial.println(cal_gyz);
    delay(5000);
  }
}

void mpu6050_init(bool calibrating)
{
  _I2C.init();
  _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00); // set to zero (wakes up the MPU-6050)
  _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x18); // FS_SEL=3
  _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x10); // AFS_SEL=2
  _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x00); //Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz

  if (calibrating) {
    _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x03); // Selection Clock 'PLL with Z axis gyroscope reference'
    mpu6050_calibrate();
  } else {
    _I2C.writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01); // Selection Clock 'PLL with X axis gyroscope reference'
  }
}

void mpu6050_scaling(bool use_eeprom = false)
{
  if (use_eeprom) {
    axg = (float)(AcX - imu_offset[0]) / MPU6050_AXGAIN;
    ayg = (float)(AcY - imu_offset[1]) / MPU6050_AYGAIN;
    azg = (float)(AcZ - imu_offset[2]) / MPU6050_AZGAIN;
    gxrs = (float)(GyX - imu_offset[3]) / MPU6050_GXGAIN * 0.01745329;   //degree to radians
    gyrs = (float)(GyY - imu_offset[4]) / MPU6050_GYGAIN * 0.01745329;   //degree to radians
    gzrs = (float)(GyZ - imu_offset[5]) / MPU6050_GZGAIN * 0.01745329;   //degree to radians
  } else {
    axg = (float)(AcX - MPU6050_AXOFFSET) / MPU6050_AXGAIN;
    ayg = (float)(AcY - MPU6050_AYOFFSET) / MPU6050_AYGAIN;
    azg = (float)(AcZ - MPU6050_AZOFFSET) / MPU6050_AZGAIN;
    gxrs = (float)(GyX - MPU6050_GXOFFSET) / MPU6050_GXGAIN * 0.01745329;   //degree to radians
    gyrs = (float)(GyY - MPU6050_GYOFFSET) / MPU6050_GYGAIN * 0.01745329;   //degree to radians
    gzrs = (float)(GyZ - MPU6050_GZOFFSET) / MPU6050_GZGAIN * 0.01745329;   //degree to radians
  }
}

/*Unused
  //#define MPU6050_AXOFFSET 0
  //#define MPU6050_AYOFFSET 0
  //#define MPU6050_AZOFFSET 0
  //#define MPU6050_AXGAIN 16384.0 // AFS_SEL = 0, +/-2g, MPU6050_ACCEL_FS_2
  //#define MPU6050_AYGAIN 16384.0 // AFS_SEL = 0, +/-2g, MPU6050_ACCEL_FS_2
  //#define MPU6050_AZGAIN 16384.0 // AFS_SEL = 0, +/-2g, MPU6050_ACCEL_FS_2
  //#define MPU6050_AXGAIN 8192.0 // AFS_SEL = 1, +/-4g, MPU6050_ACCEL_FS_4
  //#define MPU6050_AYGAIN 8192.0 // AFS_SEL = 1, +/-4g, MPU6050_ACCEL_FS_4
  //#define MPU6050_AZGAIN 8192.0 // AFS_SEL = 1, +/-4g, MPU6050_ACCEL_FS_4

  //#define MPU6050_AXGAIN 2048.0 // AFS_SEL = 3, +/-16g, MPU6050_ACCEL_FS_16
  //#define MPU6050_AYGAIN 2048.0 // AFS_SEL = 3, +/-16g, MPU6050_ACCEL_FS_16
  //#define MPU6050_AZGAIN 2048.0 // AFS_SEL = 3, +/-16g, MPU6050_ACCEL_FS_16

  //#define MPU6050_GXOFFSET 0
  //#define MPU6050_GYOFFSET 0
  //#define MPU6050_GZOFFSET 0
  //#define MPU6050_GXGAIN 131.072 // FS_SEL = 0, +/-250degree/s, MPU6050_GYRO_FS_250
  //#define MPU6050_GYGAIN 131.072 // FS_SEL = 0, +/-250degree/s, MPU6050_GYRO_FS_250
  //#define MPU6050_GZGAIN 131.072 // FS_SEL = 0, +/-250degree/s, MPU6050_GYRO_FS_250
  //#define MPU6050_GXGAIN 65.536 // FS_SEL = 1, +/-500degree/s, MPU6050_GYRO_FS_500
  //#define MPU6050_GYGAIN 65.536 // FS_SEL = 1, +/-500degree/s, MPU6050_GYRO_FS_500
  //#define MPU6050_GZGAIN 65.536 // FS_SEL = 1, +/-500degree/s, MPU6050_GYRO_FS_500
  //#define MPU6050_GXGAIN 32.768 // FS_SEL = 2, +/-1000degree/s, MPU6050_GYRO_FS_1000
  //#define MPU6050_GYGAIN 32.768 // FS_SEL = 2, +/-1000degree/s, MPU6050_GYRO_FS_1000
  //#define MPU6050_GZGAIN 32.768 // FS_SEL = 2, +/-1000degree/s, MPU6050_GYRO_FS_1000
*/