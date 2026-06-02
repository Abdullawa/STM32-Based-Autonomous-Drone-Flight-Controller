#include "main.h"
#include "IMU.h"
#include <stdio.h>
#include <math.h>

#define MPU6050_ADDR  0xD0
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B

static double gz_bias = 0.0;

void IMU_Init(void)
{
  MPU6050_Init();
}

void MPU6050_Init(void)
{
  uint8_t data = 0x00;
  // Try a few times to write power management register
  for (int i = 0; i < 3; i++)
  {
    if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &data, 1, 100) == HAL_OK)
      break;
    HAL_Delay(10);
  }
}

void MPU6050_Read(int16_t *ax, int16_t *ay, int16_t *az,
                  int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t buf[14];
  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H, 1, buf, 14, 100) != HAL_OK)
  {
    // On failure, return zeroed outputs
    *ax = *ay = *az = *gx = *gy = *gz = 0;
    return;
  }

  *ax = (int16_t)(buf[0]  << 8 | buf[1]);
  *ay = (int16_t)(buf[2]  << 8 | buf[3]);
  *az = (int16_t)(buf[4]  << 8 | buf[5]);
  *gx = (int16_t)(buf[8]  << 8 | buf[9]);
  *gy = (int16_t)(buf[10] << 8 | buf[11]);
  *gz = (int16_t)(buf[12] << 8 | buf[13]);
}

IMUData IMU_Calibrate(int samples)
{
  IMUData offset = {0, 0, 0};
  int16_t ax, ay, az, gx, gy, gz;
  double gz_sum = 0;

  for (int i = 0; i < samples; i++)
  {
    MPU6050_Read(&ax, &ay, &az, &gx, &gy, &gz);

    offset.pitch += atan2((double)ay, (double)az) * 180.0 / M_PI;
    offset.roll  += atan2((double)ax, (double)az) * 180.0 / M_PI;
    gz_sum       += gz;

    HAL_Delay(1);
  }

  offset.pitch /= samples;
  offset.roll  /= samples;
  gz_bias       = gz_sum / samples; // store bias globally

  return offset;
}

IMUData IMU_GetAngles(float dt)
{
  static double pitch = 0.0;
  static double roll  = 0.0;

  int16_t ax, ay, az, gx, gy, gz;

  MPU6050_Read(&ax, &ay, &az, &gx, &gy, &gz);

  double accel_pitch = atan2((double)ay, (double)az) * 180.0 / M_PI;
  double accel_roll  = atan2((double)ax, (double)az) * 180.0 / M_PI;

  double gyro_pitch = (gx / 131.0) * dt;
  double gyro_roll  = (gy / 131.0) * dt;

  pitch = 0.98 * (pitch + gyro_pitch) + 0.02 * accel_pitch;
  roll  = 0.98 * (roll  + gyro_roll)  + 0.02 * accel_roll;

  double yaw_rate = (gz - gz_bias) / 131.0; // bias subtracted

  IMUData data = {pitch, roll, yaw_rate};
  return data;
}
