#ifndef IMU_H
#define IMU_H

#include "main.h"

typedef struct {
  double pitch;
  double roll;
  double yaw_rate;
} IMUData;

void IMU_Init(void);
void MPU6050_Init(void);
void MPU6050_Read(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
IMUData IMU_Calibrate(int samples);
IMUData IMU_GetAngles(float dt);

#endif
