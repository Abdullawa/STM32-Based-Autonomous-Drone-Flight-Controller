#ifndef IMU_H
#define IMU_H

#include "main.h"

typedef struct {
    double pitch;
    double roll;
    double yaw;
    double yaw_rate;
    double pitch_rate;
    double roll_rate;
} IMUData;

void    IMU_Init(void);
void    MPU6050_Init(void);
void    MPU6050_Read(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);
void    HMC5883L_Init(void);
void    HMC5883L_Read(int16_t *mx, int16_t *my, int16_t *mz);
void    IMU_Calibrate(int samples);
IMUData IMU_GetAngles(void);
void    IMU_PrintTelemetry(IMUData data);

#endif