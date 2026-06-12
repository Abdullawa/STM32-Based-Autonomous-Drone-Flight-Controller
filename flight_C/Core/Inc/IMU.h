#ifndef IMU_H
#define IMU_H

#include "main.h"

typedef struct {
    float pitch;
    float roll;
    float yaw;
    float yaw_rate;
    float pitch_rate;
    float roll_rate;
} IMUData;

void    IMU_Init(void);
void    MPU6050_Init(void);
void    MPU6050_Read(int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz);
void    IMU_Calibrate(int samples);
IMUData IMU_GetAngles(float dt);
void    IMU_PrintTelemetry(IMUData data);



#endif