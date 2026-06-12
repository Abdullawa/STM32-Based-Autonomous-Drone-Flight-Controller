#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "main.h"
#include "IMU.h"
#include "rc_manager.h"

typedef struct {
    float kp;
    float ki;
    float kd;
    float prev_measurement;
    float integral;
    float out_max;
} PID;

void   PID_Init(PID *pid, float kp, float ki, float kd, float out_max);
float  PID_Update(PID *pid, float setpoint, float measurement, float dt);
void   flight_control_loop(IMUData imu_data, dataPacket controller, float dt);

#endif