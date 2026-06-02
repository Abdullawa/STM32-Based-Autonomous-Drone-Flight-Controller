#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "main.h"

typedef struct {
  double kp;
  double ki;
  double kd;
  double prev_error;
  double integral;
  double out_max;
} PID;

double tuned_output(double max, double input);
void PID_Init(PID *pid, double kp, double ki, double kd, double out_max);
double PID_update(PID *pid, double setpoint, double measurement, double dt);
void flight_control_loop(IMUData imu_data, dataPacket controller);




#endif