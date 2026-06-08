#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include "PID_Controller.h"

// Maximum mixer output (units used by PID/mixer). Adjust to match PID tuning.
#define MAX_MIX_OUTPUT 1000.0

typedef struct{
    double motor_front_left;
    double motor_front_right;
    double motor_back_left;
    double motor_back_right;
} MotorControl;

MotorControl motor_mixing(double throttle, double pitch_output, double roll_output, double yaw_output);
int16_t pwm_from_output(double output);
void Motor_PWM_Init(void);
void Motor_Arm(void);

void motor_apply_outputs(MotorControl *mc);
void Motor_PrintTelemetry(MotorControl data);

#endif
