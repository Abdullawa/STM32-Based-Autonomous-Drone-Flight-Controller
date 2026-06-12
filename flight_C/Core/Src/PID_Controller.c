#include "PID_Controller.h"
#include "IMU.h"
#include <stdio.h>
#include <math.h>
#include "motor_control.h"
#include "tim.h"
#include "telemetry.h"
#include <stdlib.h>

// -----------------------------------------------------------------------
// GAIN TABLE
// Start with ki=kd=0, tune kp only until stable, then add kd.
// -----------------------------------------------------------------------
float rate_kp  = 0.75;
float rate_ki  = 0.0;
float rate_kd  = 0.0;

float angle_kp = 2.0;
float angle_ki = 0.0;
float angle_kd = 0.0;

void PID_Init(PID *pid, float kp, float ki, float kd, float out_max)
{
    pid->kp               = kp;
    pid->ki               = ki;
    pid->kd               = kd;
    pid->out_max          = out_max;
    pid->prev_measurement = 0.0;
    pid->integral         = 0.0;
}

float PID_Update(PID *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // Integral with anti-windup
    pid->integral += error * dt;
    if (pid->ki != 0.0)
    {
        float int_limit = pid->out_max / fabsf(pid->ki);
        if (pid->integral >  int_limit) pid->integral =  int_limit;
        if (pid->integral < -int_limit) pid->integral = -int_limit;
    }

    // Derivative on measurement — no kick on setpoint change
    float derivative = 0.0;
    if (dt > 0.0)
        derivative = -(measurement - pid->prev_measurement) / dt;
    pid->prev_measurement = measurement;

    float output = (pid->kp * error)
                  + (pid->ki * pid->integral)
                  + (pid->kd * derivative);

    if (output >  pid->out_max) output =  pid->out_max;
    if (output < -pid->out_max) output = -pid->out_max;

    return output;
}

void flight_control_loop(IMUData imu_data, dataPacket controller, float dt)
{
    static PID pitch_angle_pid;
    static PID roll_angle_pid;
    static PID pitch_rate_pid;
    static PID roll_rate_pid;
    static PID yaw_rate_pid;
    static bool pid_initialized = false;

    if (!pid_initialized)
    {
        PID_Init(&pitch_angle_pid, angle_kp, angle_ki, angle_kd, 50.0);
        PID_Init(&roll_angle_pid,  angle_kp, angle_ki, angle_kd, 50.0);
        PID_Init(&pitch_rate_pid,  rate_kp,  rate_ki,  rate_kd,  300.0);
        PID_Init(&roll_rate_pid,   rate_kp,  rate_ki,  rate_kd,  300.0);
        PID_Init(&yaw_rate_pid,    rate_kp,  rate_ki,  rate_kd,  300.0);
        pid_initialized = true;
    }

    // Outer loop: angle error -> rate setpoint
    float pitch_rate_setpoint = PID_Update(&pitch_angle_pid, controller.pitch, imu_data.pitch, dt);
    float roll_rate_setpoint  = PID_Update(&roll_angle_pid,  controller.roll,  imu_data.roll,  dt);

    // Inner loop: rate error -> motor command
    float pitch_cmd = PID_Update(&pitch_rate_pid, pitch_rate_setpoint, imu_data.pitch_rate, dt);
    float roll_cmd  = PID_Update(&roll_rate_pid,  roll_rate_setpoint,  imu_data.roll_rate,  dt);
    float yaw_cmd   = PID_Update(&yaw_rate_pid,   controller.yaw,      imu_data.yaw_rate,   dt);

    // Throttle mapping
    float throttle_mapped = controller.throttle;
    if (throttle_mapped < 0.0)            throttle_mapped = 0.0;
    if (throttle_mapped > MAX_MIX_OUTPUT) throttle_mapped = MAX_MIX_OUTPUT;

    MotorControl mc = motor_mixing(throttle_mapped, pitch_cmd, roll_cmd, yaw_cmd);
    motor_apply_outputs(&mc);

    // ------------------------------------------------------------------
    // TELEMETRY over NRF24 at 5Hz (200ms)
    // Non-blocking; packed into 12-byte NRF24 payload
    // ------------------------------------------------------------------
    Telemetry_Send(imu_data.pitch, imu_data.roll, 
               imu_data.pitch_rate, imu_data.roll_rate,
               imu_data.yaw_rate, controller.throttle);
}