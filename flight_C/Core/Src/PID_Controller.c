#include "PID_Controller.h"
#include "IMU.h"
#include <stdio.h>
#include <math.h>
#include "rc_manager.h"
#include "motor_control.h"
#include "tim.h"
#include <stdlib.h>  // add this
double rate_kp = 2.0;
double rate_ki = 0.0;
double rate_kd = 0.0;

double angle_kp = 8.0;
double angle_ki = 0.0;
double angle_kd = 0.0;






void PID_Init(PID *pid, double kp, double ki, double kd, double out_max)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->out_max = out_max;
  pid->prev_measurement = 0.0;
  pid->integral = 0.0;
}

double PID_update(PID *pid, double setpoint, double measurement, double dt)
{
    double error = setpoint - measurement;

    // integrate (with anti-windup clamp so ki * integral can't exceed out_max)
    pid->integral += error * dt;
    if (pid->ki != 0.0)
    {
        double int_limit = pid->out_max / fabs(pid->ki);
        if (pid->integral > int_limit) pid->integral = int_limit;
        if (pid->integral < -int_limit) pid->integral = -int_limit;
    }

    // derivative (differentiate measurement to avoid derivative kick)
    double derivative = 0.0;
    if (dt > 0.0)
    {
        derivative = -(measurement - pid->prev_measurement) / dt;
    }
    pid->prev_measurement = measurement;

    // output
    double output =
        (pid->kp * error) +
        (pid->ki * pid->integral) +
        (pid->kd * derivative);

    // clamp output
    if (output > pid->out_max) output = pid->out_max;
    if (output < -pid->out_max) output = -pid->out_max;

    return output;
}

void flight_control_loop(IMUData imu_data, dataPacket controller, double dt)
{
    // Example control loop using PID controllers for pitch and roll
    static PID pitch_angle_pid;
    static PID roll_angle_pid;
    static PID pitch_rate_pid;
    static PID roll_rate_pid;
    static PID yaw_rate_pid;

    static bool pid_initialized = false;

    if (!pid_initialized)
    {
        PID_Init(&pitch_angle_pid, angle_kp, angle_ki, angle_kd, 50.0); // max 50 deg/s outer-loop
        PID_Init(&roll_angle_pid, angle_kp, angle_ki, angle_kd, 50.0); // max 50 deg/s outer-loop
        PID_Init(&pitch_rate_pid, rate_kp, rate_ki, rate_kd, 300.0); // max 30 degrees/second output
        PID_Init(&roll_rate_pid, rate_kp, rate_ki, rate_kd, 300.0); // max 30 degrees/second output
        PID_Init(&yaw_rate_pid, rate_kp, rate_ki, rate_kd, 300.0); // max 30 degrees/second output
        pid_initialized = true;
    }

    // Angle control (outer loop)
    double pitch_rate_setpoint = PID_update(
    &pitch_angle_pid,
    controller.pitch,
    imu_data.pitch,
    dt
);

double roll_rate_setpoint = PID_update(
    &roll_angle_pid,
    controller.roll,
    imu_data.roll,
    dt
);

    // Rate control (inner loop)
    double pitch_cmd = PID_update(
    &pitch_rate_pid,
    pitch_rate_setpoint,
    imu_data.pitch_rate,
    dt
);

double roll_cmd = PID_update(
    &roll_rate_pid,
    roll_rate_setpoint,
    imu_data.roll_rate,
    dt
);

double yaw_cmd = PID_update(
    &yaw_rate_pid,
    controller.yaw,
    imu_data.yaw_rate,
    dt
);


    // Use pitch_output and roll_output to adjust motor speeds accordingly
    // Map throttle from controller to mixer throttle (assume controller.throttle ~ -2000..2000)
    double throttle_mapped = ((double)controller.throttle + 2000.0) / 4000.0 * MAX_MIX_OUTPUT;
    if (throttle_mapped < 0.0) throttle_mapped = 0.0;
    if (throttle_mapped > MAX_MIX_OUTPUT) throttle_mapped = MAX_MIX_OUTPUT;

    MotorControl mc = motor_mixing(throttle_mapped, pitch_cmd, roll_cmd, yaw_cmd);

    static uint32_t telem_last = 0;
        if ((HAL_GetTick() - telem_last) > 100) { // 10 Hz is readable, 200ms is too slow
    telem_last = HAL_GetTick();

    // scale everything to integers for snprintf (no %f on STM32 without extra linker flags)
    int p    = (int)round(imu_data.pitch * 100.0);
    int r    = (int)round(imu_data.roll  * 100.0);
    int y    = (int)round(imu_data.yaw   * 100.0);
    int pr   = (int)round(imu_data.pitch_rate * 100.0);
    int rr   = (int)round(imu_data.roll_rate  * 100.0);
    int yr   = (int)round(imu_data.yaw_rate   * 100.0);
    int pc   = (int)round(pitch_cmd  * 100.0);
    int rc_v = (int)round(roll_cmd   * 100.0);
    int yc   = (int)round(yaw_cmd    * 100.0);
    int thr  = (int)round(throttle_mapped * 100.0);
    int dt_us = (int)round(dt * 1000000.0); // dt in microseconds

    char msg[200];
    int len = snprintf(msg, sizeof(msg),
        "ANG  P:%d.%02d R:%d.%02d Y:%d.%02d | "
        "RATE pr:%d.%02d rr:%d.%02d yr:%d.%02d | "
        "CMD  pc:%d.%02d rc:%d.%02d yc:%d.%02d thr:%d.%02d | "
        "PWM  FL:%d FR:%d BL:%d BR:%d | "
        "dt:%dus\r\n",
        p/100,  abs(p%100),
        r/100,  abs(r%100),
        y/100,  abs(y%100),
        pr/100, abs(pr%100),
        rr/100, abs(rr%100),
        yr/100, abs(yr%100),
        pc/100, abs(pc%100),
        rc_v/100, abs(rc_v%100),
        yc/100, abs(yc%100),
        thr/100, abs(thr%100),
        pwm_from_output(mc.motor_front_left),
        pwm_from_output(mc.motor_front_right),
        pwm_from_output(mc.motor_back_left),
        pwm_from_output(mc.motor_back_right),
        dt_us);

    HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 50);

    // After your existing human-readable print, add:
char csv[128];
int csv_len = snprintf(csv, sizeof(csv),
    "/*%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d,%d,%d,%d*/\r\n",
    p/100,  abs(p%100),   // pitch
    r/100,  abs(r%100),   // roll
    y/100,  abs(y%100),   // yaw
    pr/100, abs(pr%100),  // pitch rate
    rr/100, abs(rr%100),  // roll rate
    yr/100, abs(yr%100),  // yaw rate
    pwm_from_output(mc.motor_front_left),
    pwm_from_output(mc.motor_front_right),
    pwm_from_output(mc.motor_back_left),
    pwm_from_output(mc.motor_back_right));
HAL_UART_Transmit(&huart1, (uint8_t*)csv, csv_len, 50);
}

    motor_apply_outputs(&mc);
    //Motor_PrintTelemetry(mc);
    //IMU_PrintTelemetry(imu_data);

    // (Optional) convert other motors to pwms here when additional timers/channels are available


}

