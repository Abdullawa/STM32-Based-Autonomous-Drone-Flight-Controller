#include "PID_Controller.h"
#include "IMU.h"
#include <stdio.h>
#include <math.h>
#include "rc_manager.h"

double rate_kp = 0.0;
double rate_ki = 0.0;
double rate_kd = 0.0;

double angle_kp = 0.0;
double angle_ki = 0.0;
double angle_kd = 0.0;




double tuned_output(double max,double input)
{
  if (input > max)
    return max;
  else if (input < -max)
    return -max;
  else
    return input;

}


void PID_Init(PID *pid, double kp, double ki, double kd, double out_max)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->out_max = out_max;
  pid->prev_error = 0.0;
  pid->integral = 0.0;
}

double PID_update(PID *pid, double setpoint, double measurement, double dt)
{
  double error = setpoint - measurement;
  double integral = pid->integral + error * dt;
  if (integral < pid->out_max && integral > -pid->out_max) {
    pid->integral = integral;
  } 
  double derivative = (error - pid->prev_error) / dt;
  pid->prev_error = error;
  double output = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
  return tuned_output(pid->out_max, output);
}

void flight_control_loop(IMUData imu_data, dataPacket controller)
{
   // Calculate time delta for PID update
   static uint32_t last_time_ms = 0;
    uint32_t current_time_ms = HAL_GetTick();
    double dt = (current_time_ms - last_time_ms) / 1000.0; // convert ms to seconds
    last_time_ms = current_time_ms;

    // Example control loop using PID controllers for pitch and roll
    static PID pitch_angle_pid;
    static PID roll_angle_pid;
    static PID pitch_rate_pid;
    static PID roll_rate_pid;
    static PID yaw_rate_pid;

    static bool pid_initialized = false;

    if (!pid_initialized)
    {
        PID_Init(&pitch_angle_pid, angle_kp, angle_ki, angle_kd, 30.0); // max 30 degrees output
        PID_Init(&roll_angle_pid, angle_kp, angle_ki, angle_kd, 30.0); // max 30 degrees output
        PID_Init(&pitch_rate_pid, rate_kp, rate_ki, rate_kd, 30.0); // max 30 degrees/second output
        PID_Init(&roll_rate_pid, rate_kp, rate_ki, rate_kd, 30.0); // max 30 degrees/second output
        PID_Init(&yaw_rate_pid, rate_kp, rate_ki, rate_kd, 30.0); // max 30 degrees/second output
        pid_initialized = true;
    }

    // Angle control (outer loop)
    double pitch_angle_output = PID_update(&pitch_angle_pid, controller.pitch, imu_data.pitch, dt);
    double roll_angle_output = PID_update(&roll_angle_pid, controller.roll, imu_data.roll, dt); 

    // Rate control (inner loop)
    double pitch_cmd = PID_update(&pitch_rate_pid, pitch_angle_output, imu_data.pitch_rate, dt);
    double roll_cmd = PID_update(&roll_rate_pid, roll_angle_output, imu_data.roll_rate, dt);
    double yaw_cmd = PID_update(&yaw_rate_pid, controller.yaw, imu_data.yaw_rate, dt);


    // Use pitch_output and roll_output to adjust motor speeds accordingly
}

