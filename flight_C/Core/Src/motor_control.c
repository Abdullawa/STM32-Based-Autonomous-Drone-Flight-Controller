#include "motor_control.h"
#include <math.h>
#include "tim.h"
#include <stdio.h>

MotorControl motor_mixing(float throttle, float pitch_output, float roll_output, float yaw_output)
{
    MotorControl output;

    // Basic mixer for X configuration
    output.motor_front_left  = throttle + pitch_output + roll_output - yaw_output; // FL
    output.motor_front_right = throttle + pitch_output - roll_output + yaw_output; // FR
    output.motor_back_left   = throttle - pitch_output + roll_output + yaw_output; // BL
    output.motor_back_right  = throttle - pitch_output - roll_output - yaw_output; // BR
    // Proportional rescaling mixer to preserve ratios while keeping outputs in [0, MAX_MIX_OUTPUT]
    float min_out = output.motor_front_left;
    if (output.motor_front_right < min_out) min_out = output.motor_front_right;
    if (output.motor_back_left  < min_out) min_out = output.motor_back_left;
    if (output.motor_back_right < min_out) min_out = output.motor_back_right;

    // If any output is negative, shift all up so the minimum is zero
    if (min_out < 0.0)
    {
        float shift = -min_out;
        output.motor_front_left  += shift;
        output.motor_front_right += shift;
        output.motor_back_left   += shift;
        output.motor_back_right  += shift;
    }

    // If any output exceeds MAX_MIX_OUTPUT, scale all down proportionally
    float max_out = output.motor_front_left;
    if (output.motor_front_right > max_out) max_out = output.motor_front_right;
    if (output.motor_back_left  > max_out) max_out = output.motor_back_left;
    if (output.motor_back_right > max_out) max_out = output.motor_back_right;

    if (max_out > MAX_MIX_OUTPUT)
    {
        float scale = MAX_MIX_OUTPUT / max_out;
        output.motor_front_left  *= scale;
        output.motor_front_right *= scale;
        output.motor_back_left   *= scale;
        output.motor_back_right  *= scale;
    }

    return output;
}

// Wait for stick gesture (throttle low + yaw right held for 2s) while holding motors at idle
void Motor_Arm(void)
{
    // Ensure PWM running and motors at idle
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);

    uint32_t gesture_start = 0;
    const uint32_t required_hold_ms = 2000;

    // Wait for arm gesture (or proceed if no RC packets available)
    while (1)
    {
        dataPacket pkt;
        if (RC_GetLatestPacket(&pkt))
        {
            // thresholds assume controller throttle/yaw range around -2000..2000
            bool throttle_low = (pkt.throttle < -1500);
            bool yaw_right    = (pkt.yaw > 1500);

            if (throttle_low && yaw_right)
            {
                if (gesture_start == 0) gesture_start = HAL_GetTick();
                if ((HAL_GetTick() - gesture_start) >= required_hold_ms)
                {
                    break; // armed
                }
            }
            else
            {
                gesture_start = 0;
            }
        }

        HAL_Delay(20);
    }
}

int16_t pwm_from_output(float output)
{
    if (output > MAX_MIX_OUTPUT) output = MAX_MIX_OUTPUT;
    if (output < 0.0) output = 0.0;

    float pwm = 1000.0 + (output / MAX_MIX_OUTPUT) * 1000.0;
    if (pwm > 2000.0) pwm = 2000.0;
    if (pwm < 1000.0) pwm = 1000.0;

    return (int16_t)round(pwm);
}

// Apply motor outputs to hardware PWM channels.
// Currently drives front-left motor on TIM2_CH2 (PA1). Expand when more channels available.
void Motor_PWM_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);
    HAL_Delay(3000);
}

void motor_apply_outputs(MotorControl *mc)
{
    if (mc == NULL) return;

    int16_t pwm_fl = pwm_from_output(mc->motor_front_left);
    int16_t pwm_fr = pwm_from_output(mc->motor_front_right);
    int16_t pwm_bl = pwm_from_output(mc->motor_back_left);
    int16_t pwm_br = pwm_from_output(mc->motor_back_right);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)pwm_fl); // PA0 - Front Left
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, (uint32_t)pwm_fr); // PA1 - Front Right
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (uint32_t)pwm_bl); // PA2 - Back Left
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint32_t)pwm_br); // PA3 - Back Right
}



void Motor_PrintTelemetry(MotorControl data)
{
    char msg[128];

    int16_t pwm_fl = pwm_from_output(data.motor_front_left);
    int16_t pwm_fr = pwm_from_output(data.motor_front_right);
    int16_t pwm_bl = pwm_from_output(data.motor_back_left);
    int16_t pwm_br = pwm_from_output(data.motor_back_right);

    int len = snprintf(msg, sizeof(msg),
        "PWM FL:%d FR:%d BL:%d BR:%d\r\n",
        pwm_fl,
        pwm_fr,
        pwm_bl,
        pwm_br);

    //HAL_UART_Transmit(&huart1, (uint8_t*)msg, (uint16_t)len, 100);
}