/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "IMU.h"
#include "receiver.h"
#include "tim.h"
#include "rc_manager.h"
#include "PID_Controller.h"
#include "motor_control.h"
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */
#include <math.h>  // add this
/* USER CODE BEGIN PV */
static inline void MX_IWDG_Init(void) { /* no-op */ }
static inline void Watchdog_Refresh(void) { /* no-op */ }
/* USER CODE END PV */

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_IWDG_Init();
    Motor_PWM_Init();

    Receiver_Init();
    IMU_Init();

    // WHO_AM_I check
    uint8_t who = 0;
    HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x75, 1, &who, 1, 100);
    char dbg[32];
    int dlen = snprintf(dbg, sizeof(dbg), "MPU WHO_AM_I: 0x%02X\r\n", who);
    HAL_UART_Transmit(&huart1, (uint8_t*)dbg, dlen, 100);

    IMU_Calibrate(500);

    // Motor arming sequence - hold idle until stick gesture detected
    //Motor_Arm();

    // Main loop: compute dt once and pass into IMU and control loop
    uint32_t last_time_ms = HAL_GetTick();
    while (1)
    {
        Receiver_Process();
        RC_Update();

        uint32_t now = HAL_GetTick();
        double dt = (now - last_time_ms) / 1000.0;
        if (dt <= 0.0) dt = 0.001;
        if (dt > 0.05) dt = 0.05;
        last_time_ms = now;

        IMUData imu = IMU_GetAngles(dt);

        dataPacket controller;
        controller.pitch = 0.0;
        controller.roll  = 0.0;
        controller.yaw   = 0.0;
        controller.throttle = 550.0;
        controller.kill = 0;

            flight_control_loop(imu, controller, dt);

        /*if (!RC_IsKillActive())
        {
            flight_control_loop(imu, controller, dt);
        }
        else
        {
            // Kill switch active — zero all motors
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);
        }*/
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Error_Handler();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (Receiver_IsIrqPin(GPIO_Pin))
    {
        Receiver_OnIrq();
        HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
