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
#include "dma.h" 
#include "rc_manager.h"
#include "PID_Controller.h"
#include "telemetry.h"
#include "motor_control.h"
#include "tim.h"
#include "nrf24l01p.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h> 
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
static inline void MX_IWDG_Init(void) { /* no-op */ }
static inline void Watchdog_Refresh(void) { /* no-op */ }

// dt helper — call this once per loop iteration
static inline float get_dt_seconds(void)
{
    static uint32_t last_tick = 0;
    uint32_t now  = __HAL_TIM_GET_COUNTER(&htim5);
    uint32_t diff = (now >= last_tick)
                  ? (now - last_tick)
                  : (0xFFFFFFFFU - last_tick + now + 1U); // overflow wrap
    last_tick = now;
    return (float)diff * 1e-6f; // µs → seconds
}
/* USER CODE END PV */

void SystemClock_Config(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();           // ← must be before UART
    MX_TIM2_Init();
    MX_TIM5_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();
    MX_IWDG_Init();
    Motor_PWM_Init();

    //Receiver_Init();
    Telemetry_Init();
    IMU_Init();

    // WHO_AM_I check
    uint8_t who = 0;
    HAL_I2C_Mem_Read(&hi2c1, (0x68 << 1), 0x75, 1, &who, 1, 100);
    char dbg[32];
    int dlen = snprintf(dbg, sizeof(dbg), "MPU WHO_AM_I: 0x%02X\r\n", who);
    //HAL_UART_Transmit(&huart1, (uint8_t*)dbg, dlen, 100);

    IMU_Calibrate(500);

    // Motor arming sequence - hold idle until stick gesture detected
    //Motor_Arm();

    // Main loop: compute dt once and pass into IMU and control loop
    HAL_TIM_Base_Start(&htim5);  // free-running, no IRQ
    get_dt_seconds();
    //HAL_UART_Transmit(&huart1, (uint8_t*)"entering while loop\r\n", 20, 100);


    while (1)
    {

        // RX RC packets from remote
        //Receiver_Process();

        // Switch to RX and receive telemetry briefly
        //nrf24l01p_switch_to_rx();
        //Telemetry_ReceiveCheck();

        // Switch back to TX for next telemetry send
        //nrf24l01p_switch_to_tx();

        float dt = get_dt_seconds();

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

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        Telemetry_TxCpltCallback();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (Receiver_IsIrqPin(GPIO_Pin))
    {
        Receiver_OnIrq();
        HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
    }
}
/* USER CODE END 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
