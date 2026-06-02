/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "IMU.h"
#include "receiver.h"
#include <stdio.h>
#include <stdlib.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// safety and sequencing
#define LINK_LOSS_TIMEOUT_MS 200
static uint32_t last_packet_ms = 0;
static bool latched_kill = false;
static uint8_t last_seq = 0;
static bool seq_initialized = false;

/* Watchdog: HAL IWDG driver not present in this project tree. */
/* Provide no-op stubs so the code builds. If you enable IWDG in CubeMX,
   replace these with the HAL IWDG implementation. */
static inline void MX_IWDG_Init(void)
{
  /* no-op: IWDG not available */
}

static inline void Watchdog_Refresh(void)
{
  /* no-op: IWDG not available */
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
 
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  Receiver_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Service deferred receiver work (read SPI payloads outside IRQ)
    Receiver_Process();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

    dataPacket controller;

    if (Receiver_ReadPacket(&controller) && controller.sync == 0xAA && controller.type == 0x01)
    {
      // sequence handling (simple acceptance)
      if (!seq_initialized || controller.seq != last_seq)
      {
        seq_initialized = true;
        last_seq = controller.seq;
      }

      // update last received timestamp
      last_packet_ms = HAL_GetTick();

      // handle kill switch
      if (controller.kill)
      {
        latched_kill = true;
      }

      char msg[128];
      int len = snprintf(msg, sizeof(msg),
                         "seq:%u pitch:%d roll:%d yaw:%d throttle:%d kill:%u\r\n",
                         controller.seq,
                         controller.pitch,
                         controller.roll,
                         controller.yaw,
                         controller.throttle,
                         controller.kill);

      HAL_UART_Transmit(&huart1, (uint8_t *)msg, (uint16_t)len, 100);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    }

    // Link-loss failsafe
    if ((HAL_GetTick() - last_packet_ms) > LINK_LOSS_TIMEOUT_MS)
    {
      latched_kill = true;
    }

    // LED indicates latched kill
    if (latched_kill)
      HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_SET);
    else
      HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

    // refresh watchdog only when not latched kill (or refresh always if prefer automatic restart)
    if (!latched_kill)
    {
      Watchdog_Refresh();
    }

    HAL_Delay(1);
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (Receiver_IsIrqPin(GPIO_Pin))
  {
    Receiver_OnIrq();
    HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
  }
}

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
