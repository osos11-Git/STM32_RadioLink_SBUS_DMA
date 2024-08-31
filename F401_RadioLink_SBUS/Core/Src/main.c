/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t failsafe_status;
uint16_t SBUS_footer;
uint8_t buf[25];
uint16_t CH[18];

uint8_t pre_catch_buf[1];
int pre_init_counter = 0;
bool start_counting = false;
bool catch_signal = false;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (pre_catch_buf[0] == 0x0F) start_counting = true;

	if (pre_init_counter < 24 && start_counting == true)
		{
			pre_init_counter++;
			return;
		}
	else if (catch_signal == false)
		{
			catch_signal = true;
			HAL_UART_AbortReceive(&huart2);
			HAL_UART_Receive_DMA(&huart2, buf, 25);
			return;
		}

	if (buf[0] == 0x0F)
		{
			CH[0] = (buf[1] >> 0 | (buf[2] << 8)) & 0x07FF;
			CH[1] = (buf[2] >> 3 | (buf[3] << 5)) & 0x07FF;
			CH[2] = (buf[3] >> 6 | (buf[4] << 2) | buf[5] << 10) & 0x07FF;
			CH[3] = (buf[5] >> 1 | (buf[6] << 7)) & 0x07FF;
			CH[4] = (buf[6] >> 4 | (buf[7] << 4)) & 0x07FF;
			CH[5] = (buf[7] >> 7 | (buf[8] << 1) | buf[9] << 9) & 0x07FF;
			CH[6] = (buf[9] >> 2 | (buf[10] << 6)) & 0x07FF;
			CH[7] = (buf[10] >> 5 | (buf[11] << 3)) & 0x07FF;
			CH[8] = (buf[12] << 0 | (buf[13] << 8)) & 0x07FF;
			CH[9] = (buf[13] >> 3 | (buf[14] << 5)) & 0x07FF;
			CH[10] = (buf[14] >> 6 | (buf[15] << 2) | buf[16] << 10) & 0x07FF;
			CH[11] = (buf[16] >> 1 | (buf[17] << 7)) & 0x07FF;
			CH[12] = (buf[17] >> 4 | (buf[18] << 4)) & 0x07FF;
			CH[13] = (buf[18] >> 7 | (buf[19] << 1) | buf[20] << 9) & 0x07FF;
			CH[14] = (buf[20] >> 2 | (buf[21] << 6)) & 0x07FF;
			CH[15] = (buf[21] >> 5 | (buf[22] << 3)) & 0x07FF;

			if (buf[23] & (1 << 0))
				{
					CH[16] = 1;
				}
			else
				{
					CH[16] = 0;
				}

			if (buf[23] & (1 << 1))
				{
					CH[17] = 1;
				}
			else
				{
					CH[17] = 0;
				}

			// Failsafe
			failsafe_status = SBUS_SIGNAL_OK;
			if (buf[23] & (1 << 2))
				{
					failsafe_status = SBUS_SIGNAL_LOST;
				}

			if (buf[23] & (1 << 3))
				{
					failsafe_status = SBUS_SIGNAL_FAILSAFE;
				}

			SBUS_footer = buf[24];

		}
	else
		{
			catch_signal = false;
			start_counting = false;
			catch_signal = false;
			pre_init_counter = 0;
			HAL_UART_AbortReceive(&huart2);
			HAL_UART_Receive_DMA(&huart2, pre_catch_buf, 1);
			return;
		}
}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart2, pre_catch_buf, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
