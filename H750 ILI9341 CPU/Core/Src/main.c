/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "quadspi.h"
#include "rng.h"
#include "sdmmc.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "H750 ILI9341 CPU.h"
#include "Hangle.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_SYSTICK_Callback(void)
{
	static int cnt = -1000;

	cnt++;
	if(cnt > 500)
	{
		cnt = 0;

		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
}

void Demo_Line(void)
{
	uint16_t i, x1, y1, x2, y2, color, w, h;
	uint32_t rn;

	w = LCD_GetWidth();
	h = LCD_GetHeight();

	LCD_Clear(WHITE);

	for(i = 0; i < 5000; i++)
	{
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		x1 = rn % w;
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		x2 = rn % w;
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		y1 = rn % h;
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		y2 = rn % h;
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		color = rn % 0x10000;

		LCD_draw_line(x1, y1, x2, y2, color);
	}
}

void Demo_Circle(void)
{
	uint16_t i, x, y, r, color, w, h;
	uint32_t rn;

	w = LCD_GetWidth();
	h = LCD_GetHeight();

	LCD_Clear(WHITE);

	for(i = 0; i < 5000; i++)
	{
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		x = rn % w;
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		y = rn % h;
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		r = rn % 100;
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		color = rn % 0x10000;

		LCD_draw_circle(x, y, r, color);
	}
}

void Demo_Fill(void)
{
	uint16_t i, x1, y1, x2, y2, color, w, h;
	uint32_t rn;

	w = LCD_GetWidth();
	h = LCD_GetHeight();

	LCD_Clear(WHITE);

	for(i = 0; i < 5000; i++)
	{
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		x1 = rn % w;
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		x2 = rn % w;
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		y1 = rn % h;
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		y2 = rn % h;
		HAL_RNG_GenerateRandomNumber(&hrng, &rn);
		color = rn % 0x10000;

		LCD_fill(x1, y1, x2, y2, color);
	}
}

void Demo_Text(void)
{
	LCD_Clear(YELLOW);

	LCD_puts26x48(0, 0,   (uint8_t *)"Hello World.", TFT_STRING_MODE_BACKGROUND);
	LCD_puts18x32(0, 48,  (uint8_t *)"Hello World.", TFT_STRING_MODE_BACKGROUND);
	LCD_puts14x24(0, 80,  (uint8_t *)"Hello World.", TFT_STRING_MODE_BACKGROUND);
	LCD_puts8x16( 0, 104, (uint8_t *)"Hello World.", TFT_STRING_MODE_BACKGROUND);

	LCD_SetTextColor(BLUE);
	LCD_SetBackgroundColor(GREEN);
	LCD_puts14x24(0, 120,  (uint8_t *)"Blue Text.", TFT_STRING_MODE_BACKGROUND);
	LCD_puts14x24(0, 150,  (uint8_t *)"No Background Text.", TFT_STRING_MODE_NO_BACKGROUND);

	DisplayHangle(0, 180, "한글입니다. ABCDEFG");
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	int pos = 0;

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
  MX_QUADSPI_Init();
  MX_SDMMC1_SD_Init();
  MX_USART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_FMC_Init();
  MX_SPI2_Init();
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();

  volatile uint32_t id = LCD_Read_ID();
  (void)id;

  LCD_direction(1);

  LCD_Clear(RED);
  HAL_Delay(500);
  LCD_Clear(GREEN);
  HAL_Delay(500);
  LCD_Clear(BLUE);
  HAL_Delay(500);

  Demo_Line();
  HAL_Delay(500);
  Demo_Circle();
  HAL_Delay(500);
  Demo_Fill();
  HAL_Delay(500);
  Demo_Text();
  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	LCD_scroll(pos);
    HAL_Delay(50);

  	pos++;
  	if(pos >= LCD_GetWidth()) pos = 0;

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_RNG
                              |RCC_PERIPHCLK_SPI2|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_QSPI
                              |RCC_PERIPHCLK_FMC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 30;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 10;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.FmcClockSelection = RCC_FMCCLKSOURCE_D1HCLK;
  PeriphClkInitStruct.QspiClockSelection = RCC_QSPICLKSOURCE_PLL2;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL;
  PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
  PeriphClkInitStruct.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable USB Voltage detector 
  */
  HAL_PWREx_EnableUSBVoltageDetector();
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
