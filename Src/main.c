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
#define ARM_MATH_CM4
#include "ssd1306.h"
#include "fonts.h"
#include "test.h"

#include "C:\Users\zulha\OneDrive\Documents\STM32\I2C\Drivers\CMSIS\DSP\Include\arm_math.h"
#include "math.h"
#define arm_rfft_fast_f32
#define arm_rfft_fast_init_f32
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void runFastft(void) ;
int display_bar(void);

uint16_t receive_buf [4096];
float fft_in_buf [1024];
float fft_out_buf [1024];
arm_rfft_fast_instance_f32 fft_handler;
int i;
float real_fsample = 47831;
uint8_t callback_state = 0;
uint8_t outarray[11];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init(); /* Initialize all configured peripherals */
  MX_I2C1_Init();
  MX_I2S1_Init();
  //MX_USART2_UART_Init();
	HAL_I2S_Receive_DMA(&hi2s1, receive_buf, 2048); //4096 /2
  arm_rfft_fast_init_f32(&fft_handler, 2048); ///1024 +1024

	
	
	
  /* USER CODE BEGIN 2 */
	if (HAL_I2S_Init(&hi2s1) == HAL_OK && HAL_I2C_Init(&hi2c1) == HAL_OK) {
		//HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(100);		
		SSD1306_Init();
		SSD1306_GotoXY (10,10); // goto 10, 10 
		SSD1306_Puts ("Spectrum", &Font_11x18, 1); // print Hello 
		SSD1306_GotoXY (10, 30); 
		SSD1306_Puts ("Analyzer ", &Font_11x18, 1); 
		SSD1306_UpdateScreen(); // update screen
		HAL_Delay(100);
		SSD1306_Fill(0);
		SSD1306_UpdateScreen();
	}
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		//volatile HAL_StatusTypeDef result = HAL_I2S_Receive(&hi2s1, receive_buf, 4096, 100);
		//if (result == HAL_OK) {
		int fftin_pointer = 0;
				 for (i=0; i<4096; i = i+4) {
		    
					 fft_in_buf[fftin_pointer] = (float)((int) (receive_buf[i]<<16)|receive_buf[i+1]);
					 fft_in_buf[fftin_pointer] += (float)((int) (receive_buf[i+2]<<16)|receive_buf[i+3]);
					 fftin_pointer++;
				 }
				 runFastft();
				 display_bar();
				 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		//	 }

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
int display_bar()
{
	char str[6];
	for (i=1;i < 11; i++){
		if(outarray[i] < 8)
		{

			SSD1306_DrawFilledRectangle(10,0,2,i+3,1);
			SSD1306_UpdateScreen();
		}
		else if(outarray[i] < 16)
		{
			SSD1306_DrawFilledRectangle(20,0,2,i+3,1);
			SSD1306_UpdateScreen();
		}
		else if(outarray[i] < 24)
		{
			SSD1306_DrawFilledRectangle(30,0,2,i+3,1);
			SSD1306_UpdateScreen();
		}
		else if(outarray[i] < 28)
		{
			SSD1306_DrawFilledRectangle(40,0,2,i+3,1);
			SSD1306_UpdateScreen();
		}
		else if(outarray[i] < 35)
		{
			SSD1306_DrawFilledRectangle(50,0,2,i+3,1);
			SSD1306_UpdateScreen();
		}
		else if(outarray[i] < 40)
		{
			SSD1306_DrawFilledRectangle(60,0,2,i+3,1);
			SSD1306_UpdateScreen();
		}
		else if(outarray[i] < 47)
		{
			SSD1306_DrawFilledRectangle(70,0,2,i+3,1);
			SSD1306_UpdateScreen();
		}
		else if(outarray[i] < 55)
		{
			SSD1306_DrawFilledRectangle(80,0,2,i+3,1);
			SSD1306_UpdateScreen();
		}
		else {
		SSD1306_GotoXY (10,10); // goto 10, 10 
		SSD1306_Puts ("NOTHING", &Font_11x18, 1); // print Hello 
		SSD1306_UpdateScreen();
		HAL_Delay(10);
		SSD1306_Fill(0);
		SSD1306_UpdateScreen();
		
		}
	}
	//HAL_Delay(50);
	//SSD1306_Clear();
}

	float complex_absolute(float real, float complex) {
		return sqrtf(real*real+complex*complex);
	}
	
	void runFastft() {
		
		//Construct fft//
		arm_rfft_fast_f32(&fft_handler,fft_in_buf,fft_out_buf,0);
		//arm_rfft_fast_f32(&fft_handler,fft_in_buf,fft_out_buf,0); // flag =0 forward transform
		
		int freqs[512];
		int freqpoint = 0;
		int offset = 150;//variable noise floor
		
		for (i =0; i<1024; i=i+2) {
	  freqs[freqpoint] = (int)(20*log10f(complex_absolute(fft_out_buf[i], fft_out_buf[i+1])))- offset;
		if (freqs[freqpoint]<0) freqs[freqpoint] =0;
			freqpoint++;
		}
		outarray[0] = 0xff;//255
		outarray[1] = (uint8_t)freqs[1];
		outarray[2] = (uint8_t)freqs[3];
		outarray[3] = (uint8_t)freqs[5];
		outarray[4] = (uint8_t)freqs[11];
		outarray[5] = (uint8_t)freqs[22];
		outarray[6] = (uint8_t)freqs[44];
		outarray[7] = (uint8_t)freqs[96];
		outarray[8] = (uint8_t)freqs[197];
		outarray[9] = (uint8_t)freqs[393]; 
		outarray[10] =  (uint8_t)freqs[655];
		//outarray[10] = 50;
		
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB2;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 192;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLI2SP_DIV2;
  PeriphClkInitStruct.PLLI2S.PLLI2SM = 16;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.I2sApb2ClockSelection = RCC_I2SAPB2CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S1_Init(void)
{

  /* USER CODE BEGIN I2S1_Init 0 */

  /* USER CODE END I2S1_Init 0 */

  /* USER CODE BEGIN I2S1_Init 1 */

  /* USER CODE END I2S1_Init 1 */
  hi2s1.Instance = SPI1;
  hi2s1.Init.Mode = I2S_MODE_MASTER_RX;
  hi2s1.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s1.Init.DataFormat = I2S_DATAFORMAT_24B;
  hi2s1.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s1.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s1.Init.CPOL = I2S_CPOL_LOW;
  hi2s1.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s1.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S1_Init 2 */

  /* USER CODE END I2S1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Blink_GPIO_Port, Blink_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Blink_Pin */
  GPIO_InitStruct.Pin = Blink_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Blink_GPIO_Port, &GPIO_InitStruct);

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
  while (1)
  {
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
