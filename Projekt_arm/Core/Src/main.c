/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <cstdlib>
#include <stdio.h>

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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */

static void MX_NVIC_Init(void);

void NUM_RESET(void);
void write_NUM_2 (char *str_s, char *str_s10, char *str_m, char *str_m10);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t disp1ay[10][8]={
{0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x3C},//0
{0x10,0x30,0x50,0x10,0x10,0x10,0x10,0x7c},//1
{0x7E,0x2,0x2,0x7E,0x40,0x40,0x40,0x7E},//2
{0x3E,0x2,0x2,0x3E,0x2,0x2,0x3E,0x0},//3
{0x8,0x18,0x28,0x48,0xFE,0x8,0x8,0x8},//4
{0x3C,0x20,0x20,0x3C,0x4,0x4,0x3C,0x0},//5
{0x3C,0x20,0x20,0x3C,0x24,0x24,0x3C,0x0},//6
{0x3E,0x22,0x4,0x8,0x8,0x8,0x8,0x8},//7
{0x0,0x3E,0x22,0x22,0x3E,0x22,0x22,0x3E},//8
{0x3E,0x22,0x22,0x3E,0x2,0x2,0x2,0x3E},//9
};

void write_byte(uint8_t byte){
	for (int i=0;i<8;i++){
		HAL_GPIO_WritePin(GPIOD, CLK_Pin, 0);
		HAL_GPIO_WritePin(GPIOD, DATA_Pin, byte&0x80);
		byte = byte<<1;
		HAL_GPIO_WritePin(GPIOD, CLK_Pin, 1);
	}
}


void write_max(uint8_t adress, uint8_t data){
		HAL_GPIO_WritePin(GPIOD, CS_Pin, 0);
		write_byte(adress);
		write_byte(data);
		HAL_GPIO_WritePin(GPIOD, CS_Pin, 1);
}


void init_max(void){
	write_max(0x09, 0x00);
	write_max(0x0a, 0x03);
	write_max(0x0b, 0x07);
	write_max(0x0c, 0x01);
	write_max(0x0f, 0x00);
}


void write_string (char *str)
{
	while (*str)
	{
		for(int i=1;i<9;i++)
			   { // 4 * 8bit bo rejestr 32 bitowy 
					 // inaczej sa bledy
			      write_max (i,disp1ay[(*str - 48)][i-1]);
					  write_max (i,disp1ay[(*str - 48)][i-1]);
					  write_max (i,disp1ay[(*str - 48)][i-1]);
					  write_max (i,disp1ay[(*str - 48)][i-1]);
					  //write_max (1,disp1ay[(*str - 48)][i-1]);
			   }
		*str++;
	}
}

void write_NUM (char *str_s, char *str_s10, char *str_m, char *str_m10)
{
	while (*str_m10)
	{
		// jednal liczba na wyswietlaczu to 8*16bit= 128bitow
		for(int i=1;i<9;i++) // 8 razy bo tyle bitow ma jedna liczba
			   { 
			    write_max (i,disp1ay[(*str_m10 - 48)][i-1]); // wysyla 16bitow 8bitow adresu i 8bitow danych
			   }
				 
		*str_m10++;
	}
	
		while (*str_m)
	{
		for(int i=1;i<9;i++)
			   { 
			    write_max (i,disp1ay[(*str_m - 48)][i-1]);
			   }
				 
		*str_m++;
	}
	
		while (*str_s10)
	{
		for(int i=1;i<9;i++)
			   { 
			    write_max (i,disp1ay[(*str_s10 - 48)][i-1]);
			   }
				 
		*str_s10++;
	}
	
	while (*str_s)
	{
		for(int i=1;i<9;i++)
			   { 
			    write_max (i,disp1ay[(*str_s - 48)][i-1]);
			   }
				 
		*str_s++;
	}
	
}

void write_NUM_2 (char *str_s, char *str_s10, char *str_m, char *str_m10)
{
	while (*str_s)
	{
		for(int i=1;i<9;i++)
			   { 
			      write_max (i,disp1ay[(*str_s - 48)][i-1]);
					  write_max (i+8,disp1ay[(*str_s10 - 48)][i-1]);
					  write_max (i+16,disp1ay[(*str_m - 48)][i-1]);
					  write_max (i+24,disp1ay[(*str_m10 - 48)][i-1]);
					  
			   }
				 *str_s++;
				 *str_s10++;
				 *str_m++;
				 *str_m10++;
	}
}

	volatile uint8_t START = 1;

	volatile uint8_t s = 0;
	volatile uint8_t s10 = 0;
	volatile uint8_t m = 0;
	volatile uint8_t m10 = 0;

void NUM_RESET(void)
		{
			s = 0;
			m = 0;
			s10 = 0;
			m10 = 0;
		}
		
		void START_STOP(void)
		{
			if(START == 0)
				{START = 1;
				}
			if(START == 1)
				{START = 0;
				}
		}

char *my_itoa(char *dest, uint8_t i) 
	{
  sprintf(dest, "%d", i);
  return dest;
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	
 MX_NVIC_Init();
 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	uint8_t reset;
	uint8_t start_stop;
	
	init_max();
	//volatile uint8_t s = 0;
	//volatile uint8_t s10 = 0, m = 0, m10 = 0;
	char * number;
	
	char SEK;
	char SEK_10;
	char MIN;
	char MIN_10;
		
		sprintf(&SEK, "%d", s);
		sprintf(&SEK_10, "%d", s10);
		sprintf(&MIN, "%d", m);
		sprintf(&MIN_10, "%d", m10);

		
  while (1)
  {
		
		if(START)
{
							if (s >= 10){
							s10++;
							s = 0;
							//my_itoa(SEK_10, s10);
								sprintf(&SEK_10, "%d", s10);
			
							if (s10 >= 6){
								m++;
								s10 = 0;
								//my_itoa(MIN, m);
								sprintf(&MIN, "%d", m);
								
								if (m >= 10){
									m10++;
									m = 0;
									//my_itoa(MIN_10, m10);
									sprintf(&MIN_10, "%d", m10);
									
										if (m10 >= 6){
										s = 0;
										m = 0;
										s10 = 0;
										m10 = 0;
										}
									}
								}
							}
					s = s + 1;		
					//my_itoa(SEK, s);
					sprintf(&SEK, "%d", s);
							
					//write_string(&SEK_10);
							write_string(&SEK);
							
					//write_string(my_itoa(number, s10));
					
					//write_NUM(&SEK, &SEK_10, &MIN, &MIN_10);
							
						//write_NUM(&SEK, &SEK_10, &MIN, &MIN_10);
							
					HAL_Delay(1000);
}

		

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
	
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */


	
  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, CS_Pin|CLK_Pin|DATA_Pin, GPIO_PIN_RESET);


  /*Configure GPIO pins : PC13 PC12 */ //BUTTONS
  //GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_12;
  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 // GPIO_InitStruct.Pull = GPIO_NOPULL;
  //HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	
  /*Configure GPIO pins : PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin CLK_Pin DATA_Pin */
  GPIO_InitStruct.Pin = CS_Pin|CLK_Pin|DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


// --------------------------------------------------
 #define BUTTON_1_INT_Pin GPIO_PIN_13
#define BUTTON_1_INT_GPIO_Port GPIOC
#define BUTTON_1_INT_EXTI_IRQn EXTI15_10_IRQn
#define BUTTON_2_INT_Pin GPIO_PIN_12
#define BUTTON_2_INT_GPIO_Port GPIOC
#define BUTTON_2_INT_EXTI_IRQn EXTI15_10_IRQn
 
 /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = BUTTON_1_INT_Pin|BUTTON_2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

	static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
		
	HAL_GPIO_ReadPin(GPIOC, 13);
{
 
}
		
	HAL_GPIO_ReadPin(GPIOC, 12);
{
	NUM_RESET();
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
