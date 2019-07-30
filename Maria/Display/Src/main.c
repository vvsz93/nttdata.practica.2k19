/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t a[8]={0x18,0x24,0x42,0x42,0x7E,0x42,0x42,0x42};
uint8_t disp1ay[38][8]={
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
{0x18,0x24,0x42,0x42,0x7E,0x42,0x42,0x42},//A 10
{0x3C,0x22,0x22,0x3c,0x22,0x22,0x3C,0x0},//B 11
{0x3C,0x40,0x40,0x40,0x40,0x40,0x40,0x3C},//C12
{0x7C,0x22,0x22,0x22,0x22,0x22,0x22,0x7C},//D13
{0x7C,0x40,0x40,0x7C,0x40,0x40,0x40,0x7C},//E14
{0x7C,0x40,0x40,0x7C,0x40,0x40,0x40,0x40},//F15
{0x3C,0x40,0x40,0x40,0x4c,0x44,0x44,0x3C},//G16
{0x44,0x44,0x44,0x7C,0x44,0x44,0x44,0x44},//H17
{0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x7C},//I18
{0x3C,0x8,0x8,0x8,0x8,0x8,0x48,0x30},//J19
{0x0,0x24,0x28,0x30,0x20,0x30,0x28,0x24},//K20
{0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x7C},//L21
{0x81,0xC3,0xA5,0x99,0x81,0x81,0x81,0x81},//M22
{0x0,0x42,0x62,0x52,0x4A,0x46,0x42,0x0},//N23
{0x3C,0x42,0x42,0x42,0x42,0x42,0x42,0x3C},//O24
{0x3C,0x22,0x22,0x22,0x3C,0x20,0x20,0x20},//P25
{0x1C,0x22,0x22,0x22,0x22,0x26,0x22,0x1D},//Q26
{0x3C,0x22,0x22,0x22,0x3C,0x24,0x22,0x21},//R27
{0x0,0x1E,0x20,0x20,0x3E,0x2,0x2,0x3C},//S28
{0x0,0x3E,0x8,0x8,0x8,0x8,0x8,0x8},//T29
{0x42,0x42,0x42,0x42,0x42,0x42,0x22,0x1C},//U30
{0x42,0x42,0x42,0x42,0x42,0x42,0x24,0x18},//V31
{0x0,0x49,0x49,0x49,0x49,0x2A,0x1C,0x0},//W32
{0x0,0x41,0x22,0x14,0x8,0x14,0x22,0x41},//X33
{0x41,0x22,0x14,0x8,0x8,0x8,0x8,0x8},//Y34
{0x0,0x7F,0x2,0x4,0x8,0x10,0x20,0x7F},//Z35
};

unsigned char disp1[19][8]={
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // Heart Pattern
  0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x80, 0x40, 0x40, 0x00, 0x00, 0x00,
  0x00, 0x80, 0x80, 0x40, 0x40, 0x00, 0x00, 0x00,
  0x40, 0x80, 0x80, 0x40, 0x40, 0x00, 0x00, 0x00,
  0x60, 0x80, 0x80, 0x40, 0x40, 0x00, 0x00, 0x00,
  0x60, 0x90, 0x80, 0x40, 0x40, 0x00, 0x00, 0x00,
  0x60, 0x90, 0x88, 0x40, 0x40, 0x00, 0x00, 0x00,
  0x60, 0x90, 0x88, 0x44, 0x40, 0x00, 0x00, 0x00,
  0x60, 0x90, 0x88, 0x44, 0x44, 0x00, 0x00, 0x00,
  0x60, 0x90, 0x88, 0x44, 0x44, 0x08, 0x00, 0x00,
  0x60, 0x90, 0x88, 0x44, 0x44, 0x08, 0x10, 0x00,
  0x60, 0x90, 0x88, 0x44, 0x44, 0x08, 0x10, 0x20,
  0x60, 0x90, 0x88, 0x44, 0x44, 0x08, 0x10, 0x60,
  0x60, 0x90, 0x88, 0x44, 0x44, 0x08, 0x90, 0x60,
  0x60, 0x90, 0x88, 0x44, 0x44, 0x88, 0x90, 0x60, // Heart Pattern

};
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
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void write_byte(uint8_t byte)
{
	    for(int i=0;i<8;i++)
          {
	    	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 0);  // Pull the CLK LOW
	    	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, byte&0x80);// Write one BIT data MSB first
             byte = byte<<1;  // shift the data to the left
             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, 1);  // Pull the CLK HIGH
           }
}

void write_max (uint8_t address, uint8_t data)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);   // Pull the CS pin LOW
    write_byte(address);   //  write address
    write_byte(data);  //  write data
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);  // pull the CS HIGH
}


void max_init(void)
{
 write_max(0x09, 0x00);       //  no decoding
 write_max(0x0a, 0x03);       //  brightness intensity
 write_max(0x0b, 0x07);       //  scan limit = 8 LEDs
 write_max(0x0c, 0x01);       //  power down =0，normal mode = 1
 write_max(0x0f, 0x00);       //  no test display
}
/*void adxl_write(uint8_t address,uint8_t value)
{
	uint8_t data[2];
	data[0]=address | 0x40;
	data[1]=value;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, data, 2, 100);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);
}*/

/*void adxl_init(void)
{
	adxl_write(0x31,0x01);//data format range
	adxl_write(0x2d,0x00);//reset all bits
	adxl_write(0x2d,0x08);//power_cntl measure
}*/
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
  /* USER CODE BEGIN 2 */
  max_init();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


	 	   for(int i=1;i<9;i++)
	 	   {
	 	       write_max (i,disp1ay[22][i-1]);
	 	   }
	 	   HAL_Delay (500);
	 	  for(int i=1;i<9;i++)
	 	   {
	 	  	 write_max (i,disp1ay[10][i-1]);
	 	   }
	 	  	HAL_Delay (500);
	 	  	for(int i=1;i<9;i++)
	 	  	 {
	 	  	   write_max (i,disp1ay[27][i-1]);
	 	  	 }
	 	  	HAL_Delay (500);
	 	  	for(int i=1;i<9;i++)
	 	    {
	 	  	 write_max (i,disp1ay[18][i-1]);
	 	    }
	 	  	 HAL_Delay (500);
	 	  	for(int i=1;i<9;i++)
	 	  	{
	 	  	 write_max (i,disp1ay[10][i-1]);
	 	    }
	 	  	HAL_Delay (500);
	 	  	for(int j=0; j<19; j++)
	 	  	{
	 		for(int i=1;i<9;i++)

	 		 	  	{
	 		 	  	 write_max (i,disp1[j][i-1]);

	 		 	  	HAL_Delay (10);
	 		 	  	 }
	 	  	}
	 	  	HAL_Delay(1000);
	 	   //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	   //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

 }
    /* USER CODE BEGIN 3 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA12 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
