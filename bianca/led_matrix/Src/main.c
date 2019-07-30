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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

int matrix[8][8] = {{0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0}};
int point[8] = {0x0000000000000001, 0x0000000000000002,
0x0000000000000004, 0x0000000000000008, 0x0000000000000010,
0x0000000000000020, 0x0000000000000040, 0x0000000000000080};
typedef struct BallPosition{
	int i;
	int j;
}position;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
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

void clearMatrix ()
{
  	for(int i=2;i<8;i++)
  	 {
  	   write_max (i,0x00);
  	 }
}

void max_init(void)
{
 write_max(0x09, 0x00);       //  no decoding
 write_max(0x0a, 0x03);       //  brightness intensity
 write_max(0x0b, 0x07);       //  scan limit = 8 LEDs
 write_max(0x0c, 0x01);       //  power down =0，normal mode = 1
 write_max(0x0f, 0x00);       //  no test display
}

void init_matrix(){
	for(int i=0;i<8;i++){
		for(int j=0;j<8;j++){
			matrix[i][j] = 0;
		}
	}
}

void set_one_on_matrix(int line, int a, int b, int c){
	matrix[line][a]=1;
	matrix[line][b]=1;
	matrix[line][c]=1;
}

void player_bar(uint8_t g_ADCValue, uint8_t prev_value){
	  if(prev_value - g_ADCValue > 10){
		  if(g_ADCValue <= 800 ){
			  clearMatrix();
			  write_max (1,0x0000000000000007);
		  } else if(g_ADCValue > 800 && g_ADCValue <= 1500){
			  clearMatrix();
			  write_max (1,0x000000000000000e);
	      } else if(g_ADCValue > 1500 && g_ADCValue <= 2300){
	    	  clearMatrix();
	    	  write_max (1,0x000000000000001c);
	      } else if(g_ADCValue > 2300 && g_ADCValue <= 3000){
			  clearMatrix();
			  write_max (1,0x0000000000000038);
	      } else if(g_ADCValue > 3000  && g_ADCValue <= 3600){
	    	  clearMatrix();
	    	  write_max (1,0x0000000000000070);
	      } else if(g_ADCValue > 3600) {
	    	  clearMatrix();
	    	  write_max (1,0x00000000000000e0);
	      }
	  } else {
		  if(g_ADCValue <= 800 ){
			  clearMatrix();
			  write_max (1,0x0000000000000007);
		  } else if(g_ADCValue > 800 && g_ADCValue <= 1500){
			  clearMatrix();
			  write_max (1,0x000000000000000e);
	      } else if(g_ADCValue > 1500 && g_ADCValue <= 2300){
	    	  clearMatrix();
	    	  write_max (1,0x000000000000001c);
	      } else if(g_ADCValue > 2300 && g_ADCValue <= 3000){
			  clearMatrix();
			  write_max (1,0x0000000000000038);
	      } else if(g_ADCValue > 3000  && g_ADCValue <= 3600){
	    	  clearMatrix();
	    	  write_max (1,0x0000000000000070);
	      } else if(g_ADCValue > 3600) {
	    	  clearMatrix();
	    	  write_max (1,0x00000000000000e0);
	      }
	  }
}
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  max_init();
/*  clearMatrix();
  write_max (1,0x0000000000000007);
  write_max (1,0x000000000000000e);
  write_max (1,0x000000000000001c);
  write_max (1,0x0000000000000038);
  write_max (1,0x0000000000000070);
  write_max (1,0x00000000000000e0);*/

  position currpos;
  position lastpos;
  position aux;
  currpos.i = 3;
  currpos.j = 4;
  lastpos.i = 2;
  lastpos.j = 3;
  int neigh_i[4] = {-1,-1,1,1};
  int neigh_j[4] = {-1,1,1,-1};
  uint32_t g_ADCValue;
  uint32_t g_ADCValue_2;
  uint32_t prev_value = 0;
  uint32_t prev_value_2 = 0;
  while (1)
  {

	          HAL_ADC_Start(&hadc1);
	    	  HAL_ADC_PollForConversion(&hadc1, 100);
	    	  g_ADCValue = HAL_ADC_GetValue(&hadc1);
	    	  if(prev_value - g_ADCValue > 10){
	    		  if(g_ADCValue <= 800 ){
	    			  clearMatrix();
	    			  write_max (1,0x0000000000000007);
	    		  } else if(g_ADCValue > 800 && g_ADCValue <= 1500){
	    			  clearMatrix();
	    			  write_max (1,0x000000000000000e);
	    	      } else if(g_ADCValue > 1500 && g_ADCValue <= 2300){
	    	    	  clearMatrix();
	    	    	  write_max (1,0x000000000000001c);
	    	      } else if(g_ADCValue > 2300 && g_ADCValue <= 3000){
	    			  clearMatrix();
	    			  write_max (1,0x0000000000000038);
	    	      } else if(g_ADCValue > 3000  && g_ADCValue <= 3600){
	    	    	  clearMatrix();
	    	    	  write_max (1,0x0000000000000070);
	    	      } else if(g_ADCValue > 3600) {
	    	    	  clearMatrix();
	    	    	  write_max (1,0x00000000000000e0);
	    	      }
	    	  } else {
	    		  if(g_ADCValue <= 800 ){
	    			  clearMatrix();
	    			  write_max (1,0x0000000000000007);
	    		  } else if(g_ADCValue > 800 && g_ADCValue <= 1500){
	    			  clearMatrix();
	    			  write_max (1,0x000000000000000e);
	    	      } else if(g_ADCValue > 1500 && g_ADCValue <= 2300){
	    	    	  clearMatrix();
	    	    	  write_max (1,0x000000000000001c);
	    	      } else if(g_ADCValue > 2300 && g_ADCValue <= 3000){
	    			  clearMatrix();
	    			  write_max (1,0x0000000000000038);
	    	      } else if(g_ADCValue > 3000  && g_ADCValue <= 3600){
	    	    	  clearMatrix();
	    	    	  write_max (1,0x0000000000000070);
	    	      } else if(g_ADCValue > 3600) {
	    	    	  clearMatrix();
	    	    	  write_max (1,0x00000000000000e0);
	    	      }
	    	  }
	    	  prev_value = g_ADCValue;
	    	  HAL_Delay(100);
	          HAL_ADC_Stop(&hadc1);

	    	  HAL_ADC_Start(&hadc2);
	    	  HAL_ADC_PollForConversion(&hadc2, 100);
	    	  g_ADCValue_2 = HAL_ADC_GetValue(&hadc2);
	    	  if(prev_value_2 - g_ADCValue_2 > 10){
	    		  if(g_ADCValue_2 <= 800 ){
	    			  clearMatrix();
	    			  write_max (8,0x0000000000000007);
	    		  } else if(g_ADCValue_2 > 800 && g_ADCValue_2 <= 1500){
	    			  clearMatrix();
	    			  write_max (8,0x000000000000000e);
	    	      } else if(g_ADCValue_2 > 1500 && g_ADCValue_2 <= 2300){
	    	    	  clearMatrix();
	    	    	  write_max (8,0x000000000000001c);
	    	      } else if(g_ADCValue_2 > 2300 && g_ADCValue_2 <= 3000){
	    			  clearMatrix();
	    			  write_max (8,0x0000000000000038);
	    	      } else if(g_ADCValue_2 > 3000  && g_ADCValue_2 <= 3600){
	    	    	  clearMatrix();
	    	    	  write_max (8,0x0000000000000070);
	    	      } else if(g_ADCValue_2 > 3600) {
	    	    	  clearMatrix();
	    	    	  write_max (8,0x00000000000000e0);
	    	      }
	    	  } else {
	    		  if(g_ADCValue_2 <= 800 ){
	    			  clearMatrix();
	    			  write_max (8,0x0000000000000007);
	    		  } else if(g_ADCValue_2 > 800 && g_ADCValue_2 <= 1500){
	    			  clearMatrix();
	    			  write_max (8,0x000000000000000e);
	    	      } else if(g_ADCValue_2 > 1500 && g_ADCValue_2 <= 2300){
	    	    	  clearMatrix();
	    	    	  write_max (8,0x000000000000001c);
	    	      } else if(g_ADCValue_2 > 2300 && g_ADCValue_2 <= 3000){
	    			  clearMatrix();
	    			  write_max (8,0x0000000000000038);
	    	      } else if(g_ADCValue_2 > 3000  && g_ADCValue_2 <= 3600){
	    	    	  clearMatrix();
	    	    	  write_max (8,0x0000000000000070);
	    	      } else if(g_ADCValue_2 > 3600) {
	    	    	  clearMatrix();
	    	    	  write_max (8,0x00000000000000e0);
	    	      }
	    	  }
	    	  prev_value_2 = g_ADCValue_2;
	    	  HAL_Delay(100);
	          HAL_ADC_Stop(&hadc2);
	          /*
	           */
	  for(int x=1; x<9;x++){
		  for(int y=1;y<9;y++){
			  if(x==currpos.i && y==currpos.j){
				  for(int k=0;k<4;k++){
					  if((y==1 || y==8) && currpos.i - lastpos.i == 1 && currpos.i+neigh_i[k] == lastpos.i && currpos.j+neigh_j[k]==lastpos.j){
						  aux = currpos;
						  currpos.i = lastpos.i+2;
						  currpos.j = lastpos.j;
						  lastpos = aux;
						  clearMatrix();
						  write_max(currpos.i,point[currpos.j-1]);
						  HAL_Delay(200);
						  break;
					  } else if((x==1 || x==8) && currpos.j - lastpos.j == 1 && currpos.i+neigh_i[k] == lastpos.i && currpos.j+neigh_j[k]==lastpos.j){
						  aux = currpos;
						  currpos.i = lastpos.i;
						  currpos.j = lastpos.j+2;
						  lastpos = aux;
						  clearMatrix();
						  write_max(currpos.i,point[currpos.j-1]);
						  HAL_Delay(200);
						  break;
					  } else if((y==1 || y==8) && currpos.i+neigh_i[k] == lastpos.i && currpos.j+neigh_j[k]==lastpos.j){
						  aux = currpos;
						  currpos.i = lastpos.i-2;
						  currpos.j = lastpos.j;
						  lastpos = aux;
						  clearMatrix();
						  write_max(currpos.i,point[currpos.j-1]);
						  HAL_Delay(200);
						  break;
					  } else if((x==1 || x==8) && currpos.i+neigh_i[k] == lastpos.i && currpos.j+neigh_j[k]==lastpos.j){
						  aux = currpos;
						  currpos.i = lastpos.i;
						  currpos.j = lastpos.j-2;
						  lastpos = aux;
						  clearMatrix();
						  write_max(currpos.i,point[currpos.j-1]);
						  HAL_Delay(200);
						  break;
					  } else if(currpos.i+neigh_i[k] == lastpos.i && currpos.j+neigh_j[k]==lastpos.j){
						  lastpos = currpos;
						  currpos.i -= neigh_i[k];
						  currpos.j -= neigh_j[k];
						  clearMatrix();
						  write_max(currpos.i,point[currpos.j-1]);
						  HAL_Delay(200);
						  break;
					  }
				  }
			  }

	  }
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
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
