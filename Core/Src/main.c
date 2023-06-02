/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "image.h"

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
DMA_HandleTypeDef hdma_adc1;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */

uint16_t ID=0;

volatile uint16_t * LCD_MEM_DATA = (uint16_t*)LCD_DATA;
volatile uint16_t * LCD_MEM_ADDRESS = (uint16_t*)LCD_ADDRESS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_FSMC_Init(void);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(100);

	//
	//GETTING DISPLAY ID
	//
	*LCD_MEM_ADDRESS = 0x00;
	*LCD_MEM_DATA;
	ID = *LCD_MEM_DATA;
	//
	//
	//

	LCD_INIT();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_Delay(5000);
	  LCD_Block_Write(0, 239, 0, 399);
	  for(int i=0; i<400; i++)
	  	  {
	  		  for(int j=0; j<240; j++)
	  		  {
	  			  LCD_Write_Data(RGB(255,0,0));
	  		  }
	  	  }
	  HAL_Delay(5000);
	  LCD_Block_Write(0, 239, 0, 399);
	  for(int i=0; i<400; i++)
	  	  {
	  		  for(int j=0; j<240; j++)
	  		  {
	  			  LCD_Write_Data(RGB(0,255,0));
	  		  }
	  	  }
	  HAL_Delay(5000);
	  LCD_Block_Write(0, 239, 0, 399);
	  for(int i=0; i<400; i++)
	  	  {
	  		  for(int j=0; j<240; j++)
	  		  {
	  			  LCD_Write_Data(RGB(0,0,255));
	  		  }
	  	  }
	  HAL_Delay(5000);
	  LCD_Block_Write(0,239,0,399);
	  for(int i=0; i<400; i++)
	  {
		  for(int j=0; j<240; j++)
		  {
			  LCD_Write_Data(RGB(i*255/400,i*255/400,i*255/400));
		  }
	  }
	  HAL_Delay(5000);
	  	  LCD_Block_Write(0,239,0,399);
	  	  for(int i=0; i<400; i++)
	  	  {
	  		  for(int j=0; j<240; j++)
	  		  {
	  			  LCD_Write_Data(kittenBMP[i*240+j]);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
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
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD2 PD3 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 1;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 2;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
void LCD_Write(uint16_t address, uint16_t data)
{
	*LCD_MEM_ADDRESS = address;
	*LCD_MEM_DATA = data;
}

uint16_t LCD_Read(uint16_t address)
{
	uint16_t data=0x0000;

	*LCD_MEM_ADDRESS = address;
	*LCD_MEM_DATA;
	data = *LCD_MEM_DATA;

	return data;
}

void LCD_Write_Comm(uint16_t command)
{
	*LCD_MEM_ADDRESS = command;
}

void LCD_Write_Data(uint16_t data)
{
	*LCD_MEM_DATA = data;
}

void LCD_INIT(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_Delay(20);

	     //HX8352B FOR CMO3.2(Pd032MNML-0403)

	    //V03----2011.6.7/PengWu
		//1ЎўРЮёД E2-EF �?ДґжЖч
		//2Ўў�?хµНVCOMHµзС№Ј¬�?хµ­Й«°Я

		LCD_Write(0xE2, 0x15);      //VREFsetting
		LCD_Write(0xE5, 0x18);
		LCD_Write(0xE7, 0x18);
		LCD_Write(0xE8, 0x40);
		LCD_Write(0xEC, 0x09);
		LCD_Write(0xED, 0x06);
		LCD_Write(0xEE, 0x20);
		LCD_Write(0xEF, 0x50);
		LCD_Write(0x29, 0x01);
		LCD_Write(0x2B, 0x03);
		LCD_Write(0x2E, 0x85);


		// Power on Setting
		LCD_Write(0x23, 0x75);     //0x79
		LCD_Write(0x24, 0x50);  // 0x50  �?УЙо¶Ф±�?¶�?
		LCD_Write(0x25, 0x71);
		LCD_Write(0x1B, 0x1E);    //VREG1 = 4.5V
		LCD_Write(0x01, 0x00);
		LCD_Write(0x1C, 0x04);

		// Power on sequence
		LCD_Write(0x18, 0xdd);  //
		LCD_Write(0x19, 0x01);
		HAL_Delay(5);
		LCD_Write(0x1F, 0x8C);
		LCD_Write(0x1F, 0x84);
		HAL_Delay(10);
		LCD_Write(0x1F, 0x94);
		HAL_Delay(10);
		LCD_Write(0x1F, 0xD4);
		HAL_Delay(5);

		// Gamma Setting
		LCD_Write(0x40, 0x08);
		LCD_Write(0x41, 0x31);
		LCD_Write(0x42, 0x2F);
		LCD_Write(0x43, 0x3E);
		LCD_Write(0x44, 0x3D);
		LCD_Write(0x45, 0x3F);
		LCD_Write(0x46, 0x2F);
		LCD_Write(0x47, 0x79);
		LCD_Write(0x48, 0x08);
		LCD_Write(0x49, 0x06);
		LCD_Write(0x4A, 0x08);
		LCD_Write(0x4B, 0x0E);
		LCD_Write(0x4C, 0x17);


		LCD_Write(0x50, 0x00);
		LCD_Write(0x51, 0x02);
		LCD_Write(0x52, 0x01);
		LCD_Write(0x53, 0x10);
		LCD_Write(0x54, 0x0E);
		LCD_Write(0x55, 0x37);
		LCD_Write(0x56, 0x06);
		LCD_Write(0x57, 0x50);
		LCD_Write(0x58, 0x08);
		LCD_Write(0x59, 0x11);
		LCD_Write(0x5A, 0x17);
		LCD_Write(0x5B, 0x19);
		LCD_Write(0x5C, 0x17);
		LCD_Write(0x5D, 0xFF);

		// Display ON Setting
		LCD_Write(0x60, 0x00); //FMARK 18ЈєOPEN
		LCD_Write(0x16,0x0b);
		LCD_Write(0x17, 0x05);
		LCD_Write(0x36, 0x00);
		LCD_Write(0x28, 0x20);
		HAL_Delay(40);
		LCD_Write(0x28, 0x38);
		HAL_Delay(40);                                  // Waiting 2 frames at least
		LCD_Write(0x28, 0x3C);
}

void LCD_Block_Write(uint16_t Xstart, uint16_t Xend, uint16_t Ystart, uint16_t Yend)
{
	LCD_Write_Comm(0x02);	LCD_Write_Data(Xstart >> 8);
	LCD_Write_Comm(0x03);	LCD_Write_Data(Xstart);  //Column Start
	LCD_Write_Comm(0x04);	LCD_Write_Data(Xend >> 8);
	LCD_Write_Comm(0x05);	LCD_Write_Data(Xend);  //Column End

	LCD_Write_Comm(0x06);	LCD_Write_Data(Ystart >> 8);
	LCD_Write_Comm(0x07);	LCD_Write_Data(Ystart);  //Row Start
	LCD_Write_Comm(0x08);	LCD_Write_Data(Yend >> 8);
	LCD_Write_Comm(0x09);	LCD_Write_Data(Yend);  //Row End

	LCD_Write_Comm(0x80);	LCD_Write_Data(Xstart >> 8);
	LCD_Write_Comm(0x81);	LCD_Write_Data(Xstart);  //Column Start
	LCD_Write_Comm(0x82);	LCD_Write_Data(Ystart >> 8);
	LCD_Write_Comm(0x83);	LCD_Write_Data(Ystart);  //Row Start

	LCD_Write_Comm(0x22);
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
