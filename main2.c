/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include <string.h>
#include "main.h"
#include <stdbool.h> // For `true` and `false` macros
#include <stdint.h>  // For standard integer types
#include "stm32f4xx.h"

#define RS_Pin GPIO_PIN_4
#define RS_GPIO_Port GPIOC
#define EN_Pin GPIO_PIN_14
#define EN_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_15
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_1
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_2
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_12
#define D7_GPIO_Port GPIOB
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rx_index;
char rx_data[2];
const char* Command = "11";    // &_EN_*
uint8_t rx_buffer[100];
uint8_t transfer_cplt;
uint32_t tick;
uint32_t running=0;       // flag to show if function running
volatile uint16_t adcResultsDMA [2];
volatile int adcConversionComplete = 0; // set by callback
uint16_t send = 0;        // flag for end of sensing
uint32_t previousMillis = 0;   // for debounce
uint32_t currentMillis = 0;
uint32_t pulseCount = 0;     //for digital sensor
uint32_t lastpulsetime=0;    // for pulse count
float digiTemp = 0.0;
static volatile uint32_t lastDebounceTime = 0;
int DEBOUNCE_DELAY = 100; // Debounce delay in milliseconds
uint32_t PB8_high = 0;
uint32_t bounce_tick = 0;
uint32_t allow_press = true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void SenseThings(void);                                 // running led function
bool debounce_on_lift(uint16_t debounce_duration);      // debounce func
void LCD_SendCommand(uint8_t command);                  // lcd funs folllow
void LCD_SendData(uint8_t data);
void LCD_Clear(void);
void LCD_WriteString(char* str);

void LCD_Init(void) {
  LCD_SendCommand(0x02);
  LCD_SendCommand(0x28);
  LCD_SendCommand(0x0C);
  LCD_SendCommand(0x06);
}


void LCD_SendCommand(uint8_t command) {
  HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (command >> 4) & 1);
  HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (command >> 5) & 1);
  HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (command >> 6) & 1);
  HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (command >> 7) & 1);
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, command & 1);
  HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (command >> 1) & 1);
  HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (command >> 2) & 1);
  HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (command >> 3) & 1);
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
}

void LCD_SendData(uint8_t data) {
  HAL_GPIO_WritePin(RS_GPIO_Port, RS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, (data >> 4) & 1);
  HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (data >> 5) & 1);
  HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (data >> 6) & 1);
  HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (data >> 7) & 1);
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, data & 1);
  HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, (data >> 1) & 1);
  HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, (data >> 2) & 1);
 HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (data >> 3) & 1);
  //HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, (data & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_SET);
HAL_Delay(1);
HAL_GPIO_WritePin(EN_GPIO_Port, EN_Pin, GPIO_PIN_RESET);
HAL_Delay(1);
}

void LCD_Clear(void) {
LCD_SendCommand(0x01);
HAL_Delay(2);
}

void LCD_WriteString(char* str) {
while(*str) {
	LCD_SendData(*str++);
	HAL_Delay(1);
	}
}

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  char studnum[8] = "23835990";
  HAL_Delay(100);
  HAL_UART_Transmit(&huart2,"&_",2,100);
  HAL_UART_Transmit(&huart2,studnum,8,100);
  HAL_UART_Transmit(&huart2,"_*",2,100);
  HAL_UART_Transmit(&huart2,"\n\r",1,100);
  HAL_UART_Receive_IT(&huart2, rx_data, 2);

  //LCD//
  /* Enable clock for GPIOB and GPIOC */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
/* Initialize GPIOB pins */
GPIO_InitTypeDef GPIO_InitStruct = {0};
GPIO_InitStruct.Pin = EN_Pin | D4_Pin | D5_Pin | D6_Pin | D7_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* Initialize GPIOC pin */
GPIO_InitStruct.Pin = RS_Pin; // RS_Pin on GPIOC
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); // Initialize pin on GPIOC

LCD_Init();

LCD_Clear();
LCD_WriteString("Hello World!");

 //LCD end//
//running=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  debounce_on_lift(DEBOUNCE_DELAY);
	  HAL_UART_Receive_IT(&huart2, rx_data, 2);
	  if(running==1){


		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResultsDMA,2); //HAL_ADC_Start_DMA(hadc, pData, Length)
		  send = 1;
		  SenseThings();                 // just flashing led

	  }
	  if (send==1){
		  float Temp = (adcResultsDMA[0] * 3.3) / 4096.0 * 100.0 - 273;    // Convert to string and print
		  float Lux = (adcResultsDMA[1]*30000)/4096.0;                     // convert to lux
		  char uart_buffer[20];
		  int intTempADC = (int)Temp;
		  int intdigiTemp = (int)digiTemp;
		  int intLux = (int)Lux;
		  sprintf(uart_buffer, "&_%03d_%03d_%05d_*\r\n", intTempADC,intdigiTemp,intLux);
		  HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET); // leave led on after sensing
		  send=0;                                    //finished sending

	  }



}

    /* USER CODE END WHILE */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_ODD;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */





void SenseThings(void) {
	// This function will be executed when the button is pressed

	uint32_t tick = HAL_GetTick(); // set tick

	  // The function is running continuously until the flag is set to 0
	while (running) {

		if((((HAL_GetTick()-tick)>=50))) {            // led flash period
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_6);
			tick = HAL_GetTick();
		}

		HAL_UART_Receive_IT(&huart2, rx_data, 2);
		debounce_on_lift(DEBOUNCE_DELAY);
	}

}

bool debounce_on_lift(uint16_t DEBOUNCE_DELAY){
	PB8_high = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8);
	if(!PB8_high){
		bounce_tick = HAL_GetTick();
	}
	if(HAL_GetTick() - bounce_tick > DEBOUNCE_DELAY){
		allow_press = true;                                  //debounce on uppress
	}
	else{
		allow_press = false;
	}
	return allow_press;
}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
// Read the current button state
	previousMillis = currentMillis;
		currentMillis = HAL_GetTick();
		if((GPIO_Pin == GPIO_PIN_8)&&((currentMillis - previousMillis)>=100)&&(allow_press)){  // pb8 button
			if((running==0)){
				running=1;
			}
			else{
				running=0;
			}
			previousMillis = currentMillis;
		}

	if(GPIO_Pin == GPIO_PIN_12){                 //digital sensor
		pulseCount++;
		if((HAL_GetTick()-lastpulsetime)>= 50){
			digiTemp = 256.000 * pulseCount / 4096.000 - 50;
			lastpulsetime = HAL_GetTick();
			pulseCount=0;
		}
	}

}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){        // uart receive command
	UNUSED(huart);
	//rx_buffer[sizeof(rx_buffer) - 1] = '\0';
	if (strcmp(rx_data,Command) == 0){        //&_EN_*
		if(running==0){
			running=1;
		}
		else{
			running=0;
		}
		memset(rx_data, 0, sizeof(rx_data));
	}
	//memset(rx_data, 0, sizeof(rx_data));

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
