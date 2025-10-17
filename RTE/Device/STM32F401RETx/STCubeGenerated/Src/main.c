/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NewLineMssg HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, HAL_MAX_DELAY);
#define PIN_RS (1 << 0)
#define PIN_EN (1 << 2)
#define BACKLIGHT (1 << 3)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t i2c_scan_msg[] = "I2C bus scanning\r\n";
static uint16_t lcd_addr = 0x27 << (uint16_t)1;
static uint16_t gy_addr = 0x77 << (uint16_t)1;
static uint8_t t_buffer[2];
static uint8_t p_buffer[3];
static int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
static uint16_t AC4, AC5, AC6;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
static void TransmitData(uint8_t data);
static void TransmitCmd(uint8_t cmd);
static void LCD_Init(void);
static void TransmitLCD(uint8_t data, uint8_t flags);
//static uint16_t FindAddress(void);
void LCD_SendString(char *str);
static void Barometer_Init(void);
static void Barometer_GetData(bool Mode);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Barometer_GetData(bool Mode) {
	if (Mode == true) {
		HAL_I2C_Mem_Read(&hi2c1, gy_addr, 0xF6, 1, t_buffer, 2, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, t_buffer, sizeof(t_buffer)-1, HAL_MAX_DELAY);
		NewLineMssg
	}
}

static void Barometer_Init(void) {
	uint8_t cmd = 0x2E;
	if (HAL_I2C_Mem_Write(&hi2c1, gy_addr, 0xF4, 1, &cmd, 1, HAL_MAX_DELAY) == HAL_OK) {
		HAL_UART_Transmit(&huart2, (const uint8_t*)"Barometer init is done\r\n", sizeof("Barometer init is done\r\n")-1, HAL_MAX_DELAY);
	}
	HAL_Delay(5);
}

void LCD_SendString(char *str) {
    while(*str) {
        TransmitData((uint8_t)(*str));
        str++;
    }
}

static uint16_t FindAddress(void) {
	char msg[64] = {0};
	uint16_t i;
	for(i = 0; i < (uint16_t)128; i++) {
		if(HAL_I2C_IsDeviceReady(&hi2c1, i << (uint16_t)1, 1, HAL_MAX_DELAY) == HAL_OK) {
			lcd_addr = i << (uint16_t)1;
			snprintf(msg, sizeof(msg), "0x%02X", i);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);
			break;
		}
	}
	NewLineMssg
	return i;
}

static void TransmitLCD(uint8_t data, uint8_t flags) {
	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;
	
	uint8_t data_arr[4];                              
	data_arr[0] = up|flags|BACKLIGHT|PIN_EN; 
	data_arr[1] = up|flags|BACKLIGHT;         
	data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
	data_arr[3] = lo|flags|BACKLIGHT;
	HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
	HAL_Delay(10);
}

static void TransmitData(uint8_t data) {
	TransmitLCD(data, PIN_RS);
}

static void TransmitCmd(uint8_t cmd) {
	TransmitLCD(cmd, 0);
}

static void LCD_Init(void) {
  HAL_Delay(50);
  TransmitCmd(0x33);
  HAL_Delay(5);
  TransmitCmd(0x32);
  HAL_Delay(1);
  TransmitCmd(0x28);
  HAL_Delay(1);
  TransmitCmd(0x08);
  HAL_Delay(1);
  TransmitCmd(0x01);
  HAL_Delay(2);
  TransmitCmd(0x06);
  HAL_Delay(1);
  TransmitCmd(0x0C);
  HAL_Delay(1);
	HAL_UART_Transmit(&huart2, (const uint8_t*)"LCD init is done\r\n", sizeof("LCD init is done\r\n")-1, HAL_MAX_DELAY);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	HAL_Delay(500);
	//let x1 = (rx_word as i32 - calib_coeffs.ac6 as i32) * (calib_coeffs.ac5 as i32) >> 15;
	//int x1 = (int32_t)rx_buffer - 
	//uint32_t UP_raw = ((uint32_t)rx_buffer[0] << 16) | ((uint32_t)rx_buffer[1] << 8) | rx_buffer[2];
	//Barometer_GetData(true);
	Barometer_Init();
	uint8_t calib_data[22];
	HAL_I2C_Mem_Read(&hi2c1, gy_addr, 0xAA, 1, calib_data, 22, HAL_MAX_DELAY);
	
	AC1 = (int16_t)((calib_data[0] << 8) | calib_data[1]);
	AC2 = (int16_t)((calib_data[2] << 8) | calib_data[3]);
	AC3 = (int16_t)((calib_data[4] << 8) | calib_data[5]);
	AC4 = (uint16_t)((calib_data[6] << 8) | calib_data[7]);
	AC5 = (uint16_t)((calib_data[8] << 8) | calib_data[9]);
	AC6 = (uint16_t)((calib_data[10] << 8) | calib_data[11]);
	B1  = (int16_t)((calib_data[12] << 8) | calib_data[13]);
	B2  = (int16_t)((calib_data[14] << 8) | calib_data[15]);
	MB  = (int16_t)((calib_data[16] << 8) | calib_data[17]);
	MC  = (int16_t)((calib_data[18] << 8) | calib_data[19]);
	MD  = (int16_t)((calib_data[20] << 8) | calib_data[21]);
	
	/*HAL_I2C_Mem_Read(&hi2c1, gy_addr, 0xF6, 1, t_buffer, 2, HAL_MAX_DELAY);
	long UT = (t_buffer[0] << 8) + t_buffer[1];
	int32_t X1 = ((UT-AC6)*AC5) >> 15;
	int32_t X2 = (MC << 11) / (X1 + MD);
	int32_t B5 = X1 + X2;
	int32_t T = (B5 + 8) >> 4;
	char msg[32];
	char msg_lcd[32];
	float temperature = T / 10.0f;
	//sprintf(msg, "temperature = %f\r\n", temperature);
	HAL_UART_Transmit(&huart2, (uint8_t*)msg, sizeof(msg)-1, HAL_MAX_DELAY);
	sprintf(msg_lcd, "t = %f", temperature);*/
	
	HAL_I2C_Mem_Read(&hi2c1, gy_addr, 0xF6, 1, t_buffer, 2, HAL_MAX_DELAY);
	long UT = (t_buffer[0] << 8) + t_buffer[1];

	int32_t X1 = ((UT-AC6)*AC5) >> 15;
	int32_t X2 = (MC << 11) / (X1 + MD);
	int32_t B5 = X1 + X2;
	int32_t T = (B5 + 8) >> 4;

	HAL_Delay(100);
	uint8_t OSS = 0;
	uint8_t pressure_cmd = 0x34 + (OSS << 6);
	HAL_I2C_Mem_Write(&hi2c1, gy_addr, 0xF4, 1, &pressure_cmd, sizeof(pressure_cmd), HAL_MAX_DELAY);
	HAL_Delay(100);
	HAL_I2C_Mem_Read(&hi2c1, gy_addr, 0xF6, 1, p_buffer, 3, HAL_MAX_DELAY);
	long UP = (((long)p_buffer[0] << 16) | ((long)p_buffer[1] << 8) | (long)p_buffer[2]) >> 8;
	char msg[32];
	char msg_lcd[32];
	
	long B6 = B5 - 4000;
	X1 = (B2 * ((B6 * B6) >> 12)) >> 11;
	X2 = (AC2 * B6) >> 11;
	long X3 = X1 + X2;
	long B3 = ((((long)AC1 * 4 + X3) << OSS) + 2) >> 2;
	X1 = (AC3 * B6) >> 13;
	X2 = (B1 * ((B6 * B6) >> 12)) >> 16;
	X3 = ((X1 + X2) + 2) >> 2;
	unsigned long B4 = (AC4 * (unsigned long)(X3 + 32768)) >> 15;
	unsigned long B7 = ((unsigned long)UP - B3) * (50000UL >> OSS);
	long p;
	if (B7 < 0x80000000)
			p = (B7 << 1) / B4;
	else
			p = (B7 / B4) << 1;

	X1 = (p >> 8) * (p >> 8);
	X1 = (X1 * 3038L) >> 16;
	X2 = (-7357L * p) >> 16;
	p = p + ((X1 + X2 + 3791L) >> 4);
	
	
	sprintf(msg_lcd, "p = %.2f mmHg", p / 133.322f);
	
	TransmitCmd(0b10000000);
	//LCD_SendString("mike");
	LCD_SendString(msg_lcd);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
