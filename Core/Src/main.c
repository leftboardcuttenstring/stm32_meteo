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
#include "date/date.h"
#include "transmit_and_recieve_control/transmit_and_recieve_control.h"
#include "init_peripherials/init_peripherials.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PIN_RS (1 << 0)
#define PIN_EN (1 << 2)
#define BACKLIGHT (1 << 3)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t aht10_init_command[1] = {0xe1};
uint8_t aht10_measurement_command[3] = {0xac, 0x33, 0x00};

extern uint8_t cmd;

Data journal[5] = {0};
int count = 0;
uint16_t lcd1604_addr = 0x27 << (uint16_t)1;
uint16_t bmp180_addr = 0x77 << (uint16_t)1;
uint16_t aht10_addr = 0x38 << (uint16_t)1;
uint8_t AHT10_RX_Data[6] = {0};
uint32_t AHT10_ADC_Raw;
float AHT10_Temperature; 
float AHT10_Humidity = 0.0;
//uint8_t AHT10_MeasCmd[3] = {0xAC, 0x33, 0x00};
//uint8_t AHT10_InitCmd[3] = {0xE1, 0x08, 0x00};

uint8_t AHT10_TmpHum_Cmd[3] = {0xAC, 0x33, 0x00};

uint8_t calib_data[22];
uint8_t temperature_buf[2];
uint8_t pressure_buf[3];
char GLOBAL_MESSAGE_BUFFER[60] = {0};
int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
uint16_t AC4, AC5, AC6;
int32_t X1 = 0;
int32_t X2 = 0;
int32_t B5 = 0; //global stuff

//static char msg[32];
char msg_time[32];
//static char msg_lcd[32];
int32_t temperature = 0;
int32_t pressure = 0;

RTC_TimeTypeDef time;
RTC_DateTypeDef date;

char aht10_initialization_command = 0b11100001;
char aht10_trigger_measurment_command = 0b01101100;
char aht10_soft_reset_command = 0b10111010;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void bmp180_get_global_coefficients(void);
void FindAddress(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void FindAddress(void) {
	char msg_address[64] = {0};
	uint16_t i;
	for(i = 0; i < (uint16_t)128; i++) {
		if(HAL_I2C_IsDeviceReady(&hi2c1, i << (uint16_t)1, 1, HAL_MAX_DELAY) == HAL_OK) {
			snprintf(msg_address, sizeof(msg_address), "0x%02X", i);
			HAL_UART_Transmit(&huart2, (uint8_t*)msg_address, sizeof(msg_address)-1, HAL_MAX_DELAY);
		}
	}
}

void bmp180_get_global_coefficients(void) {
	HAL_I2C_Mem_Read(&hi2c1, bmp180_addr, 0xAA, 1, calib_data, 22, HAL_MAX_DELAY);
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
}

#define RTC_INIT_MAGIC 0x32F2

void init_rtc_once(void)
{
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != RTC_INIT_MAGIC) {
        RTC_TimeTypeDef sTime = { .Hours = 11, .Minutes = 10, .Seconds = 30 };
        RTC_DateTypeDef sDate = { .WeekDay = RTC_WEEKDAY_SUNDAY, .Month = RTC_MONTH_OCTOBER,
                                  .Date = 19, .Year = 25 };

        HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, RTC_INIT_MAGIC);
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
  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_USART2_UART_Init();
  //MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  lcd1602_init();
  init_rtc_once();
  bmp180_get_global_coefficients();
  //aht10_init();
  //HAL_UART_Transmit(&huart2, (uint8_t*)"Hello\r\n", 7, HAL_MAX_DELAY);
  //HAL_Delay(1000);
  lcd1602_transmit_command(0b10000000);
  HAL_I2C_Master_Transmit(&hi2c1, aht10_addr, (uint8_t*)aht10_init_command, 1, HAL_MAX_DELAY);

  /*HUMIDITY IS:
    second byte,
    third byte,
    first hald of fourth one
    DO NOT FUCKING BELIEVE TO EVERYONE ELSE
    IF HE'S TELLING YOU SOMETHING ELSE ABOUT
    THAT YOU CAN BREAK HIS FACE WITH A BRICK */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
