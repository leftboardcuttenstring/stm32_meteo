/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include "transmit_and_recieve_control/transmit_and_recieve_control.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HUMIDITY_PUT 19849
#define HUMIDITY_GET 19949

#define TEMPERATURE_PUT 19950
#define TEMPERATURE_GET 19969

#define PRESSURE_PUT 19970
#define PRESSURE_GET 20000

#define OSS 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static unsigned int SysTick_20Sec_Counter = 0;
static unsigned int SysTick_1Sec_Counter_mode = 0;
static unsigned int SysTick_1Sec_Counter_mode1 = 0;
static unsigned int SysTick_1Sec_Counter_mode2 = 0;
static unsigned int SysTick_30Msec_Counter = 0;
char hour_modifying_string[10] = {0};
char minutes_modifying_string[10] = {0};
//static unsigned int SysTick_1Minute_Counter = 0;
//static Data Log[LOG_SIZE] = {0};
//static unsigned char LogCounter = 0;
extern RTC_TimeTypeDef time;
RTC_TimeTypeDef local_modifying_time;
extern RTC_DateTypeDef date;
RTC_DateTypeDef local_modifying_date;
extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart2;
extern char msg_time[32];
extern Data journal[5];
extern int count;
extern int32_t temperature;
extern int32_t pressure;
extern I2C_HandleTypeDef hi2c1;
char message_count[32] = {0};
bool current_state_is_write = false;
extern uint8_t cmd;
extern uint16_t lcd1604_addr;
extern uint16_t bmp180_addr;
extern char GLOBAL_MESSAGE_BUFFER[30];
int32_t temperature_result = 0;
int32_t pressure_result = 0;

extern uint16_t aht10_addr;
extern uint8_t AHT10_RX_Data[6];
extern uint32_t AHT10_ADC_Raw;
extern float AHT10_Temperature;
extern float AHT10_Humidity;
extern uint8_t AHT10_MeasCmd[3];
extern uint8_t AHT10_InitCmd[3];
extern char aht10_initialization_command;
extern uint8_t aht10_init_command[1];
extern uint8_t aht10_measurement_command[3];
//unsigned int counter_1s = 0;

static unsigned int current_mode = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void GetData(void);
int AveragingData(void);
void LogData(Data Current);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  if (current_mode == 0) {
    SysTick_1Sec_Counter_mode1 = 0;
    SysTick_1Sec_Counter_mode2 = 0;
    if (SysTick_20Sec_Counter == HUMIDITY_PUT) {
      HAL_I2C_Master_Transmit(&hi2c1, aht10_addr, (uint8_t*)aht10_measurement_command, 3, HAL_MAX_DELAY);
    }
    if (SysTick_20Sec_Counter == HUMIDITY_GET) {
      HAL_I2C_Master_Receive(&hi2c1, aht10_addr, AHT10_RX_Data, 6, HAL_MAX_DELAY);
      AHT10_ADC_Raw = ((uint32_t)AHT10_RX_Data[1] << 12) | ((uint32_t)AHT10_RX_Data[2] << 4) | (AHT10_RX_Data[3] >> 4);
    }
    if (SysTick_20Sec_Counter == TEMPERATURE_PUT) {
      HAL_I2C_Mem_Write(&hi2c1, bmp180_addr, 0xF4, 1, &cmd, 1, HAL_MAX_DELAY);
    }
    if (SysTick_20Sec_Counter == TEMPERATURE_GET) {
      temperature_result = bmp180_get_temperature();
    }
    if (SysTick_20Sec_Counter == PRESSURE_PUT) {
      uint8_t pressure_cmd = 0x34 + (OSS << 6);
      HAL_I2C_Mem_Write(&hi2c1, bmp180_addr, 0xF4, 1, &pressure_cmd, sizeof(pressure_cmd), HAL_MAX_DELAY);
    }
    if (SysTick_20Sec_Counter == PRESSURE_GET) {
      SysTick_20Sec_Counter = 0;
      pressure_result = bmp180_get_pressure() / 133.322f;
    }
    SysTick_20Sec_Counter++;
    if (SysTick_1Sec_Counter_mode == 1000) {
      SysTick_1Sec_Counter_mode = 0;
      HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
      HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
      lcd1602_transmit_command(0b10000000);
      snprintf(GLOBAL_MESSAGE_BUFFER, sizeof(GLOBAL_MESSAGE_BUFFER), "%02d:%02d:%02d", \
        time.Hours, time.Minutes, time.Seconds);
      lcd1602_send_string(GLOBAL_MESSAGE_BUFFER);
      lcd1602_transmit_command(0b11000000);
      snprintf(GLOBAL_MESSAGE_BUFFER, sizeof(GLOBAL_MESSAGE_BUFFER), "%.2ld, %.2ld, %.2f", \
        pressure_result, temperature_result, ((float)AHT10_ADC_Raw / 1048576.0) * 100.0);
      lcd1602_send_string(GLOBAL_MESSAGE_BUFFER);
    }
    SysTick_1Sec_Counter_mode++;
  } 
  if (current_mode == 1) {
    SysTick_1Sec_Counter_mode = 0;
    if (SysTick_1Sec_Counter_mode1 == 100) {
      /*Here works other block of code. If ya wanna see
        - look at 'HAL_GPIO_EXTI_Callback' function
      */
    }
    SysTick_1Sec_Counter_mode1++;
  }
  if (current_mode == 2) {
    SysTick_1Sec_Counter_mode = 0;
    SysTick_1Sec_Counter_mode1 = 0;
    if (SysTick_1Sec_Counter_mode2 == 100) {
      SysTick_1Sec_Counter_mode2 = 0;
      /*Here works other block of code. If ya wanna see
        - look at 'HAL_GPIO_EXTI_Callback' function
      */
    }
    SysTick_1Sec_Counter_mode2++;
  }

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim->Instance == TIM3) {
    HAL_TIM_Base_Stop_IT(&htim3); 
    current_mode++;
    if (current_mode > 2) {
      current_mode = 0;
    }
  }
}

int modifying_hours = 0;
int modifying_minutes = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == GPIO_PIN_13) {
    current_mode++;
    if (current_mode > 2) {
      current_mode = 0;
    }
    //HAL_TIM_Base_Start_IT(&htim3); 
  } else if (GPIO_Pin == GPIO_PIN_7) {
    /*HAL_RTC_GetTime(&hrtc, &local_modifying_time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &local_modifying_date, RTC_FORMAT_BIN);
    modifying_hours = local_modifying_time.Hours;
    modifying_minutes = local_modifying_time.Minutes;*/

    if (current_mode == 1) {
      modifying_hours++;
      if (modifying_hours >= 23) {
        modifying_hours = 0;
      }
      snprintf(hour_modifying_string, sizeof(hour_modifying_string), "  %d", modifying_hours);
      lcd1602_transmit_command(0b00000001);
      lcd1602_transmit_command(0b10000000);
      lcd1602_send_string(hour_modifying_string);
    } else if (current_mode == 2) {
      modifying_minutes++;
      if (modifying_minutes >= 59) {
        modifying_minutes = 0;
      }
      snprintf(minutes_modifying_string, sizeof(minutes_modifying_string), "  %d", modifying_minutes);
      lcd1602_transmit_command(0b00000001);
      lcd1602_transmit_command(0b10000000);
      lcd1602_send_string(minutes_modifying_string);
    } else if (current_mode == 0) {
      RTC_TimeTypeDef sTime = { .Hours = modifying_hours, .Minutes = modifying_minutes, .Seconds = 0 };
      RTC_DateTypeDef sDate = { .WeekDay = RTC_WEEKDAY_SUNDAY, .Month = RTC_MONTH_OCTOBER,
                                .Date = 19, .Year = 25 };

      HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
      HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);


      time.Hours = modifying_hours;
      time.Minutes = modifying_minutes;
      time.Seconds = 0;
    } else {
      current_mode = 0;
    }
  } else {
    __NOP();
  }
}

/* USER CODE END 1 */
