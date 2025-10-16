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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
typedef struct {
	short Temperature;
	float Pressure;
	unsigned short Humdity;
} Data;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYSTICK_1SEC_VALUE 1000
#define SYSTICK_20SEC_VALUE (SYSTICK_1SEC_VALUE * 20)
#define TEMP_MIN  (-30)
#define TEMP_MAX  (40)
#define PRESS_MIN (700.0)
#define PRESS_MAX (790.0)
#define HUMD_MIN  (10)
#define HUMD_MAX  (90)
#define LOG_SIZE  (10)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
unsigned int SysTick_1Sec_Counter = 0;
unsigned int SysTick_20Sec_Counter = 0;
Data Log[LOG_SIZE] = {0};
unsigned char LogCounter = 0;
extern UART_HandleTypeDef huart2;
//unsigned int counter_1s = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void GetData(void);
void AveragingData(void);
void LogData(Data Current);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

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
	/*GetData();
	AveragingData();*/
	/*GetData();
	AveragingData();*/
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
	if(SysTick_1Sec_Counter == 1000) {
		/*SysTick_1Sec_Counter = 0;
		HAL_UART_Transmit(&huart2, "1s\r\n", sizeof("1s\r\n")-1, HAL_MAX_DELAY);*/
		GetData();
	}
	SysTick_1Sec_Counter++;	
	if(SysTick_20Sec_Counter == 20000) {
		/*SysTick_20Sec_Counter = 0;
		HAL_UART_Transmit(&huart2, "20s\r\n", sizeof("20s\r\n")-1, HAL_MAX_DELAY);*/
		AveragingData();
	}
	SysTick_20Sec_Counter++;

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
	//HAL_UART_Transmit(&huart2, (const uint8_t*)"Processing the button pushing...\r\n", sizeof("Processing the button pushing...\r\n")-1, HAL_MAX_DELAY);
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void GetData(void)
{
	if (SysTick_1Sec_Counter == SYSTICK_1SEC_VALUE)
	{
		SysTick_1Sec_Counter = 0;
		//optionally
		//HAL_UART_Transmit(&huart2, (const uint8_t*)"Getting the data...\r\n", sizeof("Getting the data...\r\n")-1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		for(int i=0; i<500000; i++);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	}
	SysTick_1Sec_Counter++;
}

void AveragingData(void)
{
	if (SysTick_20Sec_Counter == SYSTICK_20SEC_VALUE)
	{
		SysTick_20Sec_Counter = 0;
		//optionally
		//HAL_UART_Transmit(&huart2, (const uint8_t*)"Averaging the data...\r\n", sizeof("Averaging the data...\r\n")-1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		for(int i=0; i<500000; i++);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
	}
	SysTick_20Sec_Counter++;
}

void LogData(Data Current)
{
	if (LogCounter > LOG_SIZE)
	{
		LogCounter = 0;
	}
	Log[LogCounter].Humdity = Current.Humdity;
	Log[LogCounter].Pressure = Current.Pressure;
	Log[LogCounter].Temperature = Current.Temperature;
	LogCounter++;
}
/* USER CODE END 1 */
