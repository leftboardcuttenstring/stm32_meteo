#include "init_peripherials.h"

uint8_t cmd = 0x2E;

void lcd1602_init(void) {	
  HAL_Delay(50);
  lcd1602_transmit_command(0x33);
  HAL_Delay(5);
  lcd1602_transmit_command(0x32);
  HAL_Delay(1);
  lcd1602_transmit_command(0x28);
  HAL_Delay(1);
  lcd1602_transmit_command(0x08);
  HAL_Delay(1);
  lcd1602_transmit_command(0x01);
  HAL_Delay(2);
  lcd1602_transmit_command(0x06);
  HAL_Delay(1);
  lcd1602_transmit_command(0x0C);
  HAL_Delay(1);
	HAL_UART_Transmit(&huart2, (const uint8_t*)"LCD init is done\r\n", sizeof("LCD init is done\r\n")-1, HAL_MAX_DELAY);
}

void bmp180_init(void) {
	if (HAL_I2C_Mem_Write(&hi2c1, bmp180_addr, 0xF4, 1, &cmd, 1, HAL_MAX_DELAY) == HAL_OK) {
		//HAL_UART_Transmit(&huart2, (const uint8_t*)"Barometer init is done\r\n", sizeof("Barometer init is done\r\n")-1, HAL_MAX_DELAY);
	}
}

int aht10_init(void) {
  if (HAL_I2C_Master_Transmit(&hi2c1, aht10_addr, (uint8_t *)aht10_initialization_command, sizeof(aht10_initialization_command), HAL_MAX_DELAY) == HAL_OK) {
    //HAL_Delay(5);
    if (HAL_I2C_IsDeviceReady(&hi2c1, aht10_addr, 3, 100) == HAL_OK) {
      snprintf((char *)GLOBAL_MESSAGE_BUFFER, sizeof(GLOBAL_MESSAGE_BUFFER), "Success: AHT10 initalization is done\n");
      HAL_UART_Transmit(&huart2, (const uint8_t *)GLOBAL_MESSAGE_BUFFER, strlen((char *)GLOBAL_MESSAGE_BUFFER), HAL_MAX_DELAY);
      return 0;
    } else {
      snprintf((char *)GLOBAL_MESSAGE_BUFFER, sizeof(GLOBAL_MESSAGE_BUFFER), "Failure: AHT10 does not respond\n");
      HAL_UART_Transmit(&huart2, (const uint8_t *)GLOBAL_MESSAGE_BUFFER, strlen((char *)GLOBAL_MESSAGE_BUFFER), HAL_MAX_DELAY);
      return 1;
    }
  } else {
    snprintf((char *)GLOBAL_MESSAGE_BUFFER, sizeof(GLOBAL_MESSAGE_BUFFER), "Failure: error transmitting AHT10 initialization command\n");
    HAL_UART_Transmit(&huart2, (const uint8_t *)GLOBAL_MESSAGE_BUFFER, strlen((char *)GLOBAL_MESSAGE_BUFFER), HAL_MAX_DELAY);
    return 1;
  }
  return 0;
}