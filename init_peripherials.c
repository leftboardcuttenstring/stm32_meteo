#include "init_peripherials.h"

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
	uint8_t cmd = 0x2E;
	if (HAL_I2C_Mem_Write(&hi2c1, bmp180_addr, 0xF4, 1, &cmd, 1, HAL_MAX_DELAY) == HAL_OK) {
		HAL_UART_Transmit(&huart2, (const uint8_t*)"Barometer init is done\r\n", sizeof("Barometer init is done\r\n")-1, HAL_MAX_DELAY);
	}
	HAL_Delay(5);
}
