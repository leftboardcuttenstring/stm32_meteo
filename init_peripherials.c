#include "init_peripherials.h"

void lcd1602_init(void) {
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
