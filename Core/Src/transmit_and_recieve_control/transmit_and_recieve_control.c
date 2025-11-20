#include "transmit_and_recieve_control.h"

void lcd1602_send_string(char *str) {
  if (sizeof(*str) == 0) {
    HAL_UART_Transmit(&huart2, (const uint8_t*)"Warning: in the send function 'lcd1602_send_string' you pass an empty string\n", \
    strlen((char *)"Warning: in the send function 'lcd1602_send_string' you pass an empty string\n"), HAL_MAX_DELAY);
  }
  while(*str) {
    lcd1602_transmit_data((uint8_t)(*str));
    str++;
  }
}

HAL_StatusTypeDef lcd1602_transmit(uint8_t data, uint8_t flags) {
	uint8_t upper_bits = data & 0xF0;
	uint8_t lower_bits = (data << 4) & 0xF0;
	uint8_t data_arr[4];                              
	data_arr[0] = upper_bits | flags | BACKLIGHT | PIN_EN; 
	data_arr[1] = upper_bits | flags | BACKLIGHT;         
	data_arr[2] = lower_bits | flags | BACKLIGHT | PIN_EN;
	data_arr[3] = lower_bits | flags | BACKLIGHT;
	if(HAL_I2C_Master_Transmit(&hi2c1, lcd1604_addr, data_arr, sizeof(data_arr), HAL_MAX_DELAY) != HAL_OK) {
    Error_Handler("Error: an error occurred while executing the 'lcd1602_transmit' function\n");
    return HAL_ERROR;
  } else {
    return HAL_OK;
  }
}

void lcd1602_transmit_data(uint8_t data) {
	lcd1602_transmit(data, PIN_RS);
}

void lcd1602_transmit_command(uint8_t cmd) {
	lcd1602_transmit(cmd, 0);
}

int32_t bmp180_get_temperature(void) {
	HAL_I2C_Mem_Read(&hi2c1, bmp180_addr, 0xF6, 1, temperature_buf, 2, HAL_MAX_DELAY); //i forgot what the fuck it is
	int32_t UT = (temperature_buf[0] << 8) + temperature_buf[1];
	X1 = ((UT-AC6)*AC5) >> 15;
	X2 = (MC << 11) / (X1 + MD);
	B5 = X1 + X2;
	int32_t T = ((B5 + 8) >> 4) / 10.0;
	return T;
}

int32_t bmp180_get_pressure(void) {
	HAL_I2C_Mem_Read(&hi2c1, bmp180_addr, 0xF6, 1, pressure_buf, 3, HAL_MAX_DELAY);
	long UP = (((long)pressure_buf[0] << 16) | ((long)pressure_buf[1] << 8) | (long)pressure_buf[2]) >> 8;
	
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
	return p;
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