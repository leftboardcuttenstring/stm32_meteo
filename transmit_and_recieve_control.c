#include "transmit_and_recieve_control.h"

void lcd1602_send_string(char *str) {
    while(*str) {
        lcd1602_transmit_data((uint8_t)(*str));
        str++;
    }
}

void lcd1602_transmit(uint8_t data, uint8_t flags) {
	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;
	
	uint8_t data_arr[4];                              
	data_arr[0] = up|flags|BACKLIGHT|PIN_EN; 
	data_arr[1] = up|flags|BACKLIGHT;         
	data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
	data_arr[3] = lo|flags|BACKLIGHT;
	HAL_I2C_Master_Transmit(&hi2c1, lcd1604_addr, data_arr, sizeof(data_arr), HAL_MAX_DELAY);
	HAL_Delay(10);
}

int lcd1602_transmit_data(uint8_t data) {
	lcd1602_transmit(data, PIN_RS);
}

int lcd1602_transmit_command(uint8_t cmd) {
	lcd1602_transmit(cmd, 0);
}

int32_t bmp180_get_temperature(void) {
	HAL_I2C_Mem_Read(&hi2c1, bmp180_addr, 0xF6, 1, temperature_buf, 2, HAL_MAX_DELAY); //i forgot what the fuck it is
	int32_t UT = (temperature_buf[0] << 8) + temperature_buf[1];
	X1 = ((UT-AC6)*AC5) >> 15;
	X2 = (MC << 11) / (X1 + MD);
	B5 = X1 + X2;
	int32_t T = ((B5 + 8) >> 4);
	
	if (T / 10.0 > MAX_TEMPERATURE) {
		HAL_UART_Transmit(&huart2, (const uint8_t*)"The temperature is above the maximum! (38 celsius)\r\n", \
			sizeof("The temperature is above the maximum! (38 celsius)\r\n")-1, HAL_MAX_DELAY);
		return T;
	} else if (T / 10.0 < MIN_TEMPERATURE) {
		HAL_UART_Transmit(&huart2, (const uint8_t*)"The temperature is below the minimum! (-40 celsius)\r\n", \
			sizeof("The temperature is below the minimum! (-40 celsius)\r\n")-1, HAL_MAX_DELAY);
		return T;
	}
	
	return T;
}

int32_t bmp180_get_pressure(void) {
	uint32_t pressure_cmd = 0x34 + (OSS << 6);
	
	HAL_I2C_Mem_Write(&hi2c1, bmp180_addr, 0xF4, 1, &pressure_cmd, sizeof(pressure_cmd), HAL_MAX_DELAY);
	HAL_Delay(100);
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
	
	if (p / 133.322f > MAX_PRESSURE) {
		HAL_UART_Transmit(&huart2, (const uint8_t*)"The pressure is above the maximum! (770 millimeters of mercury)\r\n", \
			sizeof("The pressure is above the maximum! (770 millimeters of mercury)\r\n")-1, HAL_MAX_DELAY);
		return p;
	} else if (p / 133.322f < MIN_PRESSURE) {
		HAL_UART_Transmit(&huart2, (const uint8_t*)"The pressure is below the minimum! (730 millimeters of mercury)\r\n", \
			sizeof("The pressure is below the minimum! (730 millimeters of mercury)\r\n")-1, HAL_MAX_DELAY);
		return p;
	}
	
	return p;
}