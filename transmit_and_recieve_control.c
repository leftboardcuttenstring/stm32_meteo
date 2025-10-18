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
