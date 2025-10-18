#include "transmit_and_recieve_control.h"
#include "main.h"

extern uint8_t calib_data[22];

extern UART_HandleTypeDef huart2;
extern int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
extern uint16_t AC4, AC5, AC6;

/*
 * @brief function for init 1602 lcd
 *        display
 * @param void
 * @return void
 */
void lcd1602_init(void);

/*
 * @brief function of transmitting a
 *        string to show it on 1602
 * @param void
 * @return void
 */
void bmp180_init(void);
