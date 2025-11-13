#include "../transmit_and_recieve_control/transmit_and_recieve_control.h"
#include "../../Inc/main.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

extern uint8_t calib_data[22];

extern UART_HandleTypeDef huart2;
extern int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
extern uint16_t AC4, AC5, AC6;
extern char GLOBAL_MESSAGE_BUFFER[60];
extern uint16_t aht10_addr;
extern uint8_t aht10_init_command[1];

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
int bmp180_init(void);

/*
 * @brief function of init a aht10
          sesor
 * @param void
 * @return void
 */
int aht10_init(void);
