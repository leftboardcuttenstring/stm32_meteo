#include "main.h"

#define PIN_RS (1 << 0)
#define PIN_EN (1 << 2)
#define BACKLIGHT (1 << 3)
#define OSS 0

#define MAX_TEMPERATURE 38
#define MIN_TEMPERATURE -30
#define MAX_PRESSURE 770
#define MIN_PRESSURE 730

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern uint16_t lcd1604_addr;
extern uint16_t bmp180_addr;
extern uint8_t temperature_buf[2];
extern uint8_t pressure_buf[3];
extern int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
extern uint16_t AC4, AC5, AC6;
extern int32_t X1, X2, B5;

/*
 * @brief function for transmitting
 *        a data via I2C to 1602
 * @param uint8_t data - transmitting data
 * @return int         - result success flag
 *
 */
int lcd1602_transmit_data(uint8_t data);

/*
 * @brief function for transmitting
 *        a command via I2C to 1602
 * @param uint8_t cmd - transmitting command
 * @return int        - result success flag
 */
int lcd1602_transmit_command(uint8_t cmd);

/*
 * @brief general purpose function for 
 *        transmitting any shit what
 *        you wanna transmit
 * @param uint8_t data  - transmitting data
 * @param uint8_t flags - flags of transmitting
 * @return void
 */
void lcd1602_transmit(uint8_t data, uint8_t flags);

/*
 * @brief function of transmitting a
 *        string to show it on 1602
 * @param void
 * @return void
 */
void lcd1602_send_string(char *str);

int32_t bmp180_get_temperature(void);

int32_t bmp180_get_pressure(void);