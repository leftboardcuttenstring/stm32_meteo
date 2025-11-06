#include <string.h>
#include <stdio.h>

#include "../transmit_and_recieve_control/transmit_and_recieve_control.h"
#include "../../Inc/main.h"

extern RTC_TimeTypeDef time;
extern RTC_DateTypeDef date;
extern HAL_StatusTypeDef res;
extern RTC_HandleTypeDef hrtc;
extern HAL_StatusTypeDef res;
extern char msg_time[32];

int rtc_set_time(uint8_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t dow);

void rtc_get_time(void);