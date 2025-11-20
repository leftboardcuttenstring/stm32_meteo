#include <string.h>
#include <stdio.h>

#include "../transmit_and_recieve_control/transmit_and_recieve_control.h"
#include "../../Inc/main.h"

#define RTC_INIT_MAGIC 0x32F2

extern RTC_TimeTypeDef time;
extern RTC_DateTypeDef date;
extern HAL_StatusTypeDef res;
extern RTC_HandleTypeDef hrtc;
extern HAL_StatusTypeDef res;

void init_rtc_once(void);