#include "date.h"

int rtc_set_time(
        uint8_t year, uint8_t month, uint8_t day,
        uint8_t hour, uint8_t min, uint8_t sec,
        uint8_t dow) {
    HAL_StatusTypeDef res;
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    memset(&time, 0, sizeof(time));
    memset(&date, 0, sizeof(date));

    date.WeekDay = dow;
    date.Year = year;
    date.Month = month;
    date.Date = day;
    res = HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
    if(res != HAL_OK) {
        //HAL_UART_Transmit(&huart2, (uint8_t*)"Time set error", sizeof("Time set error")-1, HAL_MAX_DELAY);
        return -1;
    }
    time.Hours = hour;
    time.Minutes = min;
    time.Seconds = sec;

    res = HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
    if(res != HAL_OK) {
        //HAL_UART_Transmit(&huart2, (uint8_t*)"Time set error", sizeof("Time set error")-1, HAL_MAX_DELAY);
        return -2;
    }

    return 0;
}

void rtc_get_time(void) {
	//HAL_UART_Transmit(&huart2, (uint8_t*)"Time getting...", sizeof("Time getting...")-1, HAL_MAX_DELAY);
	res = HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	if(res != HAL_OK) {
    //HAL_UART_Transmit(&huart2, (uint8_t*)"Time get error", sizeof("Time get error")-1, HAL_MAX_DELAY);
	}
	sprintf(msg_time, "%d:%d", time.Hours, time.Minutes);
	lcd1602_transmit_command(0b10000000);
	lcd1602_send_string(msg_time);
}