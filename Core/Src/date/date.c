#include "date.h"

void init_rtc_once(void) {
  if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != RTC_INIT_MAGIC) {
      RTC_TimeTypeDef sTime = { .Hours = 11, .Minutes = 10, .Seconds = 30 };
      RTC_DateTypeDef sDate = { .WeekDay = RTC_WEEKDAY_SUNDAY, .Month = RTC_MONTH_OCTOBER,
                                .Date = 19, .Year = 25 };

      HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
      HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

      HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, RTC_INIT_MAGIC);
  }
}