#ifndef MT_PORT
#define MT_PORT

#include "../../../Inc/main.h"

void MT_PORT_SetTimerModule(TIM_HandleTypeDef* timer);
void MT_PORT_SetUartModule(UART_HandleTypeDef* uart);

#endif