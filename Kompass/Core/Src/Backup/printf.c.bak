//
// Created by VolkerTenta on 12/12/2023.
//

#include "printf.h"

extern UART_HandleTypeDef huart2;
//Retargeting so the printf function is usable
PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
