#ifndef COMPASS_H
#define COMPASS_H

#include "stm32l4xx_hal.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c1;

#ifdef __cplusplus
extern "C" {
#endif

#define COMPASS_DEFAULT_ADDR 0x0C

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr7;
    float declination;
    float heading;
} Compass_t;

HAL_StatusTypeDef compass4clickInit();
HAL_StatusTypeDef heading(int16_t *deg);
HAL_StatusTypeDef compass_recover(void);

#ifdef __cplusplus
}
#endif

#endif
