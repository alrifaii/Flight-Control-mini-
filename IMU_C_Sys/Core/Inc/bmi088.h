/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bmi088.h
  * @brief          : Header for BMI088 IMU driver
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BMI088_H
#define __BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/* BMI088 I2C Addresses */
#define BMI088_GYROSCOPE_I2C_ADDR(x) ((0x68 | ((x) & 1)) << 1)
#define BMI088_ACCELEROMETER_I2C_ADDR(x) ((0x18 | ((x) & 1)) << 1)

#define BMI088_REG_ADDR_LEN 1

/* Accelerometer Registers */
#define BMI088_REG_ACC_CHIP_ID                      0x00
#define BMI088_REG_ACC_CHIP_ID_VALUE                0x1E
#define BMI088_REG_ACC_ERR_REG                      0x02
#define BMI088_REG_ACC_STATUS                       0x03
#define BMI088_REG_ACC_DATA0                        0x12
#define BMI088_REG_ACC_CONF                         0x40
#define BMI088_REG_ACC_CONF_BWP_NORMAL              (0x0A << 4)
#define BMI088_REG_ACC_CONF_BWP_OSR2                (0x09 << 4)
#define BMI088_REG_ACC_CONF_BWP_OSR4                (0x08 << 4)
#define BMI088_REG_ACC_CONF_ODR_100Hz               (0x08)
#define BMI088_REG_ACC_CONF_ODR_200Hz               (0x09)
#define BMI088_REG_ACC_CONF_ODR_400Hz               (0x0A)
#define BMI088_REG_ACC_CONF_ODR_800Hz               (0x0B)
#define BMI088_REG_ACC_CONF_ODR_1600Hz              (0x0C)
#define BMI088_REG_ACC_RANGE                        0x41
#define BMI088_REG_ACC_RANGE_3G                     0x00
#define BMI088_REG_ACC_RANGE_6G                     0x01
#define BMI088_REG_ACC_RANGE_12G                    0x02
#define BMI088_REG_ACC_RANGE_24G                    0x03
#define BMI088_REG_ACC_PWR_CTRL                     0x7D
#define BMI088_REG_ACC_PWR_CTRL_ACCELEROMETER_ON    0x04
#define BMI088_REG_ACC_PWR_CONF                     0x7C
#define BMI088_REG_ACC_PWR_CONF_ACTIVE_MODE         0x00
#define BMI088_REG_ACC_INT1_IO_CONF                 0x53
#define BMI088_REG_ACC_INT1_IO_CONF_INPUT           (1<<4)
#define BMI088_REG_ACC_INT1_IO_CONF_OUTPUT          (1<<3)
#define BMI088_REG_ACC_INT1_IO_CONF_OD              (1<<2)
#define BMI088_REG_ACC_INT1_IO_CONF_ACTIVE_HIGH     (1<<1)
#define BMI088_REG_ACC_INT2_IO_CONF                 0x54
#define BMI088_REG_ACC_INT2_IO_CONF_INPUT           (1<<4)
#define BMI088_REG_ACC_INT2_IO_CONF_OUTPUT          (1<<3)
#define BMI088_REG_ACC_INT2_IO_CONF_OD              (1<<2)
#define BMI088_REG_ACC_INT2_IO_CONF_ACTIVE_HIGH     (1<<1)
#define BMI088_REG_ACC_INT1_INT2_MAP_DATA           0x58
#define BMI088_REG_ACC_INT1_INT2_MAP_DATA_INT1_DRDY (1<<2)

/* Gyroscope Registers */
#define BMI088_REG_GYR_CHIP_ID              0x00
#define BMI088_REG_GYR_CHIP_VALUE           0x0F
#define BMI088_REG_GYR_GYRO_RANGE           0x0F
#define BMI088_REG_GYR_GYRO_RANGE_2000DPS   0x00
#define BMI088_REG_GYR_GYRO_BANDWIDTH       0x10
#define BMI088_REG_GYR_GYRO_BANDWIDTH_23Hz  0x04
#define BMI088_REG_GYR_GYRO_BANDWIDTH_47Hz  0x03
#define BMI088_REG_GYR_GYRO_BANDWIDTH_116Hz 0x02
#define BMI088_REG_GYR_DATA0                0x02
#define BMI088_REG_GYR_INT3_INT4_IO_CONF    0x16
#define BMI088_REG_GYR_INT3_INT4_IO_CONF_INT3_OD          (1<<1)
#define BMI088_REG_GYR_INT3_INT4_IO_CONF_INT3_ACTIVE_HIGH (1<<0)
#define BMI088_REG_GYR_INT3_INT4_IO_MAP     0x18
#define BMI088_REG_GYR_INT3_INT4_IO_MAP_INT3  0x01
#define BMI088_REG_GYR_INT_CTRL             0x15
#define BMI088_REG_GYR_INT_CTRL_ON          0x80

/* Configuration */
#define ACC_RANGE BMI088_REG_ACC_RANGE_24G

/* Data Structure */
typedef struct {
    float acc[3];  // Acceleration in G
    float gyr[3];  // Angular velocity in DPS
} imu_measurement_t;

/* Function Prototypes */
int BMI088_Init(SPI_HandleTypeDef *hspi);
int BMI088_Read(SPI_HandleTypeDef *hspi, imu_measurement_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_H */
