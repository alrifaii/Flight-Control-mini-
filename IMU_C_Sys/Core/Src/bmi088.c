/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bmi088.c
  * @brief          : BMI088 IMU driver implementation
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bmi088.h"
#include "main.h"
#include <string.h>

/* Pin definitions */
#define BMI088_ACCEL_CS_PIN CSB_ACCEL_Pin
#define BMI088_GYRO_CS_PIN CSB_GYRO_Pin
#define BMI088_CS_GPIO GPIOA

/* Sensor selection enum */
enum sensor {
    BMI088_GYRO  = BMI088_GYRO_CS_PIN,
    BMI088_ACCEL = BMI088_ACCEL_CS_PIN
};

/* Private function prototypes */
static int bmi088_spi_write(uint8_t addr, uint8_t *data, int len, enum sensor sensor, SPI_HandleTypeDef *hspi);
static int bmi088_spi_read(uint8_t addr, uint8_t *data, int len, enum sensor sensor, SPI_HandleTypeDef *hspi);

/**
  * @brief  Write data to BMI088 via SPI
  * @param  addr: Register address
  * @param  data: Pointer to data buffer
  * @param  len: Length of data to write
  * @param  sensor: Sensor selection (ACCEL or GYRO)
  * @param  hspi: Pointer to SPI handle
  * @retval 0 on success, -1 on error
  */
static int bmi088_spi_write(uint8_t addr, uint8_t *data, int len, enum sensor sensor, SPI_HandleTypeDef *hspi) {
    static uint8_t buf[17];
    if (len > sizeof(buf)-1) return -1;
    
    buf[0] = addr;
    memcpy(buf+1, data, len);
    
    /* Set CS */
    HAL_GPIO_WritePin(BMI088_CS_GPIO, sensor, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, buf, len+1, HAL_MAX_DELAY);
    /* Unset CS */
    HAL_GPIO_WritePin(BMI088_CS_GPIO, BMI088_GYRO_CS_PIN|BMI088_ACCEL_CS_PIN, GPIO_PIN_SET);
    
    return 0;
}

/**
  * @brief  Read data from BMI088 via SPI
  * @param  addr: Register address
  * @param  data: Pointer to data buffer
  * @param  len: Length of data to read
  * @param  sensor: Sensor selection (ACCEL or GYRO)
  * @param  hspi: Pointer to SPI handle
  * @retval 0 on success, -1 on error
  */
static int bmi088_spi_read(uint8_t addr, uint8_t *data, int len, enum sensor sensor, SPI_HandleTypeDef *hspi) {
    static uint8_t obuf[18];
    static uint8_t ibuf[18];
    
    if (len > sizeof(obuf)-2) return -1;
    
    memset(obuf, 0, sizeof(obuf));
    memset(ibuf, 0, sizeof(ibuf));
    
    obuf[0] = addr | 0x80; // Set the read bit for SPI communication
    
    int offset = (sensor == BMI088_GYRO) ? 1 : 2;
    
    /* Set CS */
    HAL_GPIO_WritePin(BMI088_CS_GPIO, sensor, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, obuf, ibuf, len + offset, HAL_MAX_DELAY);
    /* Unset CS */
    HAL_GPIO_WritePin(BMI088_CS_GPIO, BMI088_GYRO_CS_PIN | BMI088_ACCEL_CS_PIN, GPIO_PIN_SET);
    
    /* Skip the dummy byte(s) */
    memcpy(data, ibuf + offset, len);
    
    return 0;
}

/**
  * @brief  Initialize BMI088 IMU
  * @param  hspi: Pointer to SPI handle
  * @retval 0 on success, -1 on error
  */
int BMI088_Init(SPI_HandleTypeDef *hspi) {
    int rv_acc = 1;
    int rv_gyr = 1;
    uint8_t acc_chip_id = 0;
    uint8_t gyr_chip_id = 0;
    uint8_t data;
    
    HAL_Delay(1); // Datasheet PG 13F
    
    /* Read chip IDs */
    (void) bmi088_spi_read(BMI088_REG_ACC_CHIP_ID, &acc_chip_id, 1, BMI088_ACCEL, hspi); // dummy read to wake ACCEL
    rv_acc = bmi088_spi_read(BMI088_REG_ACC_CHIP_ID, &acc_chip_id, 1, BMI088_ACCEL, hspi);
    (void) bmi088_spi_read(BMI088_REG_GYR_CHIP_ID, &gyr_chip_id, 1, BMI088_GYRO, hspi); // dummy read to wake GYRO
    rv_gyr = bmi088_spi_read(BMI088_REG_GYR_CHIP_ID, &gyr_chip_id, 1, BMI088_GYRO, hspi);
    
    /* Verify chip IDs */
    if (rv_acc || (acc_chip_id != BMI088_REG_ACC_CHIP_ID_VALUE)) {
        return -1;
    }
    
    if (rv_gyr || (gyr_chip_id != BMI088_REG_GYR_CHIP_VALUE)) {
        return -1;
    }
    
    /* Initialize Accelerometer */
    data = BMI088_REG_ACC_PWR_CTRL_ACCELEROMETER_ON;
    rv_acc = bmi088_spi_write(BMI088_REG_ACC_PWR_CTRL, &data, sizeof(data), BMI088_ACCEL, hspi);
    
    HAL_Delay(50); // Datasheet PG 13
    
    data = BMI088_REG_ACC_PWR_CONF_ACTIVE_MODE;
    rv_acc = bmi088_spi_write(BMI088_REG_ACC_PWR_CONF, &data, sizeof(data), BMI088_ACCEL, hspi);
    
    data = ACC_RANGE;
    rv_acc |= bmi088_spi_write(BMI088_REG_ACC_RANGE, &data, sizeof(data), BMI088_ACCEL, hspi);
    
    /* Configure accel filter to normal mode and output rate to 100 Hz and LPF to normal mode */
    data = BMI088_REG_ACC_CONF_BWP_NORMAL | BMI088_REG_ACC_CONF_ODR_100Hz;
    rv_acc |= bmi088_spi_write(BMI088_REG_ACC_CONF, &data, sizeof(data), BMI088_ACCEL, hspi);
    
    /* IO1 inactive */
    data = 0;
    rv_acc |= bmi088_spi_write(BMI088_REG_ACC_INT1_IO_CONF, &data, sizeof(data), BMI088_ACCEL, hspi);
    
    /* IO2 inactive */
    data = 0;
    rv_acc |= bmi088_spi_write(BMI088_REG_ACC_INT2_IO_CONF, &data, sizeof(data), BMI088_ACCEL, hspi);
    
    /* map_data */
    data = BMI088_REG_ACC_INT1_INT2_MAP_DATA_INT1_DRDY;
    rv_acc |= bmi088_spi_write(BMI088_REG_ACC_INT2_IO_CONF, &data, sizeof(data), BMI088_ACCEL, hspi);
    
    /* Initialize Gyroscope */
    /* Configure gyro range to 2000 DPS */
    data = BMI088_REG_GYR_GYRO_RANGE_2000DPS;
    rv_gyr = bmi088_spi_write(BMI088_REG_GYR_GYRO_RANGE, &data, sizeof(data), BMI088_GYRO, hspi);
    
    /* Configure gyro bandwidth to 47 Hz */
    data = BMI088_REG_GYR_GYRO_BANDWIDTH_47Hz;
    rv_gyr |= bmi088_spi_write(BMI088_REG_GYR_GYRO_BANDWIDTH, &data, sizeof(data), BMI088_GYRO, hspi);
    
    /* IO3 inactive */
    data = 0;
    rv_gyr |= bmi088_spi_write(BMI088_REG_GYR_INT3_INT4_IO_CONF, &data, sizeof(data), BMI088_GYRO, hspi);
    
    /* IO3 map */
    data = BMI088_REG_GYR_INT3_INT4_IO_MAP_INT3;
    rv_gyr |= bmi088_spi_write(BMI088_REG_GYR_INT3_INT4_IO_MAP, &data, sizeof(data), BMI088_GYRO, hspi);
    
    /* int off */
    data = 0;
    rv_gyr |= bmi088_spi_write(BMI088_REG_GYR_INT_CTRL, &data, sizeof(data), BMI088_GYRO, hspi);
    
    if (rv_acc || rv_gyr) {
        return -1;
    }
    
    return 0;
}

/**
  * @brief  Read IMU data from BMI088
  * @param  hspi: Pointer to SPI handle
  * @param  data: Pointer to imu_measurement structure
  * @retval 0 on success, -1 on error
  */
int BMI088_Read(SPI_HandleTypeDef *hspi, imu_measurement_t *data) {
    int16_t gyro_raw[3];
    int16_t acc_raw[3];
    
    /* Get measured raw data */
    int rv_gyr = bmi088_spi_read(BMI088_REG_GYR_DATA0, (uint8_t *)gyro_raw, sizeof(gyro_raw), BMI088_GYRO, hspi);
    int rv_acc = bmi088_spi_read(BMI088_REG_ACC_DATA0, (uint8_t *)acc_raw, sizeof(acc_raw), BMI088_ACCEL, hspi);
    
    if (rv_acc || rv_gyr) {
        return -1;
    }
    
    /* Scale data according to settings */
    for (int i = 0; i < 3; i++) {
        data->acc[i] = acc_raw[i] / 32768.0f * 1.5f * (1 << (ACC_RANGE + 1));
        data->gyr[i] = 2000.0f * gyro_raw[i] / 32767.0f;
    }
    
    return 0;
}
