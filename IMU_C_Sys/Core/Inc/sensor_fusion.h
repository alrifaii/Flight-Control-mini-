/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sensor_fusion.h
  * @brief          : Sensor fusion (complementary filter) for IMU orientation
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SENSOR_FUSION_H
#define __SENSOR_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bmi088.h"

/* Output structure for orientation angles */
typedef struct {
    float roll;   /* Rotation around X axis (degrees) */
    float pitch;  /* Rotation around Y axis (degrees) */
    float yaw;    /* Rotation around Z axis (degrees) */
} orientation_t;

/* Function Prototypes */
void SensorFusion_Init(void);
void SensorFusion_Update(const imu_measurement_t *imu_data, orientation_t *orientation);

#ifdef __cplusplus
}
#endif

#endif /* __SENSOR_FUSION_H */
