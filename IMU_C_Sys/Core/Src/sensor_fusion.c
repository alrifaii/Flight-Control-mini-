/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sensor_fusion.c
  * @brief          : Complementary filter sensor fusion implementation
  ******************************************************************************
  */
/* USER CODE END Header */

#include "sensor_fusion.h"
#include <math.h>
#include <string.h>

/* Complementary filter state */
static struct {
    float theta_gx;  /* Gyro integrated roll */
    float theta_gy;  /* Gyro integrated pitch */
    float theta_gz;  /* Gyro integrated yaw */
    float _ax[4];    /* Accel buffer for moving average */
    float _ay[4];
    float _az[4];
    uint32_t last_tick;
} fusion_state = {0};

/* Filter parameter: 75% accel, 25% gyro */
#define FUSION_ALPHA 0.25f

/**
  * @brief Initialize sensor fusion
  */
void SensorFusion_Init(void) {
    memset(&fusion_state, 0, sizeof(fusion_state));
    fusion_state.last_tick = HAL_GetTick();
}

/**
  * @brief Calculate time delta
  */
static float get_dt(void) {
    uint32_t current_tick = HAL_GetTick();
    float dt = (float)(current_tick - fusion_state.last_tick) / 1000.0f;
    fusion_state.last_tick = current_tick;
    
    if (dt < 0.001f) dt = 0.001f;  /* Minimum 1ms */
    if (dt > 1.0f) dt = 1.0f;      /* Maximum 1s (safety) */
    
    return dt;
}

/**
  * @brief Low-pass filter using moving average
  */
static void apply_moving_average(float *buffer, float new_sample, float *output) {
    /* Shift buffer down */
    memmove(buffer+1, buffer, 3*sizeof(float));
    buffer[0] = new_sample;
    
    /* Calculate average */
    *output = 0.0f;
    for (int i = 0; i < 4; i++) {
        *output += buffer[i] * 0.25f;
    }
}

/**
  * @brief Update sensor fusion with new IMU data
  * @param imu_data: Pointer to new IMU measurement
  * @param orientation: Pointer to output orientation angles
  */
void SensorFusion_Update(const imu_measurement_t *imu_data, orientation_t *orientation) {
    float dt = get_dt();
    
    /* (1) Apply low-pass filter to accelerometer data */
    float ax, ay, az;
    apply_moving_average(fusion_state._ax, imu_data->acc[0], &ax);
    apply_moving_average(fusion_state._ay, imu_data->acc[1], &ay);
    apply_moving_average(fusion_state._az, imu_data->acc[2], &az);
    
    /* (2) Convert gyro from DPS to rad/s */
    float gx = imu_data->gyr[0] / 180.0f * M_PI;
    float gy = imu_data->gyr[1] / 180.0f * M_PI;
    float gz = imu_data->gyr[2] / 180.0f * M_PI;
    
    /* (3) Calculate accelerometer angles */
    float theta_ax = atan2f(ay, az);
    float theta_ay = atan2f(-ax, sqrtf(ay*ay + az*az));
    
    /* (4) Integrate gyroscope to get angle changes */
    fusion_state.theta_gx += gx * dt;
    fusion_state.theta_gy += gy * dt;
    fusion_state.theta_gz += gz * dt;
    
    /* (5) Reset gyro drift if too large (accel provides reference) */
    if (fabsf(fusion_state.theta_gx - theta_ax) > M_PI/32.0f) {
        fusion_state.theta_gx = theta_ax;
    }
    if (fabsf(fusion_state.theta_gy - theta_ay) > M_PI/32.0f) {
        fusion_state.theta_gy = theta_ay;
    }
    
    /* (6) Complementary filter: weighted average */
    float theta_x = FUSION_ALPHA * fusion_state.theta_gx + (1.0f - FUSION_ALPHA) * theta_ax;
    float theta_y = FUSION_ALPHA * fusion_state.theta_gy + (1.0f - FUSION_ALPHA) * theta_ay;
    float theta_z = fusion_state.theta_gz;
    
    /* (7) Align to board orientation (USB connector points backwards) */
    float roll = -theta_y;
    float pitch = theta_x;
    float yaw = theta_z;
    
    /* Convert to degrees */
    orientation->roll = roll * 180.0f / M_PI;
    orientation->pitch = pitch * 180.0f / M_PI;
    orientation->yaw = yaw * 180.0f / M_PI;
}
