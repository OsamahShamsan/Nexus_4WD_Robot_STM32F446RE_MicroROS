#pragma once
/*
#include "stm32f4xx_hal.h"
#include <stdint.h>

// ----- I2C device addresses -----
#define MPU6050_ADDR_AD0_LOW  (0x68 << 1) // AD0 pin = GND
#define MPU6050_ADDR_AD0_HIGH (0x69 << 1) // AD0 pin = VCC

// ----- MPU6050 Register map -----
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_ACCEL_XOUT_H  0x3B

// ----- IMU handle -----
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t addr;
    uint8_t raw[14];       // raw data buffer from sensor
    volatile uint8_t ready; // set to 1 when DMA read finishes
} mpu6050_t;

// ----- Function declarations -----
HAL_StatusTypeDef mpu6050_init(mpu6050_t *dev);
HAL_StatusTypeDef mpu6050_read_dma(mpu6050_t *dev);
void mpu6050_parse(const mpu6050_t *dev,
                   float *ax_g, float *ay_g, float *az_g,
                   float *gx_dps, float *gy_dps, float *gz_dps,
                   float *temp_c);
void mpu6050_calibrate(mpu6050_t *dev, unsigned int samples,
                       float *bias_ax, float *bias_ay, float *bias_az,
                       float *bias_gx, float *bias_gy, float *bias_gz,
                       float expected_gz);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);

*/
