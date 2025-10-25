/*

#include "cmsis_os.h"   // for osDelay during calibration
#include "stm32f4xx_hal.h"
#include "mpu6050.h"

extern I2C_HandleTypeDef hi2c1;

extern void MX_I2C1_Init(void);

// Choose full-scale ranges
static inline uint8_t accel_cfg_val(void) { return 0 << 3; } // ±2g
static inline uint8_t gyro_cfg_val(void)  { return 0 << 3; } // ±250 dps


// ----------------- Initialization -----------------
HAL_StatusTypeDef mpu6050_init(mpu6050_t *dev)
{
    uint8_t d;

    // Wake up device (clear sleep bit)
    d = 0x00;
    if (HAL_I2C_Mem_Write(dev->hi2c, dev->addr, MPU6050_REG_PWR_MGMT_1, 1,
                          &d, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    // DLPF: 20 Hz (CONFIG = 4)
    d = 4;
    if (HAL_I2C_Mem_Write(dev->hi2c, dev->addr, MPU6050_REG_CONFIG, 1,
                          &d, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    // Sample rate divider (1kHz / (1 + 4) = 200 Hz)
    d = 4;
    if (HAL_I2C_Mem_Write(dev->hi2c, dev->addr, MPU6050_REG_SMPLRT_DIV, 1,
                          &d, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    // Accel ±2g
    d = accel_cfg_val();
    if (HAL_I2C_Mem_Write(dev->hi2c, dev->addr, MPU6050_REG_ACCEL_CONFIG, 1,
                          &d, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    // Gyro ±250 dps
    d = gyro_cfg_val();
    if (HAL_I2C_Mem_Write(dev->hi2c, dev->addr, MPU6050_REG_GYRO_CONFIG, 1,
                          &d, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}


// ----------------- DMA read -----------------
HAL_StatusTypeDef mpu6050_read_dma(mpu6050_t *dev)
{
    dev->ready = 0;
    return HAL_I2C_Mem_Read_DMA(dev->hi2c, dev->addr,
                                MPU6050_REG_ACCEL_XOUT_H, 1,
                                dev->raw, 14);
}

// ----------------- Parsing -----------------
static int16_t be16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]);
}


*/

/*
void mpu6050_parse(const mpu6050_t *dev,
                   float *ax_g, float *ay_g, float *az_g,
                   float *gx_dps, float *gy_dps, float *gz_dps,
                   float *temp_c)
{
    int16_t ax = be16(&dev->raw[0]);
    int16_t ay = be16(&dev->raw[2]);
    int16_t az = be16(&dev->raw[4]);
    int16_t t  = be16(&dev->raw[6]);
    int16_t gx = be16(&dev->raw[8]);
    int16_t gy = be16(&dev->raw[10]);
    int16_t gz = be16(&dev->raw[12]);

    const float accel_sens = 16384.0f; // LSB/g for ±2g
    const float gyro_sens  = 131.0f;   // LSB/(°/s) for ±250 dps

    if (ax_g) *ax_g = ax / accel_sens;
    if (ay_g) *ay_g = ay / accel_sens;
    if (az_g) *az_g = az / accel_sens;
    if (gx_dps) *gx_dps = gx / gyro_sens;
    if (gy_dps) *gy_dps = gy / gyro_sens;
    if (gz_dps) *gz_dps = gz / gyro_sens;
    if (temp_c) *temp_c = (t / 340.0f) + 36.53f;
}

*/


/*
// ----------------- Calibration -----------------
void mpu6050_calibrate(mpu6050_t *dev, unsigned int samples,
                       float *bias_ax, float *bias_ay, float *bias_az,
                       float *bias_gx, float *bias_gy, float *bias_gz,
                       float expected_gz)
{
    double sax = 0, say = 0, saz = 0;
    double sgx = 0, sgy = 0, sgz = 0;

    for (unsigned int i = 0; i < samples; i++) {
        // Wait until DMA transfer completes
        while (!dev->ready) { osDelay(1); }
        dev->ready = 0;

        float ax, ay, az, gx, gy, gz, temp;
        mpu6050_parse(dev, &ax, &ay, &az, &gx, &gy, &gz, &temp);

        sax += ax; say += ay; saz += az;
        sgx += gx; sgy += gy; sgz += gz;

        // Trigger next DMA read
        mpu6050_read_dma(dev);
    }

    if (bias_ax) *bias_ax = (float)(sax / samples);
    if (bias_ay) *bias_ay = (float)(say / samples);
    if (bias_az) *bias_az = (float)(saz / samples) - expected_gz;

    if (bias_gx) *bias_gx = (float)(sgx / samples);
    if (bias_gy) *bias_gy = (float)(sgy / samples);
    if (bias_gz) *bias_gz = (float)(sgz / samples);
}

*/
