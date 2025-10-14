

#include <stdint.h>
#include "imu_interface.h"
#include "robot_params.h"

/*
extern I2C_HandleTypeDef hi2c1;

// ----------------------------------------------------------------------------------
// ----------- IMU sensor (MPU6050)  ------------------------------------------------
// ----------------------------------------------------------------------------------
bool mpu_init(void)
{
  HAL_Delay(50);
  uint8_t who = 0;
  if (!mpu_read(0x75, &who, 1) || who != 0x68) return false;

  if (!mpu_write(0x6B, 0x01)) return false; // PWR_MGMT_1: PLL, wake
  HAL_Delay(10);
  if (!mpu_write(0x6C, 0x00)) return false; // PWR_MGMT_2: enable all axes
  if (!mpu_write(0x1A, 0x03)) return false; // CONFIG: DLPF=3
  if (!mpu_write(0x1B, 0x18)) return false; // GYRO_CONFIG: 2000 dps
  if (!mpu_write(0x1C, 0x08)) return false; // ACCEL_CONFIG: 4g
  if (!mpu_write(0x19, 0x04)) return false; // SMPLRT_DIV: 200 Hz
  HAL_Delay(10);
  return true;
}

bool mpu_write(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK;
}

bool mpu_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK;
}


bool mpu_read_sample(float *ax, float *ay, float *az,
                            float *gx, float *gy, float *gz,
                            float *temp_c)
{
  uint8_t b[14];
  if (!mpu_read(0x3B, b, 14)) return false;

  int16_t raw_ax = (int16_t)((b[0] << 8) | b[1]);
  int16_t raw_ay = (int16_t)((b[2] << 8) | b[3]);
  int16_t raw_az = (int16_t)((b[4] << 8) | b[5]);
  int16_t raw_t  = (int16_t)((b[6] << 8) | b[7]);
  int16_t raw_gx = (int16_t)((b[8] << 8) | b[9]);
  int16_t raw_gy = (int16_t)((b[10] << 8) | b[11]);
  int16_t raw_gz = (int16_t)((b[12] << 8) | b[13]);

  *ax = ((float)raw_ax / 8192.0f) * GRAVITY;
  *ay = ((float)raw_ay / 8192.0f) * GRAVITY;
  *az = ((float)raw_az / 8192.0f) * GRAVITY;

  *gx = ((float)raw_gx / 16.4f) * DEG2RAD;
  *gy = ((float)raw_gy / 16.4f) * DEG2RAD;
  *gz = ((float)raw_gz / 16.4f) * DEG2RAD;

  *temp_c = ((float)raw_t / 340.0f) + 36.53f;
  return true;
}

*/
