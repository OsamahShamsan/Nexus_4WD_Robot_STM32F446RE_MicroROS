#ifndef IMU_INTERFACE_H_
#define IMU_INTERFACE_H_

#include <stdbool.h>

// ----------- IMU MPU6050 -----------------------------------------------------------
#define MPU_ADDR_7B  0x68
#define MPU_ADDR     (MPU_ADDR_7B << 1)   // HAL uses 8-bit address
#define GRAVITY      9.80665f
#define DEG2RAD      0.017453292519943295f


// ----------------------------------------------------------------------------------
// ----------- IMU sensor (MPU6050)  ------------------------------------------------
// ----------------------------------------------------------------------------------
bool mpu_init(void);
bool mpu_write(uint8_t reg, uint8_t val);
bool mpu_read(uint8_t reg, uint8_t *buf, uint16_t len);
bool mpu_read_sample(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *temp_c);


#endif /* IMU_INTERFACE_H_ */
