#ifndef MYSENSORS_H_
#define MYSENSORS_H_

#include "main.h"
#include "global_definitions.h"



// ----------------------------------------------------------------------------------
// ----------- Sonar (Dual Ultrasonic Sensor (DUS))  --------------------------------
// ----------------------------------------------------------------------------------
typedef struct {
    UART_HandleTypeDef *huart;   // UART handle (e.g. &huart5)
    uint8_t addr;                // device address
    uint8_t rxBuf[SONAR_RX_BUF_SIZE];
} SONAR_HandleTypeDef;

void SONAR_Init(SONAR_HandleTypeDef *hsonar, UART_HandleTypeDef *huart, uint8_t addr);

int16_t SONAR_Trigger(SONAR_HandleTypeDef *hsonar);
int16_t SONAR_ReadDistance(SONAR_HandleTypeDef *hsonar);
int16_t SONAR_ReadTemperature(SONAR_HandleTypeDef *hsonar);



// ----------------------------------------------------------------------------------
// ----------- IMU sensor (MPU6050)  ------------------------------------------------
// ----------------------------------------------------------------------------------
bool mpu_init(void);
bool mpu_write(uint8_t reg, uint8_t val);
bool mpu_read(uint8_t reg, uint8_t *buf, uint16_t len);
bool mpu_read_sample(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *temp_c);

#endif /* MYSENSORS_H_ */
