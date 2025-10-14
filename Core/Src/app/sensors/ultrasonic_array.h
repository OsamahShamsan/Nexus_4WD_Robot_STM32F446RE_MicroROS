#ifndef ULTRASONIC_ARRAY_H_
#define ULTRASONIC_ARRAY_H_

#include "main.h"
// ----------- SONAR ----------------------------------------------------------------
#define NUM_SONARS  					4					// number of sensors: 4x Dual ultrasonic sensors (DUS)

#define SONAR_HEADER1   				0x55
#define SONAR_HEADER2   				0xAA

// Default device address
#define SONAR_DEFAULT_ADDR 				0x11

// RS485 DE/RE control pin
#define SONAR_DE_RE_GPIO_Port 			GPIOC
#define SONAR_DE_RE_Pin 				GPIO_PIN_5

// Buffer sizes
#define SONAR_RX_BUF_SIZE 				16

// Error codes
#define SONAR_TIMEOUT     				-2
#define SONAR_INVALID     				-3

// SONAR_ReadDistance() units: 1.0 if meters, 0.01 if cm, 0.001 if mm.
#define SONAR_DIST_SCALE_M 				0.01f
#define SONAR_MIN_RANGE_M  				0.02f
#define SONAR_MAX_RANGE_M  				4.00f
#define SONAR_FOV_RAD      				0.52f      // ~30 degrees

// ----------- RS485 ----------------------------------------------------------------
#define RS485_DIR_GPIO_Port 			GPIOC
#define RS485_DIR_Pin       			GPIO_PIN_5

#define COLLISION_THRESHOLD 			150   				// stop if obstacle closer than 150 mm


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

#endif /* ULTRASONIC_ARRAY_H_ */
