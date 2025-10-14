#include "ultrasonic_array.h"

// ----------------------------------------------------------------------------------
// ----------- Sonar (Dual Ultrasonic Sensor (DUS))  --------------------------------
// ----------------------------------------------------------------------------------
static void RS485_SetTX(void) { HAL_GPIO_WritePin(SONAR_DE_RE_GPIO_Port, SONAR_DE_RE_Pin, GPIO_PIN_SET); }
static void RS485_SetRX(void) { HAL_GPIO_WritePin(SONAR_DE_RE_GPIO_Port, SONAR_DE_RE_Pin, GPIO_PIN_RESET); }

static uint8_t calcChecksum(uint8_t *data, uint8_t len) {
	uint16_t sum = 0;
	for (uint8_t i = 0; i < len; i++) sum += data[i];
	return (uint8_t)(sum & 0xFF);
}

void SONAR_Init(SONAR_HandleTypeDef *hsonar, UART_HandleTypeDef *huart, uint8_t addr) {
	hsonar->huart = huart;
	hsonar->addr = addr;
	memset(hsonar->rxBuf, 0, SONAR_RX_BUF_SIZE);
	RS485_SetRX();
}


int16_t SONAR_Trigger(SONAR_HandleTypeDef *hsonar) {
    uint8_t cmd[6] = {SONAR_HEADER1, SONAR_HEADER2, hsonar->addr, 0x00, 0x01, 0x00};
    cmd[5] = calcChecksum(cmd, 5);

    RS485_SetTX();
    HAL_UART_Transmit(hsonar->huart, cmd, sizeof(cmd), 10);
    RS485_SetRX();

    //HAL_Delay(30); // wait measurement time
    return 0;
}

int16_t SONAR_ReadDistance(SONAR_HandleTypeDef *hsonar) {
    uint8_t cmd[6] = {SONAR_HEADER1, SONAR_HEADER2, hsonar->addr, 0x00, 0x02, 0x00};
    cmd[5] = calcChecksum(cmd, 5);

    RS485_SetTX();
    HAL_UART_Transmit(hsonar->huart, cmd, sizeof(cmd), 10);
    RS485_SetRX();

    if (HAL_UART_Receive(hsonar->huart, hsonar->rxBuf, 8, 10) != HAL_OK)
        return SONAR_TIMEOUT;

    // Checksum
    uint8_t sum = calcChecksum(hsonar->rxBuf, 7);
    if (sum != hsonar->rxBuf[7]) return SONAR_INVALID;

    if (hsonar->rxBuf[5] == 0xFF && hsonar->rxBuf[6] == 0xFF)
        return -1; // out of range

    int16_t dist = ((hsonar->rxBuf[5] << 8) | hsonar->rxBuf[6]);
    return dist;
}

int16_t SONAR_ReadTemperature(SONAR_HandleTypeDef *hsonar) {
    uint8_t cmd[6] = {SONAR_HEADER1, SONAR_HEADER2, hsonar->addr, 0x00, 0x03, 0x00};
    cmd[5] = calcChecksum(cmd, 5);

    RS485_SetTX();
    HAL_UART_Transmit(hsonar->huart, cmd, sizeof(cmd), 10);
    RS485_SetRX();

    if (HAL_UART_Receive(hsonar->huart, hsonar->rxBuf, 8, 10) != HAL_OK)
        return SONAR_TIMEOUT;

    uint8_t sum = calcChecksum(hsonar->rxBuf, 7);
    if (sum != hsonar->rxBuf[7]) return SONAR_INVALID;

    if (hsonar->rxBuf[5] == 0xFF && hsonar->rxBuf[6] == 0xFF)
        return -999; // invalid

    int16_t raw = ((hsonar->rxBuf[5] & 0x0F) << 8) | hsonar->rxBuf[6];
    if ((hsonar->rxBuf[5] & 0xF0) == 0) {
        return raw; // positive temp (x0.1 °C)
    } else {
        return -raw; // negative temp
    }
}
