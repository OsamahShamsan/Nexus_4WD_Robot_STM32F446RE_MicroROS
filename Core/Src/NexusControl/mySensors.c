

#include "mySensors.h"


extern I2C_HandleTypeDef hi2c1;


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

