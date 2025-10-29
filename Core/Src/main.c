/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microros/rmw_microros.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <builtin_interfaces/msg/time.h>

#include <std_msgs/msg/header.h>
#include <std_msgs/msg/float32_multi_array.h>
#include "std_msgs/msg/detail/multi_array_layout__functions.h"
#include "std_msgs/msg/detail/multi_array_dimension__functions.h"

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/quaternion.h>

#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/temperature.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/imu.h>

#include <nav_msgs/msg/odometry.h>


#include "robot_params.h"			// wheel geometry, PID constants, topic settings
#include "motor_driver.h"			// PWM, direction control, Mecanum_Control()
//#include "ultrasonic_array.h"		// trigger/echo handling for multiple sensors
//#include "odom_handler.h"			// encoder integration, pose update
//#include "debug_pub.h"

//#include "motor_currents_msg.h"
//#include "mpu6050.h"
//#include "bringup.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for myMicroROSTask */
osThreadId_t myMicroROSTaskHandle;
const osThreadAttr_t myMicroROSTask_attributes = {
  .name = "myMicroROSTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */

rcl_allocator_t allocator;
rclc_support_t  support;
rcl_node_t      node;
rclc_executor_t executor;

rcl_node_t node_base_controller;

rcl_subscription_t 		 twist_sub;     	// Subscription object → listens to /twist_nexus topic
geometry_msgs__msg__Twist twist_msg;  // Struct holding the received Twist message data

rcl_publisher_t 			 odom_pub;         // Publisher object → publishes /odom topic
nav_msgs__msg__Odometry   odom_msg; // Struct holding odometry data to send back to ROS2
rcl_timer_t timer_odom;

rcl_publisher_t 			  	imu_pub;
rcl_publisher_t 			  	imu_temp_pub;
sensor_msgs__msg__Imu         	imu_msg;
sensor_msgs__msg__Temperature 	imu_temp_msg;

rcl_publisher_t 			  	 sonar_pub;
std_msgs__msg__Float32MultiArray sonar_msg;
rcl_timer_t 					 sonar_timer;
rcl_publisher_t 				 sonar_temp_pub;
std_msgs__msg__Float32MultiArray sonar_temp_msg;

rcl_timer_t 					 motor_timer;


bool cubemx_transport_open(struct uxrCustomTransport * transport);             // Initialize UART/USB/UDP port
bool cubemx_transport_close(struct uxrCustomTransport * transport);            // Close or power down link
size_t cubemx_transport_write(struct uxrCustomTransport* transport,            // Send raw bytes to agent
                              const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport,             // Read bytes from agent (blocking or timeout)
                             uint8_t* buf, size_t len, int timeout, uint8_t* err);


void * microros_allocate(size_t size, void * state);                            // Allocates 'size' bytes
void microros_deallocate(void * pointer, void * state);                         // Frees memory
void * microros_reallocate(void * pointer, size_t size, void * state);          // Resize allocation
void * microros_zero_allocate(size_t n, size_t size_of_elem, void * state);     // Allocates and zeros array


//void odom_timer_cb(rcl_timer_t * timer, int64_t last_call_time);	 // Publish callback   (Timer callback to compute and publish odometry)
void twist_callback(const void * msgin);							 // Subscribe callback


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
void StartMicroROSTask(void *argument);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#define CHECK_RCL(x) do { \
    rcl_ret_t _rc = (x);  \
    if (_rc != RCL_RET_OK) { \
        /* optional error handling or logging */ \
    } \
} while(0)

static inline void fill_ros_time(builtin_interfaces__msg__Time *t)
{
    int64_t ns = rmw_uros_epoch_nanos();     // requires rmw_uros_sync_session() earlier (you already call it)
    if (ns > 0) {
        t->sec     = (int32_t)(ns / 1000000000LL);
        t->nanosec = (uint32_t)(ns % 1000000000LL);
    } else {
        uint32_t ms = HAL_GetTick();
        t->sec     = (int32_t)(ms / 1000U);
        t->nanosec = (uint32_t)((ms % 1000U) * 1000000U);
    }
}

// ===== Micro-ROS API =====
void MicroROS_TransportInit(UART_HandleTypeDef *uart);
void MicroROS_AllocatorInit(void);
void MicroROS_CoreInit(void);
void MicroROS_TopicsInit(void);
void MicroROS_ExecutorInit(uint32_t executor_capacity);

// ---- transport hooks you already have somewhere ----
extern bool   cubemx_transport_open(struct uxrCustomTransport * transport);
extern bool   cubemx_transport_close(struct uxrCustomTransport * transport);
extern size_t cubemx_transport_write(struct uxrCustomTransport * transport,
                                     const uint8_t * buf, size_t len, uint8_t * err);
extern size_t cubemx_transport_read(struct uxrCustomTransport * transport,
                                    uint8_t * buf, size_t len, int timeout, uint8_t * err);

// ---- FreeRTOS-safe allocator hooks you already have ----
extern void * microros_allocate(size_t size, void * state);
extern void   microros_deallocate(void * pointer, void * state);
extern void * microros_reallocate(void * pointer, size_t size, void * state);
extern void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);



// ---------------------------------------------------------------------------------------------------------------
// ---------------------------------------- Start IMU Code -------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------

// ---------- IMU sensor struct ----------
#define MPU6050_ADDR_AD0_LOW  (0x68 << 1) // HAL expects 8-bit address
#define MPU6050_ADDR_AD0_HIGH (0x69 << 1)

#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_SMPLRT_DIV   0x19
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_GYRO_CONFIG  0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint16_t addr;
    uint8_t raw[14];          // AX,AY,AZ,Temp,GX,GY,GZ (big-endian)
    volatile uint8_t ready;   // set in DMA complete callback
} mpu6050_t;

HAL_StatusTypeDef mpu6050_init(mpu6050_t *dev);
HAL_StatusTypeDef mpu6050_read_dma(mpu6050_t *dev);
void mpu6050_parse(const mpu6050_t *dev,
                   float *ax_g, float *ay_g, float *az_g,
                   float *gx_dps, float *gy_dps, float *gz_dps, float *temp_c);
void i2c1_reset(void);


volatile uint8_t i2c1_need_reset = 0;

mpu6050_t imu = {
    .hi2c = &hi2c1,
    .addr = MPU6050_ADDR_AD0_LOW
};

typedef struct { float ax, ay, az, gx, gy, gz; } ImuBias;
 ImuBias bias = {0};

// To choose the full-scale ranges (see MPU6050 Joy-it datasheet/user manual):
static inline uint8_t accel_cfg_val(void) { return 0 << 3; } // ±2g

static inline uint8_t gyro_cfg_val(void)  { return 0 << 3; } // ±250 dps

//void imu_calibrate(mpu6050_t *imu, unsigned n);


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

HAL_StatusTypeDef mpu6050_read_dma(mpu6050_t *dev)
{
    dev->ready = 0;
    // Burst read 14 bytes starting at ACCEL_XOUT_H using DMA
    return HAL_I2C_Mem_Read_DMA(dev->hi2c, dev->addr, MPU6050_REG_ACCEL_XOUT_H, 1, dev->raw, 14);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) i2c1_need_reset = 1;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        imu.ready = 1;
    }
}

// static (bias) calibration
/*
void imu_calibrate(mpu6050_t *imu, unsigned n)
{
    const unsigned throw_away = 20;
    double sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;
    unsigned used = 0;

    for (unsigned i=0; i<n+throw_away; i++) {
        while (!imu->ready) {}
        imu->ready = 0;

        float ax, ay, az, gx, gy, gz, t;
        mpu6050_parse(imu, &ax, &ay, &az, &gx, &gy, &gz, &t);

        if (i >= throw_away) {
            sax += ax; say += ay; saz += az;
            sgx += gx; sgy += gy; sgz += gz;
            used++;
        }
        (void)mpu6050_read_dma(imu);
    }

    // expected gravity at rest while level is (0,0,+1g)
    float mean_ax = (float)(sax/used);
    float mean_ay = (float)(say/used);
    float mean_az = (float)(saz/used);

    bias.ax = mean_ax - 0.0f;
    bias.ay = mean_ay - 0.0f;
    bias.az = mean_az - 1.0f;

    bias.gx = (float)(sgx/used);
    bias.gy = (float)(sgy/used);
    bias.gz = (float)(sgz/used);
}
*/

static int16_t be16(const uint8_t *p) { return (int16_t)((p[0] << 8) | p[1]); }

void mpu6050_parse(const mpu6050_t *dev,
                   float *ax_g, float *ay_g, float *az_g,
                   float *gx_dps, float *gy_dps, float *gz_dps, float *temp_c)
{
    int16_t ax = be16(&dev->raw[0]);
    int16_t ay = be16(&dev->raw[2]);
    int16_t az = be16(&dev->raw[4]);
    int16_t t  = be16(&dev->raw[6]);
    int16_t gx = be16(&dev->raw[8]);
    int16_t gy = be16(&dev->raw[10]);
    int16_t gz = be16(&dev->raw[12]);

    // Sensitivities for ±2g and ±250 dps:
    const float accel_sens = 16384.0f; // LSB/g
    const float gyro_sens  = 131.0f;   // LSB/(°/s)

    if (ax_g) *ax_g = ax / accel_sens;
    if (ay_g) *ay_g = ay / accel_sens;
    if (az_g) *az_g = az / accel_sens;

    if (gx_dps) *gx_dps = gx / gyro_sens;
    if (gy_dps) *gy_dps = gy / gyro_sens;
    if (gz_dps) *gz_dps = gz / gyro_sens;

    if (temp_c) *temp_c = (t / 340.0f) + 36.53f;
}

// ---- read/compute/publish (call from your task loop) ----
void imu_func(void)
{
	const float DEG2RAD = 0.01745329251f;
	const float G_TO_MS2 = 9.80665f;

	if (!imu.ready) return;
    imu.ready = 0;

    float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c;
    mpu6050_parse(&imu, &ax_g, &ay_g, &az_g, &gx_dps, &gy_dps, &gz_dps, &temp_c);

    // Remove static bias
    ax_g -= bias.ax;
    ay_g -= bias.ay;
    az_g -= bias.az;
    gx_dps -= bias.gx;
    gy_dps -= bias.gy;
    gz_dps -= bias.gz;

    // Publish IMU raw data
    fill_ros_time(&imu_msg.header.stamp);
    imu_msg.orientation_covariance[0] = -1;

    imu_msg.angular_velocity.x = gx_dps * DEG2RAD;
    imu_msg.angular_velocity.y = gy_dps * DEG2RAD;
    imu_msg.angular_velocity.z = gz_dps * DEG2RAD;

    imu_msg.linear_acceleration.x = ax_g * G_TO_MS2;
    imu_msg.linear_acceleration.y = ay_g * G_TO_MS2;
    imu_msg.linear_acceleration.z = az_g * G_TO_MS2;

    CHECK_RCL(rcl_publish(&imu_pub, &imu_msg, NULL));

    // Publish temperature (optional)
    fill_ros_time(&imu_temp_msg.header.stamp);
    imu_temp_msg.temperature = temp_c;
    imu_temp_msg.variance = 0.1;
    CHECK_RCL(rcl_publish(&imu_temp_pub, &imu_temp_msg, NULL));

    // Trigger next DMA read
    mpu6050_read_dma(&imu);
}

void i2c1_reset(void){

    i2c1_need_reset = 0;
    HAL_I2C_DeInit(&hi2c1);
    MX_I2C1_Init();
    mpu6050_read_dma(&imu);
}

// ---------------------------------------------------------------------------------------------------------------
// ---------------------------------------- End IMU Code ---------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------



// ---------------------------------------------------------------------------------------------------------------
// ---------------------------------------- Start Current Sensor Code --------------------------------------------
// ---------------------------------------------------------------------------------------------------------------

/*

static rcl_publisher_t motor_current_pub;
static sensor_msgs__msg__JointState joint_msg;


    CHECK_RCL(rclc_timer_init_default2(
        &motor_timer,
        &support,
        RCL_MS_TO_NS(100),        // publish every 100 ms = 10 Hz
        timer_motor_current_cb,
        true));                   // autostart
    CHECK_RCL(rclc_executor_add_timer(&executor, &motor_timer));

// ------------------- Prototypes -------------------
static inline float adc_to_current_ch(uint16_t adc_value, int ch);
static void publish_motor_currents(void);
void timer_motor_current_cb(rcl_timer_t * timer, int64_t last_call_time);
static void current_zero_calibrate(uint32_t samples);

// ------------------- Configuration Constants -------------------
#ifndef MOTOR_MAP_H
#define MOTOR_MAP_H

#define IDX_RL 0
#define IDX_FL 1
#define IDX_FR 2
#define IDX_RR 3

#endif

#define ADC_VREF            3.3f          // ADC reference voltage (V)
#define ADC_RESOLUTION      4095.0f       // 12-bit ADC
#define CS_SENSITIVITY      0.14f        // V/A for R_SENSE = 1.5kΩ
#define CURRENT_LIMIT_A     2.0f          // Max allowed motor current (A)
#define CURRENT_OFFSET_V    0.02f         // Zero-current offset (V), adjust experimentally
#define NUM_MOTORS          4

static uint16_t adc_values[NUM_MOTORS];
static float currents[NUM_MOTORS];
static volatile uint8_t adc_data_ready = 0;


static uint16_t adc_zero[NUM_MOTORS] = {0};   // per-channel zero ADC code
#define ZERO_DEADBAND_A  0.05f                // show 0 if |I| < 50 mA (tune)

// Optional simple LPF
static float i_filt[NUM_MOTORS] = {0};
#define I_ALPHA 0.2f                          // 0..1 (higher = less smoothing)

// ------------------- One-time init (call this during bringup) -------------------
void motor_current_pub_init()
{
    // Init message
    sensor_msgs__msg__JointState__init(&joint_msg);

    // Allocate and set names ONCE (no allocs in the fast path)
    rosidl_runtime_c__String__Sequence__init(&joint_msg.name, 4);
    rosidl_runtime_c__String__assign(&joint_msg.name.data[IDX_RL], "rear_left_rev");
    rosidl_runtime_c__String__assign(&joint_msg.name.data[IDX_FL], "front_left_rev");
    rosidl_runtime_c__String__assign(&joint_msg.name.data[IDX_FR], "front_right_rev");
    rosidl_runtime_c__String__assign(&joint_msg.name.data[IDX_RR], "rear_right_rev");

    // Allocate position[4]; leave velocity/effort empty
    rosidl_runtime_c__double__Sequence__init(&joint_msg.effort, 4);
    // (Optional) set initial zeros
    for (int i = 0; i < 4; ++i) joint_msg.position.data[i] = 0.0;

    // frame_id optional
    rosidl_runtime_c__String__assign(&joint_msg.header.frame_id, "base_link");

    // Publisher
    CHECK_RCL(rclc_publisher_init_default(
        &motor_current_pub,
        &node_base_controller,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), // <-- CHANGED
        "motor_currents")); // keep your topic name

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_values, NUM_MOTORS);

    // One-time time sync with Agent (so header.stamp is meaningful)
    current_zero_calibrate(200U);
    rmw_uros_sync_session(1000);  // 1s timeout is fine
}

// ------------------- Conversion Function -------------------
static inline float adc_to_current_ch(uint16_t adc_value, int ch)
{
    int32_t diff_counts = (int32_t)adc_value - (int32_t)adc_zero[ch]; // can be +/-
    float volts = ((float)diff_counts) * (ADC_VREF / ADC_RESOLUTION);
    float amps  = volts / CS_SENSITIVITY;
    return amps; // for unidirectional sensors, clamp negative to 0 below
}

// --- Add a one-time calibration (call at boot, with motors OFF) ---
static void current_zero_calibrate(uint32_t samples)
{
    uint32_t acc[NUM_MOTORS] = {0};
    uint32_t n = 0;

    while (n < samples) {
        if (adc_data_ready) {
            adc_data_ready = 0;
            for (int i = 0; i < NUM_MOTORS; i++) {
                acc[i] += adc_values[i];     // adc_values is uint16_t -> safe widen
            }
            n++;
        }
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        adc_zero[i] = (uint16_t)(acc[i] / samples);  // <- no sign conversion now
    }
}

// ------------------- DMA Callback (ISR) -------------------
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        adc_data_ready = 1;  // Set flag only — do not block here!
    }
}


void timer_motor_current_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer; (void)last_call_time;
    if (!adc_data_ready) return;
    adc_data_ready = 0;

    for (int i = 0; i < NUM_MOTORS; i++) {
        float i_raw = adc_to_current_ch(adc_values[i], i);

        // Optional LPF
        i_filt[i] += I_ALPHA * (i_raw - i_filt[i]);

        // If your sensor is unidirectional, clamp negatives:
        float i_clamped = i_filt[i];
        if (i_clamped < 0.0f) i_clamped = 0.0f;

        // Deadband to show clean zeros
        currents[i] = (fabsf(i_clamped) < ZERO_DEADBAND_A) ? 0.0f : i_clamped;
    }

    publish_motor_currents();
}

// ------------------- Publish function -------------------
static void publish_motor_currents(void)
{
    // header.stamp from Agent-synced epoch
    if (rmw_uros_epoch_synchronized()) {
        int64_t now_ns = rmw_uros_epoch_nanos();
        if (now_ns > 0) {
            joint_msg.header.stamp.sec     = (int32_t)(now_ns / 1000000000LL);
            joint_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);
        }
    }

    // Put electrical current (A) into effort[]
    joint_msg.effort.data[IDX_RL] = (double)currents[IDX_RL];
    joint_msg.effort.data[IDX_FL] = (double)currents[IDX_FL];
    joint_msg.effort.data[IDX_FR] = (double)currents[IDX_FR];
    joint_msg.effort.data[IDX_RR] = (double)currents[IDX_RR];

    CHECK_RCL(rcl_publish(&motor_current_pub, &joint_msg, NULL));
}

	motor_current_pub_init();


*/

// ---------------------------------------------------------------------------------------------------------------
// ---------------------------------------- End Current Sensor Code ----------------------------------------------
// ---------------------------------------------------------------------------------------------------------------



// ---------------------------------------------------------------------------------------------------------------
// ---------------------------------------- Start SONAR & MAX485 Code --------------------------------------------
// ---------------------------------------------------------------------------------------------------------------

#define NUM_SENSORS 			 4
#define SONAR_HEADER1 			0x55
#define SONAR_HEADER2 			0xAA
#define SONAR_DEFAULT_ADDR 		0x11
#define SONAR_TRIGGER_CMD 		0x01
#define SONAR_READ_DIST_CMD 	0x02
#define SONAR_READ_TEMP_CMD 	0x03
#define SONAR_DE_RE_GPIO_Port 	GPIOC
#define SONAR_DE_RE_Pin 		GPIO_PIN_5
#define SONAR_RX_BUF_SIZE 		16
#define SONAR_TIMEOUT 			-2
#define SONAR_INVALID 			-3
#define SONAR_CMD_ERROR 		-1

typedef struct {
	UART_HandleTypeDef *huart;
	uint8_t addr;
	uint8_t rxBuf[SONAR_RX_BUF_SIZE];
} SONAR_HandleTypeDef;

SONAR_HandleTypeDef sonar[NUM_SENSORS];

// Global buffers (keep yours)
int16_t distBuf[NUM_SENSORS];
float   tempBuf[NUM_SENSORS];
uint32_t timeout = 0xFFFF;

// Forward decls (keep your existing versions)
static void RS485_SetTX(void);
static void RS485_SetRX(void);
static uint8_t calcChecksum(uint8_t *data, uint8_t len);

// -------- Helpers (unchanged contract: they own the flags) --------
static void RS485_SetTX(void) {  HAL_GPIO_WritePin(SONAR_DE_RE_GPIO_Port, SONAR_DE_RE_Pin, GPIO_PIN_SET);  }

static void RS485_SetRX(void) {  HAL_GPIO_WritePin(SONAR_DE_RE_GPIO_Port, SONAR_DE_RE_Pin, GPIO_PIN_RESET); }

static uint8_t calcChecksum(uint8_t *data, uint8_t len) {
	uint16_t sum = 0;
	for (uint8_t i = 0; i < len; i++) sum += data[i];
	return (uint8_t)(sum & 0xFF);
}

void SONAR_Init(SONAR_HandleTypeDef *hsonar, UART_HandleTypeDef *huart, uint8_t addr) {

	hsonar->huart = huart;
	hsonar->addr = addr;
	memset(hsonar->rxBuf, 0, SONAR_RX_BUF_SIZE);
}

void SONAR_TransmitCommand(SONAR_HandleTypeDef *hsonar, uint8_t cmd_code)
{
    uint8_t cmd[6] = { SONAR_HEADER1, SONAR_HEADER2, hsonar->addr, 0x00, cmd_code, 0x00 };
    cmd[5] = calcChecksum(cmd, 5);

    RS485_SetTX();
    HAL_UART_Transmit(hsonar->huart, cmd, sizeof(cmd), timeout);
    RS485_SetRX();
}

int16_t SONAR_ReadDistance(SONAR_HandleTypeDef *hsonar)
{

    SONAR_TransmitCommand(hsonar, SONAR_READ_DIST_CMD);

    if (HAL_UART_Receive(hsonar->huart, hsonar->rxBuf, 8, timeout) != HAL_OK)
    	return SONAR_TIMEOUT;

    // Checksum
	uint8_t sum = calcChecksum(hsonar->rxBuf, 7);
	if (sum != hsonar->rxBuf[7]) return SONAR_INVALID;

	if (hsonar->rxBuf[5] == 0xFF && hsonar->rxBuf[6] == 0xFF)
		return -1; // out of range

    return ((hsonar->rxBuf[5] << 8) | hsonar->rxBuf[6]);
}

int16_t SONAR_ReadTemperature(SONAR_HandleTypeDef *hsonar)
{

     SONAR_TransmitCommand(hsonar, SONAR_READ_TEMP_CMD);

     if (HAL_UART_Receive(hsonar->huart, hsonar->rxBuf, 8, timeout) != HAL_OK)
    	 return SONAR_TIMEOUT;

     // Checksum
 	uint8_t sum = calcChecksum(hsonar->rxBuf, 7);
 	if (sum != hsonar->rxBuf[7]) return SONAR_INVALID;

 	if (hsonar->rxBuf[5] == 0xFF && hsonar->rxBuf[6] == 0xFF)
 		return -1; // out of range

    int16_t raw = ((hsonar->rxBuf[5] & 0x0F) << 8) | hsonar->rxBuf[6];
    return ((hsonar->rxBuf[5] & 0xF0) == 0) ? raw : -raw;
}

void sonar_timer_cb(rcl_timer_t *timer, int64_t last_call_time){

	static uint8_t turns = 1;

	turns = (turns % 4) + 1;

	switch (turns){
		case 1:
			distBuf[1] = SONAR_ReadDistance(&sonar[1]);
			tempBuf[1] = SONAR_ReadTemperature(&sonar[1]);
			SONAR_TransmitCommand(&sonar[1], SONAR_TRIGGER_CMD);
			break;

		case 2:
			distBuf[2] = SONAR_ReadDistance(&sonar[2]);
			tempBuf[2] = SONAR_ReadTemperature(&sonar[2]);
			SONAR_TransmitCommand(&sonar[2], SONAR_TRIGGER_CMD);
			break;

		case 3:
			distBuf[3] = SONAR_ReadDistance(&sonar[3]);
			tempBuf[3] = SONAR_ReadTemperature(&sonar[3]);
			SONAR_TransmitCommand(&sonar[3], SONAR_TRIGGER_CMD);
			break;

		default:
			distBuf[0] =  SONAR_ReadDistance(&sonar[0]);
			tempBuf[0] = SONAR_ReadTemperature(&sonar[0]);
			SONAR_TransmitCommand(&sonar[0], SONAR_TRIGGER_CMD);

			for (int i = 0; i < NUM_SENSORS; i++){
				sonar_msg.data.data[i] = distBuf[i];
			}
			CHECK_RCL(rcl_publish(&sonar_pub, &sonar_msg, NULL));

			for (int i = 0; i < NUM_SENSORS; i++){
						sonar_temp_msg.data.data[i] =  tempBuf[i];
			}
			CHECK_RCL(rcl_publish(&sonar_temp_pub, &sonar_temp_msg, NULL));
			break;
	}
}

/*

typedef enum { ST_TX_DIST, RX_DIST_TX_TEMP, RX_TEMP, ST_TX_TRIG } SBState;
static volatile uint8_t txDone = 1;
static SBState curr_state = ST_TX_DIST;
//static uint8_t order[4] = {1,2,3,0};     // sensor order
static uint8_t order[4] = {0,1,2,3};
static uint8_t pos = 0;                   // index in order[]
static SONAR_HandleTypeDef *sonars[4] = { &sonar1, &sonar2, &sonar3, &sonar4 };


static void start_rx(SONAR_HandleTypeDef *s)
{
    rxReady = 0;
    HAL_UART_Receive_IT(s->huart, s->rxBuf, 8);
}


static inline int16_t rx_dist(SONAR_HandleTypeDef *s)
{
    if (calcChecksum(s->rxBuf,7) == s->rxBuf[7])
        return (int16_t)((s->rxBuf[5] << 8) | s->rxBuf[6]);

    return distBuf[order[pos]]; // keep last
}

static inline float rx_temp(SONAR_HandleTypeDef *s)
{
    if (calcChecksum(s->rxBuf,7) == s->rxBuf[7]) {
        int16_t raw = ((s->rxBuf[5] & 0x0F) << 8) | s->rxBuf[6];
        return ((s->rxBuf[5] & 0xF0) == 0) ? (float)raw : (float)(-raw);
    }
    return tempBuf[order[pos]];
}

static void send_cmd(SONAR_HandleTypeDef *hsonar, uint8_t cmd)
{
    uint8_t buf[6] = {SONAR_HEADER1, SONAR_HEADER2, hsonar->addr, 0x00, cmd, 0x00};
    buf[5] = calcChecksum(buf, 5);

    txDone = 0;

    HAL_UART_Transmit_IT(hsonar->huart, buf, sizeof(buf));
}


// Call this from your 50 ms micro-ROS timer
void sonar_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
	SONAR_HandleTypeDef *sonars[NUM_SENSORS] = { &sonar1, &sonar2, &sonar3, &sonar4 };

    uint8_t idx = order[pos];

    switch (curr_state) {
        case ST_TX_DIST:
            if (txDone) {
                RS485_SetTX();
            	send_cmd(s, SONAR_READ_DIST_CMD);
            	curr_state = ST_RX_DIST;
            }
            break;

        case ST_RX_DIST:
            if (txDone) {

                send_cmd(s, SONAR_READ_TEMP_CMD);
            	curr_state = ST_RX_TEMP;
            }
            break;

        case ST_RX_TEMP:
            if (txDone) {
            	start_rx(s);
            	curr_state = ST_TX_TRIG;
            }
            break;

        case ST_TX_TRIG:
            if (txDone) {
                tempBuf[idx] = rx_temp(s);

                send_cmd(s, SONAR_TRIGGER_CMD);
                pos = (pos + 1) & 3; // 0..3 in ring  [1,2,3,0]
                curr_state = ST_TX_DIST;
            }
            break;

        default: break;
    }
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5) {
    	RS485_SetRX();
    	txDone = 1;
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart5) {
    	//rxReady = 1;
    }
}
*/

void SONAR_SystemInit(void)
{
    SONAR_Init(&sonar[0], &huart5, 0x11);
    SONAR_Init(&sonar[1], &huart5, 0x12);
    SONAR_Init(&sonar[2], &huart5, 0x13);
    SONAR_Init(&sonar[3], &huart5, 0x14);
}

// ---------------------------------------------------------------------------------------------------------------
// ---------------------------------------- END SONAR & MAX485 Code ----------------------------------------------
// ---------------------------------------------------------------------------------------------------------------


// ---------------------------------------------------------------------------------------------------------------
// ---------------------------------------- Start Motors, Encoders & Odom Code -----------------------------------------
// ---------------------------------------------------------------------------------------------------------------
#define ENCODER_RESOULTION   			12.0f
#define GEARBOX_RATIO 					64.0f
#define ENCODER_QUADRATURE    			4.0f
#define TICKS_PER_WHEEL_REV     		(ENCODER_RESOULTION * ENCODER_QUADRATURE * GEARBOX_RATIO)	// 3072
#define RAD_PER_TICK         			(2.0f * 3.1416f / TICKS_PER_WHEEL_REV)
#define DeltaT       		   			0.001f						// control time for motor/PID control  1 [ms]
#define RAD_PER_TICK_PER_SEC  			RAD_PER_TICK / DeltaT

typedef struct {
    float Kc;               // Controller gain
    float tau_i;            // Integral time constant (s)
    float tau_d;            // Derivative time constant (s)

    float cof_A;            // Discrete PID coefficients
    float cof_B;
    float cof_C;

    float last_error;       // e(k-1)
    float prev_error;       // e(k-2)
    float prev_output;      // u(k-1)
    float output;           // Current output

    float output_max;		// Output limits
    float output_min;
} PID_t;

float vx = 0;
float vy = 0;
float wz = 0;
volatile float w_meas[4];
//volatile uint32_t g_ccr_applied[4] = {0, 0, 0, 0};

float vx_odom = 0;
float vy_odom = 0;
float wz_odom = 0;


const uint32_t TIM_CHANNELS[4] = {
	TIM_CHANNEL_1, // RL
	TIM_CHANNEL_2, // FL
	TIM_CHANNEL_3, // FR
	TIM_CHANNEL_4  // RR
};

volatile uint32_t g_ccr_applied[4];

PID_t pid_wheel[4];

rcl_subscription_t twist_sub;     // Subscription object → listens to /twist_nexus topic
geometry_msgs__msg__Twist twist_msg;  // Struct holding the received Twist message data

#define DEBUG_ARRAY_SIZE  16

rcl_publisher_t debug_publisher;
std_msgs__msg__Float32MultiArray debug_msg;

float data_buffer[DEBUG_ARRAY_SIZE];


float clamp(float v, float lo, float hi);
void setMotorDir(GPIO_TypeDef* INxA_Port, uint16_t INxA_Pin, GPIO_TypeDef* INxB_Port, uint16_t INxB_Pin, float speed);

void init_motors(uint32_t CCR);
void Mecanum_Control(float vx_target, float vy_target, float w_target);
void Emergency_Stop(void);
void stm32_debug_publish(void);


void PID_Init(float Kp, float Ki, float Kd);
void PID_WheelInit(PID_t *pid, float Kp, float Ki, float Kd, float dt);
float PID_Update(PID_t *pid, float setpoint, float input);

void odom_publish(void);


float clamp(float v, float lo, float hi){
    if (v < lo) 		return lo;
    else if (v > hi) 	return hi;
    else 				return v;
}


void setMotorDir(GPIO_TypeDef* INxA_Port, uint16_t INxA_Pin, GPIO_TypeDef* INxB_Port, uint16_t INxB_Pin, float speed) {

	if (speed > 0.0001f) { // Forward
		HAL_GPIO_WritePin(INxB_Port, INxB_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INxA_Port, INxA_Pin, GPIO_PIN_SET);

	} else if (speed < -0.0001f) { // Reverse
		HAL_GPIO_WritePin(INxA_Port, INxA_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(INxB_Port, INxB_Pin, GPIO_PIN_SET);

	} else { // Stop / Brake
		//HAL_GPIO_WritePin(INxA_Port, INxA_Pin, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(INxB_Port, INxB_Pin, GPIO_PIN_RESET);
	}
}

void init_motors(uint32_t CCR){

	//uint32_t CCR_clamped = (uint32_t)clamp((float)CCR, 0 , 300);
	uint32_t CCR_clamped = CCR;

	// Set the direction to Forward (INA = 1) & (INB = 0)
	// Rear Left
	HAL_GPIO_WritePin(RL_INB_GPO_GPIO_Port, RL_INB_GPO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RL_INA_GPO_GPIO_Port, RL_INA_GPO_Pin, GPIO_PIN_SET);
	// Front Left
	HAL_GPIO_WritePin(FL_INB_GPO_GPIO_Port, FL_INB_GPO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FL_INA_GPO_GPIO_Port, FL_INA_GPO_Pin, GPIO_PIN_SET);
	// Rear Right
	HAL_GPIO_WritePin(RR_INB_GPO_GPIO_Port, RR_INB_GPO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RR_INA_GPO_GPIO_Port, RR_INA_GPO_Pin, GPIO_PIN_SET);
	// Front Right
	HAL_GPIO_WritePin(FR_INB_GPO_GPIO_Port, FR_INB_GPO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(FR_INA_GPO_GPIO_Port, FR_INA_GPO_Pin, GPIO_PIN_SET);

	// Enable the full bridges of the motor drivers (VDD)
	HAL_GPIO_WritePin(RL_VDD_GPO_GPIO_Port, RL_VDD_GPO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FR_VDD_GPO_GPIO_Port, FR_VDD_GPO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RR_VDD_GPO_GPIO_Port, RR_VDD_GPO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(FL_VDD_GPO_GPIO_Port, FL_VDD_GPO_Pin, GPIO_PIN_SET);

	// Start the PWM signals
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// RL Motor
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);	// FL Motor
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	// FR Motor
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);	// RR Motor

	// Set RL, FL, FR and RR motors to initial speed. For example 0 PWM = 0 CCR = 0% Duty Cycle => t_on = 0 µs
	/*
	for(uint32_t i = 0; i <= CCR_clamped; i++){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, i+200UL);	// RL Motor
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, i+100UL);	// FL Motor
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, i);		// FR Motor
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);		// RR Motor
	  	HAL_Delay(15);
	  }
	*/
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, CCR_clamped);	// RL Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, CCR_clamped);	// FL Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, CCR_clamped);	// FR Motor
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, CCR_clamped);	// RR Motor


}


void Mecanum_Control(float vx_twist, float vy_twist, float wz_twist)
{

    float w_target[4];	    // Desired wheel angular velocities
    w_target[0] = (+vx_twist + vy_twist - (A_SUM * wz_twist)) / WHEEL_R;  // RL
    w_target[1] = (+vx_twist - vy_twist - (A_SUM * wz_twist)) / WHEEL_R;  // FL
    w_target[2] = (+vx_twist + vy_twist + (A_SUM * wz_twist)) / WHEEL_R;  // FR
    w_target[3] = (+vx_twist - vy_twist + (A_SUM * wz_twist)) / WHEEL_R;  // RR

    float pid_output[4];

    for (int i = 0; i < 4; i++) {
        pid_output[i] = PID_Update(&pid_wheel[i], w_target[i], w_meas[i]);
    }

    // Motor directions
    setMotorDir(RL_INA_GPO_GPIO_Port, RL_INA_GPO_Pin,
                RL_INB_GPO_GPIO_Port, RL_INB_GPO_Pin, pid_output[0]);
    setMotorDir(FL_INA_GPO_GPIO_Port, FL_INA_GPO_Pin,
                FL_INB_GPO_GPIO_Port, FL_INB_GPO_Pin, pid_output[1]);
    setMotorDir(FR_INA_GPO_GPIO_Port, FR_INA_GPO_Pin,
                FR_INB_GPO_GPIO_Port, FR_INB_GPO_Pin, pid_output[2]);
    setMotorDir(RR_INA_GPO_GPIO_Port, RR_INA_GPO_Pin,
                RR_INB_GPO_GPIO_Port, RR_INB_GPO_Pin, pid_output[3]);

    uint32_t CCR[4];					// Convert PID output to PWM
    for (int i = 0; i < 4; i++) {
        CCR[i] = (uint32_t)((fabsf(pid_output[i]) / MAX_WHEEL_ANGULAR_V_RADPS) * CCR_MAX);
        CCR[i] = (uint32_t)clamp((float)CCR[i], 0.0f, CCR_MAX);
    }

    for (int i = 0; i < 4; i++) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNELS[i], CCR[i]);
        g_ccr_applied[i] = CCR[i];
    }

    stm32_debug_publish();
}


/*
void Mecanum_Control(float vx_twist, float vy_twist, float wz_twist)
{

  //  static float vx_twist_ramped = 0, vy_twist_ramped = 0, wz_twist_ramped = 0;
    vx_twist_ramped = ramp(vx_twist, vx_twist_ramped, g_v_step);
  //  vy_twist_ramped = ramp(vy_twist, vy_twist_ramped, g_v_step);
    wz_twist_ramped = ramp(wz_twist, wz_twist_ramped, g_wz_step_radps);


    // Desired wheel angular velocities
    float w_target[4];
    w_target[0] = (+vx_twist_ramped + vy_twist_ramped - (A_SUM * wz_twist_ramped)) / WHEEL_R;  // RL
 //   w_target[1] = (+vx_twist_ramped - vy_twist_ramped - (A_SUM * wz_twist_ramped)) / WHEEL_R;  // FL
    w_target[2] = (+vx_twist_ramped + vy_twist_ramped + (A_SUM * wz_twist_ramped)) / WHEEL_R;  // FR
    w_target[3] = (+vx_twist_ramped - vy_twist_ramped + (A_SUM * wz_twist_ramped)) / WHEEL_R;  // RR


    float w_target[4];	    // Desired wheel angular velocities
    w_target[0] = (+vx_twist + vy_twist - (A_SUM * wz_twist)) / WHEEL_R;  // RL
    w_target[1] = (+vx_twist - vy_twist - (A_SUM * wz_twist)) / WHEEL_R;  // FL
 //   w_target[2] = (+vx_twist + vy_twist + (A_SUM * wz_twist)) / WHEEL_R;  // FR
    w_target[3] = (+vx_twist - vy_twist + (A_SUM * wz_twist)) / WHEEL_R;  // RR

    // Motor directions

 //   setMotorDir(RL_INA_GPO_GPIO_Port, RL_INA_GPO_Pin,
                RL_INB_GPO_GPIO_Port, RL_INB_GPO_Pin, w_target[0]);
    //setMotorDir(FL_INA_GPO_GPIO_Port, FL_INA_GPO_Pin,
                FL_INB_GPO_GPIO_Port, FL_INB_GPO_Pin, w_target[1]);
    setMotorDir(FR_INA_GPO_GPIO_Port, FR_INA_GPO_Pin,
                FR_INB_GPO_GPIO_Port, FR_INB_GPO_Pin, w_target[2]);
    setMotorDir(RR_INA_GPO_GPIO_Port, RR_INA_GPO_Pin,
   //             RR_INB_GPO_GPIO_Port, RR_INB_GPO_Pin, w_target[3]);


    setMotorDir(RL_INA_GPO_GPIO_Port, RL_INA_GPO_Pin,
                RL_INB_GPO_GPIO_Port, RL_INB_GPO_Pin, vx_twist);
    setMotorDir(FL_INA_GPO_GPIO_Port, FL_INA_GPO_Pin,
                FL_INB_GPO_GPIO_Port, FL_INB_GPO_Pin, vx_twist);
    setMotorDir(FR_INA_GPO_GPIO_Port, FR_INA_GPO_Pin,
                FR_INB_GPO_GPIO_Port, FR_INB_GPO_Pin, vx_twist);
    setMotorDir(RR_INA_GPO_GPIO_Port, RR_INA_GPO_Pin,
                RR_INB_GPO_GPIO_Port, RR_INB_GPO_Pin, vx_twist);

    // Convert PID output to PWM
    uint32_t CCR[4];
    for (int i = 0; i < 4; i++) {
        CCR[i] = (uint32_t)((fabsf(w_target[i]) / MAX_WHEEL_ANGULAR_V_RADPS) * CCR_MAX);
        CCR[i] = (uint32_t)clamp((float)CCR[i], 0.0f, CCR_MAX);
    }

    // Apply PWM
	for (int i = 0; i < 4; i++) {
		//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNELS[i], CCR[i]);
		//g_ccr_applied[i] = CCR[i];

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNELS[i], (uint32_t)(fabsf(vx_twist)));
		//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNELS[i], 250);
		g_ccr_applied[i] = (uint32_t)((vx_twist));
	}

    stm32_debug_publish();
}

*/


void PID_Init(float Kc, float taui, float taud)
{
    if (Kc == 0.0f)   Kc   = KC;
    if (taui == 0.0f) taui = TAUI;
    if (taud == 0.0f) taud = TAUD;

    PID_WheelInit(&pid_wheel[0], Kc, taui, taud, DeltaT);   // RL
    PID_WheelInit(&pid_wheel[1], Kc, taui, taud, DeltaT);   // FL
    PID_WheelInit(&pid_wheel[2], Kc, taui, taud, DeltaT);   // FR
    PID_WheelInit(&pid_wheel[3], Kc, taui, taud, DeltaT);   // RR

}

void PID_WheelInit(PID_t *pid, float Kc, float tau_i, float tau_d, float dt)
{
    pid->Kc = Kc;
    pid->tau_i = tau_i;
    pid->tau_d = tau_d;

    // Precompute coefficients (Tustin/bilinear transform)
    pid->cof_A =  Kc * (1.0f + (dt / (2.0f * tau_i)) + (tau_d / dt));
    pid->cof_B = -Kc * (1.0f - (dt / (2.0f * tau_i)) + (2.0f * tau_d / dt));
    pid->cof_C = Kc * (tau_d / dt);

    // Initialize state
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_output = 0.0f;
    pid->output = 0.0f;

    // Default limits (can be overridden)
    pid->output_max =  MAX_WHEEL_ANGULAR_V_RADPS;
    pid->output_min =  MIN_WHEEL_ANGULAR_V_RADPS;
}

float PID_Update(PID_t *pid, float setpoint, float input)
{
    float error = setpoint - input;

    float output = pid->prev_output
                 + pid->cof_A * error
                 + pid->cof_B * pid->last_error
                 + pid->cof_C * pid->prev_error;

    // Clamp to limits
    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;

    // Shift states
    pid->prev_error = pid->last_error;
    pid->last_error = error;
    pid->prev_output = output;
    pid->output = output;

    return output;
}


void twist_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;

    //static uint32_t last_cmd_tick = 0;

    // Extract velocity commands (SI units)
    vx = (float)msg->linear.x;   // forward/backward [m/s]
    vy = (float)msg->linear.y;   // lateral [m/s]
    wz = (float)msg->angular.z;  // rotation [rad/s]


    // Pass values to the motion controller
    //if ((osKernelGetTickCount() - last_cmd_tick) > CMD_TIMEOUT_MS) {
    //	Mecanum_Control(0.0f, 0.0f, 0.0f);  // stop robot
	//}
}

volatile int32_t deltaEncoder[4] = {0};		// {RL, FL, FR, RR}
volatile uint32_t currCount[4]	 = {0};		// {RL, FL, FR, RR}
volatile uint32_t pastCount[4] 	 = {0};		// {RL, FL, FR, RR}

volatile bool encUpdateFlag 	 = 0;

volatile float x_odom = 0.0f;
volatile float y_odom = 0.0f;
volatile float th_odom = 0.0f;

void odom_publish(void)
{
  int64_t t_ns = rmw_uros_epoch_nanos();
  if (t_ns > 0) {
	odom_msg.header.stamp.sec     = (int32_t)(t_ns / 1000000000LL);
	odom_msg.header.stamp.nanosec = (uint32_t)(t_ns % 1000000000LL);
  } else {
	TickType_t tk = xTaskGetTickCount();
	odom_msg.header.stamp.sec     = (int32_t)(tk / configTICK_RATE_HZ);
	odom_msg.header.stamp.nanosec = (uint32_t)((tk % configTICK_RATE_HZ) * (1000000000UL / configTICK_RATE_HZ));
  }

  odom_msg.pose.pose.position.x = (double)x_odom;
  odom_msg.pose.pose.position.y = (double)y_odom;

  float half = 0.5f * th_odom;

  odom_msg.pose.pose.orientation.z = (double)sinf(half);
  odom_msg.pose.pose.orientation.w = (double)cosf(half);

  odom_msg.twist.twist.linear.x  = (double)vx_odom;
  odom_msg.twist.twist.linear.y  = (double)vy_odom;
  odom_msg.twist.twist.angular.z = (double)wz_odom;

  CHECK_RCL(rcl_publish(&odom_pub, &odom_msg, NULL));
}

// ---------------------------------------------------------------------------------------------------------------
// ---------------------------------------- End Motors, Encoders & Odom Code -----------------------------------------
// ---------------------------------------------------------------------------------------------------------------

// ---------------------------------------------------------------------------------------------------------------
// ---------------------------------------- Start micro-ROS Code  ------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------

void MicroROS_TransportInit(UART_HandleTypeDef *uart)
{
    rmw_uros_set_custom_transport(
        true, (void*)uart,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);
}

void MicroROS_AllocatorInit(void)
{
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate      = microros_allocate;
    freeRTOS_allocator.deallocate    = microros_deallocate;
    freeRTOS_allocator.reallocate    = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    CHECK_RCL(rcutils_set_default_allocator(&freeRTOS_allocator));
}

void MicroROS_CoreInit(void)
{
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node_base_controller, "base_controller", "", &support);

}

void MicroROS_TopicsInit(void)
{
	// -----------------------------------------------------------------------------------
	CHECK_RCL(rclc_subscription_init_default(
		  &twist_sub,													// param1 => subscriber
		  &node_base_controller,										// param2 => node
		  ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),		// param3 => message type support	&& ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSubfolder, MsgName)
		  "/twist_nexus"));												// param4 => topic_name

	// -----------------------------------------------------------------------------------

	CHECK_RCL(rclc_publisher_init_default(
		&debug_publisher,
		&node_base_controller,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
		"/stm32_debug" ));


	std_msgs__msg__MultiArrayDimension__Sequence__init(&debug_msg.layout.dim, 1);
	rosidl_runtime_c__String__assign(&debug_msg.layout.dim.data[0].label, "Debug of STM32 Values");

	debug_msg.layout.dim.data[0].size   = DEBUG_ARRAY_SIZE;
	debug_msg.layout.dim.data[0].stride = DEBUG_ARRAY_SIZE;
	debug_msg.layout.data_offset        = 0;

	// 3. Allocate the data buffer
	debug_msg.data.data     = data_buffer;
	debug_msg.data.size     = DEBUG_ARRAY_SIZE;
	debug_msg.data.capacity = DEBUG_ARRAY_SIZE;

	// 4. Initialize values to zero
	for (uint8_t i = 0; i < DEBUG_ARRAY_SIZE; i++)  debug_msg.data.data[i] = 0.0f;



   // -----------------------------------------------------------------------------------


	CHECK_RCL(rclc_publisher_init_default(
		  &odom_pub,												// param1 => publisher
		  &node_base_controller,									// param2 => node
		  ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), 	// param3 => type_support	&& ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSubfolder, MsgName)
		  "odom"));													// param4 => topic_name

	rosidl_runtime_c__String__init(&odom_msg.header.frame_id);
	rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");

	rosidl_runtime_c__String__init(&odom_msg.child_frame_id);
	rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");

	odom_msg.header.stamp.sec = 0;
	odom_msg.header.stamp.nanosec = 0;

	odom_msg.twist.twist.linear.z  = 0.0f;

	odom_msg.twist.twist.angular.x = 0.0f;
	odom_msg.twist.twist.angular.y = 0.0f;

	for (int i = 0; i < 36; i++) {
		odom_msg.pose.covariance[i]  = 0.0;
		odom_msg.twist.covariance[i] = 0.0;
	}

	odom_msg.pose.covariance[0]  = 0;   // x
	odom_msg.pose.covariance[7]  = 0;   // y
	odom_msg.pose.covariance[35] = 0;   // yaw
	odom_msg.twist.covariance[0] = 0;   // vx
	odom_msg.twist.covariance[7] = 0;   // vy
	odom_msg.twist.covariance[35]= 0;   // wz

	// -----------------------------------------------------------------------------------
	// IMU publishers
/*
	CHECK_RCL(rclc_publisher_init_default(
			&imu_pub,
			&node_base_controller,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
			"imu/data_raw"));

    // Temperature publisher
	CHECK_RCL(rclc_publisher_init_default(
			&imu_temp_pub,
			&node_base_controller,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
			"imu/temperature"));


	sensor_msgs__msg__Imu__init(&imu_msg);
	rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

	sensor_msgs__msg__Temperature__init(&imu_temp_msg);
	rosidl_runtime_c__String__assign(&imu_temp_msg.header.frame_id, "imu_link");

	// Covariances
	for (int i=0;i<9;i++){
	   imu_msg.angular_velocity_covariance[i] = 0.0001f;
	   imu_msg.linear_acceleration_covariance[i] = 0.04f;
	   imu_msg.orientation_covariance[i] = 0.0;
	}
	imu_msg.orientation_covariance[0] = -1.0; // unknown orientation

*/
	// -----------------------------------------------------------------------------------

/*
	// SONAR publishers
	static const char * SONAR_INDEX_LABELS = "order: front,right,back,left";

	/// ---- distances ----
	CHECK_RCL(rclc_publisher_init_default(
	    &sonar_pub,
	    &node_base_controller,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	    "sonar/distances"));

	// Describe the 1-D layout (N elements)
	std_msgs__msg__MultiArrayDimension__Sequence__init(&sonar_msg.layout.dim, 1);
	rosidl_runtime_c__String__assign(&sonar_msg.layout.dim.data[0].label, SONAR_INDEX_LABELS);
	sonar_msg.layout.dim.data[0].size   = NUM_SENSORS;
	sonar_msg.layout.dim.data[0].stride = NUM_SENSORS;
	sonar_msg.layout.data_offset        = 0;

	// Allocate data
	sonar_msg.data.capacity = NUM_SENSORS;
	sonar_msg.data.size     = NUM_SENSORS;
	sonar_msg.data.data     = (float*)malloc(NUM_SENSORS * sizeof(float));
	for (uint8_t i = 0; i < NUM_SENSORS; i++) sonar_msg.data.data[i] = 0.0f;

	/// ---- temperatures ----
	CHECK_RCL(rclc_publisher_init_default(
	    &sonar_temp_pub,
	    &node_base_controller,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	    "sonar/temperatures"));

	std_msgs__msg__MultiArrayDimension__Sequence__init(&sonar_temp_msg.layout.dim, 1);
	rosidl_runtime_c__String__assign(&sonar_temp_msg.layout.dim.data[0].label, SONAR_INDEX_LABELS);
	sonar_temp_msg.layout.dim.data[0].size   = NUM_SENSORS;
	sonar_temp_msg.layout.dim.data[0].stride = NUM_SENSORS;
	sonar_temp_msg.layout.data_offset        = 0;

	sonar_temp_msg.data.capacity = NUM_SENSORS;
	sonar_temp_msg.data.size     = NUM_SENSORS;
	sonar_temp_msg.data.data     = (float*)malloc(NUM_SENSORS * sizeof(float));
	for (uint8_t i = 0; i < NUM_SENSORS; i++) sonar_temp_msg.data.data[i] = 0.0f;

*/
	// -----------------------------------------------------------------------------------

}

void MicroROS_ExecutorInit(uint32_t executor_capacity)
{
    rclc_executor_init(&executor, &support.context, executor_capacity, &allocator);

    // Here, we set up odometry @ 100 Hz.
    /*
	  rclc_timer_init_default2(
		  &timer_odom,				// create timer = timer_odom
		  &support,					// Ensures entities share the same ROS context, clock, and memory and Holds micro-ROS runtime context and allocator
		  RCL_MS_TO_NS(10),			// callback period = 10 ms
		  odom_timer_cb,			// callback function
		  true);	 				// autostart = true
     */

	rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ALWAYS);

    // Slow sonar update rate: 250 ms
	  /*
	const unsigned int sonar_timer_period_ms = 250;
	rclc_timer_init_default2(
		&sonar_timer,
		&support,
		RCL_MS_TO_NS(sonar_timer_period_ms),
		sonar_timer_cb,
		true);

	rclc_executor_add_timer(&executor, &sonar_timer);
*/

    // extern void imu_timer_cb(rcl_timer_t * timer, int64_t last_call_time);
    // rclc_timer_init_default2(&imu_timer, &support, RCL_MS_TO_NS(10), imu_timer_cb, true);
    // rclc_executor_add_timer(&executor, &imu_timer);
}


// ---------------------------------------------------------------------------------------------------------------
// ---------------------------------------- End micro-ROS Code ---------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------


void nexus_bringup(void);

void nexus_bringup(void){

    HAL_TIM_Base_Start_IT(&htim6);

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

	//SONAR_SystemInit();

	init_motors(0);

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of myMicroROSTask */
  myMicroROSTaskHandle = osThreadNew(StartMicroROSTask, NULL, &myMicroROSTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 19200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FL_INA_GPO_Pin|FL_INB_GPO_Pin|RL_INB_GPO_Pin|RL_INA_GPO_Pin
                          |RR_INB_GPO_Pin|Ultrasonic_DE_RE_Pin|RL_VDD_GPO_Pin|FL_VDD_GPO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RR_INA_GPO_Pin|FR_INB_GPO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RR_VDD_GPO_Pin|FR_VDD_GPO_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FR_INA_GPO_GPIO_Port, FR_INA_GPO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FL_INA_GPO_Pin FL_INB_GPO_Pin RL_INB_GPO_Pin RL_INA_GPO_Pin
                           RR_INB_GPO_Pin Ultrasonic_DE_RE_Pin RR_VDD_GPO_Pin FR_VDD_GPO_Pin
                           RL_VDD_GPO_Pin FL_VDD_GPO_Pin */
  GPIO_InitStruct.Pin = FL_INA_GPO_Pin|FL_INB_GPO_Pin|RL_INB_GPO_Pin|RL_INA_GPO_Pin
                          |RR_INB_GPO_Pin|Ultrasonic_DE_RE_Pin|RR_VDD_GPO_Pin|FR_VDD_GPO_Pin
                          |RL_VDD_GPO_Pin|FL_VDD_GPO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RR_INA_GPO_Pin FR_INB_GPO_Pin */
  GPIO_InitStruct.Pin = RR_INA_GPO_Pin|FR_INB_GPO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FR_INA_GPO_Pin */
  GPIO_InitStruct.Pin = FR_INA_GPO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FR_INA_GPO_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// Call this periodically from your publishing task
void stm32_debug_publish(void)
{

	data_buffer[0] = (float)currCount[0];		// RL
	data_buffer[1] = (float)currCount[1];		// FL
	data_buffer[2] = (float)currCount[2];		// FR
	data_buffer[3] = (float)currCount[3];		// RR

	data_buffer[4] = (float)deltaEncoder[0];
	data_buffer[5] = (float)deltaEncoder[1];
	data_buffer[6] = (float)deltaEncoder[2];
	data_buffer[7] = (float)deltaEncoder[3];

	data_buffer[8]  = w_meas[0];
	data_buffer[9]  = w_meas[1];
	data_buffer[10] = w_meas[2];
	data_buffer[11] = w_meas[3];

	data_buffer[12] = (float)g_ccr_applied[0];
	data_buffer[13] = (float)g_ccr_applied[1];
	data_buffer[14] = (float)g_ccr_applied[2];
	data_buffer[15] = (float)g_ccr_applied[3];

    CHECK_RCL(rcl_publish(&debug_publisher, &debug_msg, NULL));

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartMicroROSTask */
/**
  * @brief  Function implementing the myMicroROSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMicroROSTask */
void StartMicroROSTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	//TickType_t last = xTaskGetTickCount();
	//const TickType_t step = pdMS_TO_TICKS(5);   // ~200 Hz check

	  nexus_bringup();

	  // micro-ROS  init
	  MicroROS_TransportInit(&huart2);
	  MicroROS_AllocatorInit();
	  while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) osDelay(100);
	  MicroROS_CoreInit();
	  MicroROS_TopicsInit();
	  MicroROS_ExecutorInit(5);
	  rmw_uros_sync_session(1000);

	  // Initialize IMU + calibration
	  //mpu6050_init(&imu);
	  //mpu6050_read_dma(&imu);
	  //imu_calibrate(&imu, 800);

	  PID_Init(0.055f, 0.04f, 0);
  /* Infinite loop */
  for(;;)
  {
	  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

	  Mecanum_Control(vx, vy, wz);
	  odom_publish();
	  //if (i2c1_need_reset){  i2c1_reset(); }

	  //imu_func();

	  //vTaskDelayUntil(&last, step);

  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM6) {
		currCount[0] = __HAL_TIM_GET_COUNTER(&htim4);		// RL
		currCount[1] = __HAL_TIM_GET_COUNTER(&htim1);		// FL
		currCount[2] = __HAL_TIM_GET_COUNTER(&htim3);		// FR
		currCount[3] = __HAL_TIM_GET_COUNTER(&htim8);		// RR

		for (int i=0; i<4; i++) {

		  deltaEncoder[i] = (int32_t)(currCount[i] - pastCount[i]);

		  if (deltaEncoder[i] > 32767)        deltaEncoder[i] -= (int16_t) 65536;
		  else if (deltaEncoder[i] < -32768)  deltaEncoder[i] += (int16_t) 65536;

		  w_meas[i] = (((float)deltaEncoder[i] * RAD_PER_TICK) / DeltaT);
		  pastCount[i] = currCount[i];
	  }
	    vx_odom = (WHEEL_R / 4.0f)          * (w_meas[0] + w_meas[1] + w_meas[2] + w_meas[3]);
	    vy_odom = (WHEEL_R / 4.0f)          * (w_meas[0] - w_meas[1] + w_meas[2] - w_meas[3]);
	    wz_odom = (WHEEL_R / (4.0f*A_SUM))  * (-w_meas[0] - w_meas[1] + w_meas[2] + w_meas[3]);

	      const float dt = DeltaT;   // 1 ms if TIM6 = 1 kHz
	      float c = cosf(th_odom);
	      float s = sinf(th_odom);
	      x_odom += (vx_odom * c - vy_odom * s) * dt;
	      y_odom += (vx_odom * s + vy_odom * c) * dt;
	      th_odom += wz_odom * dt;

	      if (th_odom > (float)M_PI)        th_odom -= 2.0f * (float)M_PI;
	      else if (th_odom < -(float)M_PI)  th_odom += 2.0f * (float)M_PI;


		//encUpdateFlag = 1;

	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
