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
// ----------------- Micro-ROS Part -----------------
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include "std_msgs/msg/float32_multi_array.h"
#include "sensor_msgs/msg/imu.h"
#include "sensor_msgs/msg/temperature.h"
#include "rosidl_runtime_c/string_functions.h"
#include "sensor_msgs/msg/range.h"
#include "sensor_msgs/msg/joint_state.h"

#include "rosidl_runtime_c/primitives_sequence_functions.h"

// ----------------- Nexus Part -----------------
#include "global_definitions.h"
#include "myMotor.h"
#include "mySensors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#ifndef ENC_DT_S               // TIM6 @ 1 kHz -> 1 ms between encoder snapshots
#define ENC_DT_S (0.001f)
#endif


// (try/except) or assert return codes
/*
#ifdef CHECK
#undef CHECK
#endif
#define CHECK(expr) do {                                      \
  rcl_ret_t _rc = (expr);                                     \
  if (_rc != RCL_RET_OK) {                                    \
    // blink or trap so you can see which call failed       \
    while (1) { HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); osDelay(200); } \
  }                                                           \
} while (0)
*/

/*
 QoS and reliability

rclc_publisher_init_default / rclc_subscription_init_default use the default QoS (reliable, keep last depth 10).

On flaky links (Wi-Fi/Serial) you might choose best-effort. You can:

use rclc’s best-effort helpers if available (e.g., rclc_publisher_init_best_effort), or

set options manually:

rcl_publisher_options_t opts = rcl_publisher_get_default_options();
opts.qos = rmw_qos_profile_sensor_data;  // best-effort
rcl_publisher_init(&pub_encoders, &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
  "wheel_status/encoders", &opts);
 */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
// ----------------- micro-ROS entities -----------------
static rclc_executor_t executor;
static rcl_timer_t timer;

/*
static rcl_publisher_t pub_cmd_echo, pub_encoders;
static rcl_subscription_t sub_cmd;

//ROS 2 Python analogy: node.create_publisher(...),
// 	 	 	 	 	  node.create_subscription(...),
// 	 	 	 	 	  create_timer(...),
// 	 	 	 	 	  rclpy.spin(node) runs callbacks.



// Messages & backing storage //
static std_msgs__msg__Int32MultiArray msg_cmd_echo;   // TX echo
static std_msgs__msg__Int32MultiArray msg_enc;        // TX encoders
//static std_msgs__msg__Int32MultiArray msg_cmd_rx;     // RX command
//static int32_t rx_data[4]  = {0,0,0,0};
//static volatile int32_t last_cmd[4] = {0,0,0,0};

// Messages & backing storage
static std_msgs__msg__Float32MultiArray msg_cmd_rx;   // RX command
static float rx_data[3]  = {0,0,0};
static volatile float last_cmd[3] = {0,0,0};          // targets: vx, vy, wz

static int32_t cmd_data[4] = {0,0,0,0};
static int32_t enc_data[4] = {0,0,0,0};
*/

// RX commands: [vx_mmps, vy_mmps, wz_radps]
static rcl_subscription_t sub_cmd;
static std_msgs__msg__Float32MultiArray msg_cmd_rx;
static float rx_data[3] = {0,0,0};
static volatile float last_cmd[3] = {0,0,0};
static volatile uint64_t last_cmd_stamp_ns = 0;

// Optional config: [v_step_mmps, wz_step_radps]
static rcl_subscription_t sub_cfg;
static std_msgs__msg__Float32MultiArray msg_cfg_rx;
static float cfg_buf[2] = {20.0f, 0.1f};

static rcl_publisher_t pub_ccr;
static std_msgs__msg__Int32MultiArray msg_ccr;
static int32_t ccr_data[4] = {0,0,0,0};

static rcl_publisher_t pub_imu;
static rcl_publisher_t pub_temp;
static sensor_msgs__msg__Imu imu_msg;
static sensor_msgs__msg__Temperature temp_msg;

/*
Using fixed-size Int32MultiArray by pointing msg->data to pre-allocated arrays (no heap allocations at runtime).

init_multiarray_4() sets m->data.data = backing; m->data.size = 4; m->data.capacity = 4; and clears the layout (which is fine if you don’t need it).

ROS 2 Python analogy:

from std_msgs.msg import Int32MultiArray
msg = Int32MultiArray()
msg.data = [0,0,0,0]  # layout usually ignored
 */

// ----------------- Nexus Part -----------------
volatile uint16_t currCount[4] = {0};		// {RL, FL, FR,, RR}
uint16_t pastcurrCount[4] = {0};			// {RL, FL, FR,, RR}
int32_t deltaEncoder[4] = {0};

int32_t omega[4] = {0};     					// rad/s

SONAR_HandleTypeDef sonar1;
SONAR_HandleTypeDef sonar2;
SONAR_HandleTypeDef sonar3;
SONAR_HandleTypeDef sonar4;

uint8_t sonarCurr = 0;
uint16_t distBuf[4] = {0};
float tempBuf[4];

uint32_t adcBuffer[NUM_WHEELS];     // DMA stores ADC results here
uint8_t motorFaultFlags[10] = {0};  // General array for faults, first 4 are for motors: [RL, FL, FR, RR, ....]

static const char* WHEEL_NAMES[NUM_WHEELS] = {
  "wheel_rl","wheel_fl","wheel_fr","wheel_rr"
};

#ifndef NUM_MOTORS
#define NUM_MOTORS 4
#endif
volatile float g_motor_current_A[NUM_MOTORS];  // updated in Process_Motor_Currents
volatile int32_t g_motor_fault_flag[NUM_MOTORS];

int CCR[4];

// ---------- Publishers & messages ----------
rcl_publisher_t pub_sonar_range[NUM_SONARS];
rcl_publisher_t pub_sonar_temp[NUM_SONARS];
rcl_publisher_t pub_joint;
rcl_publisher_t pub_motor_current;
rcl_publisher_t pub_motor_fault;

sensor_msgs__msg__Range       sonar_range_msg[NUM_SONARS];
sensor_msgs__msg__Temperature sonar_temp_msg_[NUM_SONARS];
sensor_msgs__msg__JointState  joint_msg;
std_msgs__msg__Float32MultiArray motor_currents_msg;
std_msgs__msg__Int32MultiArray   motor_faults_msg;

// Encoder publishing state (positions)
static double wheel_pos_rad[NUM_WHEELS] = {0};

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
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
// ----------------- Micro-ROS Part -----------------
// Opens and initializes the custom transport (e.g., UART) for micro-ROS communication
bool cubemx_transport_open(struct uxrCustomTransport * transport);

// Closes and cleans up the custom transport connection
bool cubemx_transport_close(struct uxrCustomTransport * transport);

// Writes (sends) data through the custom transport
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);

// Reads (receives) data from the custom transport with a timeout
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

// Allocates a memory block of the given size (used by micro-ROS memory management)
void * microros_allocate(size_t size, void * state);

// Frees a previously allocated memory block
void microros_deallocate(void * pointer, void * state);

// Reallocates an existing memory block to a new size
void * microros_reallocate(void * pointer, size_t size, void * state);

// Allocates zero-initialized memory for an array (similar to calloc)
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

// Callback function triggered when a wheel command message is received
static void wheel_cmd_cb(const void * msgin);

// Callback function executed periodically by a timer
static void timer_cb(rcl_timer_t * t, int64_t last_call_time);

// Reads sonar sensors and publishes their range and temperature data
//static void sonar_service_and_publish(void);

// Publishes current readings and fault status from the motors
static void publish_motor_currents_and_faults(void);

// Publishes the robot’s joint states using data from the encoders
static void publish_joint_state_from_encoders(void);

// Returns the current system time in nanoseconds (high-resolution timestamp)
static inline uint64_t now_nanos(void);

// ----------------- Nexus Part -----------------
void Process_Motor_Currents(void);
uint8_t Sonar_Update(void);
void nexus_bringup(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void nexus_bringup(void){
    HAL_TIM_Base_Start_IT(&htim6);

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

	SONAR_Init(&sonar1, &huart5, 0x11);
	SONAR_Init(&sonar2, &huart5, 0x12);
	SONAR_Init(&sonar3, &huart5, 0x13);
	SONAR_Init(&sonar4, &huart5, 0x14);

	//HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, NUM_WHEELS);

	//mpu_init();

	init_motors();
}

// Initializes an Int32MultiArray message with 4 fixed elements backed by a provided buffer
static void init_int_multiarray_4(std_msgs__msg__Int32MultiArray* m, int32_t* backing)
{
  std_msgs__msg__Int32MultiArray__init(m);
  m->layout.dim.data = NULL; m->layout.dim.size = 0; m->layout.dim.capacity = 0;
  m->layout.data_offset = 0;
  m->data.data = backing; m->data.size = 4; m->data.capacity = 4;
}

// Initializes a Float32MultiArray message with a fixed number (n) of elements backed by a provided buffer
static void init_float_multiarray_fixed(std_msgs__msg__Float32MultiArray* m, float* backing, size_t n)
{
  std_msgs__msg__Float32MultiArray__init(m);
  m->layout.dim.data = NULL; m->layout.dim.size = 0; m->layout.dim.capacity = 0;
  m->layout.data_offset = 0;
  m->data.data = backing; m->data.size = n; m->data.capacity = n;
}

// Returns the current time in nanoseconds
// Uses synchronized micro-ROS epoch time if available, otherwise falls back to local HAL tick count
static inline uint64_t now_nanos(void)
{
  // Use Agent-synced time if available; fallback to local
  uint64_t t = rmw_uros_epoch_nanos();
  if (t == 0) {
    // fallback: convert HAL_GetTick() ms to ns
    t = (uint64_t)HAL_GetTick() * 1000000ULL;
  }
  return t;
}


static inline bool cmd_is_fresh(uint64_t now_ns, uint64_t max_age_ns) {
  uint64_t age = now_ns - last_cmd_stamp_ns;
  // handle startup: last_cmd_stamp_ns==0 -> treat as stale
  return last_cmd_stamp_ns != 0 && age < max_age_ns;
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
  //MX_ADC1_Init();
  MX_TIM6_Init();
  //MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  nexus_bringup();
  HAL_Delay(2000);
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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
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
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
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

/* Fixed-size 4-element Int32MultiArray bound to external storage */
/*
static void init_float_multiarray_3(std_msgs__msg__Float32MultiArray* m, float* backing) {
  std_msgs__msg__Float32MultiArray__init(m);
  m->layout.dim.data = NULL; m->layout.dim.size = 0; m->layout.dim.capacity = 0;
  m->layout.data_offset = 0;
  m->data.data = backing; m->data.size = 3; m->data.capacity = 3;
}

static void init_int_multiarray_4(std_msgs__msg__Int32MultiArray* m, int32_t* backing) {
  std_msgs__msg__Int32MultiArray__init(m);
  m->layout.dim.data = NULL; m->layout.dim.size = 0; m->layout.dim.capacity = 0;
  m->layout.data_offset = 0;
  m->data.data = backing; m->data.size = 4; m->data.capacity = 4;
}
*/

static void publish_motor_currents_and_faults(void)
{
  for (int i=0;i<NUM_MOTORS;i++) {
    motor_currents_msg.data.data[i] = g_motor_current_A[i];
    motor_faults_msg.data.data[i]   = g_motor_fault_flag[i];
  }
  rcl_publish(&pub_motor_current, &motor_currents_msg, NULL);
  rcl_publish(&pub_motor_fault,   &motor_faults_msg,   NULL);
}


/*
static void sonar_service_and_publish(void)
{

  uint8_t idx1 = Sonar_Update();  // returns 1..4 per your code
  int i = (idx1 - 1) & 0x3;

  // Convert to meters (adjust SONAR_DIST_SCALE_M to your driver)
  float dist_m = ((float)distBuf[i]) * SONAR_DIST_SCALE_M;
  float temp_c = (float)tempBuf[i];   // assume °C; adjust if needed

  // Clamp to message limits
  if (dist_m < SONAR_MIN_RANGE_M) dist_m = SONAR_MIN_RANGE_M;
  if (dist_m > SONAR_MAX_RANGE_M) dist_m = SONAR_MAX_RANGE_M;

  uint64_t t = now_nanos();

  sensor_msgs__msg__Range *rng = &sonar_range_msg[i];
  rng->header.stamp.sec     = (int32_t)(t / 1000000000ULL);
  rng->header.stamp.nanosec = (uint32_t)(t % 1000000000ULL);
  rng->range = dist_m;
  rcl_publish(&pub_sonar_range[i], rng, NULL);

  sensor_msgs__msg__Temperature *tp = &sonar_temp_msg_[i];
  tp->header.stamp.sec     = rng->header.stamp.sec;
  tp->header.stamp.nanosec = rng->header.stamp.nanosec;
  tp->temperature = temp_c;
  tp->variance = 0.5;  // optional
  rcl_publish(&pub_sonar_temp[i], tp, NULL);

}
*/

const float TICK_TO_RAD = (2.0 * M_PI) / (double)TICKS_PER_REV;

static void publish_joint_state_from_encoders(void)
{
  // Snapshot encoder deltas atomically
  int16_t delta[NUM_WHEELS];

  //__disable_irq();
	  delta[0] = __HAL_TIM_GET_COUNTER(&htim4);
	  delta[1] = __HAL_TIM_GET_COUNTER(&htim1);
	  delta[2] = __HAL_TIM_GET_COUNTER(&htim3);
	  delta[3] = __HAL_TIM_GET_COUNTER(&htim8);
  //__enable_irq();

  // velocity in rad/s, position in rad (integrated)
  for (int i=0;i<NUM_WHEELS;i++) {
    double vel = ((double)delta[i]) * TICK_TO_RAD / (double)ENC_DT_S;
    joint_msg.velocity.data[i] = vel;

    wheel_pos_rad[i] += ((double)delta[i]) * TICK_TO_RAD;
    joint_msg.position.data[i] = wheel_pos_rad[i];

    joint_msg.effort.data[i] = 0.0;
  }

  uint64_t t = now_nanos();
  joint_msg.header.stamp.sec     = (int32_t)(t / 1000000000ULL);
  joint_msg.header.stamp.nanosec = (uint32_t)(t % 1000000000ULL);
  // frame_id is typically empty for JointState; leave default

  rcl_publish(&pub_joint, &joint_msg, NULL);
}


// 200 Hz timer callback IMU
/*
static void imu_timer_cb(rcl_timer_t * timer, int64_t last_call_time)
{
  (void)timer; (void)last_call_time;

  float ax, ay, az, gx, gy, gz, tc;
  if (!mpu_read_sample(&ax,&ay,&az,&gx,&gy,&gz,&tc)) return;

  uint64_t t = now_nanos();
  imu_msg.header.stamp.sec     = (int32_t)(t / 1000000000ULL);
  imu_msg.header.stamp.nanosec = (uint32_t)(t % 1000000000ULL);

  imu_msg.angular_velocity.x = gx;
  imu_msg.angular_velocity.y = gy;
  imu_msg.angular_velocity.z = gz;

  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;

  // No orientation from the chip; mark as invalid
  imu_msg.orientation.x = imu_msg.orientation.y = imu_msg.orientation.z = imu_msg.orientation.w = 0.0;
  imu_msg.orientation_covariance[0] = -1.0;  // per message spec

  // Rough diagonal covariances to start (tune later)
  const double gyro_var  = (0.02 * 0.02);   // (rad/s)^2
  const double accel_var = (0.05 * 0.05);   // (m/s^2)^2
  for (int i=0;i<9;i++) {
    imu_msg.angular_velocity_covariance[i]  = 0.0;
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
  imu_msg.angular_velocity_covariance[0] = gyro_var;
  imu_msg.angular_velocity_covariance[4] = gyro_var;
  imu_msg.angular_velocity_covariance[8] = gyro_var;
  imu_msg.linear_acceleration_covariance[0] = accel_var;
  imu_msg.linear_acceleration_covariance[4] = accel_var;
  imu_msg.linear_acceleration_covariance[8] = accel_var;

  rcl_ret_t rc1 = rcl_publish(&pub_imu, &imu_msg, NULL);
  if (rc1 != RCL_RET_OK) { // Error handle  }

  static uint8_t decim = 0;
  if (++decim >= 4) {  // publish temperature at 50 Hz
    decim = 0;
    temp_msg.header.stamp.sec     = imu_msg.header.stamp.sec;
    temp_msg.header.stamp.nanosec = imu_msg.header.stamp.nanosec;
    temp_msg.temperature = tc;
    temp_msg.variance = 0.5; // arbitrary; tune if you use it
    rcl_ret_t rc2 = rcl_publish(&pub_temp, &temp_msg, NULL);
    if (rc2 != RCL_RET_OK) { // Error handle  }

  }

}

*/


// Read encoder with wrap-safe accumulation to 32-bit
/*
static int32_t encoder_sample_accum(motor_t* m)
{
  uint16_t raw = __HAL_TIM_GET_COUNTER(m->htim_enc);
  int16_t delta = (int16_t)((int32_t)raw - (int32_t)(uint16_t)m->last_raw);
  m->last_raw = (int16_t)raw;
  m->accum += delta;              // delta can be negative; handles wrap //
  return m->accum;
}
*/

// Commands: [vx_mmps, vy_mmps, wz_radps]
static void wheel_cmd_cb(const void * msgin) {

  const std_msgs__msg__Float32MultiArray *m = msgin;

  // Enforce exactly 3 values; ignore partial/empty messages
  if (m->data.size != 3) {
    // Optional: record a diagnostic counter here
    return;
  }

  last_cmd[0] = m->data.data[0];
  last_cmd[1] = m->data.data[1];
  last_cmd[2] = m->data.data[2];
  last_cmd_stamp_ns = now_nanos();

}

// Ramp config: [v_step_mmps, wz_step_radps]
static void cfg_cb(const void * msgin)
{
  const std_msgs__msg__Float32MultiArray *m =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  float vstep  = (m->data.size >= 1) ? m->data.data[0] : g_v_step_mmps;
  float wzstep = (m->data.size >= 2) ? m->data.data[1] : g_wz_step_radps;
  ctrlparams_set_steps(vstep, wzstep);
}

// 100 Hz executor timer
static void timer_cb(rcl_timer_t * t, int64_t) {
  (void)t;

  /*
  const uint64_t now = now_nanos();
  const uint64_t CMD_TIMEOUT_NS = 200ULL * 1000ULL * 1000ULL; // 200 ms
*/

  float vx = last_cmd[0], vy = last_cmd[1], wz = last_cmd[2];

  /*
  // If no fresh command, either keep the last non-zero setpoint
  // or enter a local fallback (your choice):
  //if (!cmd_is_fresh(now, CMD_TIMEOUT_NS)) {
     //vx = vy = wz = 0.0f;
  //}
   */

  Mecanum_Control(vx, vy, wz);

  for (int i = 0; i < 4; ++i) ccr_data[i] = g_ccr_applied[i];
  (void)rcl_publish(&pub_ccr, &msg_ccr, NULL);

  //sonar_service_and_publish();
  publish_motor_currents_and_faults();
  publish_joint_state_from_encoders();
}

// ----------------- Nexus Part -----------------

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        Process_Motor_Currents();
    }
}

void Process_Motor_Currents(void)
{
	  for (int i = 0; i < NUM_MOTORS; i++) {

		float vADC = (adcBuffer[i] / ADC_RESOLUTION) * ADC_REF_VOLTAGE;
		float vCS  = vADC * VOLTAGE_DIVIDER_GAIN;
		float iSense = vCS / R_SENSE;
		float iMotor = iSense * K_SENSE;

		g_motor_current_A[i] = iMotor;               // <-- store for ROS timer
		if (iMotor > STALL_CURRENT) {
		  motorFaultFlags[i] = 1;
		  g_motor_fault_flag[i] = 1;                 // mirror for ROS
		  Emergency_Stop();
		} else {
		  motorFaultFlags[i] = 0;
		  g_motor_fault_flag[i] = 0;
		}
	  }
}

uint8_t Sonar_Update(void) {
    static uint8_t sonarCurr = 0;

    // advance to next sensor
    sonarCurr = (sonarCurr % 4) + 1;

    switch (sonarCurr) {
        case 1:
            distBuf[1] = SONAR_ReadDistance(&sonar2);
            tempBuf[1] = SONAR_ReadTemperature(&sonar2);
            SONAR_Trigger(&sonar2);
            break;
        case 2:
            distBuf[2] = SONAR_ReadDistance(&sonar3);
            tempBuf[2] = SONAR_ReadTemperature(&sonar3);
            SONAR_Trigger(&sonar3);
            break;
        case 3:
            distBuf[3] = SONAR_ReadDistance(&sonar4);
            tempBuf[3] = SONAR_ReadTemperature(&sonar4);
            SONAR_Trigger(&sonar4);
            break;
        case 4:
            distBuf[0] = SONAR_ReadDistance(&sonar1);
            tempBuf[0] = SONAR_ReadTemperature(&sonar1);
            SONAR_Trigger(&sonar1);
            break;
    }
    return sonarCurr;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	/* -------- transport -------- */
	  rmw_uros_set_custom_transport(
	      true, (void *)&huart2,
	      cubemx_transport_open, cubemx_transport_close,
	      cubemx_transport_write, cubemx_transport_read);

	  /* -------- allocators -------- */
	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate      = microros_allocate;
	  freeRTOS_allocator.deallocate    = microros_deallocate;
	  freeRTOS_allocator.reallocate    = microros_reallocate;
	  freeRTOS_allocator.zero_allocate = microros_zero_allocate;
	  rcl_ret_t rc4 =  rcutils_set_default_allocator(&freeRTOS_allocator);
	  if (rc4 != RCL_RET_OK) { /* Error handle */ }

	  /* -------- wait for agent (~5 s) -------- */
	  for (int i = 0; i < 50; ++i) {
	    if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) break;
	    osDelay(100);
	  }

	  /* -------- micro-ROS graph -------- */
	  rclc_support_t support;
	  rcl_allocator_t allocator = rcl_get_default_allocator();
	  rclc_support_init(&support, 0, NULL, &allocator);

	  rcl_node_t node_base_controller;
	  rclc_node_init_default(&node_base_controller, "base_controller", "", &support);

	  rcl_node_t node;
	  rclc_node_init_default(&node, "mpu6050_node", "", &support);

	  /* -------- publishers -------- */
	  rclc_publisher_init_default(
	      &pub_imu, &node,
	      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
	      "imu/data_raw");

	  rclc_publisher_init_default(
	      &pub_temp, &node,
	      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
	      "imu/temperature");

	  rcl_publisher_options_t qos_sensor = rcl_publisher_get_default_options();
	  qos_sensor.qos = rmw_qos_profile_sensor_data;

	  // ---- SONAR publishers (4x range + 4x temp) ----
	  for (int i=0; i<NUM_SONARS; ++i) {
	    char topic_rng[32];  snprintf(topic_rng, sizeof(topic_rng), "sonar/%d/range", i+1);
	    rcl_publisher_init(&pub_sonar_range[i], &node_base_controller,
	        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), topic_rng, &qos_sensor);

	    char topic_tmp[40];  snprintf(topic_tmp, sizeof(topic_tmp), "sonar/%d/temperature", i+1);
	    rcl_publisher_init(&pub_sonar_temp[i], &node_base_controller,
	        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), topic_tmp, &qos_sensor);
	  }

	  // Pre-fill constant fields & frame_ids
	  for (int i=0; i<NUM_SONARS; ++i) {
	    sensor_msgs__msg__Range *m = &sonar_range_msg[i];
	    memset(m, 0, sizeof(*m));
	    m->radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
	    m->field_of_view  = SONAR_FOV_RAD;
	    m->min_range      = SONAR_MIN_RANGE_M;
	    m->max_range      = SONAR_MAX_RANGE_M;
	    rosidl_runtime_c__String__init(&m->header.frame_id);
	    char fid[32]; snprintf(fid, sizeof(fid), "sonar_%d_link", i+1);
	    rosidl_runtime_c__String__assign(&m->header.frame_id, fid);

	    sensor_msgs__msg__Temperature *t = &sonar_temp_msg_[i];
	    memset(t, 0, sizeof(*t));
	    rosidl_runtime_c__String__init(&t->header.frame_id);
	    rosidl_runtime_c__String__assign(&t->header.frame_id, fid);
	  }

	  // ---- JointState publisher ----
	  rcl_publisher_init(
	    &pub_joint, &node_base_controller,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
	    "joint_states", NULL /*default QoS*/);

	  // JointState sequences (names/position/velocity/effort)
	  rosidl_runtime_c__String__Sequence__init(&joint_msg.name, NUM_WHEELS);
	  for (int i=0;i<NUM_WHEELS;++i)
	    rosidl_runtime_c__String__assign(&joint_msg.name.data[i], WHEEL_NAMES[i]);

	  rosidl_runtime_c__double__Sequence__init(&joint_msg.position, NUM_WHEELS);
	  rosidl_runtime_c__double__Sequence__init(&joint_msg.velocity, NUM_WHEELS);
	  rosidl_runtime_c__double__Sequence__init(&joint_msg.effort,   NUM_WHEELS);

	  // ---- Motor currents publishers ----
	  rcl_publisher_init(&pub_motor_current, &node_base_controller,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	    "motors/current_amps", &qos_sensor);

	  rcl_publisher_init(&pub_motor_fault, &node_base_controller,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
	    "motors/fault_flags", NULL);

	  // Init fixed-length arrays
	  rosidl_runtime_c__float__Sequence__init(&motor_currents_msg.data, NUM_MOTORS);
	  rosidl_runtime_c__int32__Sequence__init(&motor_faults_msg.data, NUM_MOTORS);
	  /* (your commented publishers remain unchanged)
	  // rclc_publisher_init_default(&pub_cmd_echo, &node, ...);
	  // rclc_publisher_init_default(&pub_encoders, &node, ...);
	  */

	  /* -------- message buffers you already use -------- */
	  init_float_multiarray_fixed(&msg_cmd_rx, rx_data, 3);

	  rclc_subscription_init_default(
	      &sub_cmd, &node_base_controller,
	      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	      "twist_nexus");

	  init_float_multiarray_fixed(&msg_cfg_rx, cfg_buf, 2);

	  rclc_subscription_init_default(
	      &sub_cfg, &node_base_controller,
	      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
	      "nexus_ctrl/config");

	  init_int_multiarray_4(&msg_ccr, ccr_data);

	  rclc_publisher_init_default(
	      &pub_ccr, &node_base_controller,
	      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
	      "wheel_status/ccr");

	  /* -------- fixed frame_id for IMU & temp -------- */
	  memset(&imu_msg, 0, sizeof(imu_msg));
	  rosidl_runtime_c__String__init(&imu_msg.header.frame_id);
	  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

	  memset(&temp_msg, 0, sizeof(temp_msg));
	  rosidl_runtime_c__String__init(&temp_msg.header.frame_id);
	  rosidl_runtime_c__String__assign(&temp_msg.header.frame_id, "imu_link");

	  /* (optional) try to sync time with Agent */
	  (void) rmw_uros_sync_session(1000);

	  /* -------- timers -------- */
	  // Your existing 100 Hz timer
	  rclc_timer_init_default2(&timer, &support, RCL_MS_TO_NS(10), timer_cb, true);

	  // New 200 Hz IMU timer
	  //rcl_timer_t timer_imu;
	  //rclc_timer_init_default2(&timer_imu, &support, RCL_MS_TO_NS(5), imu_timer_cb, true);

	  /* -------- executor (ONE instance) -------- */
	  rclc_executor_init(&executor, &support.context, /*handles*/ 4, &allocator);
	  rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd_rx, &wheel_cmd_cb, ON_NEW_DATA);
	  rclc_executor_add_subscription(&executor, &sub_cfg, &msg_cfg_rx, &cfg_cb,     ON_NEW_DATA);
	  rclc_executor_add_timer(&executor, &timer);
	  //rclc_executor_add_timer(&executor, &timer_imu);

	  /* -------- main loop -------- */
	  for (;;) {
	    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
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
/*
	if (htim->Instance == TIM6) {
		currCount[0] = __HAL_TIM_GET_COUNTER(&htim4);
		currCount[1] = __HAL_TIM_GET_COUNTER(&htim1);
		currCount[2] = __HAL_TIM_GET_COUNTER(&htim3);
		currCount[3] = __HAL_TIM_GET_COUNTER(&htim8);

		for (int i=0; i<4; i++) {

		  deltaEncoder[i] = (int16_t)(currCount[i] - pastcurrCount[i]);		// {RL, FL, FR, RR}

		  // Handling 16-bit overflow
		  if (deltaEncoder[i] > 32767)       deltaEncoder[i] -= 65536;		// underflow
		  else if (deltaEncoder[i] < -32768) deltaEncoder[i] += 65536;		// overflow

		  // Calculating wheel angular velocities (rad/s)
		  omega[i] = (int32_t)(deltaEncoder[i] * 2 );  // omega[i] = (deltaEncoder[i] * 2 * PI) / (3072 * 0.001) = deltaEncoder[i] * 2.05

		  pastcurrCount[i] = currCount[i];
	  }
	}
*/
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
