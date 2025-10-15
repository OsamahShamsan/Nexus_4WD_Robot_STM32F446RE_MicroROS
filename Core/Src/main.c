/*
 * ============================================================================
 *  STEP 1: Includes
 *  ============================================================================
*/

/*
//   Organised from hardware → middleware → user layer.
// 	 Shows what each header provides and which symbols (structs / functions) live in it.
//   	[1.1] MCU / RTOS base headers
//   	[1.2] micro-ROS core runtime
//   	[1.3] Common ROS2 message definitions
//   	[1.4] Optional utilities and user modules
// ============================================================================
*/

/*
// [1.1] MCU / RTOS Layer ------------------------------------------------------

// These headers come from CubeMX and CMSIS.
// Provide HAL peripherals and RTOS primitives (threads, delays, queues, etc.).
*/
#include "main.h"          // HAL handles (TIM_HandleTypeDef, GPIO_TypeDef, etc.)
#include "cmsis_os.h"      // osThread*, osDelay, semaphore and FreeRTOS wrappers

/*
// [1.2] micro-ROS Core Runtime -----------------------------------------------

// Core client libraries providing the embedded ROS2 interface.

// --- <rclc/rclc.h> -----------------------------------------------------------
// High-level C API wrapping the low-level rcl layer.
// Defines rcl_node_t, rcl_allocator_t, rclc_support_t, rclc_init(), rclc_node_init_default().
// Also exposes rclc_publisher_init_default(), rclc_subscription_init_default().
// headers included within are:
// #include <rcl/time.h>             // rcl_time_point_t, rcl_duration_t
// #include <rclc/timer.h>           // rcl_timer_t, rclc_timer_init_default()
// #include <rclc/publisher.h>       // rcl_publisher_t helpers (explicit include)
// #include <rclc/subscription.h>    // rcl_subscription_t helpers
*/
#include <rclc/rclc.h>

/*
// --- <rclc/executor.h> -------------------------------------------------------

// Scheduler that runs callbacks and timers.
// Provides rclc_executor_t, rclc_executor_init(), rclc_executor_spin_some().
// Links topics to user callbacks with rclc_executor_add_subscription() / add_timer().
*/
#include <rclc/executor.h>

/*
// --- <rmw_microros/rmw_microros.h> ------------------------------------------

// Transport interface for UART, UDP, CAN, SPI.
// Contains rmw_uros_set_custom_transport(), rmw_uros_ping_agent().
// Handles serialization and DDS communication toward micro-ROS Agent.
*/
#include <rmw_microros/rmw_microros.h>

/*
// --- <rosidl_runtime_c/string_functions.h> -----------------------------------

// String memory management used by all ROS messages.
// Defines rosidl_runtime_c__String struct and helpers: _assign(), _init(), _fini().
*/
#include <rosidl_runtime_c/string_functions.h>

/*
// Optional: low-level or debugging

// #include <rcl/rcl.h>                // Only if you need fine-grained control
// #include <rcl/error_handling.h>     // Optional, for printing errors
*/


/*
// [1.3] Message Definitions ----------------------------------------------------
*/

/*
// ROS2 message definitions generated from .msg files by rosidl_generator_c.
// Each header defines a C struct and its _init(), _fini(), and *_TypeSupport functions.

// --- <geometry_msgs/msg/twist.h> ---------------------------------------------
// struct geometry_msgs__msg__Twist { Vector3 linear; Vector3 angular; }
// Used for velocity commands (/cmd_vel, /twist_nexus).
*/
#include <geometry_msgs/msg/twist.h>

/*
// --- <nav_msgs/msg/odometry.h> -----------------------------------------------

// struct nav_msgs__msg__Odometry { Header header; PoseWithCovariance pose; TwistWithCovariance twist; }
// Typically used for odometry publishers.
*/
#include <nav_msgs/msg/odometry.h>

/*
// Optional future message types:

// --- IMU ---
//#include <sensor_msgs/msg/imu.h>           // orientation, angular velocity, accel
//#include <geometry_msgs/msg/vector3.h>     // used inside IMU for linear/angular
//#include <geometry_msgs/msg/quaternion.h>  // orientation quaternion
//#include <std_msgs/msg/header.h>           // timestamp + frame_id shared by all msgs

// --- Ultrasonic distance + temperature ---
//#include <sensor_msgs/msg/range.h>         // ultrasonic distance (m)
//#include <sensor_msgs/msg/temperature.h>   // ambient/sensor temperature (°C)

// #include <sensor_msgs/msg/magnetic_field.h>  // magnetometer data
// #include <std_msgs/msg/float32.h>            // scalar telemetry (battery voltage)
// #include <std_msgs/msg/bool.h>               // enable/stop flags
// #include <diagnostic_msgs/msg/key_value.h>   // system diagnostics
// #include <sensor_msgs/msg/joint_state.h>     // wheel encoder states (if needed)

*/

/*
// [1.4] User-Space Modules  ---------------------------------------------------
*/

/*
// User-developed components — hardware interfaces and control logic.
// These files should *never* include rcl* or rmw* headers.
// They implement the real robot behaviour and will grow with the system.
*/
#include "robot_params.h"			// wheel geometry, PID constants, topic settings
#include "motor_driver.h"			// PWM, direction control, Mecanum_Control()
#include "imu_interface.h"			// I2C/SPI IMU readout and filtering
#include "ultrasonic_array.h"		// trigger/echo handling for multiple sensors
#include "odom_handler.h"			// encoder integration, pose update

/*
 *  This step rarely changes except when adding new sensors or message types.
 * --------------------------------------------------------------------------------------
*/

/* --------------------------------------------------------------------------------------
 * END OF STEP 1
 * --------------------------------------------------------------------------------------
 */


TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;

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



/* ======================================================================================
 * STEP 2 — micro-ROS Core Objects and Initialization
 * ======================================================================================
 */

/*
 * This section defines the fundamental structures that every micro-ROS node running
 * on STM32 + FreeRTOS must have. These handle:
 *   - memory allocation,
 *   - node creation,
 *   - executor callback scheduling,
 *   - and topic publishing/subscription.
 *
 * Most of these are one-time system-wide objects — they exist only once per MCU.
 * Only publishers/subscribers/timers are created per topic or per feature module.
 *
 * ======================================================================================
 */

/* --------------------------------------------------------------------------------------
 * [2.1] Core micro-ROS handles — initialized once in the application.
 * --------------------------------------------------------------------------------------
 */
rcl_allocator_t allocator;   // Provides memory management functions to the ROS client
                             // Usually obtained via rcl_get_default_allocator()

rclc_support_t support;      // Aggregates allocator + init options + communication support
                             // Initialized via rclc_support_init()

rcl_node_t node;             // Represents the MCU node in the ROS graph
                             // Created once with rclc_node_init_default(&node, "node_name", "", &support)

rclc_executor_t executor;    // Task scheduler running all callbacks (timers, subscriptions, etc.)
                             // Created once via rclc_executor_init()

/* --------------------------------------------------------------------------------------
 * [2.2]Example topic handles — each topic has its own publisher or subscriber.
 * --------------------------------------------------------------------------------------
 */
extern rcl_subscription_t twist_sub;     	// Subscription object → listens to /twist_nexus topic
extern geometry_msgs__msg__Twist twist_msg;  // Struct holding the received Twist message data

extern rcl_publisher_t odom_pub;         // Publisher object → publishes /odom topic
extern nav_msgs__msg__Odometry odom_msg; // Struct holding odometry data to send back to ROS2

/*
 *  - allocator, support, node, and executor → created once in app initialization.
 *  - publishers, subscribers, timers → one per topic or feature (can be multiple).
 *  - message structs (like twist_msg, odom_msg) → user-owned, reused between callbacks.
 *
 * Keep global to avoid stack use and allow cross-module access (e.g., odometry, control).
 * --------------------------------------------------------------------------------------
 */

/* --------------------------------------------------------------------------------------
 * END OF STEP 2
 * --------------------------------------------------------------------------------------
*/


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);

void StartDefaultTask(void *argument);





/* ======================================================================================
 * STEP 3 — micro-ROS Communication and Memory Interfaces
 * ======================================================================================
 */


/* --------------------------------------------------------------------------------------
 *  [3.1] Communication and Memory Layer Management
 * --------------------------------------------------------------------------------------
*/

/*
 * This step defines all the low-level interfaces that allow the microcontroller to:
 *   1️- communicate with the ROS 2 agent on the host PC,
 *   2️- manage dynamic memory safely on an RTOS.
 *
 * These are *system-level hooks* — used by the micro-ROS middleware itself,
 * not directly by the user control logic. Usually they are implemented or provided once.
 *
 * --------------------------------------------------------------------------------------
 *  Communication Layer (transport) — used internally by Micro XRCE-DDS
 * --------------------------------------------------------------------------------------
 *  These functions implement the physical link between the MCU and the ROS 2 agent.
 *  Typical transports: UART, USB CDC, CAN, UDP, etc.
 *
 *  - They are called by the micro-ROS stack (not by user code).
 *  - You implement them once in your project, usually in transport .c
 *
 * --------------------------------------------------------------------------------------
 *  Memory Layer — used internally by rcl/rmw for allocation.
 * --------------------------------------------------------------------------------------
 *  micro-ROS allows replacing malloc/free with user-provided memory functions
 *  (to integrate with FreeRTOS heap or custom memory pools).
 *
 * --------------------------------------------------------------------------------------
 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);             // Initialize UART/USB/UDP port
bool cubemx_transport_close(struct uxrCustomTransport * transport);            // Close or power down link
size_t cubemx_transport_write(struct uxrCustomTransport* transport,            // Send raw bytes to agent
                              const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport,             // Read bytes from agent (blocking or timeout)
                             uint8_t* buf, size_t len, int timeout, uint8_t* err);

/* --------------------------------------------------------------------------------------
 *  [3.2] Memory allocation hooks (optional overrides)
 * --------------------------------------------------------------------------------------
 */

/*
 *  The micro-ROS allocator will call these instead of malloc/free if configured.
 *  They must be thread-safe under FreeRTOS.
 * --------------------------------------------------------------------------------------
 */
void * microros_allocate(size_t size, void * state);                            // Allocates 'size' bytes
void microros_deallocate(void * pointer, void * state);                         // Frees memory
void * microros_reallocate(void * pointer, size_t size, void * state);          // Resize allocation
void * microros_zero_allocate(size_t n, size_t size_of_elem, void * state);     // Allocates and zeros array

/* --------------------------------------------------------------------------------------
 *  [3.3] User callback prototypes
 * --------------------------------------------------------------------------------------
 */

/*
 *  These are custom functions the user provides for the robot’s logic.
 *  Here we declare odometry and control callbacks used in later steps.
 * --------------------------------------------------------------------------------------
 */
void odom_timer_cb(rcl_timer_t * timer, int64_t last_call_time);	 // Publish callback   (Timer callback to compute and publish odometry)
void twist_callback(const void * msgin);							 // Subscribe callback

/* --------------------------------------------------------------------------------------
 * END OF STEP 3
 * --------------------------------------------------------------------------------------
*/



/* ======================================================================================
 * STEP X: User definitions and functions prototypes
 * ======================================================================================
 */

volatile int16_t deltaEncoder[4] = {0};		// {RL, FL, FR, RR}
volatile int16_t currCount[4]	 = {0};		// {RL, FL, FR, RR}
volatile int16_t pastCount[4] 	 = {0};		// {RL, FL, FR, RR}
volatile bool encUpdateFlag 	 = 0;

void nexus_bringup(void);

void nexus_bringup(void){
    HAL_TIM_Base_Start_IT(&htim6);

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

	init_motors();

}

/* --------------------------------------------------------------------------------------
 * END OF STEP X
 * --------------------------------------------------------------------------------------
*/

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();

  osKernelInitialize();

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  osKernelStart();

  while (1)
  {

  }

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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

static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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

}

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

static void MX_TIM3_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
}

static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
}

static void MX_TIM6_Init(void)
{


  TIM_MasterConfigTypeDef sMasterConfig = {0};


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

}

static void MX_TIM8_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
}

static void MX_USART2_UART_Init(void)
{


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


}

static void MX_DMA_Init(void)
{

  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();


  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();


  HAL_GPIO_WritePin(GPIOC, FL_INA_GPO_Pin|FL_INB_GPO_Pin|RL_INB_GPO_Pin|RL_INA_GPO_Pin
                          |RR_INB_GPO_Pin|Ultrasonic_DE_RE_Pin|RL_VDD_GPO_Pin|FL_VDD_GPO_Pin, GPIO_PIN_RESET);


  HAL_GPIO_WritePin(GPIOB, RR_INA_GPO_Pin|FR_INB_GPO_Pin, GPIO_PIN_RESET);


  HAL_GPIO_WritePin(GPIOC, RR_VDD_GPO_Pin|FR_VDD_GPO_Pin, GPIO_PIN_SET);


  HAL_GPIO_WritePin(FR_INA_GPO_GPIO_Port, FR_INA_GPO_Pin, GPIO_PIN_RESET);


  GPIO_InitStruct.Pin = FL_INA_GPO_Pin|FL_INB_GPO_Pin|RL_INB_GPO_Pin|RL_INA_GPO_Pin
                          |RR_INB_GPO_Pin|Ultrasonic_DE_RE_Pin|RR_VDD_GPO_Pin|FR_VDD_GPO_Pin
                          |RL_VDD_GPO_Pin|FL_VDD_GPO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RR_INA_GPO_Pin|FR_INB_GPO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FR_INA_GPO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FR_INA_GPO_GPIO_Port, &GPIO_InitStruct);

}


/* ======================================================================================
 * STEP 4 — Timer Callback for Odometry Computation
 * ======================================================================================
 */

/*
 * Called automatically by the rclc_executor at a fixed interval (10 ms in this case).
 * This callback does not compute odometry directly — it only triggers the processing
 * function `compute_and_publish_odometry()` defined in odom_handler.c.
 *
 * Keeps callbacks lightweight — avoids blocking the micro-ROS executor.
 *
 * --------------------------------------------------------------------------------------
 */
void odom_timer_cb(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;      // Prevent unused variable warning

    if (timer == NULL)
        return;                 // Safety check: avoid null pointer crash

    compute_and_publish_odometry();   // Run the actual odometry computation (user code)
}

// --- Callback ---
void twist_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;

    // Extract velocity commands (SI units)
    float vx = msg->linear.x;   // forward/backward [m/s]
    float vy = msg->linear.y;   // lateral [m/s]
    float wz = msg->angular.z;  // rotation [rad/s]

    // Pass values to the motion controller
    /*
    if ((osKernelGetTickCount() - last_cmd_tick) > CMD_TIMEOUT_MS) {
    	Mecanum_Control(0.0f, 0.0f, 0.0f);  // stop robot
	}

     */
    Mecanum_Control(vx, vy, wz);
}
/* ======================================================================================
 * END STEP 4
 * ======================================================================================
 */





/* ======================================================================================
 * STEP 5 — micro-ROS Bring-up and Node Initialization
 * ======================================================================================
 */

/*
 *  This is the main entry point that runs once the RTOS scheduler starts (FreeRTOS task).
 *  It performs the full initialization of the micro-ROS node running on the STM32 board.
 *
 *  Summary of what happens here:
 *   1️-  Hardware bring-up (sensors, drivers, encoders, etc.)
 *   2️-  Configure transport (UART/USB/CAN) for the micro-ROS Agent
 *   3️-  Register custom FreeRTOS-safe allocator functions
 *   4️-  Ping the ROS 2 Agent to ensure communication
 *   5️-  Initialize the micro-ROS support, node, and publishers
 *   6️-  Create the timer and executor loop for periodic callbacks
 *   7️-  Enter the main executor spin loop (runs forever)
 *
 * ======================================================================================
 */

void StartDefaultTask(void *argument)
{
	 /* -------------------------------------------------------------------------
	  * 1️- Hardware bring-up (user-specific)
	  * -------------------------------------------------------------------------
	  */

	/*
	  *  Initialize peripherals, motor controllers, encoders, and sensors.
	  *  This part is specific to the robot platform.
	  */
	  nexus_bringup();  // Custom board-level init
	  for (int i = 0; i < 20; ++i) {
	        osDelay(100); // short delay to let peripherals stabilize
	  }

	/* -------------------------------------------------------------------------
	 * 2️- Configure the micro-ROS transport layer
	 * -------------------------------------------------------------------------
	 */

	/*
	 *  This binds micro-ROS to a communication channel (here UART2).
	 *  The transport functions were defined earlier in transport_cubemx.c.
	 *
	 *    Change &huart2 to &huartX if you use another UART.
	 */
	  rmw_uros_set_custom_transport(
	      true, (void *)&huart2,
	      cubemx_transport_open, cubemx_transport_close,
	      cubemx_transport_write, cubemx_transport_read);

	 /* -------------------------------------------------------------------------
	  * 3- Set up FreeRTOS-safe memory allocator
	  * -------------------------------------------------------------------------
	  */

	  /*
	  *  Replaces default malloc/free with thread-safe versions
	  *  that use FreeRTOS heap functions or user static pool.
	  */
	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate      = microros_allocate;
	  freeRTOS_allocator.deallocate    = microros_deallocate;
	  freeRTOS_allocator.reallocate    = microros_reallocate;
	  freeRTOS_allocator.zero_allocate = microros_zero_allocate;
	  rcl_ret_t rc1 =  rcutils_set_default_allocator(&freeRTOS_allocator);
	  if (rc1 != RCL_RET_OK) {
		  // Handle allocator init error (e.g. log or blink LED)
	  }

	 /* -------------------------------------------------------------------------
	  * 4️- Ping the ROS 2 Agent to ensure connectivity
	  * -------------------------------------------------------------------------
	  */

	 /*
	  *  micro-ROS communicates over XRCE-DDS; the Agent must be running
	  *  on the PC (typically `ros2 run micro_ros_agent micro_ros_agent serial ...`)
	  *  before this ping succeeds.
	  */
	  for (int i = 0; i < 50; ++i) {
	    if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) break;
	    osDelay(100);
	  }

	 /* -------------------------------------------------------------------------
	  * 5️- Create support context and nodes
	  * -------------------------------------------------------------------------
	  */

	  /*
	  *  These are the core micro-ROS entities for the MCU.
	  */
	  allocator = rcl_get_default_allocator();			 // Retrieve active allocator
	  rclc_support_init(&support, 0, NULL, &allocator);	 // Init context with default args

	  rcl_node_t node_base_controller;					 // This MCU node		  rcl_node_t * node,

	  rclc_node_init_default(
			  &node_base_controller,		 						//  param1 => Node
			  "base_controller",									//  param2 => Name
			  "",													//  param3 => Namespace
			  &support);											//  param4 => support to init the node

	  (void) rmw_uros_sync_session(1000);							// Synchronize session timing

	 /* -------------------------------------------------------------------------
	  * 6️- Initialise the Publishers and Subscribers
	  * -------------------------------------------------------------------------
	  */

	 /*
	  *  Publishes to the `/odom` topic using the standard nav_msgs/Odometry type.
	  */
	  rcl_ret_t rc7 = rclc_publisher_init_default(
	      &odom_pub,												// param1 => publisher
	      &node_base_controller,									// param2 => node
	      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), 	// param3 => type_support	&& ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSubfolder, MsgName)
	      "odom");													// param4 => topic_name

	  if (rc7 != RCL_RET_OK) {
	     // Handle publisher init error
	  }

	  /*
	   * The message header and frame ID are prepared once and reused.
	   */
	  rosidl_runtime_c__String__init(&odom_msg.header.frame_id);
	  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
	  odom_msg.header.stamp.sec = 0;
	  odom_msg.header.stamp.nanosec = 0;

	  //rcl_ret_t rc8 = rclc_subscribtion_init_defaul();
	  rcl_ret_t rc8 = rclc_subscription_init_default(
	      &twist_sub,													// param1 => subscriber
	      &node_base_controller,										// param2 => node
	      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),		// param3 => message type support	&& ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSubfolder, MsgName)
	      "/twist_nexus");												// param4 => topic_name

	  if (rc8 != RCL_RET_OK) {
	  	 // Handle publisher init error
	  }

	 /* -------------------------------------------------------------------------
	  * 7️- Initialise the Executor and Timer
	  * -------------------------------------------------------------------------
	  */

	 /*
	  *  The executor manages all callbacks (timers, subscriptions, etc.).
	  *  Each timer is a periodic callback.
	  */
	  rclc_executor_init(&executor, &support.context, 4, &allocator);

	  rcl_timer_t timer_odom;

	  // Here, we set up odometry @ 100 Hz.
	  rclc_timer_init_default2(
			  &timer_odom,				// create timer = timer_odom
			  &support,					// Ensures entities share the same ROS context, clock, and memory and Holds micro-ROS runtime context and allocator
			  RCL_MS_TO_NS(10),			// callback period = 10 ms
			  odom_timer_cb,			// callback function
			  true);	 				// autostart = true

	  rclc_executor_add_timer(&executor, &timer_odom);

	  rclc_executor_add_subscription(&executor, &twist_sub, &twist_msg, &twist_callback, ON_NEW_DATA);

	 /* -------------------------------------------------------------------------
	  * 8️- Executor main loop (runs forever)
	  * -------------------------------------------------------------------------
	  */

	 /*
	  *  Non-blocking spin_some() allows other RTOS tasks to run concurrently.
	  *  You can replace with rclc_executor_spin() for blocking loop.
	  */
	  for (;;) {
	    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));		// check callbacks every 5 ms
	    //osDelay(5);												// optional if CPU load is high
	  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM6) {
		currCount[0] = __HAL_TIM_GET_COUNTER(&htim4);
		currCount[1] = __HAL_TIM_GET_COUNTER(&htim1);
		currCount[2] = __HAL_TIM_GET_COUNTER(&htim3);
		currCount[3] = __HAL_TIM_GET_COUNTER(&htim8);

		for (int i=0; i<4; i++) {

		  deltaEncoder[i] = (int16_t)(currCount[i] - pastCount[i]);

		  if (deltaEncoder[i] > 32767)       deltaEncoder[i] -= 65536;
		  else if (deltaEncoder[i] < -32768) deltaEncoder[i] += 65536;

		  pastCount[i] = currCount[i];

	  }
		encUpdateFlag = 1;
	}
	else if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }

}


void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif
