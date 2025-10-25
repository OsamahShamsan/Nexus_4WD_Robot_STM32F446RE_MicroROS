/*
#include "bringup.h"
#include <string.h>
#include "cmsis_os.h"

// ===== Definitions (actual storage lives here) =====
rcl_allocator_t allocator;
rclc_support_t  support;
rcl_node_t      node;
rclc_executor_t executor;

rcl_node_t node_base_controller;

rcl_publisher_t pub_imu;
rcl_publisher_t pub_temp;
// rcl_publisher_t odom_pub;
// rcl_subscription_t twist_sub;

rcl_timer_t timer_imu;

sensor_msgs__msg__Imu         msg_imu;
sensor_msgs__msg__Temperature msg_temp;
// nav_msgs__msg__Odometry      odom_msg;
// geometry_msgs__msg__Twist    twist_msg;

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

// ================= Implementations =================

void Bringup_TransportInit(UART_HandleTypeDef *uart)
{
    rmw_uros_set_custom_transport(
        true, (void*)uart,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);
}

void Bringup_AllocatorInit(void)
{
    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate      = microros_allocate;
    freeRTOS_allocator.deallocate    = microros_deallocate;
    freeRTOS_allocator.reallocate    = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;
    CHECK_RCL(rcutils_set_default_allocator(&freeRTOS_allocator));
}

void Bringup_PingAgent(uint32_t attempts, uint32_t period_ms)
{
    for (uint32_t i = 0; i < attempts; ++i) {
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) break;
        osDelay(period_ms);
    }
}

void Bringup_CoreInit(void)
{
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node_base_controller, "base_controller", "", &support);
    (void) rmw_uros_sync_session(1000);
}

void Bringup_TopicsInit(void)
{
    // IMU publisher
    rclc_publisher_init_default(
        &pub_imu,
        &node_base_controller,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw");

    // Temperature publisher
    rclc_publisher_init_default(
        &pub_temp,
        &node_base_controller,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
        "imu/temperature");

    // Optional: set static message fields
    // If you want to set frame_id once using rosidl_runtime_c__String, do it here:
    // rosidl_runtime_c__String__assign(&msg_imu.header.frame_id, "imu_link");
}

void Bringup_ExecutorInit(uint32_t executor_capacity)
{
    rclc_executor_init(&executor, &support.context, executor_capacity, &allocator);

    // If you are using timers/subscribers, add them here.
    // Example (enable only if you implement timer_imu_cb):
    // extern void timer_imu_cb(rcl_timer_t * timer, int64_t last_call_time);
    // rclc_timer_init_default2(&timer_imu, &support, RCL_MS_TO_NS(10), timer_imu_cb, true);
    // rclc_executor_add_timer(&executor, &timer_imu);
}
*/
