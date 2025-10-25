#ifndef BRINGUP_H_
#define BRINGUP_H_

/*
#define CHECK_RCL(x) do { \
    rcl_ret_t _rc = (x);  \
    if (_rc != RCL_RET_OK) { \
        // optional error handling or logging / \
    } \
} while(0)

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/temperature.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

// HAL UART you use for transport (change include if your UART header differs)
#include "usart.h"
#include "stm32f4xx_hal.h"

// ===== Global micro-ROS objects (extern only here) =====
extern rcl_allocator_t allocator;
extern rclc_support_t  support;
extern rcl_node_t      node;          // name it 'node' if you like; your code used 'node'
extern rclc_executor_t executor;

// Publishers (add/remove as you need)
extern rcl_publisher_t pub_imu;
extern rcl_publisher_t pub_temp;
// extern rcl_publisher_t odom_pub;

// Subscribers (optional)
// extern rcl_subscription_t twist_sub;

// Timers (optional if you use executor timers)
extern rcl_timer_t timer_imu;

// Message storage (optional to keep around)
extern sensor_msgs__msg__Imu         msg_imu;
extern sensor_msgs__msg__Temperature msg_temp;
// extern nav_msgs__msg__Odometry       odom_msg;
// extern geometry_msgs__msg__Twist     twist_msg;

// ===== Bringup API =====
void Bringup_TransportInit(UART_HandleTypeDef *uart);
void Bringup_AllocatorInit(void);
void Bringup_PingAgent(uint32_t attempts, uint32_t period_ms);
void Bringup_CoreInit(void);
void Bringup_TopicsInit(void);
void Bringup_ExecutorInit(uint32_t executor_capacity);
*/
#endif /* BRINGUP_H_ */
