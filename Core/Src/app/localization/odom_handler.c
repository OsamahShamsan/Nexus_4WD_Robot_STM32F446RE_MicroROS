
#include "odom_handler.h"
#include "robot_params.h"

#include <math.h>
#include <stdint.h>

#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <nav_msgs/msg/odometry.h>

rcl_publisher_t odom_pub;         // Publisher object → publishes /odom topic
nav_msgs__msg__Odometry odom_msg; // Struct holding odometry data to send back to ROS2

extern volatile int16_t deltaEncoder[4];		// {RL, FL, FR, RR}
extern volatile bool encUpdateFlag;

float x = 0.0f, y = 0.0f, theta = 0.0f;

void compute_and_publish_odometry(void)
{
    if (!encUpdateFlag) return;
    encUpdateFlag = 0;

    int16_t localDelta[NUM_WHEELS];
    for (int i = 0; i < NUM_WHEELS; i++)
        localDelta[i] = deltaEncoder[i];

    const float DT = 0.001f;
    float w[NUM_WHEELS];
    for (int i = 0; i < NUM_WHEELS; i++)
        w[i] = localDelta[i] * RAD_PER_TICK / DT;

    float vx = (WHEEL_R / 4.0f) * (w[1] + w[2] + w[0] + w[3]);
    float vy = (WHEEL_R / 4.0f) * (-w[1] + w[2] + w[0] - w[3]);
    float wz = (WHEEL_R / 4.0f) * ((-w[1] + w[2] - w[0] + w[3]) / A_SUM);

    static float acc_vx = 0, acc_vy = 0, acc_wz = 0;
    static uint8_t count = 0;

    acc_vx += vx;
    acc_vy += vy;
    acc_wz += wz;
    count++;

    if (count >= 10)  {

        vx = acc_vx / count;
        vy = acc_vy / count;
        wz = acc_wz / count;
        acc_vx = acc_vy = acc_wz = 0;
        count = 0;

        float dt_publish = 0.010f;
        float dx = (vx * cosf(theta) - vy * sinf(theta)) * dt_publish;
        float dy = (vx * sinf(theta) + vy * cosf(theta)) * dt_publish;
        float dtheta = wz * dt_publish;

        x += dx;
        y += dy;
        theta += dtheta;

        if (theta > M_PI)  theta -= 2.0f * M_PI;
        if (theta < -M_PI) theta += 2.0f * M_PI;

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.orientation.z = sinf(theta / 2.0f);
        odom_msg.pose.pose.orientation.w = cosf(theta / 2.0f);

        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = wz;

        uint64_t now_ns = rmw_uros_epoch_nanos();
        odom_msg.header.stamp.sec = (int32_t)(now_ns / 1000000000ULL);
        odom_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000ULL);

        rcl_ret_t rc1 = rcl_publish(&odom_pub, &odom_msg, NULL);
        if (rc1 != RCL_RET_OK){	}
    }
}



