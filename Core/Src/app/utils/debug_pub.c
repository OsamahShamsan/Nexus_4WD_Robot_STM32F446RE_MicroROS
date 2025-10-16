#include "debug_pub.h"
#include <std_msgs/msg/float32_multi_array.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

static rcl_publisher_t dbg_pub;
static std_msgs__msg__Float32MultiArray dbg_msg;
static bool dbg_inited = false;

bool debug_init(rcl_node_t* node)
{
    if (rclc_publisher_init_default(
            &dbg_pub,
            node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/debug") != RCL_RET_OK) {
        return false;
    }

    std_msgs__msg__Float32MultiArray__init(&dbg_msg);
    // Fixed-size payload: 12 floats => [c0..c3, d0..d3, w0..w3]
    rosidl_runtime_c__float__Sequence__init(&dbg_msg.data, 12);
    dbg_msg.data.size = 12;

    dbg_inited = true;
    return true;
}

void debug_pub_cd_w(const uint16_t c[4], const int16_t d[4], const float w[4])
{
    if (!dbg_inited) return;

    float *p = dbg_msg.data.data;

    // c[4] as floats
    p[0] = (float)c[0]; p[1] = (float)c[1]; p[2] = (float)c[2]; p[3] = (float)c[3];
    // d[4] as floats
    p[4] = (float)d[0]; p[5] = (float)d[1]; p[6] = (float)d[2]; p[7] = (float)d[3];
    // w[4]
    p[8] = w[0]; p[9] = w[1]; p[10] = w[2]; p[11] = w[3];

    (void)rcl_publish(&dbg_pub, &dbg_msg, NULL);
}
