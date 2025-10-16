#pragma once
#include <rcl/rcl.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Call once after you create your node
bool debug_init(rcl_node_t* node);

// Publish c[4] (timer counts), d[4] (signed tick deltas), w[4] (rad/s)
void debug_pub_cd_w(const uint16_t c[4], const int16_t d[4], const float w[4]);

#ifdef __cplusplus
}
#endif
