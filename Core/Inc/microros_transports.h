#ifndef MICROROS_TRANSPORTS_H_
#define MICROROS_TRANSPORTS_H_

#include <uxr/client/profile/transport/custom/custom_transport.h>

#ifdef __cplusplus
extern "C" {
#endif

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport,
                              const uint8_t* buf, size_t len,
                              uint8_t* err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport,
                             uint8_t* buf, size_t len,
                             int timeout,
                             uint8_t* err);

#ifdef __cplusplus
}
#endif

#endif  // MICROROS_TRANSPORTS_H_
