/*
#include "microros_transports.h"
#include "usart.h"
#include "cmsis_os.h"

extern UART_HandleTypeDef huart2;

bool cubemx_transport_open(struct uxrCustomTransport * transport)
{
    (void) transport;
    return true; // nothing to init, HAL already did
}

bool cubemx_transport_close(struct uxrCustomTransport * transport)
{
    (void) transport;
    return true; // nothing special to close
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport,
                              const uint8_t* buf, size_t len,
                              uint8_t* err)
{
    (void) transport;
    if (HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, HAL_MAX_DELAY) == HAL_OK) {
        return len;
    } else {
        if (err) *err = 1;
        return 0;
    }
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport,
                             uint8_t* buf, size_t len,
                             int timeout,
                             uint8_t* err)
{
    (void) transport;
    if (HAL_UART_Receive(&huart2, buf, len, timeout) == HAL_OK) {
        return len;
    } else {
        if (err) *err = 1;
        return 0;
    }
}
*/
