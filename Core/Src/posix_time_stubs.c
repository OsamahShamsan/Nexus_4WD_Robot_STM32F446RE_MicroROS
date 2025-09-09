/*
 * posix_time_stubs.c
 *
 *  Created on: Sep 9, 2025
 *      Author: osamah shamsan
 */

// posix_time_stubs.c
#include <time.h>
#include <sys/time.h>
#include <errno.h>
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

// Newlib-nano may not define TIME_UTC; use 1 as per C11
#ifndef TIME_UTC
#define TIME_UTC 1
#endif

static uint32_t millis_now(void)
{
  // Use FreeRTOS ticks after scheduler starts; HAL_GetTick() before that
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
  } else {
    return HAL_GetTick();
  }
}

int _gettimeofday(struct timeval *tv, void *tzvp)
{
  (void)tzvp;
  if (!tv) { errno = EINVAL; return -1; }
  uint32_t ms = millis_now();
  tv->tv_sec  = ms / 1000U;
  tv->tv_usec = (ms % 1000U) * 1000U;
  return 0;
}


int timespec_get(struct timespec *ts, int base)
{
  if (base != TIME_UTC || !ts) return 0;
  (void)clock_gettime(0, ts);
  return base;
}
