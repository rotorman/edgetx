/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "opentx.h"
#include "mixer_scheduler.h"

#if !defined(SIMU)

// Global trigger flag

// Mixer schedule
struct MixerSchedule {

  // period in us
  volatile uint16_t period;
};

static MixerSchedule mixerSchedules[NUM_MODULES];

uint16_t getMixerSchedulerPeriod()
{
#if defined(HARDWARE_INTERNAL_MODULE)
  if (mixerSchedules[INTERNAL_MODULE].period) {
    return mixerSchedules[INTERNAL_MODULE].period;
  }
#endif
#if defined(HARDWARE_EXTERNAL_MODULE)
  if (mixerSchedules[EXTERNAL_MODULE].period) {
    return mixerSchedules[EXTERNAL_MODULE].period;
  }
#endif
  return MIXER_SCHEDULER_DEFAULT_PERIOD_US;
}

void mixerSchedulerInit()
{
  memset(mixerSchedules, 0, sizeof(mixerSchedules));
}

void mixerSchedulerSetPeriod(uint8_t moduleIdx, uint16_t periodUs)
{
  if (periodUs > 0 && periodUs < MIN_REFRESH_RATE) {
    periodUs = MIN_REFRESH_RATE;
  }
  else if (periodUs > 0 && periodUs > MAX_REFRESH_RATE) {
    periodUs = MAX_REFRESH_RATE;
  }

  mixerSchedules[moduleIdx].period = periodUs;
}

bool mixerSchedulerWaitForTrigger(uint8_t timeoutMs)
{
  uint32_t ulNotificationValue;
  const TickType_t xMaxBlockTime = pdMS_TO_TICKS( timeoutMs );

  /* Wait to be notified that the transmission is complete.  Note
     the first parameter is pdTRUE, which has the effect of clearing
     the task's notification value back to 0, making the notification
     value act like a binary (rather than a counting) semaphore.  */
  ulNotificationValue = ulTaskNotifyTakeIndexed( 1 /* task notification index */,
                                                 pdTRUE,
                                                 xMaxBlockTime );

  if( ulNotificationValue == 1 ) {
    /* The transmission ended as expected. */
    return false;

  } else {
    /* The call to ulTaskNotifyTake() timed out. */
    return true;
  }
}

void mixerSchedulerISRTrigger()
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  /* At this point xTaskToNotify should not be NULL as
     a transmission was in progress. */
  configASSERT( mixerTaskId.rtos_handle != NULL );

  /* Notify the task that the transmission is complete. */
  vTaskNotifyGiveIndexedFromISR( mixerTaskId.rtos_handle,
                                 1 /* task notification index */,
                                 &xHigherPriorityTaskWoken );

  /* If xHigherPriorityTaskWoken is now set to pdTRUE then a
     context switch should be performed to ensure the interrupt
     returns directly to the highest priority task.  The macro used
     for this purpose is dependent on the port in use and may be
     called portEND_SWITCHING_ISR(). */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

#endif
