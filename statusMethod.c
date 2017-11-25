#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "ProjectStructs.h"
#include "TCB.h"
#include "display.h"
#include "GlobalCounter.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

void StatusMethod(void * voidStatusPtr) {
  TickType_t  xLastWakeTime;
  const TickType_t xFrequency = 4000;
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
  vTaskDelayUntil(&xLastWakeTime, xFrequency);
    Status * statusPtr = voidStatusPtr;
    //lower the battery power
    *(statusPtr->batteryState) -= 1;
    //if the battery runs out, set it to full.
    if (*(statusPtr->batteryState) == 0) {
      *(statusPtr->batteryState) = 200;      
    }
   vTaskDelay(1000);
  }
}