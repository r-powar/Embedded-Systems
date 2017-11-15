#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "ProjectStructs.h"
#include "TCB.h"
#include "display.h"
#include "GlobalCounter.h"

void StatusMethod(void * voidStatusPtr) {
  
  while (1) {
    //check global counter to see if it should run
    if(globalCounter%5 == 0){
      Status * statusPtr = voidStatusPtr;
      //lower the battery power
      *(statusPtr->batteryState) -= 1;
      //if the battery runs out, set it to full.
      if (*(statusPtr->batteryState) == 0) {
        *(statusPtr->batteryState) = 200;      
      }
    }
    vTaskDelay(1000);
  }
}