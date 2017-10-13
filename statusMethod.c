#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "ProjectStructs.h"
#include "TCB.h"
#include "display.h"
#include "GlobalCounter.h"

void StatusMethod(void * voidStatusPtr) {
  if(globalCounter%5 == 0){
    Status * statusPtr = voidStatusPtr;
    *(statusPtr->batteryState) -= 1;
    if (*(statusPtr->batteryState) == 0) {
      *(statusPtr->batteryState) = 200;      
    }
  }
  
}