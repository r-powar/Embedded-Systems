#include <stdio.h>
#include <stdbool.h>
#include "ProjectStructs.h"
#include "TCB.h"
#include "measure.h"
#include "GlobalCounter.h"
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

 //global declaration
 static int sysComplete = 0;
 static int diasComplete = 0;
 void insertAtFront (int value, unsigned int * buff);
 //constant variable
 
void Measure(void * voidMeasureDataPtr) {
  TickType_t  xLastWakeTime;
  const TickType_t xFrequency = 4000;
  xLastWakeTime = xTaskGetTickCount();
  MeasureData * measureDataPtr = voidMeasureDataPtr;
  while(1) { 
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    unsigned int * tempRaw = measureDataPtr->temperatureRawBuf;
    unsigned int * sysPRaw = measureDataPtr->bloodPressRawBuf;
    unsigned int * diasPRaw = measureDataPtr->bloodPressRawBuf+8;
    unsigned int * pulseRRaw = measureDataPtr->pulseRateRawBuf;
    unsigned short int * measurementSelection = measureDataPtr-> measurementSelection;
    unsigned short int * addCompute = measureDataPtr->addCompute;
     switch (*measurementSelection){
      case 1:
        sysPressRaw(sysPRaw);
        diasPressRaw(diasPRaw);
        *addCompute = 1;
        break;
      case 2:
        calcTempRaw(tempRaw);
        *addCompute = 1;
        break;
      case 3:
        pulseRateRaw(pulseRRaw);
        *addCompute = 1;
        break;
    }
    vTaskDelay(1000);
  }
  
}

void calcTempRaw(unsigned int *tempRaw){
  static bool isReversed = true;
  static int funcCall = 0;
  
  if(isReversed){
    if(*tempRaw < 15){
      isReversed = !isReversed;
    }
  }else{
    if(*tempRaw > 50){
      isReversed = !isReversed;
    }
  }
  
  if(funcCall % 2 == 0){
    //*tempRaw += isReversed ? -2 : 2;
    insertAtFront(*tempRaw + (isReversed ? -2 : 2), tempRaw);
  }else{
    //*tempRaw += isReversed ? 1 : -1;
    insertAtFront(*tempRaw + (isReversed ? 1 : -1), tempRaw);
  }
  funcCall++;
}

void sysPressRaw(unsigned int * sysRaw){
  static int funcCall = 0;
  if(diasComplete && sysComplete){
    diasComplete = 0;
    sysComplete = 0;
    rightPressed = 1;
    //*sysRaw = 80;
    insertAtFront(80, sysRaw);
  }
  if(*sysRaw > 100){
    //set systolic variable to complete
    sysComplete = 1;
  }else{
    if (!sysComplete){
      if(funcCall % 2 == 0){
        //*sysRaw += 3;
        insertAtFront(*sysRaw + 3, sysRaw);
      }else{
        //*sysRaw--;
        insertAtFront(*sysRaw - 1, sysRaw);
      }   
    }
  }
  funcCall++;
}

void diasPressRaw(unsigned int *diasRaw){
  static int funcCall = 0;
  if(*diasRaw < 40){
    diasComplete = 1;
  }else{
    if (!diasComplete) {
      if (funcCall % 2) {
        //*diasRaw -= 2;
        insertAtFront(*diasRaw - 2, diasRaw);
      }
      else {
        //*diasRaw += 1;
        insertAtFront(*diasRaw + 1, diasRaw);
      }
    }
  }
  if (diasComplete && sysComplete) {
    //*diasRaw = 80; 
    insertAtFront(80, diasRaw);
  }
  funcCall++;
}

void pulseRateRaw(unsigned int *prRaw){
  static bool isReversed = true;
  static int funcCall = 0;
  
  if(isReversed){
    if(*prRaw > 40){
      isReversed = !isReversed;
    }
  }else{
    if(*prRaw < 15){
        isReversed = !isReversed;
    }
  }
  if(funcCall % 2 == 0){
    //*prRaw += isReversed ? -1 : 1;
    insertAtFront(*prRaw + (isReversed ? -1 : 1), prRaw);
  }else{
    //*prRaw += isReversed ? 3 : -3;
    insertAtFront(*prRaw + (isReversed ? 3 : -3), prRaw);
  }  
  funcCall++;
}

void insertAtFront (int value, unsigned int * buff) {
  for (int i = 7; i >0; i--){
    *(buff + i) = *(buff + (i-1));
  }
  *buff = value;
}