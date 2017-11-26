#include <stdio.h>
#include <stdbool.h>
#include "ProjectStructs.h"
#include "TCB.h"
#include "compute.h"
#include "GlobalCounter.h"

#define CELSIUS_BASE 5
#define CELSIUS_RATIO 0.75
#define SYSTOLIC_BASE 9
#define SYSTOLIC_RATIO 2
#define DIASTOLIC_BASE 6
#define DIASTOLIC_RATIO 1.5
#define PULSE_RATE_BASE 8
#define PULSE_RATE_RATIO 3

void Compute(void * voidComputeDataPtr) {
  while(1) { 
    // Maddie: changed to voidComputeDataPtr
    ComputeData * computeDataPtr = voidComputeDataPtr;
    //perform conversions and corrections
    convertToCelsius(computeDataPtr, computeDataPtr->temperatureRawBuf);
    diasPressConversion(computeDataPtr, computeDataPtr->bloodPressRawBuf);
    sysPressConversion(computeDataPtr, computeDataPtr->bloodPressRawBuf+8);
    pulseRateConversion(computeDataPtr, computeDataPtr->pulseRateRawBuf);  
    vTaskDelay(1000);
  }
  
}

void sysPressConversion(ComputeData *compute, unsigned int *sysRaw)
{
  int temp = SYSTOLIC_BASE + (SYSTOLIC_RATIO * *sysRaw);
  //*compute->bloodPressCorrectedBuf = temp;
  insertAtFront(temp, compute->bloodPressCorrectedBuf);
  if(updateSys == 1) {
    if ((180 - (15*(10 - (cuff/10))))){
      staticSys = temp;
      updateSys = 0;
    }
  }
}

void diasPressConversion(ComputeData *compute, unsigned int *diasRaw)
{
  int temp = DIASTOLIC_BASE + (DIASTOLIC_RATIO * *diasRaw);
  //*(compute->bloodPressCorrectedBuf+8) = temp;
  insertAtFront(temp,compute->bloodPressCorrectedBuf+8);
  if(updateDias == 1) {
    if ((180 - (15*(10 - (cuff/10)))) <= temp){
      staticDias = temp;
      updateDias = 0;
    }
  }
}

void pulseRateConversion(ComputeData *compute, unsigned int *pulseRaw)
{
  int temp = PULSE_RATE_BASE + (PULSE_RATE_RATIO * *pulseRaw);
  //*(compute->prCorrectedBuf) = temp;
  insertAtFront(temp,compute->prCorrectedBuf);
}

void convertToCelsius(ComputeData *compute, unsigned int *tempRaw)
{
  int temp = (CELSIUS_BASE + (CELSIUS_RATIO * *tempRaw));
  insertAtFront(temp, compute->tempCorrectedBuf);
  //*(compute->tempCorrectedBuf) = temp;
}

