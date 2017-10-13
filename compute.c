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
  // Maddie: changed to voidComputeDataPtr
  
  if(globalCounter%5 == 0){
    ComputeData * computeDataPtr = voidComputeDataPtr;
    //perform conversions and corrections
    convertToCelsius(computeDataPtr, computeDataPtr->temperatureRaw);
    diasPressConversion(computeDataPtr, computeDataPtr->diastolicPressRaw);
    sysPressConversion(computeDataPtr, computeDataPtr->systolicPressRaw);
    pulseRateConversion(computeDataPtr, computeDataPtr->pulseRateRaw);  
  }
  
}

void sysPressConversion(ComputeData *compute, int *sysRaw)
{
  int temp = SYSTOLIC_BASE + (SYSTOLIC_RATIO * *sysRaw);
  *compute->sysPressCorrected = temp;
}

void diasPressConversion(ComputeData *compute, int *diasRaw)
{
  int temp = DIASTOLIC_BASE + (DIASTOLIC_RATIO * *diasRaw);
  *(compute->diasPressCorrected) = temp;
}

void pulseRateConversion(ComputeData *compute, int *pulseRaw)
{
  int temp = PULSE_RATE_BASE + (PULSE_RATE_RATIO * *pulseRaw);
  *(compute->prCorrected) = temp;
}

void convertToCelsius(ComputeData *compute, int *tempRaw)
{
  //TODO: This needs to be made into a double or float
  int temp = (CELSIUS_BASE + (CELSIUS_RATIO * *tempRaw));
  *(compute->tempCorrected) = temp;
}