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

void computeinsertAtFront (int value, unsigned int * buff) {
  for (int i = 7; i >0; i--){
    *(buff + i) = *(buff + (i-1));
  }
  *buff = value;
}

void Compute(void * voidComputeDataPtr) {
  // Maddie: changed to voidComputeDataPtr
  ComputeData * computeDataPtr = voidComputeDataPtr;
  //perform conversions and corrections
  convertToCelsius(computeDataPtr, computeDataPtr->temperatureRawBuf);
  diasPressConversion(computeDataPtr, computeDataPtr->bloodPressRawBuf);
  sysPressConversion(computeDataPtr, computeDataPtr->bloodPressRawBuf+8);
  pulseRateConversion(computeDataPtr, computeDataPtr->pulseRateRawBuf);  
  
}

void sysPressConversion(ComputeData *compute, unsigned int *sysRaw)
{
  int temp = SYSTOLIC_BASE + (SYSTOLIC_RATIO * *sysRaw);
  //*compute->bloodPressCorrectedBuf = temp;
  computeinsertAtFront(temp, compute->bloodPressCorrectedBuf);
}

void diasPressConversion(ComputeData *compute, unsigned int *diasRaw)
{
  int temp = DIASTOLIC_BASE + (DIASTOLIC_RATIO * *diasRaw);
  //*(compute->bloodPressCorrectedBuf+8) = temp;
  computeinsertAtFront(temp,compute->bloodPressCorrectedBuf+8);
}

void pulseRateConversion(ComputeData *compute, unsigned int *pulseRaw)
{
  int temp = PULSE_RATE_BASE + (PULSE_RATE_RATIO * *pulseRaw);
  //*(compute->prCorrectedBuf) = temp;
  computeinsertAtFront(temp,compute->prCorrectedBuf);
}

void convertToCelsius(ComputeData *compute, unsigned int *tempRaw)
{
  int temp = (CELSIUS_BASE + (CELSIUS_RATIO * *tempRaw));
  computeinsertAtFront(temp, compute->tempCorrectedBuf);
  //*(compute->tempCorrectedBuf) = temp;
}

