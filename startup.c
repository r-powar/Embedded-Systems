#include <stdio.h>
#include <stdbool.h>
#include "inc/lm3s8962.h"
#include "ProjectStructs.h"
#include "TCB.h"
#include "warning.h"
#include "schedule.h"

void Initialize(){
  volatile unsigned long ulLoop;

  //
  // Enable the GPIO port that is used for the on-board LED.
  //
  SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

  //
  // Do a dummy read to insert a few cycles after enabling the peripheral.
  //
  ulLoop = SYSCTL_RCGC2_R;

  //
  // Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
  // enable the GPIO pin for digital function.
  //
  GPIO_PORTF_DIR_R = 0x01;
  GPIO_PORTF_DEN_R = 0x01;
  
  RIT128x96x4Init(1000000);
  
  unsigned int * tempratureRawBuf[8] = {75, 75, 75, 75, 75, 75, 75, 75, 75};
  unsigned int * bloodPressRawBuf[16] = {80, 80, 80, 80, 80, 80, 80, 80, 80,80, 80, 80, 80, 80, 80, 80, 80, 80};
  unsigned int * pulseRateRawBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

  unsigned char * tempCorrectedBuf[8] = {};
  unsigned char * bloodPressCorrectedBuf[16] = {};
  unsigned char * pulseRateRawBuf[8] = {};
  
  unsigned short * mode = 0;
  unsigned short * measurementSelection = 0;
  unsigned short * scroll = 0;
  unsigned short * select = 0;
  unsigned short * alarmAcknowledge = 0;
  unsigned short * batteryState = 200;
  
  unsigned char * bpOutOfRange = 0;
  unsigned char * tempOutOfRange = 0;
  unsigned char * pulseOutOfRange = 0;
  
  bool bpHigh = false;
  bool tempHigh = false;
  bool pulseHigh = false;
  
  
    //create the TCB for Measure
  MeasureData * measureDataPtr;
  measureDataPtr = (struct MeasureData *) malloc(sizeof(struct MeasureData));
  //TODO: assign measure data locals to point to the values declared at the top
  measureDataPtr->temperatureRawBuf = temperatureRawBuf;
  measureDataPtr->systolicPressRaw = systolicPressRaw;
  measureDataPtr->diastolicPressRaw = diastolicPressRaw;
  measureDataPtr->pulseRateRawBuf = pulseRateRaw;
    
  //Make a void pointer to datastruct for measure
  void * voidMeasureDataPtr = measureDataPtr;
  //instantiate Task Control Block for Measure
  TCB * TCBMeasure;
  TCBMeasure = (TCB *) malloc(sizeof(TCB));
  //Create the pointer to the Measure method
  void (* measurePtr)(void*);
  //assign pointer to function Measure
  measurePtr = Measure;
  //fill the TCB with revelvent method and data pointers
  TCBMeasure->myTask = measurePtr;
  TCBMeasure->taskDataPtr = voidMeasureDataPtr;

  
}

