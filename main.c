#include <stdio.h>
#include <stdbool.h>
#include "ProjectStructs.h"
#include "BoolInclude.h"
#include "TCB.h"
#include "inc/lm3s8962.h"
#include "GlobalCounter.h"

//declare the method that will initalize the variable to default values
void initializeVariable(int * tempRaw, int *  sysRaw, int *  diaRaw,
                        int *  prRaw, short int *  battState);

//given delay function
void delay(unsigned long aValue);

struct DataStruct{
  int * testData;
};

//declare Schedule Method
void Schedule(void * voidSchedulerDataPtr);

typedef struct DataStruct DataStruct;
void main (void)
{
  
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
  
  RIT128x96x4StringDraw("Hello", 0, 15, 15);

 
  //initialize defaults
  unsigned int tempDefault = 75;
  unsigned int sysDefault = 80;
  unsigned int diaDefault = 80;
  unsigned int prDefault = 50;
  short int battDefault = 200;
  
 
  //declare the variables that will be used for the tracking in the device
  unsigned int temperatureRaw[8];
  temperatureRaw[0] = 75;
  unsigned int bloodPressRaw[16];
  bloodPressRaw[0] = sysDefault;
  bloodPressRaw[8] = diaDefault;
  unsigned int pulseRateRaw[8];
  pulseRateRaw[0] = prDefault;
  unsigned int tempCorrected[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  unsigned int bloodPressCorrected[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  unsigned int prCorrected[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  short int * batteryState = & battDefault;
  
  
  //create the TCB for Measure
  MeasureData * measureDataPtr;
  measureDataPtr = (struct MeasureData *) malloc(sizeof(struct MeasureData));
  //TODO: assign measure data locals to point to the values declared at the top
  measureDataPtr->temperatureRawBuf = temperatureRaw;
  measureDataPtr->bloodPressRawBuf = bloodPressRaw;
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
  
  //set measure to the front of the linked list
  TCB * head = TCBMeasure;
  TCB * curr = head;
  curr->prev = NULL;
  RIT128x96x4StringDraw("Hello2", 0, 15, 15);
 
  //create the TCB for Compute
  ComputeData * computeDataPtr;
  computeDataPtr = (struct ComputeData *) malloc(sizeof(struct ComputeData));
  
  //assign compute data locals to point to the values declared at the top
  computeDataPtr->temperatureRawBuf = temperatureRaw;
  computeDataPtr->bloodPressRawBuf = bloodPressRaw;
  computeDataPtr->pulseRateRawBuf = pulseRateRaw;
  
  computeDataPtr->tempCorrectedBuf = tempCorrected;
  computeDataPtr->bloodPressCorrectedBuf = bloodPressCorrected;
  computeDataPtr->prCorrectedBuf = prCorrected;
  RIT128x96x4StringDraw("Hello3", 0, 15, 15);

  /*
  //Make a void pointer to datastruct for Compute
  void * voidComputeDataPtr = computeDataPtr;
  //instantiate Task Control Block for Compute
  TCB * TCBCompute;
  TCBCompute = (TCB *) malloc(sizeof(TCB));
  //Create the pointer to the Compute method
  void (* computePtr)(void*);
  //assign pointer to function Compute
  computePtr = Compute;
 //fill the TCB with revelvent method and data pointers
  TCBCompute->myTask = computePtr;
  TCBCompute->taskDataPtr = voidComputeDataPtr;
  
  TCBCompute->prev = curr;
  curr->next = TCBCompute;
  curr = curr->next;
  RIT128x96x4StringDraw("Hello3", 0, 15, 15);


  //create the TCB for Display
  DisplayData * displayDataPtr;
  displayDataPtr = (struct DisplayData *) malloc(sizeof(struct DisplayData));
  //TODO: assign measure data locals to point to the values declared at the top
  displayDataPtr->tempCorrectedBuf = tempCorrected;
  displayDataPtr->bloodPressCorrectedBuf = bloodPressCorrected;
  displayDataPtr->prCorrectedBuf = prCorrected;
  displayDataPtr->batteryState = batteryState;
  //Make a void pointer to datastruct for display
  void * voidDisplayDataPtr = displayDataPtr;
  //instantiate Task Control Block for display
  TCB * TCBDisplay;
  TCBDisplay = (TCB *) malloc(sizeof(TCB));
  //Create the pointer to the display method
  void (* displayPtr)(void*);
  //assign pointer to function Display
  displayPtr = Display;
  //fill the TCB with revelvent method and data pointers
  TCBDisplay->myTask = displayPtr;
  TCBDisplay->taskDataPtr = voidDisplayDataPtr;
  
  TCBDisplay->prev = curr;
  curr->next = TCBDisplay;
  curr = curr->next;
  
  //create the TCB for WarningAlarm
  WarningAlarmData * warningAlarmDataPtr;
  warningAlarmDataPtr = (struct WarningAlarmData *) 
    malloc(sizeof(struct WarningAlarmData));
  //TODO: assign measure data locals to point to the values declared at the top
  warningAlarmDataPtr->temperatureRawBuf = temperatureRaw;
  warningAlarmDataPtr->bloodPressRawBuf = bloodPressRaw;
  warningAlarmDataPtr->pulseRateRawBuf = pulseRateRaw;
  warningAlarmDataPtr->batteryState = batteryState;
  //Make a void pointer to datastruct for WarningAlarm
  void * voidWarningAlarmDataPtr = warningAlarmDataPtr;
  //instantiate Task Control Block for WarningAlarm
  TCB * TCBWarningAlarm;
  TCBWarningAlarm = (TCB *) malloc(sizeof(TCB));
  //Create the pointer to the WarningAlarm method
  void (* warningAlarmPtr)(void*);
  //assign pointer to function WarningAlarm
  warningAlarmPtr = WarningAlarm;
  //fill the TCB with revelvent method and data pointers
  TCBWarningAlarm->myTask = warningAlarmPtr;
  TCBWarningAlarm->taskDataPtr = voidWarningAlarmDataPtr;
  
  TCBWarningAlarm->prev = curr;
  curr->next = TCBWarningAlarm;
  curr = curr->next;
  
  //create the TCB for StatusMethod
  Status * statusPtr;
  statusPtr = (struct Status *) malloc(sizeof(struct Status));
  //TODO: assign measure data locals to point to the values declared at the top
  statusPtr->batteryState = batteryState;
  //Make a void pointer to datastruct for StatusMethod
  void * voidStatusPtr = statusPtr;
  //instantiate Task Control Block for StatusMethod
  TCB * TCBStatusMethod;
  TCBStatusMethod = (TCB *) malloc(sizeof(TCB));
  //Create the pointer to the StatusMethod method
  void (* statusMethodPtr)(void*);
  //assign pointer to function StatusMethod
  statusMethodPtr = StatusMethod;
  //fill the TCB with revelvent method and data pointers
  TCBStatusMethod->myTask = statusMethodPtr;
  TCBStatusMethod->taskDataPtr = voidStatusPtr;
  
  TCBStatusMethod->prev = curr;
  curr->next = TCBStatusMethod;
  curr = curr->next;
  
  //create the TCB for Scheduler

  //assign Scheduler data locals to point to the values declared at the top
  //Make a void pointer to datastruct for scheduler
  void * voidSchedulerPtr;
  //instantiate Task Control Block for scheduler
  TCB * TCBScheduler;
  TCBScheduler = (TCB *) malloc(sizeof(TCB));
  //Create the pointer to the Schedule method
  void (* schedulerPtr)(void*);
  //assign pointer to function Schedule
  schedulerPtr = Schedule;
  //fill the TCB with revelvent method and data pointers
  TCBScheduler->myTask = schedulerPtr;
  TCBScheduler->taskDataPtr = voidSchedulerPtr;
  
  curr->next = NULL;
  curr = head;
  while (1){
    while (curr != NULL) {   
      curr->myTask(curr->taskDataPtr);
      curr=curr->next;
    }
    TCBScheduler->myTask(TCBScheduler->taskDataPtr);
    curr = head;
  }
  */
}
void Schedule(void * voidSchedulerDataPtr) {
  //SchedulerData * schedulerDataPtr = schedulerDataPtr;
  delay(10);
  globalCounter++;
}

//void initializeVariable(int * tempRaw, int *  sysRaw, int *  diaRaw,
//                        int *  prRaw, short int *  battState) {
//  int tempDefault = 75;
//  int sysDefault = 80;
//  int diaDefault = 80;
//  int prDefault = 50;
//  short int battDefault = 200;
//  tempRaw = &tempDefault;
//  sysRaw = &sysDefault;
//  diaRaw = &diaDefault;
//  prRaw = &prDefault;
//  battState = &battDefault;
//}
void delay(unsigned long aValue){
  volatile unsigned long i = 0;

  volatile unsigned int j = 0;
  for (i = aValue; i > 0; i--){
    for (j = 0; j < 100000; j++);
  }
  return;
}