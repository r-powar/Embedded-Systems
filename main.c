#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "ProjectStructs.h"
#include "TCB.h"
#include "inc/lm3s8962.h"
#include "GlobalCounter.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/sysctl.h"

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
    // Set the clocking to run directly from the crystal.
    //
   SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ);

  
   RIT128x96x4Init(1000000);

   /*
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
  */
  //initialize defaults
  unsigned int tempDefault = 75;
  unsigned int sysDefault = 80;
  unsigned int diaDefault = 80;
  unsigned int prDefault = 50;
  short int battDefault = 200;
  unsigned short int modeDefault = 0;
  unsigned short int measureSelectDefault = 0;
  unsigned short int scrollDefault = 0;
  unsigned short int selectDefault = 0;
  unsigned short int alarmAcknowledgeDefault = 0;
    
  //declare the variables that will be used for the tracking in the device
  unsigned int temperatureRaw [8] = {75, 75, 75, 75, 75, 75, 75, 75};
  
  unsigned int bloodPressRaw [16] = {80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80, 80};
  
  unsigned int pulseRateRaw [8] = {50, 50, 50, 50, 50, 50, 50, 50};
  
  unsigned int tempCorrected [] = {0, 0, 0, 0, 0, 0, 0, 0};
  
  unsigned int bloodPressCorrected [] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} ;
 
  unsigned int prCorrected [] = {0, 0, 0, 0, 0, 0, 0, 0};
  
  short int * batteryState = &battDefault;
  unsigned short int * mode = &modeDefault;
  unsigned short int * measureSelect = &measureSelectDefault;
  unsigned short int * scroll = &scrollDefault;
  unsigned short int * select = &selectDefault;
  unsigned short int * alarmAcknowledge = &alarmAcknowledgeDefault;
  
  //create the TCB for Measure
  MeasureData * measureDataPtr;
  measureDataPtr = (struct MeasureData *) malloc(sizeof(struct MeasureData));
  
  //TODO: assign measure data locals to point to the values declared at the top
  measureDataPtr->temperatureRawBuf = &temperatureRaw[0];
  measureDataPtr->bloodPressRawBuf = &bloodPressRaw[0];
  measureDataPtr->pulseRateRawBuf = &pulseRateRaw[0];
    
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
  
  RIT128x96x4StringDraw("Test", 0, 0, 15);
  
  
  //create the TCB for Compute
  ComputeData * computeDataPtr;
  computeDataPtr = (struct ComputeData *) malloc(sizeof(struct ComputeData));
  //assign compute data locals to point to the values declared at the top
  computeDataPtr->temperatureRawBuf = &temperatureRaw[0];
  computeDataPtr->bloodPressRawBuf = &bloodPressRaw[0];
  computeDataPtr->pulseRateRawBuf = &pulseRateRaw[0];
  RIT128x96x4StringDraw("Test2", 0, 0, 15);
  

  computeDataPtr->tempCorrectedBuf = (unsigned int *) malloc(8 * sizeof(unsigned int));
  memcpy(computeDataPtr->tempCorrectedBuf, tempCorrected, 8 * sizeof(unsigned int));
  RIT128x96x4StringDraw("Hello", 0, 0, 15);
 
  /*
  computeDataPtr->tempCorrectedBuf = &tempCorrected[0];
  computeDataPtr->bloodPressCorrectedBuf = &bloodPressCorrected[0];
  computeDataPtr->prCorrectedBuf = &prCorrected[0];
  RIT128x96x4StringDraw("Hello", 0, 0, 15);
 
  
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
  
  
  
  //create the TCB for Keypad
  KeypadData * keypadDataPtr;
  keypadDataPtr = (struct KeypadData *) malloc(sizeof(struct KeypadData));
  keypadDataPtr->mode = mode;
  keypadDataPtr->measurementSelection = measureSelect;
  keypadDataPtr->scroll = scroll;
  keypadDataPtr->select = select;
  keypadDataPtr->alarmAcknowledge = alarmAcknowledge;    
  //Make a void pointer to datastruct for Keypad
  void * voidKeypadDataPtr = keypadDataPtr;
  //instantiate Task Control Block for Keypad
  TCB * TCBKeypad;
  TCBKeypad = (TCB *) malloc(sizeof(TCB));
  //Create the pointer to the Keypad
  void (* keypadPtr)(void*);
  //assign pointer to function Keypad
  keypadPtr = Keypad;
  //fill the TCB with revelvent method and data pointers
  TCBKeypad->myTask = keypadPtr;
  TCBKeypad->taskDataPtr = voidKeypadDataPtr;
  

  TCBKeypad->prev = curr;
  curr->next = TCBKeypad;
  curr = curr->next;
  
  RIT128x96x4StringDraw("Test3", 0, 0, 15);
  
  /*
  //create the TCB for Display
  DisplayData * displayDataPtr;
  displayDataPtr = (struct DisplayData *) malloc(sizeof(struct DisplayData));
  
  //TODO: assign measure data locals to point to the values declared at the top
  displayDataPtr->tempCorrectedBuf = &tempCorrected[0];
  displayDataPtr->bloodPressCorrectedBuf = &bloodPressCorrected[0];
  displayDataPtr->prCorrectedBuf = &prCorrected[0];
  
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
  
  RIT128x96x4StringDraw("Test4", 0, 0, 15);

  //create the TCB for WarningAlarm
  WarningAlarmData * warningAlarmDataPtr;
  warningAlarmDataPtr = (struct WarningAlarmData *) 
    malloc(sizeof(struct WarningAlarmData));
  //TODO: assign measure data locals to point to the values declared at the top
  warningAlarmDataPtr->temperatureRawBuf = &temperatureRaw[0];
  warningAlarmDataPtr->bloodPressRawBuf = &bloodPressRaw[0];
  warningAlarmDataPtr->pulseRateRawBuf = &pulseRateRaw[0];
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
  
  RIT128x96x4StringDraw("Test4", 0, 0, 15);

  //create the TCB for Communications
  CommunicationsData * communicationsDataPtr;
  communicationsDataPtr = (struct CommunicationsData *) malloc(sizeof(struct CommunicationsData));
  
  //TODO: assign measure data locals to point to the values declared at the top
  communicationsDataPtr->tempCorrectedBuf = &tempCorrected[0];
  communicationsDataPtr->bloodPressCorrectedBuf = &bloodPressCorrected[0]; 
  communicationsDataPtr->prCorrectedBuf = &prCorrected[0];
  
  //Make a void pointer to datastruct for Communications
  void * voidCommunicationsDataPtr = communicationsDataPtr;
  //instantiate Task Control Block for Communications
  TCB * TCBCommunications;
  TCBCommunications = (TCB *) malloc(sizeof(TCB));
  //Create the pointer to the Communications method
  void (* communicationsPtr)(void*);
  //assign pointer to function WarningAlarm
  communicationsPtr = Communications;
  //fill the TCB with revelvent method and data pointers
  TCBCommunications->myTask = communicationsPtr;
  TCBCommunications->taskDataPtr = voidCommunicationsDataPtr;
  
  TCBCommunications->prev = curr;
  curr->next = TCBCommunications;
  curr = curr->next;
  
  RIT128x96x4StringDraw("Test5", 0, 0, 15);

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
  
  RIT128x96x4StringDraw("Test6", 0, 0, 15);

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
  
  RIT128x96x4StringDraw("Test7", 0, 0, 15);

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
void Keypad(void * voidKeypadDataPtr) {
  
}
void Communications (void * voidCommunicationsDataPtr) { 
  
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