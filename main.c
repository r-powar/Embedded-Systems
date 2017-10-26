#include <stdio.h>
#include <stdbool.h>
#include "ProjectStructs.h"
#include "BoolInclude.h"
#include "TCB.h"
#include "inc/lm3s8962.h"
#include "GlobalCounter.h"
#include "inc/lm3s8962.h"
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "drivers/rit128x96x4.h"
#include  <utils/uartstdio.c>
#include "driverlib/pwm.h"
#include "driverlib/timer.h"

//given delay function
void delay(unsigned long aValue);

struct DataStruct{
  int * testData;
};

//declare Schedule Method
void Schedule(void * voidSchedulerDataPtr);

//interrupts

// Flags that contain the current value of the interrupt indicator as displayed
// on the OLED display.
//
//*****************************************************************************
unsigned long g_ulFlags;
unsigned int ulPeriod;

//*****************************************************************************
//
// The interrupt handler for the first timer interrupt.
//
//*****************************************************************************
void
Timer0IntHandler(void)
{
  //
  // Clear the timer interrupt.
  //
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

  //
  // Toggle the flag for the first timer.
  //
  HWREGBITW(&g_ulFlags, 0) ^= 1;

  //
  // Update the interrupt status on the display.
  //
  
  globalCounter++;
  IntMasterDisable();
  //RIT128x96x4StringDraw(HWREGBITW(&g_ulFlags, 0) ? "1" : "0", 48, 32, 15);
  IntMasterEnable();
}

//handle select interrupts
void selectPressedHandler(void){//port F pin 1
  GPIOPinIntClear(GPIO_PORTF_BASE, GPIO_PIN_1);
  selectPressed = 1;
}
//handle direction interrupts
void dirPressedHandler(void){//port E pins 0-3 up down left right
  volatile long Up = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0);
  volatile long Down = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1);
  volatile long Left = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_2);
  volatile long Right = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_3);
  if(Down == 0){
    downPressed = 1;
  }
  else if(Up == 0){
    upPressed = 1;
  }
  else if(Left == 0){
    leftPressed = 1;
  }
  else if(Right ==0) {
    rightPressed = 1;
    PWMGenDisable(PWM_BASE, PWM_GEN_0); // Turn off the speaker
  }
  GPIOPinIntClear(GPIO_PORTE_BASE, GPIO_PIN_3);
  GPIOPinIntClear(GPIO_PORTE_BASE, GPIO_PIN_1);
  GPIOPinIntClear(GPIO_PORTE_BASE, GPIO_PIN_0);
  GPIOPinIntClear(GPIO_PORTE_BASE, GPIO_PIN_2);
}


typedef struct DataStruct DataStruct;
void main (void)
{
  //
  // Set the clocking to run directly from the crystal.
  //
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                 SYSCTL_XTAL_8MHZ);
  SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
  
   //enable the pins for the direction buttons
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
   GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
   GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3, GPIO_STRENGTH_2MA,
    GPIO_PIN_TYPE_STD_WPU);
   GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3, GPIO_FALLING_EDGE);
   GPIOPinIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3);
   IntEnable(INT_GPIOE);
       
   //enable the pins for the select button
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
   GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
   GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA,
                    GPIO_PIN_TYPE_STD_WPU);
   GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_FALLING_EDGE);
   GPIOPinIntEnable(GPIO_PORTF_BASE, GPIO_PIN_1);
   IntEnable(INT_GPIOF);
  //
  // Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
  // enable the GPIO pin for digital function.
  //
   //GPIO_PORTF_DIR_R = 0x01;
   //GPIO_PORTF_DEN_R = 0x01;
   
   //set up the speaker for the alarm
   SysTickPeriodSet(SysCtlClockGet());
   SysTickEnable();
   SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);    // speaker
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
   GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
   GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
   volatile unsigned long ulPeriod = SysCtlClockGet() / 440;
   PWMGenConfigure(PWM_BASE, PWM_GEN_0,
               PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
   PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod);
   PWMPulseWidthSet(PWM_BASE, PWM_OUT_0, ulPeriod / 4);
   PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ulPeriod * 3 / 4);
   
  //
  // Enable the PWM0 and PWM1 output signals.
  //
  PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
    
  //timer interrupt stuff
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  IntMasterEnable();
  TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
  TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet());
  IntEnable(INT_TIMER0A);
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER0_BASE, TIMER_A);
  
  RIT128x96x4Init(1000000);
  
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
  unsigned short int addComputeDefault = 1;
  unsigned short int addCommunicationsDefault = 0;
  //declare the variables that will be used for the tracking in the device
  unsigned int temperatureRaw[8];
  temperatureRaw[0] = tempDefault;
  unsigned int bloodPressRaw[16];
  bloodPressRaw[0] = sysDefault;
  bloodPressRaw[8] = diaDefault;
  unsigned int pulseRateRaw[8];
  pulseRateRaw[0] = prDefault;
  unsigned int tempCorrected[8];
  unsigned int bloodPressCorrected[16];
  unsigned int prCorrected[8];
  short int * batteryState = &battDefault;
  unsigned short int * mode = &modeDefault;
  unsigned short int * measureSelect = &measureSelectDefault;
  unsigned short int * scroll = &scrollDefault;
  unsigned short int * select = &selectDefault;
  unsigned short int * alarmAcknowledge = &alarmAcknowledgeDefault;
  unsigned short int * addCompute = &addComputeDefault;
  unsigned short int * addCommunications = &addCommunicationsDefault;
  //create the TCB for Measure
  MeasureData * measureDataPtr;
  measureDataPtr = (struct MeasureData *) malloc(sizeof(struct MeasureData));
  measureDataPtr->temperatureRawBuf = temperatureRaw;
  measureDataPtr->bloodPressRawBuf = bloodPressRaw;
  measureDataPtr->pulseRateRawBuf = pulseRateRaw;
  measureDataPtr->measurementSelection = measureSelect;
  measureDataPtr->addCompute = addCompute;
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
  
  //keypad is always in the queue
  TCBKeypad->prev = curr;
  curr->next = TCBKeypad;
  curr = curr->next;
  
  //create the TCB for Display
  DisplayData * displayDataPtr;
  displayDataPtr = (struct DisplayData *) malloc(sizeof(struct DisplayData));
  //TODO: assign measure data locals to point to the values declared at the top
  displayDataPtr->tempCorrectedBuf = tempCorrected;
  displayDataPtr->bloodPressCorrectedBuf = bloodPressCorrected;
  displayDataPtr->prCorrectedBuf = prCorrected;
  displayDataPtr->batteryState = batteryState;
  displayDataPtr->mode = mode;
  displayDataPtr->scroll = scroll;
  displayDataPtr->select = select;
  displayDataPtr->measurementSelection = measureSelect;
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
  
  //display should be in the queue for the first run, then only added when keypad is pressed
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
  
  //warningalarm is always in the queue
  TCBWarningAlarm->prev = curr;
  curr->next = TCBWarningAlarm;
  curr = curr->next;
  
  //create the TCB for Communications
  CommunicationsData * communicationsDataPtr;
  communicationsDataPtr = (struct CommunicationsData *) 
    malloc(sizeof(struct CommunicationsData));
  //TODO: assign measure data locals to point to the values declared at the top
  communicationsDataPtr->tempCorrectedBuf = tempCorrected;
  communicationsDataPtr->bloodPressCorrectedBuf = bloodPressCorrected;
  communicationsDataPtr->prCorrectedBuf = prCorrected;
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
  
  //status is always in the queue
  TCBStatusMethod->prev = curr;
  curr->next = TCBStatusMethod;
  curr = curr->next;
  
  //create the TCB for Scheduler
  SchedulerData * schedulerDataPtr;
  schedulerDataPtr = (struct SchedulerData *) malloc(sizeof(struct SchedulerData));
  //assign Scheduler data locals to point to the values declared at the top
  schedulerDataPtr->addCompute = addCompute;
  schedulerDataPtr->addCommunications = addCommunications;
  //Make a void pointer to datastruct for scheduler
  void * voidSchedulerPtr = schedulerDataPtr;
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
    //need to move this functionality into the scheduler, but need to do this for now
    if (*addCompute == 1) {
      *addCompute = 0;
      TCBCompute->myTask(TCBCompute->taskDataPtr);
    }
    if (*addCommunications == 1) {
      *addCommunications = 0;
      TCBCommunications->myTask(TCBCommunications->taskDataPtr);
    }
    curr = head;
  }
}
void Schedule(void * voidSchedulerDataPtr) {
  if(globalCounter%5 == 0){
    delay(15);
    runMeasure = 1;
  }
}
//Keypad method should be moved into its own class
void Keypad(void * voidKeypadDataPtr) {
  KeypadData * keypadDataPtr = voidKeypadDataPtr;
  unsigned short int * mode = keypadDataPtr->mode;
  unsigned short int * measurementSelection = keypadDataPtr->measurementSelection;
  unsigned short int * scroll = keypadDataPtr->scroll;
  unsigned short int * select = keypadDataPtr->select;
  unsigned short int * alarmAcknowledge = keypadDataPtr->alarmAcknowledge;

  if (selectPressed == 1) {
    if (*mode == 0) {
      *mode = *scroll + 1;
      *scroll = 0;
    }
    else if (*mode == 1){
      *measurementSelection = *scroll+1;
      *select = 1;
    }
    selectPressed = 0;
  }
  if (downPressed == 1) {
    if (*mode == 0) {
      *scroll= 1;  
    }
    else if (*mode == 1) {
      if (*scroll >= 1) {
       *scroll = 2; 
      }
      else {
       *scroll = 1;
      }
    }
    downPressed = 0;
  }
  if (upPressed == 1) {
    if (*mode == 0) {
      *scroll= 0;  
    }
    else if (*mode == 1) {
      if (*scroll <= 1) {
       *scroll = 0; 
      }
      else {
       *scroll = 1;
      }
    }
    upPressed = 0;
  }
  if (leftPressed == 1) {
    *mode= 0;
    *scroll = 0;
    leftPressed = 0;
  }
}
void Communications (void * voidCommunicationsDataPtr) { 
  
}
void delay(unsigned long aValue){
  volatile unsigned long i = 0;

  volatile unsigned int j = 0;
  for (i = aValue; i > 0; i--){
    for (j = 0; j < 100000; j++);
  }
  return;
}