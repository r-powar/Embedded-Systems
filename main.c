<<<<<<< Updated upstream
#include <stdio.h>
#include <stdbool.h>
#include "ProjectStructs.h"
#include "BoolInclude.h"
#include "TCB.h"
#include "inc/lm3s8962.h"
#include "GlobalCounter.h"
#include "inc/lm3s8962.h"
#include "inc/hw_gpio.h"
=======
#define mainINCLUDE_WEB_SERVER		0

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Hardware library includes. */
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_sysctl.h"
#include "sysctl.h"
#include "gpio.h"
#include "grlib.h"
#include "rit128x96x4.h"
#include "osram128x64x4.h"
#include "formike128x128x16.h"
>>>>>>> Stashed changes
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


<<<<<<< Updated upstream
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
  //unsigned int tempDefault = 75;
  //unsigned int sysDefault = 80;
  //unsigned int diaDefault = 80;
  //unsigned int prDefault = 50;
  short int battDefault = 200;
=======

/*-----------------------------------------------------------*/

/* The time between cycles of the 'check' functionality (defined within the
tick hook. */
#define mainCHECK_DELAY						( ( TickType_t ) 5000 / portTICK_PERIOD_MS )

/* Size of the stack allocated to the uIP task. */
#define mainBASIC_WEB_STACK_SIZE            ( configMINIMAL_STACK_SIZE * 3 )

/* The OLED task uses the sprintf function so requires a little more stack too. */
#define mainOLED_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE + 50 )

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainCREATOR_TASK_PRIORITY           ( tskIDLE_PRIORITY + 3 )
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY )
#define mainGEN_QUEUE_TASK_PRIORITY			( tskIDLE_PRIORITY )

/* The maximum number of message that can be waiting for display at any one
time. */
#define mainOLED_QUEUE_SIZE					( 3 )

/* Dimensions the buffer into which the jitter time is written. */
#define mainMAX_MSG_LEN						25

/* The period of the system clock in nano seconds.  This is used to calculate
the jitter time in nano seconds. */
#define mainNS_PER_CLOCK					( ( unsigned long ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Constants used when writing strings to the display. */
#define mainCHARACTER_HEIGHT				( 9 )
#define mainMAX_ROWS_128					( mainCHARACTER_HEIGHT * 14 )
#define mainMAX_ROWS_96						( mainCHARACTER_HEIGHT * 10 )
#define mainMAX_ROWS_64						( mainCHARACTER_HEIGHT * 7 )
#define mainFULL_SCALE						( 15 )
#define ulSSI_FREQUENCY						( 3500000UL )

/*-----------------------------------------------------------*/

/*
 * The task that handles the uIP stack.  All TCP/IP processing is performed in
 * this task.
 */
extern void vuIP_Task( void *pvParameters );

/*
 * The display is written two by more than one task so is controlled by a
 * 'gatekeeper' task.  This is the only task that is actually permitted to
 * access the display directly.  Other tasks wanting to display a message send
 * the message to the gatekeeper.
 */
static void vOLEDTask( void *pvParameters );

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/*
 * Configures the high frequency timers - those used to measure the timing
 * jitter while the real time kernel is executing.
 */
extern void vSetupHighFrequencyTimer( void );

/*
 * Hook functions that can get called by the kernel.
 */
void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName );
void vApplicationTickHook( void );


/*-----------------------------------------------------------*/

/* The queue used to send messages to the OLED task. */
QueueHandle_t xOLEDQueue;
/*-----------------------------------------------------------*/


/*************************************************************************
 * Please ensure to read http://www.freertos.org/portlm3sx965.html
 * which provides information on configuring and running this demo for the
 * various Luminary Micro EKs.
 *************************************************************************/
int main( void )
{
  prvSetupHardware();
  //
  // Set the clocking to run directly from the crystal.
  //  //
  //  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
  //                 SYSCTL_XTAL_8MHZ);
  //SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
   
  //LED
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);
  
  
 //enable the pins for the direction buttons/select
 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
 GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
 GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2| GPIO_PIN_3, GPIO_STRENGTH_2MA,
  GPIO_PIN_TYPE_STD_WPU);
 IntEnable(INT_GPIOE);
 
 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
 GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_1);
 GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_STRENGTH_2MA,
                  GPIO_PIN_TYPE_STD_WPU);
 IntEnable(INT_GPIOF);
 
 RIT128x96x4Init(1000000);
 //speaker
 SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);   
 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
 GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_1);
 volatile unsigned long ulPeriod = SysCtlClockGet() / 2048;
 PWMGenConfigure(PWM_BASE, PWM_GEN_0,
                 PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
 PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod); 
 PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ulPeriod * 3 / 4);  

 PWMGenDisable(PWM_BASE, PWM_GEN_0); // Turn off the speaker
 
 //initialize defaults
  unsigned int tempDefault = 75;
  unsigned int sysDefault = 80;
  unsigned int diaDefault = 80;
  unsigned int prDefault = 50;
  unsigned short int battDefault = 200;
>>>>>>> Stashed changes
  unsigned short int modeDefault = 0;
  unsigned short int measureSelectDefault = 0;
  unsigned short int scrollDefault = 0;
  unsigned short int selectDefault = 0;
  unsigned short int alarmAcknowledgeDefault = 0;
  unsigned short int addComputeDefault = 1;
  unsigned short int addCommunicationsDefault = 0;
  //declare the variables that will be used for the tracking in the device
<<<<<<< Updated upstream
  unsigned int *temperatureRaw;
  //temperatureRaw[0] = tempDefault;
  unsigned int *bloodPressRaw;
 // bloodPressRaw[0] = sysDefault;
  //bloodPressRaw[8] = diaDefault;
  unsigned int *pulseRateRaw;
  //pulseRateRaw[0] = prDefault;
  unsigned int *tempCorrected;
  unsigned int *bloodPressCorrected;
  unsigned int *prCorrected;
  
  temperatureRaw = (unsigned int *) malloc(sizeof(unsigned int) * 8);
  bloodPressRaw = (unsigned int *) malloc(sizeof(unsigned int) * 16);
  pulseRateRaw = (unsigned int *) malloc(sizeof(unsigned int) * 8);
  
  tempCorrected = (unsigned int *) malloc(sizeof(unsigned int) * 8);
  bloodPressCorrected = (unsigned int *) malloc(sizeof(unsigned int) * 16);
  prCorrected = (unsigned int *) malloc(sizeof(unsigned int) * 8);
  
  for(volatile int i = 0; i < 8; i++){
    tempCorrected[i] = 0;
    prCorrected[i] = 0;
    temperatureRaw[i] = 75;
    pulseRateRaw[i] = 50;
  }
  
  for(volatile int i = 0; i < 16; i++){
    bloodPressRaw[i] = 80;
    bloodPressCorrected[i] = 0;
  }
  
  
  
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
  measureDataPtr->temperatureRawBuf = &temperatureRaw[0];
  measureDataPtr->bloodPressRawBuf = &bloodPressRaw[0];
  measureDataPtr->pulseRateRawBuf = &pulseRateRaw[0];
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
=======
  unsigned int * temperatureRaw = (unsigned int *) pvPortMalloc(8*sizeof(unsigned int));
  temperatureRaw[0] = tempDefault;
  //uint8_t *temperatureRaw[8] = {75, 75, 75, 75, 75, 75, 75, 75};
  unsigned int * bloodPressRaw = (unsigned int *) pvPortMalloc(16*sizeof(unsigned int));
  bloodPressRaw[0] = sysDefault;
  bloodPressRaw[8] = diaDefault;
  //unsigned int * pulseRateRaw = (unsigned int *) pvPortMalloc(8*sizeof(unsigned int));
  unsigned int pulseRateRaw[8] = {50};
  unsigned int * tempCorrected = (unsigned int *) pvPortMalloc(8*sizeof(unsigned int));
  unsigned int * bloodPressCorrected = (unsigned int *) pvPortMalloc(16*sizeof(unsigned int));
  unsigned int * prCorrected = (unsigned int *) pvPortMalloc(8*sizeof(unsigned int));
  tempCorrected[0] = 0;
  bloodPressCorrected[0] = 0;
  bloodPressCorrected[8] = 0;
  prCorrected[0] = 0;
  unsigned short int * batteryState;
  //batteryState = (unsigned short int * ) pvPortMalloc(sizeof(unsigned short int));
  batteryState = &battDefault;
  unsigned short int * mode;
  mode = &modeDefault;
  unsigned short int * measureSelect;
  measureSelect = &measureSelectDefault;
  unsigned short int * scroll;
  scroll = &scrollDefault;
  unsigned short int * select;
  select = &selectDefault;
  unsigned short int * alarmAcknowledge;
  alarmAcknowledge = &alarmAcknowledgeDefault;
  unsigned short int * addCompute;
  addCompute = &addComputeDefault;
  unsigned short int * addCommunications;
  addCommunications = &addCommunicationsDefault;
  //
  // Enable the PWM0 and PWM1 output signals.
  //
  PWMOutputState(PWM_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);

    /* Exclude some tasks if using the kickstart version to ensure we stay within
    the 32K code size limit. */
    #if mainINCLUDE_WEB_SERVER != 0
    {
        /* Create the uIP task if running on a processor that includes a MAC and
        PHY. */
        if( SysCtlPeripheralPresent( SYSCTL_PERIPH_ETH ) )
        {
                xTaskCreate( vuIP_Task, "uIP", mainBASIC_WEB_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );
        }
    }
    #endif
    
  //create the TCB for Measure
  MeasureData measureDataPtr;
//  measureDataPtr = (struct MeasureData) malloc(sizeof(struct MeasureData));
  measureDataPtr.temperatureRawBuf = temperatureRaw;
  measureDataPtr.bloodPressRawBuf = bloodPressRaw;
  measureDataPtr.pulseRateRawBuf = pulseRateRaw;
  measureDataPtr.measurementSelection = measureSelect;
  measureDataPtr.addCompute = addCompute;
  //Make a void pointer to datastruct for measure
  MeasureData * MDActual;
  MDActual = (struct MeasureData*) pvPortMalloc(sizeof(struct MeasureData));
  MDActual = &measureDataPtr;
  void * voidMeasureDataPtr = MDActual;
  
  xTaskCreate( Measure, "MEASURE", 1000, voidMeasureDataPtr, 1, NULL );
>>>>>>> Stashed changes
  
  //create the TCB for Compute
  ComputeData computeDataPtr;
//  computeDataPtr = (struct ComputeData *) malloc(sizeof(struct ComputeData));
  //assign compute data locals to point to the values declared at the top
<<<<<<< Updated upstream
  computeDataPtr->temperatureRawBuf = &temperatureRaw[0];
  computeDataPtr->bloodPressRawBuf = &bloodPressRaw[0];
  computeDataPtr->pulseRateRawBuf = &pulseRateRaw[0];
  
  computeDataPtr->tempCorrectedBuf = &tempCorrected[0];
  computeDataPtr->bloodPressCorrectedBuf = &bloodPressCorrected[0];
  computeDataPtr->prCorrectedBuf = &prCorrected[0];
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
=======
  computeDataPtr.temperatureRawBuf = temperatureRaw;
  computeDataPtr.bloodPressRawBuf = bloodPressRaw;
  computeDataPtr.pulseRateRawBuf = pulseRateRaw;
  computeDataPtr.tempCorrectedBuf = tempCorrected;
  computeDataPtr.bloodPressCorrectedBuf = bloodPressCorrected;
  computeDataPtr.prCorrectedBuf = prCorrected;
  ComputeData * CDActual;
  //CDActual = (struct ComputeData *) malloc(sizeof(struct ComputeData));
  CDActual = &computeDataPtr;
  //Make a void pointer to datastruct for Compute
  void * voidComputeDataPtr = CDActual;
  
  xTaskCreate( Compute, "COMPUTE", 500, voidComputeDataPtr, 1, NULL );
>>>>>>> Stashed changes
  
  //create the TCB for Keypad
  KeypadData keypadDataPtr;
//  keypadDataPtr = (struct KeypadData *) malloc(sizeof(struct KeypadData));
  keypadDataPtr.mode = mode;
  keypadDataPtr.measurementSelection = measureSelect;
  keypadDataPtr.scroll = scroll;
  keypadDataPtr.select = select;
  keypadDataPtr.alarmAcknowledge = alarmAcknowledge; 
  KeypadData * KDActual;
  //KDActual = (struct KeypadData *) malloc(sizeof(struct KeypadData));
  KDActual = &keypadDataPtr;
  //Make a void pointer to datastruct for Keypad
<<<<<<< Updated upstream
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
  displayDataPtr->tempCorrectedBuf = &tempCorrected[0];
  displayDataPtr->bloodPressCorrectedBuf = &bloodPressCorrected[0];
  displayDataPtr->prCorrectedBuf = &prCorrected[0];
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
=======
  void * voidKeypadDataPtr = KDActual;
  
  xTaskCreate( Keypad, "KEYPAD", 100, voidKeypadDataPtr, 1, NULL );
  
  //create the TCB for Display
  DisplayData displayDataPtr;
  displayDataPtr.tempCorrectedBuf = tempCorrected;
  displayDataPtr.bloodPressCorrectedBuf = bloodPressCorrected;
  displayDataPtr.prCorrectedBuf = prCorrected;
  displayDataPtr.batteryState = batteryState;
  displayDataPtr.mode = mode;
  displayDataPtr.scroll = scroll;
  displayDataPtr.select = select;
  displayDataPtr.measurementSelection = measureSelect;
  DisplayData * DDActual;
  DDActual = (struct DisplayData *) pvPortMalloc(sizeof(struct DisplayData));
  DDActual = &displayDataPtr;
//  //Make a void pointer to datastruct for display
  void * voidDisplayDataPtr = DDActual;
  
  xTaskCreate(Display, "DISPLAY", 1000, voidDisplayDataPtr, 1, NULL );
>>>>>>> Stashed changes
  
  //create the TCB for WarningAlarm
  WarningAlarmData warningAlarmDataPtr;
//  warningAlarmDataPtr = (struct WarningAlarmData *) 
//    malloc(sizeof(struct WarningAlarmData));
  //TODO: assign measure data locals to point to the values declared at the top
<<<<<<< Updated upstream
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
  
  //warningalarm is always in the queue
  TCBWarningAlarm->prev = curr;
  curr->next = TCBWarningAlarm;
  curr = curr->next;
=======
  warningAlarmDataPtr.temperatureRawBuf = temperatureRaw;
  warningAlarmDataPtr.bloodPressRawBuf = bloodPressRaw;
  warningAlarmDataPtr.pulseRateRawBuf = pulseRateRaw;
  warningAlarmDataPtr.batteryState = batteryState;
  WarningAlarmData * WADActual;
  WADActual =(struct WarningAlarmData *) 
    pvPortMalloc(sizeof(struct WarningAlarmData));
  WADActual = &warningAlarmDataPtr;
  //Make a void pointer to datastruct for WarningAlarm
  void * voidWarningAlarmDataPtr = WADActual;
>>>>>>> Stashed changes
  
  xTaskCreate(WarningAlarm, "WARNALARM", 500, voidWarningAlarmDataPtr, 1, NULL );
//  
  //create the TCB for Communications
  CommunicationsData communicationsDataPtr;
//  communicationsDataPtr = (struct CommunicationsData *) 
//    malloc(sizeof(struct CommunicationsData));
  //TODO: assign measure data locals to point to the values declared at the top
<<<<<<< Updated upstream
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
=======
  communicationsDataPtr.tempCorrectedBuf = tempCorrected;
  communicationsDataPtr.bloodPressCorrectedBuf = bloodPressCorrected;
  communicationsDataPtr.prCorrectedBuf = prCorrected;
  CommunicationsData * CommDActual = &communicationsDataPtr;
  CommDActual = (struct CommunicationsData *) 
     pvPortMalloc(sizeof(struct CommunicationsData));
  //Make a void pointer to datastruct for Communications
  void * voidCommunicationsDataPtr = CommDActual;
  
  xTaskCreate(Communications, "COMMUNICATIONS", 500, voidCommunicationsDataPtr, 1, NULL );
>>>>>>> Stashed changes
  
  //create the TCB for StatusMethod
  Status statusPtr;
//  statusPtr = (struct Status *) malloc(sizeof(struct Status));
  //TODO: assign measure data locals to point to the values declared at the top
  statusPtr.batteryState = batteryState;
  //Make a void pointer to datastruct for StatusMethod
<<<<<<< Updated upstream
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
=======
  Status * SActual = &statusPtr;
  SActual = (struct Status *) pvPortMalloc(sizeof(struct Status));
  void * voidStatusPtr = SActual;
  
  xTaskCreate(StatusMethod, "STATUSMETHOD", 500, voidStatusPtr, 1, NULL );
  
  /* The suicide tasks must be created last as they need to know how many
    tasks were running prior to their creation in order to ascertain whether
    or not the correct/expected number of tasks are running at any given time. */
 //   vCreateSuicidalTasks( mainCREATOR_TASK_PRIORITY );

    /* Configure the high frequency interrupt used to measure the interrupt
    jitter time. */
    //vSetupHighFrequencyTimer();

    /* Start the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task. */
    return 0;
>>>>>>> Stashed changes
}
void Schedule(void * voidSchedulerDataPtr) {
  while(globalCounter%5 == 0){
    delay(15);
    runMeasure = 1;
  }
 
}
//Keypad method should be moved into its own class
<<<<<<< Updated upstream
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
    runDisplay = 1;
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
     runDisplay = 1;
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
    runDisplay = 1;
  }
  if (leftPressed == 1) {
    *mode= 0;
    *scroll = 0;
    leftPressed = 0;
    runDisplay = 1;
  }
}
=======
//void Keypad(void * voidKeypadDataPtr) {
//  while (1) {
//    KeypadData * keypadDataPtr = voidKeypadDataPtr;
//  unsigned short int * mode = keypadDataPtr->mode;
//  unsigned short int * measurementSelection = keypadDataPtr->measurementSelection;
//  unsigned short int * scroll = keypadDataPtr->scroll;
//  unsigned short int * select = keypadDataPtr->select;
//  unsigned short int * alarmAcknowledge = keypadDataPtr->alarmAcknowledge;
//
//  if (selectPressed == 1) {
//    if (*mode == 0) {
//      *mode = *scroll + 1;
//      *scroll = 0;
//    }
//    else if (*mode == 1){
//      *measurementSelection = *scroll+1;
//      *select = 1;
//    }
//    selectPressed = 0;
//  }
//  if (downPressed == 1) {
//    if (*mode == 0) {
//      *scroll= 1;  
//    }
//    else if (*mode == 1) {
//      if (*scroll >= 1) {
//       *scroll = 2; 
//      }
//      else {
//       *scroll = 1;
//      }
//    }
//    downPressed = 0;
//  }
//  if (upPressed == 1) {
//    if (*mode == 0) {
//      *scroll= 0;  
//    }
//    else if (*mode == 1) {
//      if (*scroll <= 1) {
//       *scroll = 0; 
//      }
//      else {
//       *scroll = 1;
//      }
//    }
//    upPressed = 0;
//  }
//  if (leftPressed == 1) {
//    *mode= 0;
//    *scroll = 0;
//    leftPressed = 0;
//  }
//    vTaskDelay(1000);
//  }
//}
>>>>>>> Stashed changes
void Communications (void * voidCommunicationsDataPtr) { 
  
}
void delay(unsigned long aValue){
  volatile unsigned long i = 0;

  volatile unsigned int j = 0;
  for (i = aValue; i > 0; i--){
    for (j = 0; j < 100000; j++);
  }
  return;
<<<<<<< Updated upstream
=======
}







/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
    /* If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
    a workaround to allow the PLL to operate reliably. */
    if( DEVICE_IS_REVA2 )
    {
        SysCtlLDOSet( SYSCTL_LDO_2_75V );
    }

	/* Set the clocking to run from the PLL at 50 MHz */
	SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );

	/* 	Enable Port F for Ethernet LEDs
		LED0        Bit 3   Output
		LED1        Bit 2   Output */
//	SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOF );
//	GPIODirModeSet( GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3), GPIO_DIR_MODE_HW );
//	GPIOPadConfigSet( GPIO_PORTF_BASE, (GPIO_PIN_2 | GPIO_PIN_3 ), GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD );
//
//	vParTestInitialise();
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
//static xOLEDMessage xMessage = { "PASS" };
//static unsigned long ulTicksSinceLastDisplay = 0;
//portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//
//	/* Called from every tick interrupt.  Have enough ticks passed to make it
//	time to perform our health status check again? */
//	ulTicksSinceLastDisplay++;
//	if( ulTicksSinceLastDisplay >= mainCHECK_DELAY )
//	{
//		ulTicksSinceLastDisplay = 0;
//
//		/* Has an error been found in any task? */
//		if( xAreGenericQueueTasksStillRunning() != pdTRUE )
//		{
//			xMessage.pcMessage = "ERROR IN GEN Q";
//		}
//	    else if( xIsCreateTaskStillRunning() != pdTRUE )
//	    {
//	        xMessage.pcMessage = "ERROR IN CREATE";
//	    }
//	    else if( xAreIntegerMathsTaskStillRunning() != pdTRUE )
//	    {
//	        xMessage.pcMessage = "ERROR IN MATH";
//	    }
//		else if( xAreIntQueueTasksStillRunning() != pdTRUE )
//		{
//			xMessage.pcMessage = "ERROR IN INT QUEUE";
//		}
//		else if( xAreBlockingQueuesStillRunning() != pdTRUE )
//		{
//			xMessage.pcMessage = "ERROR IN BLOCK Q";
//		}
//		else if( xAreBlockTimeTestTasksStillRunning() != pdTRUE )
//		{
//			xMessage.pcMessage = "ERROR IN BLOCK TIME";
//		}
//		else if( xAreSemaphoreTasksStillRunning() != pdTRUE )
//		{
//			xMessage.pcMessage = "ERROR IN SEMAPHORE";
//		}
//		else if( xArePollingQueuesStillRunning() != pdTRUE )
//		{
//			xMessage.pcMessage = "ERROR IN POLL Q";
//		}
//		else if( xAreQueuePeekTasksStillRunning() != pdTRUE )
//		{
//			xMessage.pcMessage = "ERROR IN PEEK Q";
//		}
//		else if( xAreRecursiveMutexTasksStillRunning() != pdTRUE )
//		{
//			xMessage.pcMessage = "ERROR IN REC MUTEX";
//		}
//		else if( xAreQueueSetTasksStillRunning() != pdPASS )
//		{
//			xMessage.pcMessage = "ERROR IN Q SET";
//		}
//		else if( xAreEventGroupTasksStillRunning() != pdTRUE )
//		{
//			xMessage.pcMessage = "ERROR IN EVNT GRP";
//		}
//
//		configASSERT( strcmp( ( const char * ) xMessage.pcMessage, "PASS" ) == 0 );
//
//		/* Send the message to the OLED gatekeeper for display. */
//		xHigherPriorityTaskWoken = pdFALSE;
//		xQueueSendFromISR( xOLEDQueue, &xMessage, &xHigherPriorityTaskWoken );
//	}
//
//	/* Write to a queue that is in use as part of the queue set demo to
//	demonstrate using queue sets from an ISR. */
//	vQueueSetAccessQueueSetFromISR();
//
//	/* Call the event group ISR tests. */
//	vPeriodicEventGroupsProcessing();
}
/*-----------------------------------------------------------*/

void vOLEDTask( void *pvParameters )
{
xOLEDMessage xMessage;
unsigned long ulY, ulMaxY;
static char cMessage[ mainMAX_MSG_LEN ];
extern volatile unsigned long ulMaxJitter;
const unsigned char *pucImage;

/* Functions to access the OLED.  The one used depends on the dev kit
being used. */
void ( *vOLEDInit )( unsigned long ) = NULL;
void ( *vOLEDStringDraw )( const char *, unsigned long, unsigned long, unsigned char ) = NULL;
void ( *vOLEDImageDraw )( const unsigned char *, unsigned long, unsigned long, unsigned long, unsigned long ) = NULL;
void ( *vOLEDClear )( void ) = NULL;

	/* Map the OLED access functions to the driver functions that are appropriate
	for the evaluation kit being used. */
	switch( HWREG( SYSCTL_DID1 ) & SYSCTL_DID1_PRTNO_MASK )
	{
		case SYSCTL_DID1_PRTNO_6965	:
		case SYSCTL_DID1_PRTNO_2965	:	vOLEDInit = OSRAM128x64x4Init;
										vOLEDStringDraw = OSRAM128x64x4StringDraw;
										vOLEDImageDraw = OSRAM128x64x4ImageDraw;
										vOLEDClear = OSRAM128x64x4Clear;
										ulMaxY = mainMAX_ROWS_64;
										pucImage = pucBasicBitmap;
										break;

		case SYSCTL_DID1_PRTNO_1968	:
		case SYSCTL_DID1_PRTNO_8962 :	vOLEDInit = RIT128x96x4Init;
										vOLEDStringDraw = RIT128x96x4StringDraw;
										vOLEDImageDraw = RIT128x96x4ImageDraw;
										vOLEDClear = RIT128x96x4Clear;
										ulMaxY = mainMAX_ROWS_96;
										pucImage = pucBasicBitmap;
										break;

		default						:	vOLEDInit = vFormike128x128x16Init;
										vOLEDStringDraw = vFormike128x128x16StringDraw;
										vOLEDImageDraw = vFormike128x128x16ImageDraw;
										vOLEDClear = vFormike128x128x16Clear;
										ulMaxY = mainMAX_ROWS_128;
										pucImage = pucGrLibBitmap;
										break;

	}

	ulY = ulMaxY;

	/* Initialise the OLED and display a startup message. */
	vOLEDInit( ulSSI_FREQUENCY );
	vOLEDStringDraw( "POWERED BY FreeRTOS", 0, 0, mainFULL_SCALE );
	vOLEDImageDraw( pucImage, 0, mainCHARACTER_HEIGHT + 1, bmpBITMAP_WIDTH, bmpBITMAP_HEIGHT );

	for( ;; )
	{
		/* Wait for a message to arrive that requires displaying. */
		xQueueReceive( xOLEDQueue, &xMessage, portMAX_DELAY );

		/* Write the message on the next available row. */
		ulY += mainCHARACTER_HEIGHT;
		if( ulY >= ulMaxY )
		{
			ulY = mainCHARACTER_HEIGHT;
			vOLEDClear();
			//vOLEDStringDraw( pcWelcomeMessage, 0, 0, mainFULL_SCALE );
		}

		/* Display the message along with the maximum jitter time from the
		high priority time test. */
		sprintf( cMessage, "%s [%uns]", xMessage.pcMessage, ulMaxJitter * mainNS_PER_CLOCK );
		vOLEDStringDraw( cMessage, 0, ulY, mainFULL_SCALE );
	}
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t *pxTask, signed char *pcTaskName )
{
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char *pcFile, unsigned long ulLine )
{
volatile unsigned long ulSetTo1InDebuggerToExit = 0;

	taskENTER_CRITICAL();
	{
		while( ulSetTo1InDebuggerToExit == 1 )
		{
			/* Nothing do do here.  Set the loop variable to a non zero value in
			the debugger to step out of this function to the point that caused
			the assertion. */
			( void ) pcFile;
			( void ) ulLine;
		}
	}
	taskEXIT_CRITICAL();
>>>>>>> Stashed changes
}