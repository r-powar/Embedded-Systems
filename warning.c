#include <stdio.h>
#include <stdbool.h>
#include "inc/lm3s8962.h"
#include "ProjectStructs.h"
#include "TCB.h"
#include "warning.h"
#include "GlobalCounter.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "drivers/rit128x96x4.h"

// Maddie: Warning/Annunciate
#define HIGH_TEMP_RAW 43.73 //(37.8/0.75 - 5/0.75)
#define LOW_TEMP_RAW 41.46 //(36.1/0.75 - 5/0.75)
#define SYSTOLIC_RAW 55.5 //(120/2 - 9/2)
#define DIASTOLIC_RAW 50.33 //(80/1.5 - 6/2)
#define HIGH_PULSE_RAW 30.67 //(100/3 - 8/3)
#define LOW_PULSE_RAW 17.33 //(60/3 - 8/3)
#define LOW_BATTERY 40

#define PULSE_DELAY 20
#define TEMP_DELAY 10
#define BP_DELAY 5
#define BATTERY_DELAY 1


bool LEDOn = false;

void WarningAlarm(void * voidWarningAlarmDataPtr) {
  // Maddie: changed to voidWarningAlarmDataPtr
  WarningAlarmData * warningAlarmDataPtr = voidWarningAlarmDataPtr;
  int alarmDelay;
  bool alarmOn = false;
  
  if (*warningAlarmDataPtr->pulseRateRawBuf >= HIGH_PULSE_RAW ||
      *warningAlarmDataPtr->pulseRateRawBuf <= LOW_PULSE_RAW)
  {
    alarmOn = true;
    alarmDelay = PULSE_DELAY;
  }
  if (*warningAlarmDataPtr->temperatureRawBuf >= HIGH_TEMP_RAW ||
      *warningAlarmDataPtr->temperatureRawBuf <= LOW_TEMP_RAW)
  {
    alarmOn = true;
    alarmDelay = TEMP_DELAY;
  }
  if (*warningAlarmDataPtr->bloodPressRawBuf >= 100 ||
      *warningAlarmDataPtr->bloodPressRawBuf+8 <= DIASTOLIC_RAW)
  {
    if(rightPressed != 1) {
      PWMGenEnable(PWM_BASE, PWM_GEN_0); // Turn on the speaker
      
    }
    alarmOn = true;
    alarmDelay = BP_DELAY;
  }
  if (*warningAlarmDataPtr->batteryState <= LOW_BATTERY)
  {
    alarmOn = true;
    alarmDelay = BATTERY_DELAY;
  }
  blinky(alarmDelay, alarmOn);
}

void blinky(unsigned long aValue, bool alarmOn)
{  
  //if alarm is off and it is time to turn on...
 if (alarmOn == true) {
    if (globalCounter % aValue == 0){
      if (!LEDOn) {
        // Turn on the LED.
        GPIO_PORTF_DATA_R |= 0x01;
        LEDOn = !LEDOn;
      }
      else {
        // Turn off the LED.
        GPIO_PORTF_DATA_R &= ~(0x01);
        LEDOn = !LEDOn;
      }
    }
  }
}