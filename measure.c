#include <stdio.h>
#include <stdbool.h>
#include "ProjectStructs.h"
#include "TCB.h"
#include "measure.h"
#include "GlobalCounter.h"

 //global declaration
 static int sysComplete = 0;
 static int diasComplete = 0;
 
 //constant variable
 
void Measure(void * voidMeasureDataPtr) {
  
  if(globalCounter%5 == 0){
    
    MeasureData * measureDataPtr = voidMeasureDataPtr;
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
    *tempRaw += isReversed ? -2 : 2;
  }else{
    *tempRaw += isReversed ? 1 : -1;
  }
  funcCall++;
}

void sysPressRaw(unsigned int * sysRaw){
  static int funcCall = 0;
  if(diasComplete && sysComplete){
    diasComplete = 0;
    sysComplete = 0;
    *sysRaw = 80;
  }
  if(*sysRaw > 100){
    //set systolic variable to complete
    sysComplete = 1;
  }else{
    if (!sysComplete){
      if(funcCall % 2 == 0){
        *sysRaw += 3;
      }else{
        *sysRaw--;
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
        *diasRaw -= 2;
      }
      else {
        *diasRaw += 1;
      }
    }
  }
  if (diasComplete && sysComplete) {
    *diasRaw = 80; 
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
    *prRaw += isReversed ? -1 : 1;
  }else{
    *prRaw += isReversed ? 3 : -3;
  }  
  funcCall++;
}