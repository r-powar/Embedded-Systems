#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "ProjectStructs.h"
#include "TCB.h"
#include "display.h"
#include "GlobalCounter.h"
#include "utils/ustdlib.h"

void Display(void * voidDisplayDataPtr) {
  if(globalCounter%5 == 0){
    DisplayData * displayDataPtr = voidDisplayDataPtr;
    
    //Updated to pull the most recent data from the measurement buffers
    int * temp = displayDataPtr->tempCorrectedBuf;
    int * sys = displayDataPtr->bloodPressCorrectedBuf;
    int * dias = displayDataPtr->bloodPressCorrectedBuf+8;
    int * pRate = displayDataPtr->prCorrectedBuf;
    short int * bStatus = displayDataPtr->batteryState;
    
    //build the string that will be output on the top line
    char sysTemp [10];
    usprintf(sysTemp, "%d", *sys);
    char diasTemp [10];
    usprintf(diasTemp, "%d", *dias);
    char * bpUnits = "mm Hg";
    strcat(sysTemp, bpUnits); 
    strcat(sysTemp, "/");
    strcat(sysTemp, diasTemp);
    strcat(sysTemp, bpUnits);
    strcat(sysTemp, "      ");
    
    RIT128x96x4StringDraw(sysTemp, 0, 0, 15);
    //build the string that will be output on the second like
    char valTemp [10];
    usprintf(valTemp, "%d", *temp);
    char pTemp [10];
    usprintf(pTemp, "%d", *pRate);
    char battTemp [10];
    usprintf(battTemp, "%d", *bStatus);
    char * tempUnits = "C";
    char * pUnits = "BPM";
    char * battUnits = "Charge:";
    strcat(valTemp, tempUnits);
    strcat(valTemp, " ");
    strcat(valTemp, pTemp);
    strcat(valTemp, pUnits);
    strcat(valTemp, " ");
    strcat(valTemp, battUnits);
    strcat(valTemp, battTemp);
    strcat(valTemp, "    ");
    RIT128x96x4StringDraw(valTemp, 0, 15, 15);
    
  }
}