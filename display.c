#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "ProjectStructs.h"
#include "TCB.h"
#include "display.h"
#include "GlobalCounter.h"
#include "utils/ustdlib.h"

void Display(void * voidDisplayDataPtr) {
  while(1) {
    DisplayData * displayDataPtr = voidDisplayDataPtr;
  
    //Updated to pull the most recent data from the measurement buffers
    unsigned int * temp = displayDataPtr->tempCorrectedBuf;
    unsigned int * sys = displayDataPtr->bloodPressCorrectedBuf;
    unsigned int * dias = displayDataPtr->bloodPressCorrectedBuf+8;
    unsigned int * pRate = displayDataPtr->prCorrectedBuf;
    short int * bStatus = displayDataPtr->batteryState;
    unsigned short int * mode = displayDataPtr->mode;
    unsigned short int * scroll = displayDataPtr->scroll;
    unsigned short int * measureSelect = displayDataPtr->measurementSelection;
    unsigned short int * select = displayDataPtr->select;
    static unsigned short int currentMode= 1;
    static unsigned short int currentScroll= 0;
    //if display needs to change
    if (currentMode != *mode || currentScroll != *scroll || *select == 1 ) {
        RIT128x96x4Clear();
        currentMode = *mode;
        currentScroll = *scroll;
        *select = 0;

        //Home Screen
        if (*mode == 0) {
          //menu is highlighted  
          if (*scroll == 0) {
            char * menuLine =" * Menu";
            RIT128x96x4StringDraw(menuLine, 0, 0, 15);
            char * annunciateLine ="   Annunciate";
            RIT128x96x4StringDraw(annunciateLine, 0, 15, 15);
          } else {
            char * menuLine ="   Menu";
            RIT128x96x4StringDraw(menuLine, 0, 0, 15);
            char * annunciateLine =" * Annunciate";
            RIT128x96x4StringDraw(annunciateLine, 0, 15, 15);
          }
          //menu screen
        } else if (*mode == 1) {
          if (*measureSelect == 1) {
            char * mattRules = "                 *";
            RIT128x96x4StringDraw(mattRules, 0, 0, 15);
          } else if (*measureSelect == 2) { 
            char * mattRules = "              *";
            RIT128x96x4StringDraw(mattRules, 0, 15, 15);
          } else if (*measureSelect == 3) {
            char * mattRules = "            *";
            RIT128x96x4StringDraw(mattRules, 0, 30, 15);
          }
          if (*scroll == 0) {
            char * bpLine = " * Blood Pressure";
            RIT128x96x4StringDraw(bpLine, 0, 0, 15);
            char * tempLine = "   Temperature";
            RIT128x96x4StringDraw(tempLine, 0, 15, 15);
            char * hrLine = "   HeartRate";
            RIT128x96x4StringDraw(hrLine, 0, 30, 15);
          } else if (*scroll == 1) {
            char * bpLine = "   Blood Pressure";
            RIT128x96x4StringDraw(bpLine, 0, 0, 15);
            char * tempLine = " * Temperature";
            RIT128x96x4StringDraw(tempLine, 0, 15, 15);
            char * hrLine = "   HeartRate";
            RIT128x96x4StringDraw(hrLine, 0, 30, 15);
          } else {
            char * bpLine = "   Blood Pressure";
            RIT128x96x4StringDraw(bpLine, 0, 0, 15);
            char * tempLine = "   Temperature";
            RIT128x96x4StringDraw(tempLine, 0, 15, 15);
            char * hrLine = " * HeartRate";
            RIT128x96x4StringDraw(hrLine, 0, 30, 15);
          }
        } 
    } 
    if (*mode == 2) {
      //build the string that will be output on the top line
            char sysTemp [10];
            usprintf(sysTemp, "%d", staticSys);
            char diasTemp [10];
            usprintf(diasTemp, "%d", staticDias);
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
            
            char * cuffDisp = (char *) pvPortMalloc(10*sizeof(char));
            cuffDisp = "Cuff level: ";
            char cuffTemp [10];
            usprintf(cuffTemp, "%d", cuff);
            char * percentMark = "%";
            strcat(cuffTemp, percentMark);
            strcat(cuffTemp, "   ");
            RIT128x96x4StringDraw(cuffDisp, 0, 30, 15);
            RIT128x96x4StringDraw(cuffTemp, 67, 30, 15);
            
    }
    vTaskDelay(1000);
  }
}