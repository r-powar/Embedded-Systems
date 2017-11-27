#include "ProjectStructs.h"
#include "TCB.h"
#include "ekgcapture.h"
#include "GlobalCounter.h"

void EKGCapture(void * voidEKGCaptureDataPtr)
{
  while(1) {
    if (1 == runEkgCap) {
      EKGCaptureData * ekgDataPtr = voidEKGCaptureDataPtr;
      // fill buffer with raw real values
      for (int x = 0; x < 256; x++) {
        *(ekgDataPtr->ekgRawBuf+x) = x;
      }        
      runEkgProc = 1;
      runEkgCap = 0;
    }
    vTaskDelay(1000);
  }
}