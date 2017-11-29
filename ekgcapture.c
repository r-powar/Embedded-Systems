#include "ProjectStructs.h"
#include "TCB.h"
#include "ekgcapture.h"
#include "GlobalCounter.h"
#include "math.h"
#define PI 3.141592654
#define OMEGA 2 * PI * 1000

void EKGCapture(void * voidEKGCaptureDataPtr)
{
  while(1) {
    if (1 == runEkgCap) {
      EKGCaptureData * ekgDataPtr = voidEKGCaptureDataPtr;
      float t = 0;
      // fill buffer with raw real values
      for (int x = 0; x < 256; x++) {		
        *(ekgDataPtr->ekgRawBuf+x) = (int)(30 * sin(OMEGA * t));
        t += 0.000125;
      }        
      runEkgProc = 1;
      runEkgCap = 0;
    }
    vTaskDelay(1000);
  }
}