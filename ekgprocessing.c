#include "ProjectStructs.h"
#include "TCB.h"
#include "ekgcapture.h"
#include "optfft.h"
#include "GlobalCounter.h"

//#include "tables.c"
void insertAtFrontOfBuff (int value, unsigned int * buff);
void EKGProcessing(void * voidEKGCaptureDataPtr)
{
  while(1)  {
    if (1 == runEkgProc) {
      EKGCaptureData * ekgCapturePtr = voidEKGCaptureDataPtr;
      
      // create buffer of 0s for imaginary portion
      signed int ekgImagBuf[256];
      
      for (int x = 0; x < 256; x++) {
        ekgImagBuf[x] = 0;
      }
      // calculate FFT    
      insertAtFrontOfBuff(optfft(ekgCapturePtr->ekgRawBuf, ekgImagBuf),
                          ekgCapturePtr->ekgFreqBuf);  
      runEkgProc = 0;
    }
    vTaskDelay(1000);
  }
}

void insertAtFrontOfBuff (int value, unsigned int * buff) {
  for (int i = 7; i >0; i--){
    *(buff + i) = *(buff + (i-1));
  }
  *buff = value;
}