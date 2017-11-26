#include "ProjectStructs.h"
#include "TCB.h"
#include "ekgcapture.h"
#include "optfft.h"
#include "tables.c"

void EKGProcessing(void * voidEKGCaptureDataPtr)
{
	while(1)
	{
		EKGCaptureData * ekgCapturePtr = voidEKGCaptureDataPtr;
		
		// create buffer of 0s for imaginary portion
		unsigned int * ekgImagBuf = (unsigned int *)pvPortMalloc(256 * sizeof(unsigned int));
		
		for (int x = 0; x < 256; x++)
		{
			ekgImagBuf[x] = 0;
		}
		
		// calculate FFT
		insertAtFrontOfBuff(optfft(ekgCapturePtr->ekgRawBuf, ekgImagBuf), ekgCapturePtr->ekgFreqBuf);
		
		vTaskDelay(1000);
	}
}

void insertAtFrontOfBuff (int value, unsigned int * buff) {
  for (int i = 7; i >0; i--){
    *(buff + i) = *(buff + (i-1));
  }
  *buff = value;
}