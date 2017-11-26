#include "ProjectStructs.h"
#include "TCB.h"
#include "ekgcapture.h"

void EKGCapture(void * voidEKGCaptureDataPtr)
{
	while(1)
	{
		EKGCaptureData * ekgDataPtr = voidEKGCaptureDataPtr;
		
		// fill buffer with raw real values
		for (int x = 0; x < 256; x++)
		{
			ekgDataPtr->ekgRawBuf+x = x;
		}
		
		vTaskDelay(1000);
	}
}