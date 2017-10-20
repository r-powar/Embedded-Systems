struct MeasureData{
  unsigned int * temperatureRawBuf;
  unsigned int * bloodPressRawBuf;
  unsigned int * pulseRateRawBuf;
  unsigned short int * measurementSelection;
};
typedef struct MeasureData MeasureData;

struct ComputeData{
  unsigned int * temperatureRawBuff;
  unsigned int * bloodPressRawBuf;
  unsigned int * pulseRateRawBuf;
  unsigned char * tempCorrectedBuf;
  unsigned char * bloodPressCorrectedBuf;
  unsigned char * prCorrectedBuf;
  unsigned short int * measurementSelection;
};
typedef struct ComputeData ComputeData;

struct DisplayData{
  unsigned char * tempCorrectedBuf;
  unsigned char * bloodPressCorrectedBuf;
  unsigned char * prCorrectedBuf;
  short int * batteryState;
  
  unsigned short int * mode;
};
typedef struct DisplayData DisplayData;


struct WarningAlarmData{
  unsigned char * temperatureRawBuf;
  unsigned char * bloodPressRawBuf;
  unsigned char * pulseRateRawBuf;
  short int * batteryState;
};
typedef struct WarningAlarmData WarningAlarmData;

struct Status{
  short int * batteryState;	
};
typedef struct Status Status;

struct KeypadData{
  unsigned short int * mode;
  unsigned short int * measurementSelection;
  unsigned short int * scroll;
  unsigned short int * select;
  unsigned short int * alarmAcknowledge;
};

typedef struct KeypadData KeypadData;

struct CommunicationsData{
  unsigned char * tempCorrectedBuf;
  unsigned char * bloodPressCorrectedBuf;
  unsigned char * prCorrectedBuf;
};
typedef struct CommunicationsData CommunicationsData;

struct SchedulerData{	
  int * globalCounter;
};
typedef struct SchedulerData SchedulerData;

