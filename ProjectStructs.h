struct MeasureData{
  unsigned int * temperatureRawBuf;
  unsigned int * bloodPressRawBuf;
  unsigned int * pulseRateRawBuf;
  unsigned short int * measurementSelection;
  unsigned short int * addCompute;
};
typedef struct MeasureData MeasureData;

struct EKGCaptureData{
  int * ekgRawBuf;
  unsigned int * ekgFreqBuf;
};
typedef struct EKGCaptureData EKGCaptureData;


struct ComputeData{
  unsigned int * temperatureRawBuf;
  unsigned int * bloodPressRawBuf;
  unsigned int * pulseRateRawBuf;
  unsigned int * tempCorrectedBuf;
  unsigned int * bloodPressCorrectedBuf;
  unsigned int * prCorrectedBuf;
  unsigned short int * measurementSelection;
};
typedef struct ComputeData ComputeData;

struct DisplayData{
  unsigned int * tempCorrectedBuf;
  unsigned int * bloodPressCorrectedBuf;
  unsigned int * prCorrectedBuf;
  unsigned int * ekgFreqBuf;
  short int * batteryState;
  unsigned short int * mode;
  unsigned short int * scroll;
  unsigned short int * select;
  unsigned short int * measurementSelection;
};
typedef struct DisplayData DisplayData;


struct WarningAlarmData{
  unsigned int * temperatureRawBuf;
  unsigned int * bloodPressRawBuf;
  unsigned int * pulseRateRawBuf;
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
  unsigned int * tempCorrectedBuf;
  unsigned int * bloodPressCorrectedBuf;
  unsigned int * prCorrectedBuf;
};
typedef struct CommunicationsData CommunicationsData;

struct SchedulerData{	
  int * globalCounter;
  unsigned short int * addCompute;
  unsigned short int * addCommunications;
};
typedef struct SchedulerData SchedulerData;

