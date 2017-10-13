struct MeasureData{
  int * temperatureRaw;
  int * systolicPressRaw;
  int * diastolicPressRaw;
  int * pulseRateRaw;
};
typedef struct MeasureData MeasureData;

struct ComputeData{
  int * temperatureRaw;
  int * systolicPressRaw;
  int * diastolicPressRaw;
  int * pulseRateRaw;
  int * tempCorrected;
  int * sysPressCorrected;
  int * diasPressCorrected;
  int * prCorrected;
};
typedef struct ComputeData ComputeData;

struct DisplayData{
  int * tempCorrected;
  int * sysPressCorrected;
  int * diasPressCorrected;
  int * prCorrected;
  short int * batteryState;
};
typedef struct DisplayData DisplayData;


struct WarningAlarmData{
  int * temperatureRaw;
  int * systolicPressRaw;
  int * diastolicPressRaw;
  int * pulseRateRaw;
  short int * batteryState;
};
typedef struct WarningAlarmData WarningAlarmData;

struct Status{
  short int * batteryState;	
};
typedef struct Status Status;

struct SchedulerData{	
  int * globalCounter;
};
typedef struct SchedulerData SchedulerData;

