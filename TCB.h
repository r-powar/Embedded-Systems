struct MyStruct{
  void (*myTask)(void*);
  void* taskDataPtr;
  struct MyStruct *next;
  struct MyStruct *prev;
};

//TCB struct decl that hold method ref and data struct ref
typedef struct MyStruct TCB;

//declare the method that will simulate measurements
void Measure(void * measureDataPtr);
//declare the method that will compute corrected values
void Compute(void * voidComputeDataPtr);
//declare the method that will manage the display
void Display(void * voidDisplayDataPtr);
//declare the method that will check for warnings
void WarningAlarm(void * voidWarningAlarmDataPtr);
//declare the method that will decrement battery life
void StatusMethod(void * voidStatusPtr);
//declare KeyPad Method
void Keypad(void * voidKeypadDataPtr);
//declare Communications Method
void Communications(void * voidCommunicationsDataPtr);