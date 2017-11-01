#include <stdio.h>
#include <stdbool.h>
#include "inc/lm3s8962.h"
#include "ProjectStructs.h"
#include "TCB.h"

struct TaskQueue{
  TCB head;
  TCB tail;
  int totalTasks;
}

typedef struct TaskQueue taskqueue;

void SetUpTaskQueue(TaskQueue taskqueue);

int addTasks(TaskQueue taskqueue, TCB);

