#include <stdio.h>
#include <stdbool.h>
#include "inc/lm3s8962.h"
#include "ProjectStructs.h"
#include "TCB.h"
#include "schedule.h"

void SetUpTaskQueue(TaskQueue queue){
  queue->head = null;
  queue->tail = null;
  queue->totalTasks = 0;
}

