#include "stm32f0xx.h"
#include "freertos.h"
#include "semphr.h"
#include "task.h"
#include "dispatcher.h"
#include "protocol.h"
#include "radio.h"
#include "ui.h"

static SemaphoreHandle_t allDone;

void Dispatcher_Task(void const * argument) {
    UI_Event_t UI_Event = UI_EVENT_NONE;
    allDone = xSemaphoreCreateBinary();

    while(1) {
        taskYIELD();
    }
}

void Dispatcher_Unlock() {
    xSemaphoreGive(allDone);
}
