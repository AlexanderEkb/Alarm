#ifndef UI_H
#define UI_H

#include "FreeRTOS.h"

typedef enum {UI_EVENT_NONE, UI_EVENT_BUTTON, UI_EVENT_RADIO} UI_Event_t;

void UI_Init();
void UI_Task(void const * argument);
void UI_Notify(UI_Event_t event, uint32_t param0, uint32_t param1);
void UI_NotifyFromISR(UI_Event_t event, uint32_t param0, uint32_t param1, portBASE_TYPE *sc);
ErrorStatus UI_IsActive();
void UI_Suspend();
void UI_Resume();

#endif /* UI_H */
