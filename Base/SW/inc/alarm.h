#ifndef ALARM_H
#define ALARM_H

#include "protocol.h"

void Alarm_Init();
void Alarm_Task(void *p);
BaseType_t Alarm_SendMsg(Action_t event, uint32_t a1, uint32_t a2);

#endif /* ALARM_H */