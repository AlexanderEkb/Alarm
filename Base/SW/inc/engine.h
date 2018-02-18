#ifndef ENGINE_H
#define ENGINE_H

#include "protocol.h"

//typedef enum {
//    ENGINE_CMD_START,
//    ENGINE_CMD_STOP,
//
//    ENGINE_ALARM_PARTIALLY_DISARMED,
//    ENGINE_ALARM_ARMED,
//
//    ENGINE_IN_STATE_CHANGED,
//    ENGINE_PLUGS_ACTIVATED,
//    ENGINE_PLUGS_DEACTIVATED,
//    ENGINE_RPM_ARE_HIGH,
//    ENGINE_RPM_ARE_LOW
//} EngineEvent_t;

void Engine_Init();
void Engine_Task(void *p);
BaseType_t Engine_SendMsg(Action_t event, uint32_t a1, uint32_t a2);
FunctionalState Engine_GetState();

#endif /* ENGINE_H */
