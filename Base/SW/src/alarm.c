#include <stm32f1xx.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "alarm.h"
#include "inout.h"
#include "engine.h"
#include "protocol.h"

#define QUEUE_LENGTH    (16)
#define DEFAULT_TIMEOUT (10)

#define DEFAULT_STACK_DEPTH     (configMINIMAL_STACK_SIZE * 2)

#define GUARD               (IN_SHOCK_HI | IN_SHOCK_LO | IN_IGN | IN_DOORS)
#define GUARD_WHEN_RUNNING  (IN_DOORS)
#define SET_WHEN_ARMED      (OUT_CLOSE | OUT_LOCK)
#define SET_WHEN_PART_ARMED (OUT_CLOSE)
typedef struct {
    Action_t event;
    uint32_t a1;
    uint32_t a2;
} AlarmMessage_t;

static QueueHandle_t queue;
static AlarmState_t state;
static uint32_t tracked_inputs;

static ErrorStatus Arm(AlarmMessage_t *msg);
static ErrorStatus FullyDisarm(AlarmMessage_t *msg);
static ErrorStatus Panic(AlarmMessage_t *msg);
static ErrorStatus ShutUp(AlarmMessage_t *msg);
static ErrorStatus OnEngineStart(AlarmMessage_t *msg);
static ErrorStatus OnEngineStop(AlarmMessage_t *msg);
static ErrorStatus OnInStateChanged(AlarmMessage_t *msg);

void Alarm_Init() {
    xTaskHandle foo;

    state = STATE_DISARMED;
    queue = xQueueCreate(QUEUE_LENGTH, sizeof(AlarmMessage_t));
    xTaskCreate(&Alarm_Task, "ALARM", DEFAULT_STACK_DEPTH, NULL, 5, &foo);
}

BaseType_t Alarm_SendMsg(Action_t event, uint32_t a1, uint32_t a2) {
    AlarmMessage_t msg;

    msg.event = event;
    msg.a1 = a1;
    msg.a2 = a2;

    return xQueueSend(queue, &msg, DEFAULT_TIMEOUT);
}

void Alarm_Task(void *p) {
    AlarmMessage_t msg;

    while(1)
        if(xQueueReceive(queue, &msg, portMAX_DELAY) == pdTRUE)
            switch(msg.event) {
            case ACTION_ARM:
                Arm(&msg);
                break;
            case ACTION_DISARM:
                FullyDisarm(&msg);
                break;
            case ALARM_CMD_PANIC:
                Panic(&msg);
                break;
            case ALARM_CMD_SHUT_UP:
                ShutUp(&msg);
                break;
            case ALARM_IN_STATE_CHANGED:
                OnInStateChanged(&msg);
                break;
            case ALARM_ENGINE_START_REQ:
                OnEngineStart(&msg);
                break;
            case ALARM_ENGINE_STOPPED:
                OnEngineStop(&msg);
                break;
            }
}

static ErrorStatus Arm(AlarmMessage_t *msg) {
    uint32_t inputs;
    uint32_t outputs;
    FunctionalState engine = Engine_GetState();

    switch (state) {
    case STATE_DISARMED:
        Inout_PlaySignal(SIG_CLOSE);
        // And continue...
    case STATE_PRE_ARMED:
        state = STATE_PRE_ARMED;
        if(Engine_GetState() == ENABLE) {
            tracked_inputs = GUARD_WHEN_RUNNING;
            outputs = SET_WHEN_PART_ARMED;
        } else {
            tracked_inputs = GUARD;
            outputs = SET_WHEN_ARMED;
        }
        inputs = Inout_Get();
        if(!(inputs & tracked_inputs)) {
            state = STATE_ARMED;
            Inout_Set(outputs);
        }
        break;
    case STATE_PANIC:
        state = STATE_ARMED;
        Inout_PlaySignal(SIG_WARNING);
        break;
    }
    return SUCCESS;
}

static ErrorStatus FullyDisarm(AlarmMessage_t *msg) {
    switch (state) {
    case STATE_PRE_ARMED:
    case STATE_ARMED:
        Inout_PlaySignal(SIG_OPEN);
        state = STATE_DISARMED;
        Inout_Clear(SET_WHEN_ARMED);
        Inout_Set(OUT_OPEN);
        break;
    case STATE_PANIC:
        state = STATE_ARMED;
        Inout_PlaySignal(SIG_WARNING);
        break;
    }
    return SUCCESS;
}

static ErrorStatus Panic(AlarmMessage_t *msg) {
    Inout_PlaySignal(SIG_PANIC);
    state = STATE_PANIC;
    return SUCCESS;
}

static ErrorStatus ShutUp(AlarmMessage_t *msg) {
    return SUCCESS;
}

static ErrorStatus OnInStateChanged(AlarmMessage_t *msg) {
    uint32_t inputs = msg->a1;
    switch (state) {
    case STATE_PRE_ARMED:
        Arm(msg);
        break;
    case STATE_ARMED:
        if(inputs & tracked_inputs)
            Alarm_SendMsg(ALARM_CMD_PANIC, 0, 0);
    }
    return SUCCESS;
}

static ErrorStatus OnEngineStart(AlarmMessage_t *msg) {
    if((state == STATE_ARMED) || (state == STATE_PRE_ARMED))
        tracked_inputs = GUARD_WHEN_RUNNING;
    Engine_SendMsg(ENGINE_START_ALLOWED, 0, 0);
    return SUCCESS;
}

static ErrorStatus OnEngineStop(AlarmMessage_t *msg) {
    if((state == STATE_ARMED) || (state == STATE_PRE_ARMED))
        tracked_inputs = GUARD;
    return SUCCESS;
}
