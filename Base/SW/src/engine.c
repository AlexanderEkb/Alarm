#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "engine.h"
#include "alarm.h"
#include "inout.h"
#include "measurements.h"

#define QUEUE_LENGTH    (16)
#define DEFAULT_TIMEOUT (10)

#define DEFAULT_STACK_DEPTH     (configMINIMAL_STACK_SIZE * 2)

#define TIMEOUT_PAUSE_I         (1000)
#define TIMEOUT_PAUSE_II        (1000)
#define TIMEOUT_PREHEAT_PAUSE   (500)
#define TIMEOUT_WHEN_PREHEAT    (60000)
#define TIMEOUT_WHEN_CRANKING   (500)
#define TIMEOUT_WHEN_RUNNING    (1000)

#define MAX_CRANKING_TIME       (10000)

typedef struct {
    Action_t event;
    uint32_t a1;
    uint32_t a2;
} EngineMessage_t;

typedef enum {
    ENGINE_STATE_STOPPED,
    ENGINE_STATE_ALARM_WAIT,
    ENGINE_STATE_PAUSE_I,
    ENGINE_STATE_PREHEAT,
    ENGINE_STATE_PREHEAT_PAUSE,
    ENGINE_STATE_CRANKING,
    ENGINE_STATE_PAUSE_II,
    ENGINE_STATE_RUNNING
}EngineState_t;

static QueueHandle_t queue;
static EngineState_t engine_state;

static ErrorStatus Engine_Start(EngineMessage_t *msg);
static ErrorStatus Engine_Stop(EngineMessage_t *msg);
static void OnStartAllowed();
static BaseType_t GetTimeout();
static void OnInStateChanged();
static void CheckIfRunning();
static uint32_t repetitions;
static uint32_t glow_counter;
static uint32_t cranking_start;
static uint32_t _voltage;
static uint32_t _rpm;
void Engine_Init() {
    xTaskHandle foo;

    queue = xQueueCreate(QUEUE_LENGTH, sizeof(EngineMessage_t));
    engine_state = ENGINE_STATE_STOPPED;
    xTaskCreate(&Engine_Task, "ENGINE", DEFAULT_STACK_DEPTH, NULL, 5, &foo);
}

/*
        I. ENGINE START

    1. Command received;
    2. Check if the engine is already running. If so, then break.
    4. Check for minimal voltage - if it is less than allowed, then break.
    2. Partially deactivate alarm (shock, ign, lock);
    3. Turn on ignition;
    5. Check for engine temp and calculate (Repetitions);
    6. Play appropriate pattern (?)
    7. Wait for plugs are turned on for 1 sec. If they are not turned on - go to 12.
    8. Wait for plugs are turned off;
    9. Wait for 1 sec.
    10. Turn off ignition;
    11. Wait for 1 sec.
    12. Turn it on again;
    13. Repeat step 8-12 for (Repetitions) times;
    14. Turn on starter;
    15. Wait for RPM are above the programmed value, check for timeout;
    16. If everything is OK - go to 21;
    17. In other case, turn off ignition, fully activate alarm;
    18. Wait for programmed time;
    19. Repeat steps 2-18 for programmed number of times;
    20. If procedure still fails - turn off ignition,
        re-arm the alarm and go to Failure(ATTEMPTS_ARE_OVER);
    21. Send OK to every control device;
    22. Keep engine running until some reason to stop it;
*/

BaseType_t Engine_SendMsg(Action_t event, uint32_t a1, uint32_t a2) {
    EngineMessage_t msg;

    msg.event = event;
    msg.a1 = a1;
    msg.a2 = a2;
    return xQueueSend(queue, &msg, DEFAULT_TIMEOUT);
}

void Engine_Task(void *p) {
    uint32_t timeout;
    EngineMessage_t msg;

    while(1) {
        timeout = GetTimeout();
        if(xQueueReceive(queue, &msg, timeout) == pdTRUE) {
            switch(msg.event) {
            case ACTION_START:
                Engine_Start(&msg);
                // the next step is ENGINE_START_ALLOWED
                break;
            case ENGINE_START_ALLOWED:
                OnStartAllowed();
                // the next step is ENGINE_STATE_PAUSE_I (just pause)
                break;
            case ENGINE_IN_STATE_CHANGED:
                OnInStateChanged(&msg);
                break;
            case ACTION_STOP:
                Engine_Stop(&msg);
                break;
            }
        } else {
            switch(engine_state) {
            case ENGINE_STATE_PAUSE_I:
                engine_state = ENGINE_STATE_PREHEAT; // the next engine_state is ENGINE_STATE_PREHEAT (...)
                break;
            case ENGINE_STATE_PREHEAT:
                Inout_Clear(OUT_IGN);         // Glow plugs are remaining on for more than one minute.
                engine_state = ENGINE_STATE_STOPPED;
                break;
            case ENGINE_STATE_CRANKING:
                CheckIfRunning();
                break;
            case ENGINE_STATE_STOPPED:
            case ENGINE_STATE_RUNNING:
                _voltage = Measurement_GetVoltage();
                _rpm = Measurement_GetRPM();
                break;
            }
        }
    }
}

static BaseType_t GetTimeout() {
    uint32_t timeout;
    
    switch(engine_state) {
    case ENGINE_STATE_STOPPED:
        timeout = 1000;
        break;
    case ENGINE_STATE_ALARM_WAIT:
        timeout = portMAX_DELAY;
        break;
    case ENGINE_STATE_PAUSE_I:
        timeout = TIMEOUT_PAUSE_I;
        break;
    case ENGINE_STATE_PREHEAT:
        timeout = TIMEOUT_WHEN_PREHEAT;
        break;
    case ENGINE_STATE_PREHEAT_PAUSE:
        timeout = TIMEOUT_PREHEAT_PAUSE;
        break;
    case ENGINE_STATE_CRANKING:
        timeout = TIMEOUT_WHEN_CRANKING;
        break;
    case ENGINE_STATE_PAUSE_II:
        timeout = TIMEOUT_PAUSE_II;
        break;
    case ENGINE_STATE_RUNNING:
        timeout = TIMEOUT_WHEN_RUNNING;
        break;
    }

    return timeout;
}

static ErrorStatus Engine_Start(EngineMessage_t *msg) {
//        I. ENGINE START
//
//    1. Command received;
//    2. Check if the engine is already running. If so, then break.
    if(engine_state != ENGINE_STATE_STOPPED)
        return SUCCESS;
//    2. Partially deactivate alarm (shock, ign, lock);
    Alarm_SendMsg(ALARM_ENGINE_START_REQ, 0, 0);

    engine_state = ENGINE_STATE_ALARM_WAIT;
    return SUCCESS;
}

static void OnStartAllowed() {
    uint32_t voltage;
//    3. Turn on ignition;
    Inout_Set(OUT_IGN);
    vTaskDelay(50);
//    4. Check for minimal voltage - if it is less than allowed -
//       turn off ignition, re-arm the alarm  and go to Failure(MIN_VOLT);
    voltage = Measurement_GetVoltage();
    if(voltage < 10000) {
        engine_state = ENGINE_STATE_STOPPED;
        Inout_Clear(OUT_IGN);
        Alarm_SendMsg(ALARM_ENGINE_STOPPED, 0, 0);
        return;
    }
//    5. Check for engine temp and calculate (Repetitions);
    repetitions = 3;
//    6. Play appropriate pattern (?)
//    7. Wait for plugs are turned on for 1 sec. If they are not turned on - go to 12.
    engine_state = ENGINE_STATE_PAUSE_I;
//    8. Wait for plugs are turned off;
}
//    9. Wait for 1 sec.
//    10. Turn off ignition;
//    11. Wait for 1 sec.
//    12. Turn it on again;
//    13. Repeat step 8-12 for (Repetitions) times;
//    14. Turn on starter;
//    15. Wait for RPM are above the programmed value, check for timeout;
//    16. If everything is OK - go to 21;
//    17. In other case, turn off ignition, fully activate alarm;
//    18. Wait for programmed time;
//    19. Repeat steps 2-18 for programmed number of times;
//    20. If procedure still fails - turn off ignition,
//        re-arm the alarm and go to Failure(ATTEMPTS_ARE_OVER);
//    21. Send OK to every control device;
//    22. Keep engine running until some reason to stop it;

static void OnInStateChanged(EngineMessage_t *msg) {
    uint32_t changes = msg->a1 ^ msg->a2;
    uint32_t now = msg->a1;
    
    switch(engine_state) {
    case ENGINE_STATE_PAUSE_I:
        if((changes == IN_GLOW) && (now & IN_GLOW))
            engine_state = ENGINE_STATE_PREHEAT;
        break;
    case ENGINE_STATE_PREHEAT:
        if((changes == IN_GLOW) && !(now & IN_GLOW)) {
            cranking_start = xTaskGetTickCount();
            Inout_Set(OUT_START);
            engine_state = ENGINE_STATE_CRANKING;
        }
        break;
    case ENGINE_STATE_PREHEAT_PAUSE:
    case ENGINE_STATE_CRANKING:
    case ENGINE_STATE_PAUSE_II:
    case ENGINE_STATE_RUNNING:
        break;
    }
    if((engine_state == ENGINE_STATE_PREHEAT) && (changes & IN_GLOW)) {
        if(!(now & IN_GLOW))
            engine_state = ENGINE_STATE_CRANKING;
    }
}

static void CheckIfRunning() {
    uint32_t time = xTaskGetTickCount();
    uint32_t rpm = Measurement_GetRPM();
    uint32_t voltage = Measurement_GetVoltage();
    
    if(time - cranking_start >= MAX_CRANKING_TIME) {
        Inout_Clear(OUT_IGN | OUT_START);
        vTaskDelay(10);
        engine_state = ENGINE_STATE_STOPPED;
    } else if((rpm >= 600) || (voltage >= 10500)) {
        Inout_Clear(OUT_START);
        vTaskDelay(10);
        engine_state = ENGINE_STATE_RUNNING;
    }
}

FunctionalState Engine_GetState() {
    return engine_state == ENGINE_STATE_STOPPED?DISABLE:ENABLE;
}

static ErrorStatus Engine_Stop(EngineMessage_t *msg) {
    Inout_Clear(OUT_IGN);
    engine_state = ENGINE_STOPPED;
    return SUCCESS;
}
