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

#define MIN_OPERATING_VOLTAGE   (10000) /* mV */
#define MAX_CRANKING_TIME       (10000) /* mS */

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

static void OnStartCmd(EngineMessage_t *msg);
static void OnStartAllowed();
static void OnWarmUp();
static void OnStartCranking();
static BaseType_t GetTimeout();
static void OnInStateChanged();
static void CheckIfRunning();
static void OnTimeout();
static void OnStopCmd(EngineMessage_t *msg);

static uint32_t repetitions;
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
    3. Check for minimal voltage - if it is less than allowed, then break.
    4. Check for engine temp and calculate (Repetitions);
    5. Partially deactivate alarm (shock, ign, lock);
    6. Play appropriate pattern (?)
    7. Turn on ignition;
    8. Wait for plugs are turned on for 1 sec. If they are not turned on - 
       shut down and report about failure;
    9. Wait for plugs are turned off for 1 minute. If they are not turned off -
       shut down and report about failure;
    10. Turn off ignition;
    11. Wait for 1 sec.
    12. Turn it on again;
    13. Repeat step 7-12 for (Repetitions) times;
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
                OnStartCmd(&msg);
                break;
            case ENGINE_START_ALLOWED:
                OnStartAllowed();
                break;
            case ENGINE_IN_STATE_CHANGED:
                OnInStateChanged(&msg);
                break;
            case ACTION_STOP:
                OnStopCmd(&msg);
                break;
            }
        } else {
            switch(engine_state) {
            case ENGINE_STATE_PAUSE_II:
                // 12. Turn them on again;
                OnWarmUp();
                break;
            case ENGINE_STATE_CRANKING:
                CheckIfRunning();
                break;
            case ENGINE_STATE_STOPPED:
            case ENGINE_STATE_RUNNING:
                _voltage = Measurement_GetVoltage();
                _rpm = Measurement_GetRPM();
                break;
            default:
                OnTimeout();
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

static void OnStartCmd(EngineMessage_t *msg) {
    uint32_t voltage;
//        I. ENGINE START
//
//    1. Command received;
//    2. Check if the engine is already running. If so, then break.
    if(engine_state != ENGINE_STATE_STOPPED)
        return;
//    4. Check for minimal voltage - if it is less than allowed then break.
    voltage = Measurement_GetVoltage();
    if(voltage < MIN_OPERATING_VOLTAGE) {
        // TODO: Notify the user about low voltage
        return;
    }
//    5. Check for engine temp and calculate (Repetitions);
    repetitions = 3;
//    2. Partially deactivate alarm (shock, ign, lock);
    Alarm_SendMsg(ALARM_ENGINE_START_REQ, 0, 0);

    engine_state = ENGINE_STATE_ALARM_WAIT;
    return;
}

static void OnStartAllowed() {
//    6. Play appropriate pattern (?)
    // TODO: Do this.
    OnWarmUp();
}
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

static void OnWarmUp() {
    // 3. Turn on ignition;
    Inout_Set(OUT_IGN);
    // 7. Wait for plugs are turned on for 1 sec. If they are not turned on - go to 12.
    engine_state = ENGINE_STATE_PAUSE_I;
    // 8. Wait for plugs are turned off;
}

static void OnGlowPlugsTurnedOn() {
    engine_state = ENGINE_STATE_PREHEAT;
}

static void OnGlowPlugsTurnedOff() {
    repetitions--;
    if(repetitions) {
        // 10. Turn off ignition;
        Inout_Clear(OUT_IGN);
        // 11. Wait for 1 sec.
        engine_state = ENGINE_STATE_PAUSE_II;
    } else
        OnStartCranking();
}

static void OnStartCranking() {
    cranking_start = xTaskGetTickCount();
    Inout_Set(OUT_START);
    engine_state = ENGINE_STATE_CRANKING;
}

static void OnInStateChanged(EngineMessage_t *msg) {
    uint32_t changes = msg->a1 ^ msg->a2;
    uint32_t now = msg->a1;
    
    switch(engine_state) {
    case ENGINE_STATE_PAUSE_I:
        if((changes == IN_GLOW) && (now & IN_GLOW))
            OnGlowPlugsTurnedOn();
        break;
    case ENGINE_STATE_PREHEAT:
        if((changes == IN_GLOW) && !(now & IN_GLOW))
            OnGlowPlugsTurnedOff();
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

static void OnTimeout() {
    Inout_Clear(OUT_START | OUT_IGN);
    // TODO: Notify the user about timeout at the certain state.
    engine_state = ENGINE_STATE_STOPPED;
}
FunctionalState Engine_GetState() {
    return engine_state == ENGINE_STATE_STOPPED?DISABLE:ENABLE;
}

static void OnStopCmd(EngineMessage_t *msg) {
    Inout_Clear(OUT_IGN);
    engine_state = ENGINE_STATE_STOPPED;
    return;
}
