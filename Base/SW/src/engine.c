#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "engine.h"
#include "alarm.h"
#include "inout.h"
#include "measurements.h"
#include "protocol.h"

#define QUEUE_LENGTH    (16)
#define DEFAULT_TIMEOUT (10)

#define DEFAULT_STACK_DEPTH     (configMINIMAL_STACK_SIZE * 2)

#define TIMEOUT_PRE_PREHEAT         (1000)
#define PAUSE_BETWEEN_PREHEATS      (1000)
#define TIMEOUT_PREHEAT_PAUSE       (500)
#define TIMEOUT_WHEN_PREHEAT        (60000)
#define MAX_CRANKING_TIME           (10000) /* mS */
#define TIMEOUT_BETWEEN_ATTEMPTS    (10000)
#define DEFAULT_PERIOD              (1000)
#define MAX_ATTEMPTS_COUNT          (3)
#define PREHEAT_DURATION_THRESHOLD  (500)

#define MIN_OPERATING_VOLTAGE       (10000) /* mV */

typedef struct {
    Action_t event;
    uint32_t a1;
    uint32_t a2;
} EngineMessage_t;

static QueueHandle_t queue;
static EngineState_t engine_state;

static void OnStartCmd(EngineMessage_t *msg);
static void OnStartAllowed();
static void OnWarmUp();
static void OnStartCranking();
static void OnEngineStarted();
static BaseType_t GetTimeout();
static void OnInStateChanged();
static uint32_t GlowPlugsAreOn(EngineMessage_t *msg);
static uint32_t GlowPlugsAreOff(EngineMessage_t *msg);
static void OnCrankingTimeout();
static void OnTimeout();
static void OnStopCmd(EngineMessage_t *msg);

static uint32_t repetitions;
static uint32_t preheat_start;
static uint32_t attempts;

void Engine_Init() {
    queue = xQueueCreate(QUEUE_LENGTH, sizeof(EngineMessage_t));
    engine_state = STATE_STOPPED;
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
            case ENGINE_RPM_UP:
                OnEngineStarted();
                break;
            }
        } else {
            switch(engine_state) {
            case STATE_PAUSE_BETWEEN_PREHEATS:
                // 12. Turn them on again;
                OnWarmUp();
                break;
            case STATE_CRANKING:
                OnCrankingTimeout();
                break;
            case STATE_PAUSE_BETWEEN_ATTEMPTS:
                repetitions = 3;
                OnWarmUp();
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
    case STATE_ALARM_WAIT:
        timeout = portMAX_DELAY;
        break;
    case STATE_PRE_PREHEAT:
        timeout = TIMEOUT_PRE_PREHEAT;
        break;
    case STATE_PREHEAT:
        timeout = TIMEOUT_WHEN_PREHEAT;
        break;
    case STATE_PREHEAT_PAUSE:
        timeout = TIMEOUT_PREHEAT_PAUSE;
        break;
    case STATE_CRANKING:
        timeout = MAX_CRANKING_TIME;
        break;
    case STATE_PAUSE_BETWEEN_PREHEATS:
        timeout = PAUSE_BETWEEN_PREHEATS;
        break;
    case STATE_PAUSE_BETWEEN_ATTEMPTS:
        timeout = TIMEOUT_BETWEEN_ATTEMPTS;
        break;
    default:
        timeout = portMAX_DELAY;
    }

    return timeout;
}

static void OnStartCmd(EngineMessage_t *msg) {
    uint32_t voltage;
//        I. ENGINE START
//
//    1. Command received;
//    2. Check if the engine is already running. If so, then break.
    if(engine_state != STATE_STOPPED)
        return;
//    4. Check for minimal voltage - if it is less than allowed then break.
    voltage = Measurement_GetVoltage();
    if(voltage < MIN_OPERATING_VOLTAGE) {
        // TODO: Notify the user about low voltage
        return;
    }
//    5. Check for engine temp and calculate (Repetitions);
    repetitions = 3;
    attempts = 0;
//    2. Partially deactivate alarm (shock, ign, lock);
    Alarm_SendMsg(ALARM_ENGINE_START_REQ, 0, 0);

    engine_state = STATE_ALARM_WAIT;
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
    engine_state = STATE_PRE_PREHEAT;
    // 8. Wait for plugs are turned off;
}

static void OnGlowPlugsTurnedOn() {
    preheat_start = xTaskGetTickCount();
    engine_state = STATE_PREHEAT;
}

static void OnGlowPlugsTurnedOff() {
    uint32_t duration = xTaskGetTickCount() - preheat_start;
    
    if(duration > PREHEAT_DURATION_THRESHOLD)
        repetitions--;
    else
        repetitions = 0;
    if(repetitions) {
        // 10. Turn off ignition;
        Inout_Clear(OUT_IGN);
        // 11. Wait for 1 sec.
        engine_state = STATE_PAUSE_BETWEEN_PREHEATS;
    } else
        OnStartCranking();
}

static void OnStartCranking() {
    Inout_Set(OUT_START);
    engine_state = STATE_CRANKING;
}

static void OnEngineStarted() {
    if(engine_state == STATE_CRANKING) {
        Inout_Clear(OUT_START);
        engine_state = STATE_RUNNING;
    }
}

static void OnCrankingTimeout() {
    Inout_Clear(OUT_IGN | OUT_START);
    attempts++;
    if(attempts < MAX_ATTEMPTS_COUNT) {
        engine_state = STATE_PAUSE_BETWEEN_ATTEMPTS;
    } else {
        engine_state = STATE_STOPPED;
        // TODO: Notify the user about failure;
    }
}

static void OnInStateChanged(EngineMessage_t *msg) {
    uint32_t on = GlowPlugsAreOn(msg);
    uint32_t off = GlowPlugsAreOff(msg);

    if(on) {
        if(engine_state == STATE_PRE_PREHEAT) {
            OnGlowPlugsTurnedOn();
        } else
            on = 0;
    } else if(off && (engine_state == STATE_PREHEAT)) {
        OnGlowPlugsTurnedOff();
    }
}

static void OnTimeout() {
    Inout_Clear(OUT_START | OUT_IGN);
    // TODO: Notify the user about timeout at the certain state.
    engine_state = STATE_STOPPED;
}

FunctionalState Engine_GetState() {
    return engine_state == STATE_STOPPED?DISABLE:ENABLE;
}

static void OnStopCmd(EngineMessage_t *msg) {
    Inout_Clear(OUT_IGN | OUT_START);
    engine_state = STATE_STOPPED;
    return;
}

static uint32_t GlowPlugsAreOn(EngineMessage_t *msg) {
    uint32_t changes = msg->a1 ^ msg->a2;
    uint32_t now = msg->a1;
    
    return ((changes == IN_GLOW) && (now & IN_GLOW))?1:0;
}

static uint32_t GlowPlugsAreOff(EngineMessage_t *msg) {
    uint32_t changes = msg->a1 ^ msg->a2;
    uint32_t now = msg->a1;
    
    return ((changes == IN_GLOW) && !(now & IN_GLOW))?1:0;
}