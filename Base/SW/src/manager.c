#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "manager.h"

#include "protocol.h"

#define QUEUE_DEPTH     16
#define STACK_DEPTH     (configMINIMAL_STACK_SIZE * 2)

#define PANIC_MASK          (IN_SHOCK_LO | IN_SHOCK_HI | IN_GLOW | IN_DOORS | IN_PARKING | IN_IGN)
#define CAUSE_WHEN_SET      (IN_SHOCK_LO | IN_SHOCK_HI | IN_GLOW | IN_DOORS | IN_IGN)
#define CAUSE_WHEN_CHANGED  (IN_PARKING)
#define READY_TO_ARM        (IN_GLOW | IN_DOORS | IN_IGN)

static QueueHandle_t queue;
static TaskHandle_t mgr;

void Manager(void *p);
void WhenArmed(Event_t e);
void WhenDisarmed(Event_t e);

void Manager_Start() {
    queue = xQueueCreate(QUEUE_DEPTH, sizeof(Event_t));

}

void Manager(void *p) {
    uint32_t Status = 0;
    static Event_t event;

    Status |=
    Status |= RC_Init();


    if(!(Status & STORAGE_CS_FAILED) && (Storage_GetStatus() & STATUS_ARMED))
        Manager_Notify(POWER_FAILURE, 0, 0);
    while(1)
        if(xQueueReceive(queue, &event, portMAX_DELAY)) {
            Status = Storage_GetStatus();
            if(Status & STATUS_ARMED)
                WhenArmed(event);
            else
                WhenDisarmed(event);
        }
}

void Manager_Notify(EventType_t e, uint32_t p1, uint32_t p2) {
    Event_t evt;

    evt.Event = e;
    evt.Param1 = p1;
    evt.Param2 = p2;
    xQueueSend(queue, &evt, portMAX_DELAY);
}

void WhenArmed(Event_t e) {
    uint16_t Status = Storage_GetStatus();
    uint16_t Cause = 0;
    uint32_t Count;

    if(e.Event == CMD_ARM) {
        // If arm command received - stop panic
    } else if(e.Event == CMD_DISARM) {
        // If disarm command received - play appropriate signal and disarm
        if(Status & STATUS_AL_COUNT)
            Inout_PlaySignal(SIG_WARNING);
        else
            Inout_PlaySignal(SIG_OPEN);
        Inout_Set(OUT_LOCK | OUT_OPEN);
        Status &= ~STATUS_ARMED;
    } else if(e.Event == CMD_START) {
        // If engine start requested...
    } else if(e.Event == CMD_STOP) {
        // If engine stop requested...
    } else if (e.Event == INPUTS_CHANGED) {
        // If any defendable contour breached...
        // Some inputs leads to panic on every change of its state,
        // and the rest - only when they are set.
        uint32_t zones = ((e.Param1 & CAUSE_WHEN_CHANGED) | (e.Param2 & CAUSE_WHEN_SET));
        if(zones) {
            if(zones & IN_SHOCK_LO)
                Cause |= STATUS_LO_SHOCK;
            if(zones & IN_SHOCK_HI)
                Cause |= STATUS_HI_SHOCK;
            if(zones & IN_GLOW)
                Cause |= STATUS_GLOW;
            if(zones & IN_DOORS)
                Cause |= STATUS_DOORS;
            if(zones & IN_PARKING)
                Cause |= STATUS_BRAKE_CHANGED;
            if(zones & IN_IGN)
                Cause |= STATUS_IGN;
            if(Cause == STATUS_LO_SHOCK)
                Inout_PlaySignal(SIG_WARNING);
            else {
                Count = (Status & STATUS_AL_COUNT);
                Count += 1 << STATUS_AL_COUNT_POS;
                Count &= STATUS_AL_COUNT;
                if(!Count)
                    Count = 1 << STATUS_AL_COUNT_POS;
                Status &= ~STATUS_AL_COUNT;
                Status |= Count;
                Status |= Cause;
                Inout_PlaySignal(SIG_PANIC);
            }
        }

    }

    Storage_SetStatus(Status);
}

void WhenDisarmed(Event_t e) {
    uint16_t Status = Storage_GetStatus();
    uint32_t Inputs = Inout_Get();

    if(e.Event == CMD_ARM) {
        // If arm command received...
        if((Inputs & READY_TO_ARM) == 0) {
            Status &= ~STATUS_AL_COUNT;
            Status |= STATUS_ARMED;
            Inout_Clear(OUT_LOCK);
            Inout_Set(OUT_CLOSE);
            Inout_PlaySignal(SIG_CLOSE);
        } else {
            Inout_PlaySignal(SIG_WARNING);
        }
    } else if(e.Event == CMD_START) {
        // If engine start requested...
    } else if(e.Event == CMD_STOP) {
        // If engine stop requested...
    }
    Storage_SetStatus(Status);
}

#ifdef DEBUG
void Manager_Trap(char *r) {
    volatile static char *Reason;
    Reason = r;
    while(1);
}
#endif
