#ifndef MANAGER_H
#define MANAGER_H

#include "stdint.h"

//#define STATE_MASK          (0x07)

#define DEVICE_RESET        (0x00010000)
#define STORAGE_CS_FAILED   (0x00020000)
#define MODEM_ERR           (0x00040000)
#define TRX_ERR             (0x00080000)

typedef enum {
    STARTUP = 0x00,
    POWER_FAILURE = 0x01,
    MESSAGE_RCVD = 0x02,
    INPUTS_CHANGED = 0x03,
    CMD_ARM = 0x04,
    CMD_DISARM = 0x05,
    CMD_START = 0x06,
    CMD_STOP = 0x07
} EventType_t;

typedef struct {
    EventType_t Event;
    uint32_t Param1;
    uint32_t Param2;
} Event_t;

void Manager_Start();
void Manager_Notify(EventType_t e, uint32_t p1, uint32_t p2);

#ifdef DEBUG
#define TRAP(x) {Manager_Trap(x);}
void Manager_Trap(char *r);
#else
#define TRAP(x)
#endif

#endif /* MANAGER_H */
