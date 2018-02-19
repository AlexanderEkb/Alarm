#ifndef PROTOCOL_H
#define PROTOCOL_H

#if defined ( __GNUC__ )

#define READONLY static const
#define PACKED __attribute__ ((packed))
#define READONLY_PACKED static const __attribute__ ((packed))
#define NOINIT __attribute__ ((section (".noinit")))
#define WEAK __attribute__ ((weak))

#else

#define READONLY static const __ro_placement
#define PACKED __packed
#define READONLY_PACKED static const __ro_placement __packed
#define NOINIT __no_init
#define WEAK __weak

#endif


typedef enum {
    STATE_PRE_ARMED = 1,
    STATE_ARMED = 2,
    STATE_DISARMED = 3,
    STATE_PANIC = 4
} AlarmState_t;

/*******************************************************************************
    f e d c b a 9 8 7 6 5 4 3 2 1 0     Status word
    x x x x x x x x x x x x x x x x     ~~~~~~~~~~~
    | | | | | | | | | | | | | | | |
    | | | | | | | | | | | | | | | +---- "Armed"
    | | | | | | | | | | | | | | |
    | | | | | | | | | | | | | | +------ !!! Obsolete !!!
    | | | | | | | | | | | | | |
    | | | | | | | | | | | | | |
    | | | | | | | | | | | | | +-------- "Doors"
    | | | | | | | | | | | | |
    | | | | | | | | | | | | +---------- "Ignition"
    | | | | | | | | | | | |
    | | | | | | | | | | | +------------ "Large shock"
    | | | | | | | | | | |
    | | | | | | | | | | +-------------- "Small shock"
    | | | | | | | | | |
    | | | | | | | | | +---------------- "Glow". Set when glow plugs are on.
    | | | | | | | | |
    | | | | | | | | +------------------ "Brake changed". Set when brake/selector
    | | | | | | | |                     changes its state.
    | | | | | | | |
    | | | | | | | +-------------------- "Alarm counter ". Zero when no alarms
    | | | | | | |                       happened until disarm, becomes non-zero
    | | | | | | +---------------------- and changes at any alarm.
*******************************************************************************/

#define STATUS_ARMED                ((uint16_t) 0x0001)
#define STATUS_PANIC                ((uint16_t) 0x0002)
#define STATUS_DOORS                ((uint16_t) 0x0004)
#define STATUS_IGN                  ((uint16_t) 0x0008)
#define STATUS_HI_SHOCK             ((uint16_t) 0x0010)
#define STATUS_LO_SHOCK             ((uint16_t) 0x0020)
#define STATUS_GLOW                 ((uint16_t) 0x0040)
#define STATUS_BRAKE_CHANGED        ((uint16_t) 0x0080)
#define STATUS_AL_COUNT_POS         (8)
#define STATUS_AL_COUNT             ((uint16_t)(0x0003 << STATUS_AL_COUNT_POS))

#define RESPONCE_STATE_ARMED       ((uint8_t)0x80)
#define RESPONCE_STATE_STARTED     ((uint8_t)0x40)
#define RESPONCE_STATE_PANIC       ((uint8_t)0x20)
#define RESPONCE_STATE_DOORS       ((uint8_t)0x10)
#define RESPONCE_STATE_IGNITION    ((uint8_t)0x08)
#define RESPONCE_STATE_KNOCK_LO    ((uint8_t)0x04)
#define RESPONCE_STATE_KNOCK_HI    ((uint8_t)0x02)

#define TARGET_MASK     0xc0
#define TARGET_ALARM    0x00
#define TARGET_ENGINE   0x40
typedef enum {
    ACTION_NOTHING =                0x00,
    ACTION_ARM =                    0x01,
    ACTION_DISARM =                 0x02,
    ALARM_CMD_PANIC =               0x04,
    ALARM_CMD_SHUT_UP =             0x05,
    ALARM_ENGINE_START_REQ =        0x06,
    ALARM_ENGINE_STOPPED =          0x07,
    ALARM_IN_STATE_CHANGED =        0x08,

    ENGINE_START_ALLOWED =          0x41,
    ACTION_START =                  0x43,
    ACTION_STOP =                   0x44,
    ACTION_ENABLE_TEMP_START =      0x45,
    ACTION_ENABLE_TIME_START =      0x46,
    ENGINE_IN_STATE_CHANGED =       0x47,
    ENGINE_VOLTAGE_UP =             0x48,
    ENGINE_VOLTAGE_DOWN =           0x49,
    ENGINE_RPM_UP =                 0x4A,
    ENGINE_RPM_DOWN =               0x4B,

} Action_t;

typedef PACKED struct {
    uint16_t Status;
    uint16_t Temp;
    uint16_t RPM;
    uint8_t PanicReason;
    uint8_t _Reserved[9];
} DefaultResponse_t;

#endif /* PROTOCOL_H */
