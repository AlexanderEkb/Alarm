#ifndef INOUT_H
#define INOUT_H

//////////////////////////// Car interface outputs /////////////////////////////
#define OUT_IGN     (0x0001)
#define OUT_START   (0x0002)
#define OUT_LOCK    (0x0004)
#define OUT_ALARM   (0x0008)
#define OUT_CLOSE   (0x0010)
#define OUT_OPEN    (0x0020)
#define OUT_HEATER  (0x0040)
#define OUT_LIGHTS  (0x0080)
#define OUT_LED     (0x0100)

///////////////////////////// Car interface inputs /////////////////////////////
#define IN_EXT_220  (0x01)
#define IN_SHOCK_HI (0x02)
#define IN_SHOCK_LO (0x04)
#define IN_GLOW     (0x10)
#define IN_IGN      (0x20)
#define IN_PARKING  (0x40)
#define IN_DOORS    (0x80)

#define ALARM_MASK  (IN_SHOCK_HI | IN_SHOCK_LO | IN_GLOW | IN_IGN | IN_DOORS)

void Inout_Init();

/////////////////////////// EVERYTHING ABOUT INPUTS ////////////////////////////
typedef enum {INACTIVE = 0, ACTIVE} InputState_TypeDef;

/////////////////////////// EVERYTHING ABOUT OUTPUTS ///////////////////////////
typedef enum {SIG_OPEN, SIG_CLOSE, SIG_WARNING, SIG_PANIC, SIG_PRERUN} Signals_TypeDef;

uint32_t Inout_Get();
void Inout_Set(uint32_t d);
void Inout_Clear(uint32_t d);
void Inout_PlaySignal(Signals_TypeDef s);

void Inout_Task(void *p);


#endif /* INOUT_H */
