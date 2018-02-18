#ifndef POWER_H
#define POWER_H

#define RADIO     (0x01)
#define DISPLAY   (0x02)

#define SLEEP_IS_COMPLETELY_ENABLED (0x03)

void Power_Init();
void Power_OnIdle();
void Power_Lock();
void Power_Unlock();
void Power_OnWakeup();
void Power_InitSettings(uint32_t prescaler, uint32_t period);
void Power_InitIWDG();
uint32_t Power_IfPOR();

#endif /* POWER_H */