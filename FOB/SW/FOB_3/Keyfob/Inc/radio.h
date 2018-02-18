#ifndef RADIO_H
#define RADIO_H

void Radio_Init();
void Radio_Task(void const * argument);
ErrorStatus Radio_GetLinkStatus();
void Radio_Notify(Action_t action, uint32_t param0, uint32_t param1);

#endif /* RADIO_H */
