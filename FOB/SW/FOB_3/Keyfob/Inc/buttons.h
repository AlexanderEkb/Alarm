#ifndef BUTTONS_H
#define BUTTONS_H

#define BUTTON_LEFT     (0x00000008)
#define BUTTON_UP       (0x00000004)
#define BUTTON_RIGHT    (0x00000002)
#define BUTTON_DOWN     (0x00000001)
#define BUTTON_PUSH     (0x00000400)

void Buttons_Init();
void Buttons_Suspend();
void Buttons_Activate();
void Buttons_IRQHandler();

#endif /* BUTTONS_H */
