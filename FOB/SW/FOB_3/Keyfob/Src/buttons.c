#include "buttons.h"
#include "FreeRTOS.h"
#include "protocol.h"
#include "power.h"
#include "ui.h"

#if defined (__GNUC__)
#include "iar_to_gcc.h"
#endif

#define BUTTONS_MASK    (0x0000040f)

typedef struct {
    uint32_t prevButtons;
    uint32_t Diff;
    uint32_t State;
} ButtonsData_t;

static __no_init ButtonsData_t Data;

void Buttons_Init() {
    if(Power_IfPOR()) {
        Data.prevButtons = 0;
        Data.Diff = 0;
        Data.State = 0;
    }
}

void Buttons_Suspend() {
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_0);
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_1);
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_2);
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_3);
    LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_10);
}

void Buttons_Activate() {
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_0);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_1);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_2);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_3);
    LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_10);
}

void Buttons_IRQHandler() {
    static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

//    Buttons_Suspend();

    uint32_t buttons = (~GPIOA->IDR) & BUTTONS_MASK;

//    LL_IWDG_ReloadCounter(IWDG);
//    if(buttons != prevButtons) {
//        state = buttons;
//        diff = (buttons ^ prevButtons) & buttons;
//        prevButtons = buttons;
//        if(diff) {
//            UI_NotifyFromISR(UI_EVENT_BUTTON, state, diff, &xHigherPriorityTaskWoken);
            UI_NotifyFromISR(UI_EVENT_BUTTON, 0, buttons, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
//        }
//    }
}
