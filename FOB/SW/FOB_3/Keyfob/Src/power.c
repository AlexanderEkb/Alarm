#include "FreeRTOS.h"
#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_iwdg.h"
#include "protocol.h"
#include "power.h"
#include "radio.h"

#if defined (__GNUC__)
#include "iar_to_gcc.h"
#endif

#define CHECKSUM_SHIFT          (0x1234)  /* Any random value */
#define IWDG_DEFAULT_PRESCALER  (LL_IWDG_PRESCALER_16)
#define IWDG_DEFAULT_PERIOD     (2500)
#define IWDG_SLOWED_PRESCALER   (LL_IWDG_PRESCALER_256)
#define IWDG_SLOWED_PERIOD      (4094)
/*
    TODO: Default IWDG period is one second;
    When radio link fails, it is increased to 25 seconds, to save power.
    But when UI is active, it is set to 1 second, despite the radio link state.
*/

static uint32_t lock;
static uint32_t ifPOR = 0;

static __no_init struct {
    uint32_t Prescaler;
    uint32_t Period;
    uint32_t Checksum;
} PowerSettings;

void Power_GoToSleep();

void Power_Init() {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
#ifdef DEBUG
    // Enable DBGMCU peripheral
    RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
    // Enable debug support in STOP mode
    DBGMCU->CR |= DBGMCU_CR_DBG_STOP;
    // Suspend IWDG counter at breakpoint
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#else
    // Enable DBGMCU peripheral
    RCC->APB2ENR &= ~RCC_APB2ENR_DBGMCUEN;
#endif
    // Enable deep sleep
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Switch from STANDBY to STOP
    PWR->CR &= ~PWR_CR_PDDS;
    // Put voltage regulator to LP mode when stopped
    PWR->CR |= PWR_CR_LPDS;

    // Check if it was POR reset
    uint32_t csr = RCC->CSR | RCC_CSR_RMVF;
    RCC->CSR = csr ;

    ifPOR = (csr & (RCC_CSR_SFTRSTF | RCC_CSR_PORRSTF))?1:0;
    if(ifPOR)
        lock = 0; // Two tasks may prevent MCU from power down
    else
        lock = 0;
}

void Power_OnIdle() {
    if(!lock) {
        Power_GoToSleep();
    } else {
        LL_IWDG_ReloadCounter(IWDG);
    }
}

void Power_Unlock() {
    if(lock)
        lock--;
}

void Power_Lock() {
    lock++;
}

void Power_GoToSleep() {
    GPIOA->MODER = 0x6945A800;
    GPIOA->BRR = 0x9B00;
    GPIOB->MODER = 0x55555555;
    GPIOB->BRR = 0xfffffffc;

    EXTI->IMR = 0x0000041f;
    EXTI->PR = 0xffffffff;

    LL_TIM_DisableIT_UPDATE(TIM14);
    LL_TIM_ClearFlag_UPDATE(TIM14);
    NVIC_DisableIRQ(TIM14_IRQn);
    NVIC_ClearPendingIRQ(TIM14_IRQn);

    __WFI();
}

void Power_OnWakeup() {
    GPIOA->MODER = 0x2800A800;
    GPIOB->MODER = 0x5555A555;

    LL_TIM_EnableIT_UPDATE(TIM14);
    NVIC_EnableIRQ(TIM14_IRQn);
}

uint32_t Power_IfPOR() {
    return ifPOR;
}

void Power_InitSettings(uint32_t prescaler, uint32_t period) {
    uint32_t cs =
        PowerSettings.Checksum +
        PowerSettings.Period +
        PowerSettings.Prescaler +
        CHECKSUM_SHIFT;

    if(cs) {
        PowerSettings.Prescaler = prescaler?prescaler:IWDG_DEFAULT_PRESCALER;
        PowerSettings.Period = period?period:IWDG_DEFAULT_PERIOD;
    } else {
        PowerSettings.Prescaler = prescaler?prescaler:PowerSettings.Prescaler;
        PowerSettings.Period = period?period:PowerSettings.Period;
    }

    cs = 0 - (
        PowerSettings.Period +
        PowerSettings.Prescaler +
        CHECKSUM_SHIFT);

    PowerSettings.Checksum = cs;
}

void Power_InitIWDG() {
//    if((UI_IsActive() != SUCCESS) && (Radio_GetLinkStatus() != SUCCESS)) {
//        PowerSettings.Prescaler = IWDG_SLOWED_PRESCALER;
//        PowerSettings.Period = IWDG_SLOWED_PERIOD;
//    } else {
//        PowerSettings.Prescaler = IWDG_DEFAULT_PRESCALER;
//        PowerSettings.Period = IWDG_DEFAULT_PERIOD;
//    }

    LL_IWDG_Enable(IWDG);
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, PowerSettings.Prescaler);
    LL_IWDG_SetWindow(IWDG, 4095);
    LL_IWDG_SetReloadCounter(IWDG, PowerSettings.Period);
    while (LL_IWDG_IsReady(IWDG) != 1);
    LL_IWDG_ReloadCounter(IWDG);
}
