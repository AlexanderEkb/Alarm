#include "manager.h"
#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "inout.h"
#include "alarm.h"
#include "engine.h"
#include "storage.h"
#include "modem.h"
#include "radio.h"
#include "measurements.h"

#define DEFAULT_STACK_DEPTH     (configMINIMAL_STACK_SIZE * 2)

void SetupMasterClock();
void xPortSysTickHandler();

void SysTick_Handler(void)
{
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
#endif  /* INCLUDE_xTaskGetSchedulerState */
        xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState  == 1 )
    }
#endif  /* INCLUDE_xTaskGetSchedulerState */
}

int main()
{
    TaskHandle_t foo;
    SetupMasterClock();

    Inout_Init();
    Alarm_Init();
    Engine_Init();
    Storage_Init();
    Radio_Init();
    Modem_Init();
    Measurement_Init();

    xTaskCreate(&Inout_Task, "INOUT", DEFAULT_STACK_DEPTH, NULL, 5, &foo);
    xTaskCreate(&Alarm_Task, "ALARM", DEFAULT_STACK_DEPTH, NULL, 4, &foo);
    xTaskCreate(&Engine_Task, "ENGINE", DEFAULT_STACK_DEPTH, NULL, 3, &foo);
    xTaskCreate(&Radio_Task, "RADIO", DEFAULT_STACK_DEPTH / 2, NULL, 5, &foo);
    //xTaskCreate(&Modem_Task, "MODEM", DEFAULT_STACK_DEPTH / 2, NULL, 5, &foo);

    vTaskStartScheduler();

    while(1);
}

void SetupMasterClock() {
    static uint32_t foo, bar;

    foo = FLASH->ACR;
    foo &= ~FLASH_ACR_LATENCY;
    foo |= FLASH_ACR_LATENCY_2;
    FLASH->ACR = foo;

    foo = RCC->CR;

    RCC->CR |= RCC_CR_HSEON;
    do
        foo = RCC->CR;
    while(!(foo & RCC_CR_HSERDY));

    foo = RCC_CFGR_MCO_NOCLOCK | RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC |
       RCC_CFGR_ADCPRE_DIV2 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_PPRE1_DIV2 |
           RCC_CFGR_HPRE_DIV1;

    RCC->CFGR = foo;

    RCC->CR |= RCC_CR_PLLON;
    do
        bar = RCC->CR;
    while(!(bar & RCC_CR_PLLRDY));

    foo |= RCC_CFGR_SW_PLL;
    RCC->CFGR = foo;

    do
        foo = RCC->CFGR;
    while((foo & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}
