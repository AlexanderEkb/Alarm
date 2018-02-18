#include "stm32f0xx.h"
#include "stm32f0xx_ll_tim.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"

#include "freertos.h"
#include "task.h"
#include "semphr.h"

#include "power.h"
#include "sound.h"

static SemaphoreHandle_t semphr;

void Sound_Init() {
    LL_GPIO_InitTypeDef GPIO_InitStruct;
    LL_TIM_InitTypeDef TIM_InitStruct;
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct;

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_TIM1);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_8;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    TIM_InitStruct.Prescaler = 40;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 100;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0xff;
    LL_TIM_Init(TIM1, &TIM_InitStruct);

//    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
//    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_ENABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 50;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_LOW;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_HIGH;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);

    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_LOW;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);

    LL_TIM_EnableCounter(TIM1);

    semphr = xSemaphoreCreateBinary();
}

void Sound_Task(void const * argument) {
    while(1) {
        if(xSemaphoreTake(semphr, portMAX_DELAY) == pdTRUE) {
            Power_Lock();
            Sound_Init();
            TIM1->BDTR = 0xec00;
            vTaskDelay(100);
            Power_Unlock();
        }
    }
}

void Sound_Beep() {
    xSemaphoreGive(semphr);
}