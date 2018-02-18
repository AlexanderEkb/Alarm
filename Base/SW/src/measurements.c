#include <stdint.h>
#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "measurements.h"

#define VREF                (3.3)
#define BUFFER_LENGTH       (16)
#define TEMP_TABLE_LENGTH   (37)
#define MIN_TEMP            (-55)
#define MAX_TEMP            (125)

#define TEETH_NUM           (60)
#define PULLEY_RATIO        (2)
#define RPM_ARR_VALUE       (10000)

#define DEFAULT_STACK_DEPTH     (configMINIMAL_STACK_SIZE * 2)

/*
100...5000 RPM
85 mS ... 100 uS
Tick = 10uS
*/

typedef struct {
    int32_t temp;
    uint32_t r;
} TempRecalc_TypeDef;

typedef struct {
    uint32_t Voltage;
    uint32_t Temp;
} ADC_Data_TypeDef;

static const TempRecalc_TypeDef Table[TEMP_TABLE_LENGTH] = {
    {-55, 772850}, {-50, 549380}, {-45, 395070}, {-40, 287220}, {-35, 210990},
    {-30, 156520}, {-25, 117150}, {-20, 88541}, {-15, 67433}, {-10, 51815},
    {-5, 40099}, {0, 31283}, {5, 24569}, {10, 19438}, {15, 15475},
    {20, 12403}, {25, 10000}, {30, 8110}, {35, 6615}, {40, 5425},
    {45, 4472}, {50, 3707}, {55, 3087}, {60, 2583}, {65, 2171},
    {70, 1832}, {75, 1554}, {80, 1322}, {85, 1130}, {90, 969},
    {95, 835}, {100, 721}, {105, 623}, {110, 544}, {115, 475},
    {120, 416}, {125, 365},
};

static ADC_Data_TypeDef ADC_Data[BUFFER_LENGTH];
static uint32_t RPM_Data[TEETH_NUM];
static uint32_t rpmPtr;

static void _Task(void *p);

void TIM2_IRQHandler() {
    ADC1->SR &= ~ADC_SR_EOC;
    ADC1->CR2 |= ADC_CR2_ADON;
    TIM2->SR &= ~TIM_SR_CC2IF;
}

void TIM3_IRQHandler() {
    uint32_t SR = TIM3->SR;
    uint32_t value;
    if(SR & TIM_SR_CC1IF) {
        value = TIM3->CCR1;
        TIM3->SR &= ~TIM_SR_CC1IF;
        TIM3->CNT = 0;
    } else {
        value = RPM_ARR_VALUE;
        TIM3->SR &= ~TIM_SR_UIF;
    }
    RPM_Data[rpmPtr] = value;
    if(++rpmPtr >= TEETH_NUM)
        rpmPtr = 0;
}

void Measurement_Init() {
    uint32_t i;

    rpmPtr = 0;

    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

    GPIOA->CRL &=
        ~(GPIO_CRL_MODE1_0 | GPIO_CRL_MODE1_1 | GPIO_CRL_MODE2_0 | GPIO_CRL_MODE2_1 |
        GPIO_CRL_CNF1_0 | GPIO_CRL_CNF1_1 | GPIO_CRL_CNF2_0 | GPIO_CRL_CNF2_1);
    // Configure ADC1 to make single conversion on ch.1 and initialize Temp. buffer
    ADC1->CR1 = ADC_CR1_DISCEN;
    ADC1->CR2 = ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2 | ADC_CR2_ADON;
    ADC1->SMPR1 = 0x00;
    ADC1->SMPR2 = ADC_SMPR2_SMP0_0 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_2 |
        ADC_SMPR2_SMP1_0 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_2;
    ADC1->SQR1 = 0x00;
    ADC1->SQR2 = 0x00;
    ADC1->SQR3 = ADC_SQR3_SQ1_0;
    ADC1->SR &= ~ADC_SR_EOC;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while(!(ADC1->SR & ADC_SR_EOC));
    for(i=0;i<BUFFER_LENGTH;i++)
        ADC_Data[i].Temp = ADC1->DR;

    // Configure ADC1 to make single conversion on ch.2 and initialize Voltage buffer
    ADC1->SQR3 = ADC_SQR3_SQ1_1;
    ADC1->SR &= ~ADC_SR_EOC;
    ADC1->CR2 |= ADC_CR2_SWSTART;
    while(!(ADC1->SR & ADC_SR_EOC));
    for(i=0;i<BUFFER_LENGTH;i++)
        ADC_Data[i].Voltage = ADC1->DR;

    // Configure TIM2 to be an external trigger for ADC1.
    // Conversions will be performed every 10 mSec.
    TIM2->CR1 = TIM_CR1_CEN;
    TIM2->CR2 = 0x00;
    TIM2->SMCR = 0x00;
    TIM2->DIER = TIM_DIER_CC2IE;
    TIM2->SR = 0;
    TIM2->CCMR1 = 0x00;
    TIM2->CCMR2 = 0x00;
    TIM2->CCER = TIM_CCER_CC2E;
    TIM2->CNT = 0;
    TIM2->PSC = 11;
    TIM2->ARR = 60000;
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
    TIM2->DCR = 0;
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM2_STOP;
    TIM2->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[TIM2_IRQn / 32] |= (1 << (TIM2_IRQn % 32));
    NVIC->IP[TIM2_IRQn] = (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1) << 4;

    // Configure DMA to automatically transfer measured data to the buffer;
    DMA1_Channel1->CCR = DMA_CCR_PL_0 | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_1 |
        DMA_CCR_MINC | DMA_CCR_CIRC;
    DMA1_Channel1->CNDTR = BUFFER_LENGTH * 2;
    DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
    DMA1_Channel1->CMAR = (uint32_t)&ADC_Data[0];
    DMA1_Channel1->CCR |= DMA_CCR_EN;

    // Configure ADC1 to perform consecutive measurements, when triggered
    // by TIM2. After that, data are transferred to the raw data buffer
    // using DMA.
    ADC1->CR1 = ADC_CR1_SCAN | ADC_CR1_EOCIE;
    ADC1->CR2 = ADC_CR2_EXTTRIG | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_0 |
        ADC_CR2_DMA | ADC_CR2_CONT | ADC_CR2_ADON;
    ADC1->SMPR2 = ADC_SMPR2_SMP1_2 | ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP1_0 |
        ADC_SMPR2_SMP0_2 | ADC_SMPR2_SMP0_1 | ADC_SMPR2_SMP0_0;
    ADC1->SQR1 = ADC_SQR1_L_0;
    ADC1->SQR2 = 0x00;
    ADC1->SQR3 = ADC_SQR3_SQ1_1 | ADC_SQR3_SQ2_0;

    GPIOB->CRL &= ~(GPIO_CRL_CNF4_0 | GPIO_CRL_CNF4_1);
    GPIOB->CRL |= GPIO_CRL_CNF4_1;
    GPIOB->BRR = 1 << 4;

    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_1;

    /* TIM3 Ch1 */
    TIM3->CR1 = TIM_CR1_ARPE;
    TIM3->CR2 = 0x00;
    TIM3->DIER = TIM_DIER_CC1IE | TIM_DIER_UIE;
    TIM3->CCMR1 = TIM_CCMR1_CC1S_0;
    TIM3->CCER = /*TIM_CCER_CC1P |*/ TIM_CCER_CC1E;
    TIM3->PSC = 720;
    TIM3->ARR = RPM_ARR_VALUE;
    TIM3->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[TIM3_IRQn / 32] |= (1 << (TIM3_IRQn % 32));
    NVIC->IP[TIM3_IRQn] = (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1) << 4;

    AFIO->MAPR |= AFIO_MAPR_TIM3_REMAP_1;

    xTaskHandle foo;

    xTaskCreate(&_Task, "MEAS", DEFAULT_STACK_DEPTH, NULL, 5, &foo);
}

static void _Task(void *p) {
    while(1) {
        portYIELD();
    }
}

uint32_t Measurement_GetVoltage() {
    uint32_t result = 0;
    uint32_t i;
    float u;

    for(i=0;i<BUFFER_LENGTH;i++)
        result += ADC_Data[i].Voltage;

    result /= BUFFER_LENGTH;
    u = (result * VREF) / 0x0fff;
    result = (int)(u * 4900.0);
    return result;
}

int32_t Measurement_GetTemp() {
    uint32_t result = 0;
    float u, r, c;
    uint32_t i, T;

    for(i=0;i<BUFFER_LENGTH;i++)
        result += ADC_Data[i].Temp;

    result /= BUFFER_LENGTH;
    u = (result * VREF)/0x0fff;
    r = (u * 10e3) / (VREF - u);

    if(r >= Table[0].r)
        return (MIN_TEMP);
    if(r <= Table[TEMP_TABLE_LENGTH - 1].r)
        return (MAX_TEMP);
    i = 0;
    while(r <= Table[i].r)
        i++;
    c = 1 - (r - Table[i].r) / (Table[i -1].r - Table[i].r);
    T = (int)(Table[i - 1].temp + 5.0 * c);
    return T;
}

uint32_t Measurement_GetRPM() {
    uint32_t i;
    uint32_t Period = 0;
    uint32_t RPM;
    
    for(i=0;i<TEETH_NUM;i++) {
        Period += RPM_Data[i];
    }
    
    Period >>= 1;
    RPM = (10000000 / Period);
    return RPM;
}