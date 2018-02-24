#include "stm32f1xx.h"
#include "inout.h"
#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "engine.h"
#include "alarm.h"

#define SPI                     SPI2
#define SPI_PORT                GPIOB
#define MOSI_PIN                (1 << 15)
#define MISO_PIN                (1 << 14)
#define SCK_PIN                 (1 << 13)
#define CS_PORT                 GPIOB
#define CS_PIN                  (1 << 12)
#define BUTTON_PORT             GPIOA
#define BUTTON_PIN              (1 << 9)
#define LED_PORT                GPIOA
#define LED_PIN                 (1 << 8)

#define POLL_PERIOD             (20)

#define SET_CS                  {CS_PORT->BSRR = CS_PIN; }
#define RESET_CS                {CS_PORT->BRR = CS_PIN; }
#define CS_STATE                (CS_PORT->IDR & CS_PIN)

#define ENDLESS                 (0xffffffff)

#define INPUTS_USED             (7)
#define USED_INPUTS_MASK        (IN_EXT_220 | IN_SHOCK_HI | IN_SHOCK_LO | IN_GLOW | IN_IGN | IN_PARKING | IN_DOORS)
#define STRAIGHT_INPUTS_MASK    (IN_PARKING | IN_IGN)

typedef struct {
    const uint32_t Line;
    const uint32_t Threshold;
    uint32_t Counter;
    InputState_TypeDef State;
} InputPattern_TypeDef;

typedef struct {
    uint32_t Duty;
    uint32_t Idle;
    uint32_t Repetitions;
} OutputPattern_TypeDef;

typedef enum {DUTY, IDLE} SirenPhase_TypeDef;

typedef struct {
    uint32_t Line;
    OutputPattern_TypeDef const *SignalPattern;
    uint32_t Start;
    uint32_t Stop;
    uint32_t Repetitions;
    SirenPhase_TypeDef Phase;
} Signal_TypeDef;

static uint32_t DoInOut(uint32_t o);
static uint32_t raw_inputs, inputs;
static uint32_t outputs;
//////////////////////////// Siren-related variables ///////////////////////////
static Signal_TypeDef Siren;
static const OutputPattern_TypeDef AP_Close = {100, 0, 1};
static const OutputPattern_TypeDef AP_Open = {100, 250, 2};
static const OutputPattern_TypeDef AP_Warning = {100, 100, 8};
static const OutputPattern_TypeDef AP_PreRun = {200, 300, 5};
static const OutputPattern_TypeDef AP_Panic = {500, 500, 60};

///////////////////////////// LED-related variables ////////////////////////////
static Signal_TypeDef Led;
static const OutputPattern_TypeDef LED_Blink = {50, 5000, ENDLESS};

//////////////////////////// Lights-related variables //////////////////////////
static Signal_TypeDef Lights;
static const OutputPattern_TypeDef LP_Open = {500, 500, 2};
static const OutputPattern_TypeDef LP_Close = {500, 0, 1};
static const OutputPattern_TypeDef LP_Warning = {200, 200, 4};
static const OutputPattern_TypeDef LP_PreRun = {200, 300, 5};
static const OutputPattern_TypeDef LP_Panic = {500, 500, 60};

InputPattern_TypeDef Inputs[INPUTS_USED] = {
    {IN_SHOCK_LO, 0, 0, INACTIVE},
    {IN_SHOCK_HI, 0, 0, INACTIVE},
    {IN_PARKING, 5, 0, INACTIVE},
    {IN_EXT_220, 5, 0, INACTIVE},
    {IN_GLOW, 5, 0, INACTIVE},
    {IN_IGN, 5, 0, INACTIVE},
    {IN_DOORS, 5, 0, INACTIVE}
};

static void Inout_HandleOutput(Signal_TypeDef *s);

void Inout_Init() {
    uint32_t foo;

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

    // CS
    foo = CS_PORT->CRH;
    foo &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12);
    foo |= GPIO_CRH_MODE12_1;
    CS_PORT->CRH = foo;
    CS_PORT->BSRR = CS_PIN;

    // MOSI, MISO & SCK
    foo = SPI_PORT->CRH;
    foo &= ~(GPIO_CRH_MODE15 | GPIO_CRH_CNF15 |
             GPIO_CRH_MODE14 | GPIO_CRH_CNF14 |
             GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    foo |= GPIO_CRH_MODE15_1 | GPIO_CRH_CNF15_1 |
                               GPIO_CRH_CNF14_0 |
           GPIO_CRH_MODE13_1 | GPIO_CRH_CNF13_1;
    SPI_PORT->CRH = foo;

    // LED
    foo = LED_PORT->CRH;
    foo &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
    foo |= GPIO_CRH_MODE8_1;
    LED_PORT->CRH = foo;
    LED_PORT->BRR = LED_PIN;

    // Service button
    foo = BUTTON_PORT->CRH;
    foo &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
    foo |= GPIO_CRH_CNF9_1;
    BUTTON_PORT->CRH = foo;

    SPI->CR1 = SPI_CR1_SPE | SPI_CR1_BR | SPI_CR1_MSTR |
        SPI_CR1_CPOL | SPI_CR1_CPHA | SPI_CR1_SSM | SPI_CR1_SSI;
    SPI->CR2 = 0;

    outputs = 0x00;

    Siren.Line = OUT_ALARM;
    Siren.SignalPattern = NULL;

    Lights.Line = OUT_LIGHTS;
    Lights.SignalPattern = NULL;

    Led.Line = OUT_LED;
    Led.SignalPattern = NULL;
}

void Inout_Task(void *p) {
    uint32_t prev_inputs;
    TickType_t ticks = xTaskGetTickCount();

    // Do in-out twice to eliminate initial error.
    DoInOut(outputs);
    prev_inputs = (~DoInOut(outputs) ^ STRAIGHT_INPUTS_MASK) & USED_INPUTS_MASK;

    while(1) {
        Inout_HandleOutput(&Siren);
        Inout_HandleOutput(&Lights);
        Inout_HandleOutput(&Led);

        raw_inputs = DoInOut(outputs);

        // Handle certain outputs
        if(outputs & OUT_LED)
            LED_PORT->BSRR = LED_PIN;
        else
            LED_PORT->BRR = LED_PIN;

        if(outputs & OUT_OPEN)
            outputs &= ~OUT_OPEN;

        if(outputs & OUT_CLOSE)
            outputs &= ~OUT_CLOSE;

        // translate raw input into something useful...
        inputs = (~raw_inputs ^ STRAIGHT_INPUTS_MASK) & USED_INPUTS_MASK;
        // Clear GLOW bit if ignition is turned off, due to lack of 
        // dashboard power
        if(!(inputs & IN_IGN))
            inputs &= ~IN_GLOW;
        
        if(inputs ^ prev_inputs) {
            Engine_SendMsg(ENGINE_IN_STATE_CHANGED, inputs, prev_inputs);
            Alarm_SendMsg(ALARM_IN_STATE_CHANGED, inputs, prev_inputs);
        }
        prev_inputs = inputs;
        vTaskDelayUntil(&ticks, POLL_PERIOD);
    }
}

static uint32_t DoInOut(uint32_t o) {
    static uint32_t i;

    i = SPI->DR;
    SPI->DR = (uint8_t)o;
    SET_CS;
    vTaskDelay(1);

    RESET_CS;

    return i;
}

void Inout_Set(uint32_t d) {
    outputs |= d;
}

void Inout_Clear(uint32_t d) {
    outputs &= ~d;
}

uint32_t Inout_Get() {
    return inputs;
}

void Inout_PlaySignal(Signals_TypeDef s) {
    uint32_t time = xTaskGetTickCount();

    switch(s) {
    case SIG_OPEN:
        Siren.SignalPattern = &AP_Open;
        Lights.SignalPattern = &LP_Open;
        Led.SignalPattern = NULL;
        Inout_Clear(OUT_LED);
        break;
    case SIG_CLOSE:
        Siren.SignalPattern = &AP_Close;
        Lights.SignalPattern = &LP_Close;
        Led.SignalPattern = &LED_Blink;
        Led.Stop = time + Led.SignalPattern->Duty;
        Led.Start = Led.Stop + Led.SignalPattern->Idle;
        Led.Repetitions = Led.SignalPattern->Repetitions;
        Led.Phase = DUTY;
        Inout_Set(OUT_LED);
        break;
    case SIG_WARNING:
        Siren.SignalPattern = &AP_Warning;
        Lights.SignalPattern = &LP_Warning;
        break;
    case SIG_PRERUN:
        Siren.SignalPattern = &AP_PreRun;
        Lights.SignalPattern = &LP_PreRun;
        break;
    case SIG_PANIC:
        Siren.SignalPattern = &AP_Panic;
        Lights.SignalPattern = &LP_Panic;
        break;
    }
    Siren.Stop = time + Siren.SignalPattern->Duty;
    Siren.Start = Siren.Stop + Siren.SignalPattern->Idle;
    Siren.Repetitions = Siren.SignalPattern->Repetitions;
    Siren.Phase = DUTY;
    Inout_Set(OUT_ALARM);

    Lights.Stop = time + Lights.SignalPattern->Duty;
    Lights.Start = Lights.Stop + Lights.SignalPattern->Idle;
    Lights.Repetitions = Lights.SignalPattern->Repetitions;
    Lights.Phase = DUTY;
    Inout_Set(OUT_LIGHTS);
}

static void Inout_HandleOutput(Signal_TypeDef *s) {
    uint32_t time = xTaskGetTickCount();
    if(!s->SignalPattern)
        return;
    if(s->Phase == DUTY) {
        if(time >= s->Stop) {
            Inout_Clear(s->Line);
            if((s->Repetitions == ENDLESS) || (--s->Repetitions)) {
                s->Start = time + s->SignalPattern->Idle;
                s->Phase = IDLE;
            } else
                s->SignalPattern = NULL;
        }
    } else {
        if(time >= s->Start) {
            Inout_Set(s->Line);
            s->Phase = DUTY;
            s->Stop = time + s->SignalPattern->Duty;
        }
    }
}