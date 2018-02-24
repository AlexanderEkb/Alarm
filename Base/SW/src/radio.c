#include <string.h>
#include "radio.h"
#include "nRF24L01.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "cipher.h"
#include "rc.h"
#include "alarm.h"
#include "engine.h"
#include "protocol.h"

#define PORT            GPIOA
#define STACK_DEPTH     (configMINIMAL_STACK_SIZE)

typedef __packed struct {
    uint8_t Data[PAYLOAD_LENGTH];
    uint8_t Sync[8];
} RadioPacket_TypeDef;

static const Address_t addr = "\x01\x02\x03\x04\x05";
static xSemaphoreHandle Irq;
static volatile RadioPacket_TypeDef Data;

void Radio_Init() {
    uint32_t foo;
    uint32_t iser;

    Irq = xSemaphoreCreateBinary();

    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN
        | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_AFIOEN;

    // MOSI, MISO & SCK
    foo = GPIOA->CRL;
    foo &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5 |
             GPIO_CRL_MODE6 | GPIO_CRL_CNF6 |
             GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    foo |= (GPIO_CRL_MODE5 | GPIO_CRL_CNF5_1 |
            GPIO_CRL_CNF6_0 |
            GPIO_CRL_MODE7 | GPIO_CRL_CNF7_1);
    GPIOA->CRL = foo;

    // IRQ
    foo = GPIOB->CRL;
    foo &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0);
    foo |= GPIO_CRL_CNF0_1;
    GPIOB->CRL = foo;
    GPIOB->ODR = (1 << 0);

    // CE
    foo = GPIOA->CRL;
    foo &= ~(GPIO_CRL_MODE3 | GPIO_CRL_CNF3);
    foo |= GPIO_CRL_MODE3_1;
    GPIOA->CRL = foo;
    GPIOA->BSRR = (1 << 3);
    GPIOA->BRR = (1 << 3);

    // CS
    foo = GPIOA->CRL;
    foo &= ~(GPIO_CRL_MODE4 | GPIO_CRL_CNF4);
    foo |= GPIO_CRL_MODE4_1;
    GPIOA->CRL = foo;
    GPIOA->BSRR = (1 << 4);

    // SPI
    SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_2 | SPI_CR1_BR_0;
    SPI1->CR1 |= SPI_CR1_SPE;

    EXTI->IMR |= EXTI_IMR_MR0;
    EXTI->FTSR |= EXTI_FTSR_TR0;
    foo = AFIO->EXTICR[0];
    foo &= ~(AFIO_EXTICR1_EXTI0);
    foo |= AFIO_EXTICR1_EXTI0_PB;
    AFIO->EXTICR[0] = foo;
    iser = EXTI0_IRQn / 32;
    NVIC->ISER[iser] |= (1 << (EXTI0_IRQn % 32));
    NVIC->IP[EXTI0_IRQn] = (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1) << 4;

}

void Radio_Task(void *p) {
    DataPipe_TypeDef DataPipe;
    uint8_t len;
    volatile static uint8_t decoded[24];
    uint8_t answer[24];
    uint32_t sync;
    uint32_t *psync;
    uint8_t LastAction = ACTION_NOTHING;

    nRF24L01_Init();

    DataPipe.N = DATA_PIPE_0;
    DataPipe.State = ENABLE;
    DataPipe.Address = addr;
    DataPipe.PacketWidth = sizeof(RadioPacket_TypeDef);
    DataPipe.AutoACK = NO_AUTO_ACK;
    nRF24L01_ConfigureDataPipe(&DataPipe);

    DataPipe.N = DATA_PIPE_1;
    DataPipe.State = DISABLE;
    nRF24L01_ConfigureDataPipe(&DataPipe);
    DataPipe.N = DATA_PIPE_2;
    nRF24L01_ConfigureDataPipe(&DataPipe);
    DataPipe.N = DATA_PIPE_3;
    nRF24L01_ConfigureDataPipe(&DataPipe);
    DataPipe.N = DATA_PIPE_4;
    nRF24L01_ConfigureDataPipe(&DataPipe);
    DataPipe.N = DATA_PIPE_5;
    nRF24L01_ConfigureDataPipe(&DataPipe);

    nRF24L01_SetTXAddress(addr, ADDR_LEN);
    nRF24L01_SetRFChannel(0x10);

    nRF24L01_GetITSource();   /* to clear pending IRQ flags */
    nRF24L01_ITConfig(IT_RX_DR | IT_TX_DS, ENABLE);

    nRF24L01_TurnOn();
    while(1) {
        nRF24L01_StartReceiving();
        xSemaphoreTake(Irq, portMAX_DELAY);
        len = nRF24L01_GetFullRXPayload((void *)&Data);

        if(len == PAYLOAD_LENGTH + 8) {
            nRF24L01_Stop();
            psync = ((uint32_t *)&Data.Sync[0]);
            Cipher_Decode((uint8_t *)&Data, (uint8_t *)&decoded[0], *psync, PAYLOAD_LENGTH);
            uint8_t action = decoded[0];
            if(action != LastAction) {
                uint8_t target = action & TARGET_MASK;
                if(target == TARGET_ALARM)
                    Alarm_SendMsg((Action_t)action, ((uint32_t)&decoded[0]), 0);
                else if(target == TARGET_ENGINE)
                    Engine_SendMsg((Action_t)action, ((uint32_t)&decoded[0]), 0);
                LastAction = action;
            }
            // TODO: Encode it!
            sync = xTaskGetTickCount();
            memcpy((void *)&answer[0], "\x00answer!        ", PAYLOAD_LENGTH);
            Cipher_Encode(((uint8_t *)&answer[0]), ((uint8_t *)&Data.Data[0]), sync, PAYLOAD_LENGTH);
            memcpy((void *)&Data.Sync[0], &sync, sizeof(sync));
            memcpy((void *)&Data.Sync[4], &sync, sizeof(sync));
            nRF24L01_Transmit((void *)&Data, sizeof(Data));

            xSemaphoreTake(Irq, portMAX_DELAY);
        }
        nRF24L01_Stop();
    }
}

void EXTI0_IRQHandler() {
    static BaseType_t xHigherPriorityTaskWoken;

    EXTI->PR = EXTI_PR_PR0;
    nRF24L01_GetITSource();

    xSemaphoreGiveFromISR(Irq, &xHigherPriorityTaskWoken);

    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}