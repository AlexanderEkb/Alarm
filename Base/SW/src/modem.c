#include <string.h>

#include "stm32f1xx.h"
#include "modem.h"
#include "PDU.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define PORT                GPIOB
#define PWRKEY_PORT         PORT
#define PWRKEY_PIN          ((uint32_t)(1 << 9))
#define DTR_PORT            PORT
#define DTR_PIN             ((uint32_t)(1 << 5))
#define RING_PORT           PORT
#define RING_PIN            ((uint32_t)(1 << 8))
#define USART_PORT          PORT
#define TXD_PIN             ((uint32_t)(1 << 6))
#define RXD_PIN             ((uint32_t)(1 << 7))
#define USART               USART1

#define INBUF_SIZE          (256)
#define ATTEMPTS            (10)
#define DEFAULT_TIMEOUT     (10)
#define DISABLE_COMM_IRQ    {USART->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_RXNEIE);}
#define ENABLE_RX_IRQ       {USART->CR1 |= USART_CR1_RXNEIE;}
#define DISABLE_RX_IRQ      {USART->CR1 &= ~(USART_CR1_RXNEIE);}
#define ENABLE_TX_IRQ       {USART->CR1 |= USART_CR1_TXEIE;}
#define DISABLE_TX_IRQ      {USART->CR1 &= ~(USART_CR1_TXEIE);}
#define PULSE_PWRKEY        {PWRKEY_PORT->BRR = PWRKEY_PIN;vTaskDelay(500);PWRKEY_PORT->BSRR = PWRKEY_PIN;vTaskDelay(10);}

#define INIT_TIMEOUT    (20000)
#define MODEM_STARTUP   "MODEM:STARTUP"
#define PBREADY         "+PBREADY"
#define OK_ANSWER       "OK"

#define DISABLE_ECHO    "ATE0\r"
#define SMS_INPUT_MODE  "AT+CMGF=0\r"
#define TE_CHARSET      "AT+CSCS=\"UCS2\"\r"
#define MSG_INDICATION  "AT+CNMI=2,2,0,0,0\r"
#define CALLER_ID       "AT+CLIP=1\r"

#define MSG_RECEIVED    "+CMT:"
#define INCOMING_CALL   "+CLIP:"

typedef struct {
    uint8_t MTI;
    uint8_t OA_Length;
    uint8_t OA_Type;
    uint64_t OA;
    uint8_t length;
} IncomingSMS_TypeDef;

static volatile char inbuf[INBUF_SIZE];
static volatile uint32_t inbuf_usage = 0;
static uint32_t inbuf_pointer = 0;
static char *outbuf = NULL;
static SemaphoreHandle_t Received;
static SemaphoreHandle_t Sent;
static char dbgbuf[INBUF_SIZE*8];
static uint32_t dbgptr = 0;

void Modem_PwrOn();
uint32_t Modem_WaitForString(char *s, uint32_t timeout);
void Modem_Send(char *s);
uint32_t ToBinary(uint8_t *src, uint8_t *dst, uint32_t len);
void ToASCII(uint8_t *src, uint8_t *dst, uint32_t len);

void USART1_IRQHandler() {
    char rx;
    static BaseType_t xHigherPriorityTaskWoken;
    uint32_t sr = USART->SR;

    xHigherPriorityTaskWoken = pdFALSE;

    // Handling received data
    while (USART->SR & USART_SR_RXNE) {
        rx = USART->DR;
        if(rx)
            dbgbuf[dbgptr++] = rx;
        if(rx && (inbuf_pointer < INBUF_SIZE)) {
            if((rx == 0x0a) || (rx == 0x0d)){
                inbuf[inbuf_pointer] = 0;             // Terminate string
                if(inbuf_pointer > inbuf_usage)
                    inbuf_usage = inbuf_pointer;
                inbuf_pointer = 0;
                if(inbuf[0]) {
                    DISABLE_RX_IRQ;
                    xSemaphoreGiveFromISR(Received, &xHigherPriorityTaskWoken );
                }
            } else {
                inbuf[inbuf_pointer] = rx;
                inbuf_pointer++;
            }
        }
    }

    // Handling transmission process
    if((sr & USART_SR_TXE) && (outbuf)){
        outbuf++;
        if(*outbuf)
            USART->DR = *outbuf;
        else {
            outbuf = 0;
            DISABLE_TX_IRQ;
            xSemaphoreGiveFromISR(Sent, &xHigherPriorityTaskWoken );
        }
    }
    USART->SR = 0;
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void Modem_Init() {
    TaskHandle_t mdm;
    uint32_t foo;

    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN |
        RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
    AFIO->MAPR |= AFIO_MAPR_USART1_REMAP;

    // PWRKEY
    foo = PWRKEY_PORT->CRH;
    foo &= ~(GPIO_CRH_MODE9 | GPIO_CRH_CNF9);
    foo |= GPIO_CRH_MODE9_1;
    PWRKEY_PORT->CRH = foo;
    PWRKEY_PORT->BSRR = PWRKEY_PIN;

    //DTR
    foo = DTR_PORT->CRL;
    foo &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
    foo |= GPIO_CRL_MODE5_1;
    DTR_PORT->CRL = foo;
    DTR_PORT->BRR = DTR_PIN;

    //RING
    foo = RING_PORT->CRH;
    foo &= ~(GPIO_CRH_MODE8 | GPIO_CRH_CNF8);
    foo |= GPIO_CRH_CNF8_0;
    RING_PORT->CRH = foo;

    // TX & RX
    foo = USART_PORT->CRL;
    foo &= ~(GPIO_CRL_MODE6 | GPIO_CRL_CNF6 | GPIO_CRL_MODE7 | GPIO_CRL_CNF7);
    foo |= GPIO_CRL_MODE6 | GPIO_CRL_CNF6_1 | GPIO_CRL_CNF7_0;
    USART_PORT->CRL = foo;


    //USART->BRR = (58 << 4) | (10);
    USART->BRR = (39 << 4) | (0);
    USART->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    USART->SR = 0;

    NVIC->ISER[USART1_IRQn / 32] |= (1 << (USART1_IRQn % 32));
    NVIC->IP[USART1_IRQn] = (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1) << 4;
    inbuf_pointer = 0;

    Received = xSemaphoreCreateBinary();
    Sent = xSemaphoreCreateBinary();
    xSemaphoreGive(Sent);
    outbuf = NULL;
}

void Modem_Task (void *p) {
    SMS_TypeDef SMS;

    while(1) {
        Modem_PwrOn();
        while(1) {
            Modem_WaitForString(NULL, portMAX_DELAY);
            if(!strncmp((char const *)inbuf, MSG_RECEIVED, strlen(MSG_RECEIVED))) {
                Modem_WaitForString(NULL, portMAX_DELAY);
                PDU_Decode((char *)inbuf, &SMS);
            } else if(!strncmp((char const *)inbuf, INCOMING_CALL, strlen(INCOMING_CALL))) {
            }
        }
    }
}

uint32_t Modem_WaitForString(char *s, uint32_t timeout) {
    uint32_t GameOver = xTaskGetTickCount() + timeout;
    uint32_t result;

    ENABLE_RX_IRQ;
    while(1) {
        result = xSemaphoreTake(Received, DEFAULT_TIMEOUT);
        if(result == pdTRUE) {
            if((!s) || !strncmp((char const *)inbuf, s, strlen(s)))
                return 1;
            else
                ENABLE_RX_IRQ;
        }
        if((timeout != portMAX_DELAY) && (xTaskGetTickCount() >= GameOver)) {
            DISABLE_RX_IRQ;
            return(0);
        }
    }
}

void Modem_PwrOn() {
    while(1) {
        dbgptr = 0;
        memset(dbgbuf, 0, sizeof(dbgbuf));
        DISABLE_COMM_IRQ;
        PULSE_PWRKEY;

        if(!Modem_WaitForString(MODEM_STARTUP, INIT_TIMEOUT))
            continue;
        if(!Modem_WaitForString(PBREADY, INIT_TIMEOUT))
            continue;
        Modem_Send(TE_CHARSET);
        if(!Modem_WaitForString(OK_ANSWER, INIT_TIMEOUT))
            continue;
        Modem_Send(MSG_INDICATION);
        if(!Modem_WaitForString(OK_ANSWER, INIT_TIMEOUT))
            continue;
        Modem_Send(DISABLE_ECHO);
        if(!Modem_WaitForString(OK_ANSWER, INIT_TIMEOUT))
            continue;
        Modem_Send(CALLER_ID);
        if(!Modem_WaitForString(OK_ANSWER, INIT_TIMEOUT))
            continue;

        return;
    }
}

void Modem_Send(char *s) {
    xSemaphoreTake(Sent, portMAX_DELAY);
    outbuf = s;

    USART->DR = *s;
    ENABLE_TX_IRQ;
}

/*******************************************************************************
************************** Binary <-> ASCII conversion *************************
********************************** (MIME-like) *********************************
*******************************************************************************/

void ToASCII(uint8_t *src, uint8_t *dst, uint32_t len) {
    uint32_t triplets;
    uint8_t *pdst = dst;
    uint32_t i, j;
    uint32_t triplet;

    /* Determining the number of byte triplets */
    triplets = len / 3;
    if(len % 3)
        triplets++;

    /* then expanding eight-bit bytes to six-bit groups */
    for(i=0;i<triplets;i++) {
        triplet = 0;
        for(j=0;j<3;j++){
            triplet <<= 8;
            triplet |= ((i*3+j) < len)?src[i*3+j]:0;
        }
        *(dst++) = (triplet >> 18) & 0x3f;
        *(dst++) = (triplet >> 12) & 0x3f;
        *(dst++) = (triplet >> 6) & 0x3f;
        *(dst++) = triplet & 0x3f;

    }

    /* Move narrow six-byte groups to the acceptable ASCII table area */
    for(i=0;i<triplets*4;i++) {
        if(*pdst < 34)
            *pdst += 0x3e;
        else
            *pdst += 0x3f;
        pdst++;
    }

}

uint32_t ToBinary(uint8_t *src, uint8_t *dst, uint32_t len) {
    uint32_t quadruplets;
    uint32_t i, j;
    uint32_t quadruplet;
    uint8_t *inbuf_pointer = src;

    /* Message length should be divisible by four */
    if(len % 4)
        return 0;

    quadruplets = len / 4;

    /* Convert SMS-acceptable symbols to the binary data */
    for(i=0;i<len;i++) {
        if((*src > 0x3d) && (*src < 0x60))
            *src -= 0x3e;
        else if((*src > 0x60) && (*src < 0x7F))
            *src -= 0x3f;
        else return 0;      // if char value does not fit to the used ranges
        src++;
    }

    /* Convert data from wide representation to usual byte form */
    for(i=0;i<quadruplets;i++) {

        for(j=0;j<4;j++) {
            quadruplet <<= 8;
            quadruplet |= *(inbuf_pointer++);
        }
        *(dst++) = ((quadruplet >> 22) & 0xfc) | ((quadruplet >> 20) & 0x03);
        *(dst++) = ((quadruplet >> 12) & 0xf0) | ((quadruplet >> 10) & 0x0f);
        *(dst++) = ((quadruplet >> 02) & 0xc0) | ((quadruplet >> 00) & 0x3f);
    }

    return 1;
}

