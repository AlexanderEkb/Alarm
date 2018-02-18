#include <string.h>
#include "stm32f0xx.h"
#include "nRF24L01.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "protocol.h"
#include "radio.h"
#include "cipher.h"
#include "power.h"
#include "dispatcher.h"

#if defined (__GNUC__)
#include "iar_to_gcc.h"
#endif

#define PORT                GPIOA
#define ADDR_LEN            0x05
#define PAYLOAD_LENGTH      (16)
#define QUEUE_LENGTH        (4)
#define QUEUE_RECV_TIMEOUT  ((TickType_t)1200)   /* Must be greater than IWDG timeout */

#define NACK_THRESHOLD      (1)
#define RX_TIMEOUT          (4)
#define MAX_ATTEMPTS        (10)

typedef __packed struct {
    uint8_t Data[PAYLOAD_LENGTH];
    uint8_t Sync[8];
} RadioPacket_TypeDef;

typedef __packed struct {
    uint32_t cmd;
    uint32_t param0;
    uint32_t param1;
} Radio_Notification_t;

static __ro_placement const Address_t addr = "\x01\x02\x03\x04\x05";
static volatile uint8_t Data[32];
static QueueHandle_t queue;

static __no_init struct {
    uint32_t NACK;
    uint32_t status;
} Link;

static void Radio_Send(Radio_Notification_t *event);
static ErrorStatus Radio_ReceiveAnswer();
static void Radio_HwInit();

void Radio_Task(void const * argument) {
    uint32_t attempt;
    Radio_Notification_t event = {ACTION_NOTHING, 0, 0};
    RadioPacket_TypeDef answer;
    uint8_t decoded[PAYLOAD_LENGTH];
    ErrorStatus result;

    Radio_HwInit();

    while(1) {
		LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SPI1);

		nRF24L01_TurnOn();

		UI_Suspend();
		for(attempt=0;attempt<MAX_ATTEMPTS;attempt++) {
			Radio_Send(&event);
			result = Radio_ReceiveAnswer();
			if(result == SUCCESS)
				break;
		}

		nRF24L01_TurnOff();

		if(result == SUCCESS) {
			Link.NACK = 0;
			Link.status = 1;
			nRF24L01_GetFullRXPayload((void *)&answer);
			Cipher_Decode((uint8_t *)&answer.Data[0], &decoded[0], *((uint32_t *)&answer.Sync[0]), PAYLOAD_LENGTH);
		} else {
			if(Link.NACK < NACK_THRESHOLD)
				Link.NACK++;
			else
				Link.status = 0;
		}

		UI_Resume();

		LL_APB1_GRP2_DisableClock(LL_APB1_GRP2_PERIPH_SPI1);
		xQueueReceive(queue, &(event), QUEUE_RECV_TIMEOUT);
    }
}

static void Radio_Send(Radio_Notification_t *event) {
    uint8_t source[PAYLOAD_LENGTH];
    RadioPacket_TypeDef pkt;

    /* Create the reliable sync part */
    uint32_t rtc_time = RTC->TR;
    uint32_t rtc_subsec = RTC->SSR << 8;
    uint32_t pre_sync = rtc_time ^ rtc_subsec;
    memcpy(&pkt.Sync[0], &pre_sync, sizeof(pre_sync));
    memcpy(&pkt.Sync[4], &pre_sync, sizeof(pre_sync));
    Cipher_SimpleEncode((uint32_t *)&pkt.Sync[0]);
    uint32_t *sync = ((uint32_t *)&pkt.Sync[0]);

    source[0] = (uint8_t)event->cmd;
    memcpy(&source[1], "request", 7);

    Cipher_Encode(((uint8_t *)&source[0]), ((uint8_t *)&pkt.Data[0]), *sync, PAYLOAD_LENGTH);

    nRF24L01_Transmit(&pkt, sizeof(pkt));
}

void Radio_Init() {
    queue = xQueueCreate(QUEUE_LENGTH, sizeof(Radio_Notification_t));
}

static ErrorStatus Radio_ReceiveAnswer() {
    ErrorStatus result = ERROR;

    nRF24L01_GetITSource();
    nRF24L01_StartReceiving();
    result = nRF24L01_WaitForIRQ(RX_TIMEOUT, IT_RX_DR);
    nRF24L01_Stop();
    return result;
}

ErrorStatus Radio_GetLinkStatus() {
    return Link.status?SUCCESS:ERROR;
}

static void Radio_HwInit() {
    DataPipe_TypeDef DataPipe;

    LL_SPI_SetRxFIFOThreshold(SPI1, LL_SPI_RX_FIFO_TH_QUARTER);
    LL_SPI_Enable(SPI1);

    nRF24L01_OsInit();
    if(Power_IfPOR()) {
        Link.NACK = 0;
        Link.status = 0;

        nRF24L01_HwInit();

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
    }
}

void Radio_Notify(Action_t action, uint32_t param0, uint32_t param1) {
    Radio_Notification_t event;

    event.cmd = (uint32_t)action;
    event.param0 = param0;
    event.param1 = param1;

    xQueueSend(queue, &event, portMAX_DELAY);
}
