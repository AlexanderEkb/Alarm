#ifndef NRF24L01_H
#define NRF24L01_H

#include "stm32f0xx_hal.h"

#define R_CONFIG        0x00
#define R_EN_AA         0x01
#define R_EN_RXADDR     0x02
#define R_SETUP_AW      0x03
#define R_SETUP_RETR    0x04
#define R_RF_CH         0x05
#define R_RF_SETUP      0x06
#define R_STATUS        0x07
#define R_OBSERVE_TX    0x08
#define R_RPD           0x09
#define R_RX_ADDR_P0    0x0a
#define R_RX_ADDR_P1    0x0b
#define R_RX_ADDR_P2    0x0c
#define R_RX_ADDR_P3    0x0d
#define R_RX_ADDR_P4    0x0e
#define R_RX_ADDR_P5    0x0f
#define R_TX_ADDR       0x10
#define R_RX_PW_P0      0x11
#define R_RX_PW_P1      0x12
#define R_RX_PW_P2      0x13
#define R_RX_PW_P3      0x14
#define R_RX_PW_P4      0x15
#define R_RX_PW_P5      0x16
#define R_FIFO_STATUS   0x17
#define R_DYNPD         0x1c
#define R_FEATURE       0x1d

#define READ_REGISTER   0x00
#define WRITE_REGISTER  0x20
#define R_RX_PAYLOAD    0x61
#define W_TX_PAYLOAD    0xa0
#define FLUSH_TX        0xe1
#define FLUSH_RX        0xe2
#define REUSE_TX_PL     0xe3
#define R_RX_PL_WID     0x60
#define W_ACK_PAYLOAD   0xa8
#define W_TX_PL_NO_ACK  0xb0
#define NOP             0xff

/*!<******************  Bit definition for R_CONFIG register  *******************/
#define R_CONFIG_MASK_RX_DR                 ((uint8_t)0x40)            /*!< Mask interrupt caused by RX_DR */
#define R_CONFIG_MASK_TX_DS                 ((uint8_t)0x20)            /*!< Mask interrupt caused by TX_DS */
#define R_CONFIG_MASK_MAX_RT                ((uint8_t)0x10)            /*!< Mask interrupt caused by MAX_RT */
#define R_CONFIG_EN_CRC                     ((uint8_t)0x08)            /*!< Enable CRC */
#define R_CONFIG_CRCO                       ((uint8_t)0x04)            /*!< CRC encoding scheme*/
#define R_CONFIG_PWR_UP                     ((uint8_t)0x02)            /*!< Power up/power down */
#define R_CONFIG_PRIM_RX_CONTROL            ((uint8_t)0x01)            /*!< 1-RX, 0-TX */

/*!<******************  Bit definition for R_EN_AA register  *******************/
#define R_EN_AA_P5                          ((uint8_t)0x20)            /*!< Enable autoACK data pipe 5 */
#define R_EN_AA_P4                          ((uint8_t)0x10)            /*!< Enable autoACK data pipe 4 */
#define R_EN_AA_P3                          ((uint8_t)0x08)            /*!< Enable autoACK data pipe 3 */
#define R_EN_AA_P2                          ((uint8_t)0x04)            /*!< Enable autoACK data pipe 2 */
#define R_EN_AA_P1                          ((uint8_t)0x02)            /*!< Enable autoACK data pipe 1 */
#define R_EN_AA_P0                          ((uint8_t)0x01)            /*!< Enable autoACK data pipe 0 */

/*!<******************  Bit definition for R_EN_RXADDR register  *******************/
#define R_EN_RXADDR_P5                      ((uint8_t)0x20)            /*!< Enable data pipe 5 */
#define R_EN_RXADDR_P4                      ((uint8_t)0x10)            /*!< Enable data pipe 4 */
#define R_EN_RXADDR_P3                      ((uint8_t)0x08)            /*!< Enable data pipe 3 */
#define R_EN_RXADDR_P2                      ((uint8_t)0x04)            /*!< Enable data pipe 2 */
#define R_EN_RXADDR_P1                      ((uint8_t)0x02)            /*!< Enable data pipe 1 */
#define R_EN_RXADDR_P0                      ((uint8_t)0x01)            /*!< Enable data pipe 0 */

/*!<******************  Bit definition for R_SETUP_AW register  *******************/
#define R_SETUP_AW_3_BYTES                  ((uint8_t)0x01)            /*!< 3 bytes address width */
#define R_SETUP_AW_4_BYTES                  ((uint8_t)0x02)            /*!< 4 bytes address width */
#define R_SETUP_AW_5_BYTES                  ((uint8_t)0x03)            /*!< 5 bytes address width */
#define R_SETUP_AW_MASK                     ((uint8_t)0x03)            /*!< SETUP_AW mask */

/*!<******************  Bit definition for R_SETUP_RETR register  ******************/
#define R_SETUP_RETR_DELAY_250uS            ((uint8_t)0x00)
#define R_SETUP_RETR_DELAY_500uS            ((uint8_t)0x10)
#define R_SETUP_RETR_DELAY_750uS            ((uint8_t)0x20)
#define R_SETUP_RETR_DELAY_1000uS           ((uint8_t)0x30)
#define R_SETUP_RETR_DELAY_1250uS           ((uint8_t)0x40)
#define R_SETUP_RETR_DELAY_1500uS           ((uint8_t)0x50)
#define R_SETUP_RETR_DELAY_1750uS           ((uint8_t)0x60)
#define R_SETUP_RETR_DELAY_2000uS           ((uint8_t)0x70)
#define R_SETUP_RETR_DELAY_2250uS           ((uint8_t)0x80)
#define R_SETUP_RETR_DELAY_2500uS           ((uint8_t)0x90)
#define R_SETUP_RETR_DELAY_2750uS           ((uint8_t)0xa0)
#define R_SETUP_RETR_DELAY_3000uS           ((uint8_t)0xb0)
#define R_SETUP_RETR_DELAY_3250uS           ((uint8_t)0xc0)
#define R_SETUP_RETR_DELAY_3500uS           ((uint8_t)0xd0)
#define R_SETUP_RETR_DELAY_3750uS           ((uint8_t)0xe0)
#define R_SETUP_RETR_DELAY_4000uS           ((uint8_t)0xf0)
#define R_SETUP_RETR_DELAY_MASK             ((uint8_t)0xf0)
#define R_SETUP_RETR_DISABLED               ((uint8_t)0x00)
#define R_SETUP_RETR_1_TIME                 ((uint8_t)0x01)
#define R_SETUP_RETR_2_TIMES                ((uint8_t)0x02)
#define R_SETUP_RETR_3_TIMES                ((uint8_t)0x03)
#define R_SETUP_RETR_4_TIMES                ((uint8_t)0x04)
#define R_SETUP_RETR_5_TIMES                ((uint8_t)0x05)
#define R_SETUP_RETR_6_TIMES                ((uint8_t)0x06)
#define R_SETUP_RETR_7_TIMES                ((uint8_t)0x07)
#define R_SETUP_RETR_8_TIMES                ((uint8_t)0x08)
#define R_SETUP_RETR_9_TIMES                ((uint8_t)0x09)
#define R_SETUP_RETR_10_TIMES               ((uint8_t)0x0a)
#define R_SETUP_RETR_11_TIMES               ((uint8_t)0x0b)
#define R_SETUP_RETR_12_TIMES               ((uint8_t)0x0c)
#define R_SETUP_RETR_13_TIMES               ((uint8_t)0x0d)
#define R_SETUP_RETR_14_TIMES               ((uint8_t)0x0e)
#define R_SETUP_RETR_15_TIMES               ((uint8_t)0x0f)
#define R_SETUP_RETR_COUNT_MASK             ((uint8_t)0x0f)

/*!<************************ Data mask for R_CH register ***************************/
#define R_CH_MASK                           ((uint8_t)0x7f)

/*!<******************************  Data Rate values  *******************************/
#define DATA_RATE_250KBPS                   ((uint8_t)0x20)
#define DATA_RATE_1MBPS                     ((uint8_t)0x00)
#define DATA_RATE_2MBPS                     ((uint8_t)0x08)
#define DATA_RATE_MASK                      ((uint8_t)0xd7)

/*!<******************************  TX power values  *******************************/
#define TX_PWR_18DBM                        ((uint8_t)0x00)
#define TX_PWR_12DBM                        ((uint8_t)0x02)
#define TX_PWR_6DBM                         ((uint8_t)0x04)
#define TX_PWR_0DBM                         ((uint8_t)0x06)
#define TX_PWR_MASK                         ((uint8_t)0xf9)

/*!<******************  Bit definition for R_STATUS register  *******************/
#define R_STATUS_RX_DR                      ((uint8_t)0x40)            /*!< Data ready RX FIFO interrupt */
#define R_STATUS_TX_DS                      ((uint8_t)0x20)            /*!< Data sent TX FIFO interrupt */
#define R_STATUS_MAX_RT                     ((uint8_t)0x10)            /*!< Max no of TX retransmits interrupt */
#define R_STATUS_P_NO_0                     ((uint8_t)0x00)            /*!< Data pipe 0 */
#define R_STATUS_P_NO_1                     ((uint8_t)0x02)            /*!< Data pipe 1 */
#define R_STATUS_P_NO_2                     ((uint8_t)0x04)            /*!< Data pipe 2 */
#define R_STATUS_P_NO_3                     ((uint8_t)0x06)            /*!< Data pipe 3 */
#define R_STATUS_P_NO_4                     ((uint8_t)0x08)            /*!< Data pipe 4 */
#define R_STATUS_P_NO_5                     ((uint8_t)0x0a)            /*!< Data pipe 5 */
#define R_STATUS_P_NO_MASK                  ((uint8_t)0x0e)            /*!< Data pipe no mask */
#define R_STATUS_TX_FULL                    ((uint8_t)0x01)            /*!< TX FIFO is full */

/*!<**************************  Data pipe definitions  ***************************/
#define DATA_PIPE_0                         (R_EN_RXADDR_P0)
#define DATA_PIPE_1                         (R_EN_RXADDR_P1)
#define DATA_PIPE_2                         (R_EN_RXADDR_P2)
#define DATA_PIPE_3                         (R_EN_RXADDR_P3)
#define DATA_PIPE_4                         (R_EN_RXADDR_P4)
#define DATA_PIPE_5                         (R_EN_RXADDR_P5)

/*!<**************************  IRQ src definitions  ***************************/
#define IT_RX_DR                            ((uint8_t)0x40)            /*!< Data ready RX FIFO interrupt */
#define IT_TX_DS                            ((uint8_t)0x20)            /*!< Data sent TX FIFO interrupt */
#define IT_MAX_RT                           ((uint8_t)0x10)            /*!< Max no of TX retransmits interrupt */
#define IT_SOURCE_MASK                      ((uint8_t)0x70)

typedef enum {R_OK = 0, R_ERROR, R_BUSY} RadioResult_TypeDef;
typedef enum {NO_AUTO_ACK=0, AUTO_ACK} AutoACK_TypeDef;

typedef uint8_t* Address_t;

typedef struct {
    uint8_t N;
    FunctionalState State;
    uint8_t PacketWidth;
    Address_t Address;
    AutoACK_TypeDef AutoACK;
} DataPipe_TypeDef;

void nRF24L01_OsInit();
void nRF24L01_HwInit();
RadioResult_TypeDef nRF24L01_TurnOn();
RadioResult_TypeDef nRF24L01_TurnOff();
RadioResult_TypeDef nRF24L01_Transmit(void *txbuf, uint8_t len);
RadioResult_TypeDef nRF24L01_Receive(void *rxbuf, uint32_t timeout);
RadioResult_TypeDef nRF24L01_SetRFChannel(uint8_t ch);
RadioResult_TypeDef nRF24L01_SetDataRate(uint8_t rate);
RadioResult_TypeDef nRF24L01_SetTXPower(uint8_t pwr);

RadioResult_TypeDef nRF24L01_ConfigureDataPipe(DataPipe_TypeDef *DataPipe);
RadioResult_TypeDef nRF24L01_SetTXAddress(Address_t addr, uint8_t addr_len);

uint8_t nRF24L01_GetStatus();
uint8_t nRF24L01_GetRXPayloadWidth();
uint8_t nRF24L01_GetFullRXPayload(void *buffer);
uint8_t nRF24L01_GetRXPayload(void *buffer, uint8_t count);

void nRF24L01_StartReceiving();
void nRF24L01_Stop();

void nRF24L01_ITConfig(uint8_t IRQSource, FunctionalState NewState);
uint8_t nRF24L01_GetITSource();
ErrorStatus nRF24L01_WaitForIRQ(uint32_t timeout, uint32_t mask);

void nRF24L01_IRQHandler();
#endif /* NRF24L01_H */
