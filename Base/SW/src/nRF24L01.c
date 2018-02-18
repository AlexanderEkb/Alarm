#include "stm32f1xx.h"
#include "nRF24L01.h"
#include "FreeRTOS.h"
#include "task.h"

#define SPI                 SPI1
#define DRIVER_STACK_SIZE   256

#define DEFAULT_CHANNEL     ((uint8_t)0x00)
#define DEFAULT_TX_PWR      TX_PWR_0DBM
#define DEFAULT_DATA_RATE   DATA_RATE_250KBPS

#define Assert_CS()         {GPIOA->BRR = (1 << 4);}
#define Deassert_CS()       {GPIOA->BSRR = (1 << 4);}
#define Assert_CE()         {GPIOA->BSRR = (1 << 3);}
#define Deassert_CE()       {GPIOA->BRR = (1 << 3);}

static uint8_t Status;
static const Address_t DEFAULT_ADDR = "\x00\x01\x02\x03\x04";

void SPI_TransmitReceive(uint8_t *txbuf, uint8_t *rxbuf, uint8_t len);
void Delay(uint8_t time);
void nRF24L01_IssueCmd(uint8_t cmd);
uint8_t nRF24L01_ReadRegister(uint8_t addr);
void nRF24L01_WriteRegister(uint8_t addr, uint8_t value);
void nRF24L01_WriteTXPayload(void *buf, uint8_t len);

void Radio_Notify();

void nRF24L01_Init() {
    uint8_t cmd;
    Address_t addr = DEFAULT_ADDR;
    nRF24L01_TurnOff();

    nRF24L01_WriteRegister(0x00, 0x00);
    nRF24L01_WriteRegister(0x01, 0x00);
    nRF24L01_WriteRegister(0x02, 0x01);
    nRF24L01_WriteRegister(0x03, 0x03);
    nRF24L01_WriteRegister(0x04, 0x00);
    nRF24L01_WriteRegister(0x05, 0x00);
    nRF24L01_WriteRegister(0x06, 0x20); /* -18dBm, don't change! */
    nRF24L01_WriteRegister(0x07, 0x70);

    cmd = WRITE_REGISTER | 0x0a;
    Assert_CS();
    SPI_TransmitReceive(&cmd, &Status, 1);
    SPI_TransmitReceive((uint8_t *)&addr[0], NULL, 5);
    Deassert_CS();
    Delay(1);

    cmd = WRITE_REGISTER | 0x10;
    Assert_CS();
    SPI_TransmitReceive(&cmd, &Status, 1);
    SPI_TransmitReceive((uint8_t *)&addr[0], NULL, 5);
    Deassert_CS();
    Delay(1);

    nRF24L01_WriteRegister(0x11, 0x08);
    nRF24L01_WriteRegister(0x1c, 0x00);
    nRF24L01_WriteRegister(0x1d, 0x00);
    nRF24L01_WriteRegister(0x00, 0x19);

    nRF24L01_IssueCmd(FLUSH_RX);
//    cmd = FLUSH_RX;
//    Assert_CS();
//    SPI_TransmitReceive(&cmd, &Status, 1);
//    Deassert_CS();
    Delay(100);
}

RadioResult_TypeDef nRF24L01_TurnOn() {
    uint8_t cfg = nRF24L01_ReadRegister(R_CONFIG);

    if(!(cfg & R_CONFIG_PWR_UP)) {
        cfg |= R_CONFIG_PWR_UP;
        nRF24L01_WriteRegister(R_CONFIG, cfg);
    }

    return R_OK;
}

RadioResult_TypeDef nRF24L01_TurnOff() {
    uint8_t cfg = nRF24L01_ReadRegister(R_CONFIG);

    if(cfg & R_CONFIG_PWR_UP) {
        cfg &= ~R_CONFIG_PWR_UP;
        nRF24L01_WriteRegister(R_CONFIG, cfg);
    }

    return R_OK;
}

RadioResult_TypeDef nRF24L01_SetRFChannel(uint8_t ch) {
    nRF24L01_WriteRegister(R_RF_CH, ch & R_CH_MASK);
    return R_OK;
}

RadioResult_TypeDef nRF24L01_SetDataRate(uint8_t rate) {
    uint8_t rf_setup = nRF24L01_ReadRegister(R_RF_SETUP);
    rf_setup &= DATA_RATE_MASK;
    rf_setup |= (rate  & ~DATA_RATE_MASK);
    nRF24L01_WriteRegister(R_RF_SETUP, rf_setup);
    return R_OK;
}

RadioResult_TypeDef nRF24L01_SetTXPower(uint8_t pwr) {
    uint8_t rf_setup = nRF24L01_ReadRegister(R_RF_SETUP) & TX_PWR_MASK;
    rf_setup |= (pwr & ~TX_PWR_MASK);
    nRF24L01_WriteRegister(R_RF_SETUP, rf_setup);
    return R_OK;
}

RadioResult_TypeDef nRF24L01_ConfigureDataPipe(DataPipe_TypeDef *DataPipe) {
    uint8_t len;
    uint8_t rx_addr;
    uint8_t rx_pw_px;
    uint8_t cmd;
    uint8_t aa;
    if(DataPipe->PacketWidth>32)
        return R_ERROR;
    switch(DataPipe->N) {
    case DATA_PIPE_0:
        len = 5;
        rx_addr = R_RX_ADDR_P0;
        rx_pw_px = R_RX_PW_P0;
        aa = R_EN_AA_P0;
        break;
    case DATA_PIPE_1:
        len = 5;
        rx_addr = R_RX_ADDR_P1;
        rx_pw_px = R_RX_PW_P1;
        aa = R_EN_AA_P1;
        break;
    case DATA_PIPE_2:
        len = 1;
        rx_addr = R_RX_ADDR_P2;
        rx_pw_px = R_RX_PW_P2;
        aa = R_EN_AA_P2;
        break;
    case DATA_PIPE_3:
        len = 1;
        rx_addr = R_RX_ADDR_P3;
        rx_pw_px = R_RX_PW_P3;
        aa = R_EN_AA_P3;
        break;
    case DATA_PIPE_4:
        len = 1;
        rx_addr = R_RX_ADDR_P4;
        rx_pw_px = R_RX_PW_P4;
        aa = R_EN_AA_P4;
        break;
    case DATA_PIPE_5:
        len = 1;
        rx_addr = R_RX_ADDR_P5;
        rx_pw_px = R_RX_PW_P5;
        aa = R_EN_AA_P5;
        break;
    default:
        return R_ERROR;
    }

    uint8_t r_en_rxaddr = nRF24L01_ReadRegister(R_EN_RXADDR);
    if(DataPipe->State == ENABLE)
        r_en_rxaddr |= DataPipe->N;
    else
        r_en_rxaddr &= ~(DataPipe->N);
    nRF24L01_WriteRegister(R_EN_RXADDR, r_en_rxaddr);

    nRF24L01_WriteRegister(rx_pw_px, DataPipe->PacketWidth);

    uint8_t r_en_aa = nRF24L01_ReadRegister(R_EN_AA);
    if(DataPipe->AutoACK)
        r_en_aa |= aa;
    else
        r_en_aa &= ~aa;
    nRF24L01_WriteRegister(R_EN_AA, r_en_aa);

    cmd = WRITE_REGISTER | rx_addr;
    Assert_CS();
    SPI_TransmitReceive(&cmd, &Status, 1);
    SPI_TransmitReceive((uint8_t *)&DataPipe->Address[0], NULL, len);
    Deassert_CS();
    return R_OK;
}

RadioResult_TypeDef nRF24L01_SetTXAddress(Address_t addr, uint8_t addr_len) {
    uint8_t cmd = WRITE_REGISTER | R_TX_ADDR;
    uint8_t code;

    switch(addr_len) {
    case 3:
        code = R_SETUP_AW_3_BYTES;
        break;
    case 4:
        code = R_SETUP_AW_4_BYTES;
        break;
    case 5:
        code = R_SETUP_AW_5_BYTES;
        break;
    default:
        return R_ERROR;
    }
    nRF24L01_WriteRegister(R_SETUP_AW, code);

    Assert_CS();
    SPI_TransmitReceive(&cmd, &Status, 1);
    SPI_TransmitReceive((uint8_t *)&addr[0], NULL, addr_len);
    Deassert_CS();
    return R_OK;
}

RadioResult_TypeDef nRF24L01_Transmit(void *txbuf, uint8_t len) {
    volatile static uint32_t dly;
    nRF24L01_WriteRegister(R_STATUS, R_STATUS_MAX_RT);
    uint8_t cfg = nRF24L01_ReadRegister(R_CONFIG) & ~R_CONFIG_PRIM_RX_CONTROL;
    nRF24L01_WriteRegister(R_CONFIG, cfg);
    nRF24L01_IssueCmd(FLUSH_TX);
    nRF24L01_WriteTXPayload(txbuf, len);
    Assert_CE();
    Delay(1);
    //nRF24L01_GetStatus(Radio);             /* A kind of of delay */
    //nRF24L01_GetStatus(Radio);             /* A kind of of delay */
    //nRF24L01_GetStatus(Radio);             /* A kind of of delay */
    Deassert_CE();
    return R_OK;
}

RadioResult_TypeDef nRF24L01_Receive(void *rxbuf, uint32_t timeout) {
    uint8_t cmd = R_RX_PAYLOAD;
    uint8_t len;
    uint8_t status;

    uint8_t cfg = nRF24L01_ReadRegister(R_CONFIG) | R_CONFIG_PRIM_RX_CONTROL;
    nRF24L01_WriteRegister(R_CONFIG, cfg);
    nRF24L01_IssueCmd(FLUSH_RX);
    Assert_CE();

    while(1) {
        status = nRF24L01_ReadRegister(R_STATUS);
        if(status & R_STATUS_RX_DR)
            break;
    }
    Deassert_CE();
    nRF24L01_WriteRegister(R_STATUS, status);
    len = nRF24L01_ReadRegister(0x60);
    Assert_CS();
    SPI_TransmitReceive(&cmd, &Status, 1);
    SPI_TransmitReceive(NULL, rxbuf, len);
    Deassert_CS();

    return R_OK;
}

uint8_t nRF24L01_GetStatus() {
    uint8_t nop = NOP;
    uint8_t status;
    Assert_CS();
    SPI_TransmitReceive(&nop, &status, 1);
    Deassert_CS();
    Status = status;
    return status;
}

uint8_t nRF24L01_GetRXPayloadWidth() {
    uint8_t cmd[2] = {R_RX_PL_WID, 0xff};
    __packed struct {
        uint8_t Status;
        uint8_t PL_Width;
    } rxbuf = {0, 0};

    Assert_CS();
    SPI_TransmitReceive(cmd, (void *)&rxbuf, 2);
    Deassert_CS();
    Status = rxbuf.Status;

    return rxbuf.PL_Width;
}

uint8_t nRF24L01_GetFullRXPayload(void *buffer) {
    uint8_t cmd[2] = {R_RX_PL_WID, 0xff};
    uint8_t PL_Width = nRF24L01_GetRXPayloadWidth();

    if(PL_Width) {
        cmd[0] = R_RX_PAYLOAD;
        Assert_CS();
        SPI_TransmitReceive(&cmd[0], &Status, 1);
        SPI_TransmitReceive(NULL, buffer, PL_Width);
        Deassert_CS();
    }

    return PL_Width;
}

uint8_t nRF24L01_GetRXPayload(void *buffer, uint8_t count) {
    uint8_t cmd[2] = {R_RX_PL_WID, 0xff};
    uint8_t PL_Width = nRF24L01_GetRXPayloadWidth();

    if(PL_Width < count)
        count = PL_Width;

    if(count) {
        cmd[0] = R_RX_PAYLOAD;
        Assert_CS();
        SPI_TransmitReceive(&cmd[0], &Status, 1);
        SPI_TransmitReceive(NULL, buffer, count );
        Deassert_CS();
    }

    return count;
}

void nRF24L01_StartReceiving() {
    uint8_t cfg = nRF24L01_ReadRegister(R_CONFIG) | R_CONFIG_PRIM_RX_CONTROL;
    nRF24L01_WriteRegister(R_CONFIG, cfg);
    //nRF24L01_IssueCmd(FLUSH_RX);
    Assert_CE();
}

void nRF24L01_Stop() {
    Deassert_CE();
}

void nRF24L01_ITConfig(uint8_t IRQSource, FunctionalState NewState) {
    uint8_t r_config = nRF24L01_ReadRegister(R_CONFIG);

    if(NewState == ENABLE)
        r_config &= ~IRQSource;
    else
        r_config |= IRQSource;

    nRF24L01_WriteRegister(R_CONFIG, r_config);
}

uint8_t nRF24L01_GetITSource() {
    nRF24L01_WriteRegister(R_STATUS, IT_SOURCE_MASK);
    return Status & IT_SOURCE_MASK;
}

/* ----------------------------- Private routines ----------------------------*/

void SPI_TransmitReceive(uint8_t *txbuf, uint8_t *rxbuf, uint8_t len) {
    uint8_t i;
    uint16_t spi_status;
    uint8_t r;
    uint8_t byte;
    for(i=0;i<len;i++) {
        byte = txbuf?txbuf[i]:0xff;
        SPI->DR = byte;
        do
            spi_status = SPI->SR;
        while((spi_status & (SPI_SR_RXNE | SPI_SR_RXNE)) != (SPI_SR_RXNE | SPI_SR_RXNE));
        r = SPI->DR;
        if(rxbuf)
            rxbuf[i] = r;
    }
}

void Delay(uint8_t time) {
    vTaskDelay(time);
}

void nRF24L01_IssueCmd(uint8_t cmd) {
    uint8_t rxbuf;
    Assert_CS();
    SPI_TransmitReceive(&cmd, &rxbuf, 1);
    Deassert_CS();
    Status = rxbuf;
}

uint8_t nRF24L01_ReadRegister(uint8_t addr) {
    uint8_t txbuf[2];
    uint8_t rxbuf[2];

    txbuf[0] = READ_REGISTER | addr;
    txbuf[1] = 0xff;

    Assert_CS();
    SPI_TransmitReceive(txbuf, rxbuf, 2);
    Deassert_CS();
    Status = rxbuf[0];
    return rxbuf[1];
}

void nRF24L01_WriteRegister(uint8_t addr, uint8_t value) {
    uint8_t txbuf[2];
    uint8_t rxbuf[2];

    txbuf[0] = WRITE_REGISTER | addr;
    txbuf[1] = value;

    Assert_CS();
    SPI_TransmitReceive(txbuf, rxbuf, 2);
    Deassert_CS();
    Status = rxbuf[0];
}

void nRF24L01_WriteTXPayload(void *buf, uint8_t len) {
    uint8_t cmd = W_TX_PAYLOAD;
    Assert_CS();
    SPI_TransmitReceive(&cmd, &Status, 1);
    SPI_TransmitReceive(buf, NULL, len);
    Deassert_CS();
}
