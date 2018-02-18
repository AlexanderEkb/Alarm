/**
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2015
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#include <stdlib.h>
#include "display.h"
#include "power.h"
#include "FreeRTOS.h"
#include "task.h"

#if defined (__GNUC__)
#include "iar_to_gcc.h"
#endif

#define COMMAND (0x00)
#define DATA    (0x40)

/* Write command */
#define SSD1306_WRITECOMMAND(command)      TM_I2C_Write(COMMAND, (command))
/* Write data */
#define SSD1306_WRITEDATA(data)            TM_I2C_Write(DATA, (data))
/* Absolute value */
#define ABS(x)   ((x) > 0 ? (x) : -(x))


void TM_I2C_Write(uint32_t ctl, uint32_t cmd);
void TM_I2C_WriteMulti(uint8_t* data, uint16_t count);

/* SSD1306 data buffer */
#include "font.inc"

/* Private SSD1306 structure */
typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
} SSD1306_t;

/* Private variable */
static SSD1306_t SSD1306;

uint8_t TM_SSD1306_Init(void) {
	/* A little delay */
	vTaskDelay(100);

    LL_I2C_SetSlaveAddr(I2C1, SSD1306_I2C_ADDR);
	/* Init LCD */
	SSD1306_WRITECOMMAND(0xAE); //display off
	SSD1306_WRITECOMMAND(0x20); //Set Memory Addressing Mode
	SSD1306_WRITECOMMAND(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
	SSD1306_WRITECOMMAND(0xB0); //Set Page Start Address for Page Addressing Mode,0-7
	SSD1306_WRITECOMMAND(0xC8); //Set COM Output Scan Direction
	SSD1306_WRITECOMMAND(0x00); //---set low column address
	SSD1306_WRITECOMMAND(0x10); //---set high column address
	SSD1306_WRITECOMMAND(0x40); //--set start line address
	SSD1306_WRITECOMMAND(0x81); //--set contrast control register
	SSD1306_WRITECOMMAND(0x7F); // ...contrast value
	SSD1306_WRITECOMMAND(0xA1); //--set segment re-map 0 to 127
	SSD1306_WRITECOMMAND(0xA6); //--set normal display
	SSD1306_WRITECOMMAND(0xA8); //--set multiplex ratio(1 to 64)
	SSD1306_WRITECOMMAND(0x3F); //
	SSD1306_WRITECOMMAND(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
	SSD1306_WRITECOMMAND(0xD3); //-set display offset
	SSD1306_WRITECOMMAND(0x00); // ...no offset
	SSD1306_WRITECOMMAND(0xD5); //--set display clock divide ratio/oscillator frequency
	SSD1306_WRITECOMMAND(0xF0); //--set divide ratio
	SSD1306_WRITECOMMAND(0xD9); //--set pre-charge period
	SSD1306_WRITECOMMAND(0x22); //
	SSD1306_WRITECOMMAND(0xDA); //--set com pins hardware configuration
	SSD1306_WRITECOMMAND(0x12);
	SSD1306_WRITECOMMAND(0xDB); //--set vcomh
	SSD1306_WRITECOMMAND(0x20); //0x20,0.77xVcc
	//SSD1306_WRITECOMMAND(0x8D); //--set DC-DC enable
	SSD1306_WRITECOMMAND(0x14); //

	/* Return OK */
	return 1;
}

void TM_SSD1306_UpdateScreen(uint32_t Row, uint32_t Offset, uint32_t Length, const char *Buffer) {
    Power_Lock();
	// Set Page Start Address for Page Addressing Mode
	SSD1306_WRITECOMMAND(0xB0 + Row);
	// Set Lower Column Start Address for Page Addressing Mode
	SSD1306_WRITECOMMAND(0x00 | (Offset & 0x0000000f));
	// Set Higher Column Start Address for Page Addressing Mode
	SSD1306_WRITECOMMAND(0x10 | ((Offset & 0x000000f0) >> 4));

	/* Write multi data */
	TM_I2C_WriteMulti(&Buffer[0], Length);
    Power_Unlock();
}

void TM_SSD1306_ClearScreen(uint32_t From, uint32_t To) {
	uint8_t m;
	char buffer[SSD1306_WIDTH];

	memset(buffer, 0, 128);
    Power_Lock();
    for(m=From;m<=To;m++) {
    	// Set Page Start Address for Page Addressing Mode
    	SSD1306_WRITECOMMAND(0xB0 + m);
    	// Set Lower Column Start Address for Page Addressing Mode
    	SSD1306_WRITECOMMAND(0x00);
    	// Set Higher Column Start Address for Page Addressing Mode
    	SSD1306_WRITECOMMAND(0x10);

    	/* Write multi data */
    	TM_I2C_WriteMulti(&buffer[0], SSD1306_WIDTH);
    }
    Power_Unlock();
}

void TM_SSD1306_ToggleInvert(void) {
	/* Toggle invert */
	SSD1306.Inverted = !SSD1306.Inverted;
}

void SSD1306_On(void) {
	SSD1306_WRITECOMMAND(0x8D);
	SSD1306_WRITECOMMAND(0x14);
	SSD1306_WRITECOMMAND(0xAF);
}
void SSD1306_Off(void) {
	SSD1306_WRITECOMMAND(0x8D);
	SSD1306_WRITECOMMAND(0x10);
	SSD1306_WRITECOMMAND(0xAE);
}

void SSD1306_DrawLine(char *buffer, uint32_t line, uint32_t inverted) {
	uint32_t chr, col, addr, letter, byte;
	char Buffer[SSD1306_WIDTH];

	addr = 0;
	for(chr=0; chr<SSD1306_CHR_WIDTH; chr++) {
		letter = buffer[chr];
		for(col=0; col<8; col++) {
			byte = font[letter][col];
			if(inverted)
				byte = ~byte;
			Buffer[addr] = byte;
			addr++;
		}
	}
	TM_SSD1306_UpdateScreen(line, 0, SSD1306_WIDTH, Buffer);
}

/******************************************************************************
 * I2C Driver Miniport                                                        *
 ******************************************************************************/

static void TM_I2C_StartWrite(uint32_t len) {
    //LL_I2C_GenerateStartCondition(I2C1);
    LL_I2C_HandleTransfer(I2C1, SSD1306_I2C_ADDR, LL_I2C_ADDRSLAVE_7BIT,
                          len, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    while(I2C1->CR2 & I2C_CR2_START);
}

static void TM_I2C_TransmitOneByte(uint32_t byte, uint32_t theLastOne) {
    uint32_t ISR = 0;
    uint32_t cond = theLastOne?I2C_ISR_STOPF:I2C_ISR_TXIS;

    LL_I2C_TransmitData8(I2C1, byte);
    while(!(ISR & cond))
        ISR = I2C1->ISR;
}

void TM_I2C_Write(uint32_t ctl, uint32_t byte) {
    uint32_t ISR = 0;
    do{
        ISR = I2C1->ISR;
    } while(ISR & I2C_ISR_BUSY);
    TM_I2C_StartWrite(2);
    ISR = I2C1->ISR;
	TM_I2C_TransmitOneByte(ctl, 0);
	TM_I2C_TransmitOneByte(byte, 1);
    // Stop condition will be generated automatically
}

void TM_I2C_WriteMulti(uint8_t* data, uint16_t count) {
    uint32_t ISR = 0;
    do{
        ISR = I2C1->ISR;
    } while(ISR & I2C_ISR_BUSY);
    TM_I2C_StartWrite(count + 1);
	TM_I2C_TransmitOneByte(DATA, 0);
    while(count--)
        TM_I2C_TransmitOneByte(*(data++), !count);
    // Stop condition will be generated automatically
}
