/**
 * @author  Tilen Majerle
 * @email   tilen@majerle.eu
 * @website http://stm32f4-discovery.com
 * @link    http://stm32f4-discovery.com/2015/05/library-61-ssd1306-oled-i2c-lcd-for-stm32f4xx
 * @version v1.0
 * @ide     Keil uVision
 * @license GNU GPL v3
 * @brief   Library for 128x64 SSD1306 I2C LCD
 *
@verbatim
   ----------------------------------------------------------------------
    Copyright (C) Tilen Majerle, 2015

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
@endverbatim
 */
#ifndef TM_SSD1306_H
#define TM_SSD1306_H 100

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup TM_STM32F4xx_Libraries
 * @{
 */

/**
 * @defgroup TM_SSD1306
 * @brief    Library for 128x64 SSD1306 I2C LCD - http://stm32f4-discovery.com/2015/05/library-61-ssd1306-oled-i2c-lcd-for-stm32f4xx
 * @{
 *
 * This SSD1306 LCD uses I2C for communication
 *
 * Library features functions for drawing lines, rectangles and circles.
 *
 * It also allows you to draw texts and characters using appropriate functions provided in library.
 *
 * \par Default pinout
 *
@verbatim
SSD1306    |STM32F4xx    |DESCRIPTION

VCC        |3.3V         |
GND        |GND          |
SCL        |PA8          |Serial clock line
SDA        |PC9          |Serial data line
@endverbatim
 *
 * \par Select custom I2C settings
 *
 * Use defines.h file for custom settings
 *
@verbatim
//Select custom I2C
#define SSD1306_I2C              I2C3
#define SSD1306_I2C_PINSPACK     TM_I2C_PinsPack_1

//Select custom I2C address
#define SSD1306_I2C_ADDR         0x78

//Select custom width and height if your LCD differs in size
#define SSD1306_WIDTH            128
#define SSD1306_HEIGHT           64
@endverbatim
 *
 * \par Changelog
 *
@verbatim
 Version 1.0
  - First release
@endverbatim
 *
 * \par Dependencies
 *
@verbatim
 - STM32F4xx
 - STM32F4xx RCC
 - defines.h
 - TM I2C
 - TM FONTS
 - TM DELAY
 - string.h
 - stdlib.h
@endverbatim
 */

#include "defines.h"
#include "stm32f0xx_ll_i2c.h"
#include "tm_stm32f4_fonts.h"

#include "stdlib.h"
#include "string.h"

#if defined (__GNUC__)
#include "iar_to_gcc.h"
#endif


/**
 * @defgroup TM_SSD1306_Macros
 * @brief    Library defines
 * @{
 */

/* I2C settings */
#ifndef SSD1306_I2C
#define SSD1306_I2C              I2C1
#define SSD1306_I2C_PINSPACK     TM_I2C_PinsPack_1
#endif

/* I2C address */
#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR         0x78
/* Use defines.h for custom definitions */
//#define SSD1306_I2C_ADDR       0x7A
#endif

/* SSD1306 settings */
/* SSD1306 width in pixels */
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH            128
#define SSD1306_CHR_WIDTH		 (SSD1306_WIDTH/8)
#endif
/* SSD1306 LCD height in pixels */
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT           64
#endif

#define BUFFER_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8)
extern const __packed __ro_placement uint8_t startupLogo[BUFFER_SIZE];

/**
 * @}
 */

/**
 * @defgroup TM_SSD1306_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @}
 */

/**
 * @defgroup TM_SSD1306_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes SSD1306 LCD
 * @param  None
 * @retval Initialization status:
 *           - 0: LCD was not detected on I2C port
 *           - > 0: LCD initialized OK and ready to use
 */
uint8_t TM_SSD1306_Init(void);

void SSD1306_On(void);
void SSD1306_Off(void);
/**
 * @brief  Updates buffer from internal RAM to LCD
 * @note   This function must be called each time you do some changes to LCD, to update buffer from RAM to LCD
 * @param  None
 * @retval None
 */
void TM_SSD1306_UpdateScreen(uint32_t Row, uint32_t Offset, uint32_t Length, const char *Buffer);
void TM_SSD1306_ClearScreen(uint32_t From, uint32_t To);
/**
 * @brief  Toggles pixels invertion inside internal RAM
 * @note   @ref TM_SSD1306_UpdateScreen() must be called after that in order to see updated LCD screen
 * @param  None
 * @retval None
 */
void TM_SSD1306_ToggleInvert(void);

void SSD1306_DrawLine(char *buffer, uint32_t line, uint32_t inverted);
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
