
//	MIT License
//
//	Copyright (c) 2017 Matej Artnak
//
//	Permission is hereby granted, free of charge, to any person obtaining a copy
//	of this software and associated documentation files (the "Software"), to deal
//	in the Software without restriction, including without limitation the rights
//	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//	copies of the Software, and to permit persons to whom the Software is
//	furnished to do so, subject to the following conditions:
//
//	The above copyright notice and this permission notice shall be included in all
//	copies or substantial portions of the Software.
//
//	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//	SOFTWARE.
//
//
// Modyfy to ILI9488 by NSJ
//
//-----------------------------------

#ifndef ILI9488_STM32_DRIVER_H
#define ILI9488_STM32_DRIVER_H

#include "stm32h7xx_hal.h"


#define ILI9488_SCREEN_HEIGHT 320
#define ILI9488_SCREEN_WIDTH 	480

//SPI INSTANCE
#define HSPI_INSTANCE							&hspi1

#define BURST_MAX_SIZE 	600

#define BLACK       0x0000      
#define NAVY        0x000F      
#define DARKGREEN   0x03E0      
#define DARKCYAN    0x03EF      
#define MAROON      0x7800      
#define PURPLE      0x780F      
#define OLIVE       0x7BE0      
#define LIGHTGREY   0xC618      
#define DARKGREY    0x7BEF      
#define BLUE        0x001F      
#define GREEN       0x07E0      
#define CYAN        0x07FF      
#define RED         0xF800     
#define MAGENTA     0xF81F      
#define YELLOW      0xFFE0      
#define WHITE       0xFFFF      
#define ORANGE      0xFD20      
#define GREENYELLOW 0xAFE5     
#define PINK        0xF81F

#define SCREEN_VERTICAL_1			0
#define SCREEN_HORIZONTAL_1		1
#define SCREEN_VERTICAL_2			2
#define SCREEN_HORIZONTAL_2		3

/* Define TFT String Mode */
#define TFT_STRING_MODE_NO_BACKGROUND		0x01
#define TFT_STRING_MODE_BACKGROUND			0x00


void ILI9488_SPI_Init(void);
void ILI9488_SPI_Send(unsigned char SPI_Data);
void ILI9488_Write_Command(uint8_t Command);
void ILI9488_Write_Data(uint8_t Data);
void ILI9488_Set_Address(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2);
void ILI9488_Reset(void);
void ILI9488_Set_Rotation(uint8_t Rotation);
void ILI9488_Init(void);
void ILI9488_Fill_Screen(uint16_t Colour);
void ILI9488_Draw_Colour(uint16_t Colour);
void ILI9488_Draw_Pixel(uint16_t X,uint16_t Y,uint16_t Colour);
void ILI9488_Draw_Colour_Burst(uint16_t Colour, uint32_t Size);

uint8_t ILI9488_GetDirection(void);
uint16_t ILI9488_GetWidth(void);
uint16_t ILI9488_GetHeight(void);

void ILI9488_Draw_Fill_Rectangle(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint16_t Colour);
void ILI9488_Draw_Horizontal_Line(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Colour);
void ILI9488_Draw_Vertical_Line(uint16_t X, uint16_t Y, uint16_t Height, uint16_t Colour);
	
void ILI9488_Draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);
void ILI9488_Draw_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ILI9488_Draw_Circle(uint16_t x, uint16_t y, uint16_t radian, uint16_t color);

void ILI9488_SetTextColor(uint16_t color);
void ILI9488_SetBackgroundColor(uint16_t color);
void ILI9488_Puts8x16(uint16_t x, uint16_t y, uint8_t *string, uint8_t TFT_STRING_MODE);
void ILI9488_Puts14x24(uint16_t x, uint16_t y, uint8_t *string, uint8_t TFT_STRING_MODE);
void ILI9488_Puts18x32(uint16_t x, uint16_t y, uint8_t *string, uint8_t TFT_STRING_MODE);
void ILI9488_Puts26x48(uint16_t x, uint16_t y, uint8_t *string, uint8_t TFT_STRING_MODE);

#endif

