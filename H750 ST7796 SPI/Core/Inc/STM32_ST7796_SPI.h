
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
//


#ifndef ST7796_STM32_DRIVER_H
#define ST7796_STM32_DRIVER_H

#include "stm32h7xx_hal.h"


//SPI INSTANCE
#define HSPI_INSTANCE					&hspi1


#define BURST_MAX_SIZE 	500

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

uint16_t ST7796_GetWidth(void);
uint16_t ST7796_GetHeight(void);

void ST7796_SPI_Init(void);
void ST7796_SPI_Send(unsigned char SPI_Data);
void ST7796_Write_Command(uint8_t Command);
void ST7796_Write_Data(uint8_t Data);
void ST7796_Set_Address(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2);
void ST7796_Reset(void);
void ST7796_Set_Rotation(uint8_t Rotation);
void ST7796_Enable(void);
void ST7796_Init(void);
void ST7796_Fill_Screen(uint16_t Colour);
void ST7796_Draw_Colour(uint16_t Colour);
void ST7796_Draw_Pixel(uint16_t X,uint16_t Y,uint16_t Colour);
void ST7796_Draw_Colour_Burst(uint16_t Colour, uint32_t Size);


void ST7796_Draw_FillRectangle(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint16_t Colour);
void ST7796_Draw_Horizontal_Line(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Colour);
void ST7796_Draw_Vertical_Line(uint16_t X, uint16_t Y, uint16_t Height, uint16_t Colour);
void ST7796_Draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);
void ST7796_Draw_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void ST7796_Draw_Circle(uint16_t x, uint16_t y, uint16_t radian, uint16_t color);
void ST7796_Scroll(uint16_t position);
void BSP_LCD_SetBackLight(uint16_t brightness);	// 0 <= brightness < 100
uint8_t ST7796_GetDirection(void);

#endif

