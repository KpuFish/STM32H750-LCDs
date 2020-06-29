#ifndef __ili9341_H
#define __ili9341_H
#ifdef __cplusplus
 extern "C" {
#endif


#include "main.h"

#define LCD_W 240
#define LCD_H 320

#define USE_HORIZONTAL  	 0

#define	LCD_REG 	*(volatile uint16_t *)0xC0000000
#define	LCD_RAM 	*(volatile uint16_t *)0xC0080000


#define WHITE       0xFFFF
#define BLACK      	0x0000
#define BLUE       	0x001F
#define BRED        0XF81F
#define GRED 			 	0XFFE0
#define GBLUE			 	0X07FF
#define RED         0xF800
#define MAGENTA     0xF81F
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define BROWN 			0XBC40
#define BRRED 			0XFC07
#define GRAY  			0X8430
//GUI��ɫ

#define DARKBLUE    0X01CF
#define LIGHTBLUE   0X7D7C
#define GRAYBLUE    0X5458

#define LIGHTGREEN  0X841F
#define LIGHTGRAY   0XEF5B
#define LGRAY 			0XC618

#define LGRAYBLUE   0XA651
#define LBBLUE      0X2B12

/* Define TFT String Mode */
#define TFT_STRING_MODE_NO_BACKGROUND		0x01
#define TFT_STRING_MODE_BACKGROUND			0x00

extern uint16_t POINT_COLOR, BACK_COLOR;

#define LCD_SetTextColor(c)				(POINT_COLOR = c)
#define LCD_SetBackgroundColor(c)	(BACK_COLOR = c)
#define LCD_GetTextColor()				POINT_COLOR
#define LCD_GetBackgroundColor()	BACK_COLOR


 typedef struct
 {
	 uint16_t width;			//
	 uint16_t height;			//
	 uint16_t id;				  //
	 uint8_t  dir;			  //
	 uint16_t	 wramcmd;		//
	 uint16_t  rramcmd;   //
	 uint16_t  setxcmd;		//
	 uint16_t  setycmd;		//
 }_lcd_dev;


 void LCD_Init(void);
 uint16_t LCD_read(void);
 void LCD_Clear(uint16_t Color);
 void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);
 void LCD_DrawPoint(uint16_t x,uint16_t y);
 void LCD_draw_point(uint16_t x, uint16_t y, uint16_t color);
 uint16_t LCD_ReadPoint(uint16_t x, uint16_t y);
 void LCD_SetWindows(uint16_t xStar, uint16_t yStar,uint16_t xEnd,uint16_t yEnd);
 uint16_t LCD_RD_DATA(void);
 void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue);
 void LCD_WR_REG(uint16_t data);
 void LCD_WR_DATA(uint16_t data);
 void LCD_ReadReg(uint16_t LCD_Reg, uint8_t *Rval, int n);
 void LCD_WriteRAM_Prepare(void);
 void LCD_ReadRAM_Prepare(void);
 void Lcd_WriteData_16Bit(uint16_t Data);
 uint16_t Lcd_ReadData_16Bit(void);
 void LCD_direction(uint8_t direction );
 uint32_t LCD_Read_ID(void);
 uint16_t LCD_GetWidth(void);
 uint16_t LCD_GetHeight(void);

void LCD_puts26x48(uint16_t x, uint16_t y, uint8_t *string, uint8_t TFT_STRING_MODE);
void LCD_puts18x32(uint16_t x,uint16_t y,uint8_t *string,uint8_t TFT_STRING_MODE);
void LCD_puts14x24(uint16_t x,uint16_t y,uint8_t *string,uint8_t TFT_STRING_MODE);
void LCD_puts8x16(uint16_t x,uint16_t y,uint8_t *string,uint8_t TFT_STRING_MODE);
void LCD_scroll(uint16_t position);
void LCD_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);
void LCD_draw_h_line(uint16_t x1, uint16_t y1, uint16_t x2 ,uint16_t color);
void LCD_draw_v_line(uint16_t x1, uint16_t y1, uint16_t y2 ,uint16_t color);
void LCD_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color);
void LCD_draw_circle(uint16_t x,uint16_t y,uint16_t radian,uint16_t color);
void LCD_puts_image_pos(const unsigned char* image_arr, uint16_t x, uint16_t y, uint16_t size_x, uint16_t size_y);
void LCD_puts_image(const unsigned char* image_arr);
void LCD_putchar(uint16_t x,uint16_t y,uint8_t num,uint8_t mode);
void LCD_putchar14x24(uint16_t x,uint16_t y,uint8_t data,uint8_t mode);
void LCD_putchar18x32(uint16_t x,uint16_t y,uint8_t data,uint8_t mode);
void LCD_putchar26x48(uint16_t x,uint16_t y,uint8_t data,uint8_t mode);

void LCD_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void LCD_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);
void LCD_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
#ifdef __cplusplus
}
#endif
#endif /*__ili9341_H */
