#ifndef ILI9341_STM32_DRIVER_H
#define ILI9341_STM32_DRIVER_H

#include "stm32h7xx_hal.h"


#define ILI9341_SCREEN_HEIGHT 320
#define ILI9341_SCREEN_WIDTH 	240

//SPI INSTANCE
#define HSPI_INSTANCE							&hspi1

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

#define DARKBLUE    0X01CF
#define LIGHTBLUE   0X7D7C
#define GRAYBLUE    0X5458

#define LIGHTGREEN  0X841F
#define LIGHTGRAY   0XEF5B
#define LGRAY 			0XC618

#define LGRAYBLUE   0XA651
#define LBBLUE      0X2B12

#define SCREEN_VERTICAL_1			0
#define SCREEN_HORIZONTAL_1		1
#define SCREEN_VERTICAL_2			2
#define SCREEN_HORIZONTAL_2		3

typedef struct
{
  int16_t X;
  int16_t Y;
} Point, * pPoint;

#define ILI9341_ID                  0x9341


void 			ILI9341_Init(void);
void      ILI9341_DisplayOn(void);
void      ILI9341_DisplayOff(void);
uint16_t  ILI9341_GetLcdPixelWidth(void);
uint16_t  ILI9341_GetLcdPixelHeight(void);

void ILI9341_Clear(uint32_t Color);


void ILI9341_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code);
uint32_t ILI9341_ReadPixel(uint16_t Xpos, uint16_t Ypos);
void ILI9341_Clear(uint32_t Color);
void ILI9341_SelectLayer(uint32_t LayerIndex);
void ILI9341_SetLayerVisible(uint32_t LayerIndex, FunctionalState state);
void ILI9341_SetLayerVisible_NoReload(uint32_t LayerIndex, FunctionalState State);
void ILI9341_SetTransparency(uint32_t LayerIndex, uint8_t Transparency);
void ILI9341_SetTransparency_NoReload(uint32_t LayerIndex, uint8_t Transparency);
void ILI9341_SetLayerAddress(uint32_t LayerIndex, uint32_t Address);
void ILI9341_SetLayerAddress_NoReload(uint32_t LayerIndex, uint32_t Address);
void ILI9341_SetLayerWindow(uint16_t LayerIndex, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void ILI9341_SetLayerWindow_NoReload(uint16_t LayerIndex, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height);
void ILI9341_SetColorKeying(uint32_t LayerIndex, uint32_t RGBValue);
void ILI9341_SetColorKeying_NoReload(uint32_t LayerIndex, uint32_t RGBValue);
void ILI9341_ResetColorKeying(uint32_t LayerIndex);
void ILI9341_ResetColorKeying_NoReload(uint32_t LayerIndex);
void ILI9341_Relaod(uint32_t ReloadType);
void ILI9341_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint16_t Color);
void ILI9341_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint16_t Color);
void ILI9341_DrawLine(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2, uint16_t Color);
void ILI9341_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height, uint16_t Color);
void ILI9341_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius, uint16_t Color);
void ILI9341_DrawPolygon(pPoint Points, uint16_t PointCount, uint16_t Color);
void ILI9341_DrawEllipse(int Xpos, int Ypos, int XRadius, int YRadius, uint16_t Color);
void ILI9341_DrawBitmap(uint32_t X, uint32_t Y, uint8_t *pBmp);
void ILI9341_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height, uint16_t Color);
void ILI9341_FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius, uint16_t Color);
void ILI9341_FillTriangle(uint16_t X1, uint16_t X2, uint16_t X3, uint16_t Y1, uint16_t Y2, uint16_t Y3, uint16_t Color);
void ILI9341_FillPolygon(pPoint Points, uint16_t PointCount, uint16_t Color);
void ILI9341_FillEllipse(int Xpos, int Ypos, int XRadius, int YRadius, uint16_t Color);



#endif

