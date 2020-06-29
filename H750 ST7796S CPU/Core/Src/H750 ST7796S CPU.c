#include "H750 ST7796S CPU.h"
#include "LCD_font.h"

#define ABS(x)   ((x) > 0 ? (x) : -(x))

uint16_t POINT_COLOR = 0x0000, BACK_COLOR = 0xFFFF;
uint16_t DeviceCode;
_lcd_dev lcddev;
static uint8_t Direction;

#define LCD_WR_DATA(data)	(LCD_RAM = data)
#define LCD_WR_REG(data)	(LCD_REG = data)

void LCD_WriteReg(uint16_t LCD_Reg, uint16_t LCD_RegValue)
{
	LCD_REG = LCD_Reg;
	LCD_RAM = LCD_RegValue;
}

void LCD_ReadReg(uint16_t LCD_Reg, uint8_t *Rval, int n)
{
	LCD_REG = LCD_Reg;
	while(n--)
	{
		*(Rval++) = LCD_RAM;
	}
}

void LCD_WriteRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.wramcmd);
}

void LCD_ReadRAM_Prepare(void)
{
	LCD_WR_REG(lcddev.rramcmd);
}

uint16_t Lcd_ReadData_16Bit(void)
{
	uint16_t r;
	//dummy data
	r = LCD_RD_DATA();
	//delay_us(1);// delay 1us
	HAL_Delay(0);
	//real color
	r = LCD_RD_DATA();
	return r;
}

void LCD_On(void)
{
	LCD_WR_REG(0X29);
}

void LCD_Off(void)
{
	LCD_WR_REG(0X28);
}

void LCD_DrawPoint(uint16_t x, uint16_t y)
{
	LCD_SetCursor(x,y);
	LCD_WR_DATA(POINT_COLOR);
}

void LCD_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
	LCD_SetCursor(x,y);
	LCD_WR_DATA(color);
}


uint16_t LCD_ReadPoint(uint16_t x, uint16_t y)
{
	uint16_t color;
	if(x >= lcddev.width || y >= lcddev.height)
	{
		return 0;
	}
	LCD_SetCursor(x, y);
	LCD_ReadRAM_Prepare();
	color = Lcd_ReadData_16Bit();
	return color;
}

void LCD_Clear(uint16_t Color)
{
  unsigned int i;
  uint32_t total_point = lcddev.width * lcddev.height;

	LCD_SetWindows(0, 0, lcddev.width - 1, lcddev.height - 1);
	for(i = 0; i < total_point; i++)
	{
		LCD_RAM = Color;
	}
}

void LCD_Init(void)
{
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_SET);
	HAL_Delay(120);

  LCD_WR_REG(0x01);		//Software Reset
	HAL_Delay(120);

  LCD_WR_REG(0x11);		// Sleep Out
	HAL_Delay(120);

  LCD_WR_REG(0xF0);		//Command Set Control
  LCD_WR_DATA(0xC3);	//D[7:0] = C3h enable command 2 part I

  LCD_WR_REG(0xF0);		//Command Set Control
  LCD_WR_DATA(0x96);	//D[7:0] = 96h enable command 2 part II

  LCD_WR_REG(0x36);		//Memory Data Access Control
  LCD_WR_DATA(0x88);	//MY MX MV ML RGB MH

  LCD_WR_REG(0x3A);		//Interface Pixel Format
  LCD_WR_DATA(0x55);	//¡®101¡¯ = 16bit/pixel

  LCD_WR_REG(0xB0);		//Interface Mode Control
  LCD_WR_DATA(0x80);	//SPI_EN = 1, 3/4 wire serial interface

  LCD_WR_REG(0xB5);
  LCD_WR_DATA(8);			//VFP: Front porch of vertical lines =4
  LCD_WR_DATA(4);			//VBP: Back porch of vertical lines =2
  LCD_WR_DATA(0);
  LCD_WR_DATA(20);		//HBP: Back porch of horizontal dotclk=10

  LCD_WR_REG(0xB6);		//Display Function Control
  LCD_WR_DATA(0x20);	// A0		//D5=1:RGB,D5=0:System Interface D6=1:SYNC Mode   E0
  LCD_WR_DATA(0x02);	// 02
  LCD_WR_DATA(0x3B);

  LCD_WR_REG(0xE8);		//Display Output Ctrl Adjust
  LCD_WR_DATA(0x40);
  LCD_WR_DATA(0x8A);	// 82
  LCD_WR_DATA(0x00);	// 07
  LCD_WR_DATA(0x00);	// 18
  LCD_WR_DATA(0x25);	// 27
  LCD_WR_DATA(0x0A);	// 0A
  LCD_WR_DATA(0x38);	// B6
  LCD_WR_DATA(0x33);	// 33

  LCD_WR_REG(0xC2);		//Power Control 3
  LCD_WR_DATA(0x0A);	// A7

  LCD_WR_REG(0xC5);		// VCOM Control
  LCD_WR_DATA(0x1C);	// 3E

  LCD_WR_REG(0xC6);		//Vcom Offset Register
  LCD_WR_DATA(0x00);

	//POSITIVE GAMMA CORRECTION
	LCD_WR_REG(0xE0);
	LCD_WR_DATA(0xF0);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x02);
	LCD_WR_DATA(0x0A);
	LCD_WR_DATA(0x0D);
	LCD_WR_DATA(0x1D);
	LCD_WR_DATA(0x35);
	LCD_WR_DATA(0x55);
	LCD_WR_DATA(0x45);
	LCD_WR_DATA(0x3C);
	LCD_WR_DATA(0x17);
	LCD_WR_DATA(0x17);
	LCD_WR_DATA(0x18);
	LCD_WR_DATA(0x1B);

	//NEGATIVE GAMMA CORRECTION
	LCD_WR_REG(0xE1);
	LCD_WR_DATA(0xF0);
	LCD_WR_DATA(0x00);
	LCD_WR_DATA(0x02);
	LCD_WR_DATA(0x07);
	LCD_WR_DATA(0x06);
	LCD_WR_DATA(0x04);
	LCD_WR_DATA(0x2E);
	LCD_WR_DATA(0x44);
	LCD_WR_DATA(0x45);
	LCD_WR_DATA(0x0B);
	LCD_WR_DATA(0x17);
	LCD_WR_DATA(0x16);
	LCD_WR_DATA(0x18);
	LCD_WR_DATA(0x1B);

  LCD_WR_REG(0xF0);		//Command Set Control
  LCD_WR_DATA(0x3C);	//D[7:0] = 3Ch disable command 2 part I

  LCD_WR_REG(0xF0);		//Command Set Control
  LCD_WR_DATA(0x69);	//D[7:0] = 69h disable command 2 part II

  HAL_Delay(120);
  LCD_WR_REG(0x29);		// Display ON

  LCD_Direction(1);
}

void LCD_SetWindows(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd)
{
	LCD_WR_REG(lcddev.setxcmd);
	LCD_WR_DATA(xStar>>8);
	LCD_WR_DATA(0x00FF&xStar);
	LCD_WR_DATA(xEnd>>8);
	LCD_WR_DATA(0x00FF&xEnd);

	LCD_WR_REG(lcddev.setycmd);
	LCD_WR_DATA(yStar>>8);
	LCD_WR_DATA(0x00FF&yStar);
	LCD_WR_DATA(yEnd>>8);
	LCD_WR_DATA(0x00FF&yEnd);

	LCD_WriteRAM_Prepare();
}


void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos)
{
	LCD_SetWindows(Xpos,Ypos,Xpos,Ypos);
}

void LCD_Direction(uint8_t direction)
{
	lcddev.setxcmd = 0x2A;
	lcddev.setycmd = 0x2B;
	lcddev.wramcmd = 0x2C;
	lcddev.rramcmd = 0x2E;

	Direction = direction;

	switch(direction)
	{
		case 0:
			lcddev.width = LCD_W;
			lcddev.height = LCD_H;
			LCD_WriteReg(0x36, (1<<7) | (1<<3));	//0 degree
			break;
		case 1:
			lcddev.width = LCD_H;
			lcddev.height = LCD_W;
			LCD_WriteReg(0x36, (1<<7)|(1<<6)|(1<<5)|(1<<3)); //90 degree
			break;
		case 2:
			lcddev.width = LCD_W;
			lcddev.height = LCD_H;
			LCD_WriteReg(0x36, (1<<6)|(1<<3));//180 degree
			break;
		case 3:
			lcddev.width = LCD_H;
			lcddev.height = LCD_W;
			LCD_WriteReg(0x36, (1<<5)|(1<<3));//270 degree
			break;
		default:
			break;
	}
}

uint8_t LCD_GetDirection(void)
{
	return Direction;
}

/*
uint32_t LCD_Read_ID(void)
{
	uint8_t val[4] = {0};

	LCD_ReadReg(0xD3, val, 4);
	return ((val[1] << 16) | (val[2] << 8) | val[3]);
}
*/
uint32_t LCD_Read_ID(void)
{
	uint8_t val[5] = {0};
	uint32_t id;

	LCD_ReadReg(0xD3, val, 5);
	id = (val[2] << 16) | (val[3] << 8) | val[4];

	return (id);
}

uint16_t LCD_GetWidth(void)
{
	return lcddev.width;
}

uint16_t LCD_GetHeight(void)
{
	return lcddev.height;
}

void LCD_puts26x48(uint16_t x, uint16_t y, uint8_t *string, uint8_t TFT_STRING_MODE)
{
  uint8_t i = 0;
  uint8_t font_w = 26;
  uint8_t font_h = 48;

  while(*(string + i) != '\0')
  {
    if(*(string + i) == 0)
    {
      return;
    }

    if(*(string + i) == '\n')
    {
      y += font_h;
      x = 0;
      string++;
    }

    if(x > lcddev.width - font_w)
    {
      x = 0;
      y += font_h;
    }

    if(y > lcddev.height - font_h)
    {
      x = y = 0;
    }

    LCD_putchar26x48(x, y, *(string+i), TFT_STRING_MODE);
    x += font_w;
    i++;
  }
}


void LCD_puts18x32(uint16_t x, uint16_t y, uint8_t *string, uint8_t TFT_STRING_MODE)
{
  uint8_t i=0;
  uint8_t font_w=18;
  uint8_t font_h=32;

  while(*(string+i)!='\0')
  {

    if(*(string+i)==0)
    {
      return;
    }

    if(*(string+i)=='\n')
    {
      x += font_h;
      y = 0;
      string++;
    }

    if(x > lcddev.width-font_w)
    {
      x = 0;
      y += font_h;
    }

    if(y > lcddev.height-font_h)
    {
      x = y = 0;
    }

    LCD_putchar18x32(x, y, *(string+i),TFT_STRING_MODE);
    x += font_w;
    i++;
  }
}


void LCD_puts14x24(uint16_t x, uint16_t y, uint8_t *string,uint8_t TFT_STRING_MODE)
{
  uint8_t i=0;
  uint8_t font_w=14;
  uint8_t font_h=24;

  while(*(string+i)!='\0')
  {

    if(*(string+i)==0)
    {
      return;
    }

    if(*(string+i)=='\n')
    {
      y += font_h;
      x = 0;
      string++;
    }

    if(x > lcddev.width-font_w)
    {
      x = 0;
      y += font_h;
    }

    if(y > lcddev.height-font_h)
    {
      x = y = 0;
    }

    LCD_putchar14x24(x, y, *(string+i), TFT_STRING_MODE);
    x += font_w;
    i++;
  }
}

void LCD_puts8x16(uint16_t x, uint16_t y, uint8_t *string,uint8_t TFT_STRING_MODE)
{
  uint8_t i=0;
  uint8_t font_w=8;
  uint8_t font_h=16;

  while(*(string+i)!='\0')
  {

    if(*(string+i)==0)
    {
      return;
    }

    if(*(string+i)=='\n')
    {
      y += font_h;
      x = 0;
      string++;
    }

    if(x > lcddev.width-font_w)
    {
      x = 0;
      y += font_h;
    }

    if(y > lcddev.height-font_h)
    {
      x = y = 0;
    }

    LCD_putchar(x, y,*(string+i),TFT_STRING_MODE);
    x += font_w;
    i++;
  }
}

void LCD_scroll_vertical(uint16_t position)
{
  LCD_WR_REG(0x37);
  LCD_WR_DATA(position >> 8);
  LCD_WR_DATA(position & 0xFF);
}

void LCD_fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  uint16_t i, j;

  LCD_SetWindows(x1, y1, x2, y2);
  for(i = x1; i <= x2; i++)
  {
    for(j = y1; j <= y2; j++)
    {
      LCD_WR_DATA(color);
    }
  }
}

void LCD_draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
  uint16_t t;
  int xerr = 0, yerr = 0, delta_x, delta_y, distance;
  int incx, incy, uRow, uCol;

  delta_x = x2 - x1;
  delta_y = y2 - y1;
  uRow = x1;
  uCol = y1;

  if(delta_x > 0) incx = 1;
  else if(delta_x == 0) incx = 0;
  else
  {
    incx = -1;
    delta_x = -delta_x;
  }

  if(delta_y > 0) incy = 1;
  else if(delta_y == 0) incy=0;
  else
  {
    incy = -1; delta_y = -delta_y;
  }

  if( delta_x > delta_y) distance = delta_x;
  else distance = delta_y;

  for(t = 0; t <= distance + 1; t++ )
  {
    LCD_draw_point(uRow, uCol, color);
    xerr += delta_x ;
    yerr += delta_y ;
    if(xerr > distance)
    {
      xerr -= distance;
      uRow += incx;
    }
    if(yerr > distance)
    {
      yerr -= distance;
      uCol += incy;
    }
  }
}

void LCD_draw_h_line(uint16_t x1, uint16_t y1, uint16_t x2 ,uint16_t color)
{
	uint16_t i;

	if(x2 < x1)
	{
		uint16_t tmp = x1;
		x1 = x2;
		x2 = tmp;
	}

	LCD_SetWindows(x1, y1, x2, y1);
  for(i = x1; i <= x2; i++)
  {
     LCD_WR_DATA(color);
  }
}

void LCD_draw_v_line(uint16_t x1, uint16_t y1, uint16_t y2 ,uint16_t color)
{
	uint16_t i;

	if(y2 < y1)
	{
		uint16_t tmp = y1;
		y1 = y2;
		y2 = tmp;
	}

	LCD_SetWindows(x1, y1, x1, y2);
  for(i = y1; i <= y2; i++)
  {
     LCD_WR_DATA(color);
  }
}

void LCD_draw_rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  LCD_draw_line(x1,y1,x1,y2,color);
  LCD_draw_line(x1,y2,x2,y2,color);
  LCD_draw_line(x2,y2,x2,y1,color);
  LCD_draw_line(x2,y1,x1,y1,color);
}

void LCD_draw_circle(uint16_t x, uint16_t y, uint16_t radian, uint16_t color)
{
  int a, b;
  int di;

  a = 0;
  b = radian;
  di = 3 - (radian << 1);

  while(a <= b)
  {
    LCD_draw_point(x-b,y-a,color); //3
    LCD_draw_point(x+b,y-a,color); //0
    LCD_draw_point(x-a,y+b,color); //1
    LCD_draw_point(x-b,y-a,color); //7
    LCD_draw_point(x-a,y-b,color); //2
    LCD_draw_point(x+b,y+a,color); //4
    LCD_draw_point(x+a,y-b,color); //5
    LCD_draw_point(x+a,y+b,color); //6
    LCD_draw_point(x-b,y+a,color);
    a++;
    //Bresenham
    if(di<0)
    {
      di +=4*a+6;
    }
    else
    {
      di+=10+4*(a-b);
      b--;
    }
    LCD_draw_point(x+a,y+b,color);
  }
}

void LCD_puts_image_pos(const unsigned char* image_arr, uint16_t x, uint16_t y, uint16_t size_x, uint16_t size_y)
{
  uint32_t i;
  uint16_t temp;
  uint8_t high_byte,low_byte;
  LCD_SetWindows(x,y,x+size_x-1,y+size_y-1);
  for(i=x;i<(size_x*size_y);i++)
  {
    low_byte=*(image_arr+i*2);
    high_byte=*(image_arr+i*2+1);
    temp=((high_byte)<<8|(low_byte));
    LCD_WR_DATA(temp);
  }
}

void LCD_puts_image(const unsigned char* image_arr)
{
  uint32_t i;
  uint16_t temp;
  uint8_t high_byte,low_byte;
  LCD_SetWindows(0,0,lcddev.width-1,lcddev.height-1);
  for(i=0;i<(lcddev.width*lcddev.height);i++)
  {
    low_byte=*(image_arr+i*2);
    high_byte=*(image_arr+i*2+1);
    temp=((high_byte)<<8|(low_byte));
    LCD_WR_DATA(temp);
  }
}

void LCD_putchar(uint16_t x,uint16_t y,uint8_t num,uint8_t mode)
{
  uint8_t temp;
  uint8_t pos,t;
  //uint16_t x0=x;
  uint16_t colortemp=POINT_COLOR;
  if(x>lcddev.width-8||y>lcddev.height-16)return;
  num=num-' ';
  LCD_SetWindows(x,y,x+8-1,y+16-1);
  if(!mode)
  {
    for(pos=0;pos<16;pos++)
    {
      temp=asc2_1608[(uint16_t)num*16+pos];
      for(t=0;t<8;t++)
      {
        if(temp&0x01)POINT_COLOR=colortemp;
        else POINT_COLOR=BACK_COLOR;
        LCD_WR_DATA(POINT_COLOR);
        temp>>=1;
      }
    }
  }
  else
  {
    for(pos=0;pos<16;pos++)
    {
      temp=asc2_1608[(uint16_t)num*16+pos];
      for(t=0;t<8;t++)
      {
        if(temp&0x01)LCD_draw_point(x+t,y+pos,POINT_COLOR);
        temp>>=1;
      }
    }
  }
  POINT_COLOR=colortemp;
}

void LCD_putchar14x24(uint16_t x,uint16_t y,uint8_t data,uint8_t mode)
{
  if((x>lcddev.width-14)||(y>lcddev.height-24)) return;
  uint8_t i,j,k,temp;
  LCD_SetWindows(x,y,x+14-1,y+24-1);
  for(i=0;i<24/8;i++)
  {
    for(j=0;j<8;j++)
    {
      for(k=0;k<14;k++)
      {
        temp=Consolas14x24[(data-' ')*(24/8)*14+k*(24/8)+i];
        if(mode==TFT_STRING_MODE_BACKGROUND)
        {
          if(temp&(0x01<<j))
          {
            LCD_WR_DATA(POINT_COLOR);
          }
          else
          {
            LCD_WR_DATA(BACK_COLOR);
          }
        }
        else
        {
          if(temp&(0x01<<j))
          {
            LCD_draw_point(x+k,y+(8*i+j),POINT_COLOR);
          }
        }
      }
    }
  }
}

void LCD_putchar18x32(uint16_t x,uint16_t y,uint8_t data,uint8_t mode)
{
  if((x>lcddev.width-18)||(y>lcddev.height-32)) return;
  uint8_t i,j,k,temp;
  LCD_SetWindows(x,y,x+18-1,y+32-1);
  for(i=0;i<32/8;i++)
  {
    for(j=0;j<8;j++)
    {
      for(k=0;k<18;k++)
      {
        temp=Consolas18x32[(data-' ')*(32/8)*18+k*(32/8)+i];
        if(mode==TFT_STRING_MODE_BACKGROUND)
        {
          if(temp&(0x01<<j))
          {
            LCD_WR_DATA(POINT_COLOR);
          }
          else
          {
            LCD_WR_DATA(BACK_COLOR);
          }
        }
        else
        {
          if(temp&(0x01<<j))
          {
            LCD_draw_point(x+k,y+(8*i+j),POINT_COLOR);
          }
        }
      }
    }
  }
}

void LCD_putchar26x48(uint16_t x,uint16_t y,uint8_t data,uint8_t mode)
{
  if((x>lcddev.width-26)||(y>lcddev.height-48)) return;
  uint8_t i,j,k,temp;
  LCD_SetWindows(x,y,x+26-1,y+48-1);
  for(i=0;i<48/8;i++)
  {
    for(j=0;j<8;j++)
    {
      for(k=0;k<26;k++)
      {
        temp=Consolas26x48[(data-' ')*(48/8)*26+k*(48/8)+i];
        if(mode==TFT_STRING_MODE_BACKGROUND)
        {
          if(temp&(0x01<<j))
          {
            LCD_WR_DATA(POINT_COLOR);
          }
          else
          {
            LCD_WR_DATA(BACK_COLOR);
          }
        }
        else
        {
          if(temp&(0x01<<j))
          {
            LCD_draw_point(x+k,y+(8*i+j),POINT_COLOR);
          }
        }
      }
    }
  }
}


void LCD_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
	/* Draw lines */
	LCD_draw_line(x1, y1, x2, y2, color);
	LCD_draw_line(x2, y2, x3, y3, color);
	LCD_draw_line(x3, y3, x1, y1, color);
}

void LCD_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		LCD_draw_line(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

void LCD_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	LCD_draw_point(x0, y0 + r, color);
	LCD_draw_point(x0, y0 - r, color);
	LCD_draw_point(x0 + r, y0, color);
	LCD_draw_point(x0 - r, y0, color);
	LCD_draw_h_line(x0 - r, y0, x0 + r, color);

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		LCD_draw_h_line(x0 - x, y0 + y, x0 + x, color);
		LCD_draw_h_line(x0 + x, y0 - y, x0 - x, color);

		LCD_draw_h_line(x0 + y, y0 + x, x0 - y, color);
		LCD_draw_h_line(x0 + y, y0 - x, x0 - y, color);
	}
}

