/**
  * @brief  ILI9341 Registers
  */

/* Level 1 Commands */
#define LCD_SWRESET             0x01   /* Software Reset */
#define LCD_READ_DISPLAY_ID     0x04   /* Read display identification information */
#define LCD_RDDST               0x09   /* Read Display Status */
#define LCD_RDDPM               0x0A   /* Read Display Power Mode */
#define LCD_RDDMADCTL           0x0B   /* Read Display MADCTL */
#define LCD_RDDCOLMOD           0x0C   /* Read Display Pixel Format */
#define LCD_RDDIM               0x0D   /* Read Display Image Format */
#define LCD_RDDSM               0x0E   /* Read Display Signal Mode */
#define LCD_RDDSDR              0x0F   /* Read Display Self-Diagnostic Result */
#define LCD_SPLIN               0x10   /* Enter Sleep Mode */
#define LCD_SLEEP_OUT           0x11   /* Sleep out register */
#define LCD_PTLON               0x12   /* Partial Mode ON */
#define LCD_NORMAL_MODE_ON      0x13   /* Normal Display Mode ON */
#define LCD_DINVOFF             0x20   /* Display Inversion OFF */
#define LCD_DINVON              0x21   /* Display Inversion ON */
#define LCD_GAMMA               0x26   /* Gamma register */
#define LCD_DISPLAY_OFF         0x28   /* Display off register */
#define LCD_DISPLAY_ON          0x29   /* Display on register */
#define LCD_COLUMN_ADDR         0x2A   /* Colomn address register */
#define LCD_PAGE_ADDR           0x2B   /* Page address register */
#define LCD_GRAM                0x2C   /* GRAM register */
#define LCD_RGBSET              0x2D   /* Color SET */
#define LCD_RAMRD               0x2E   /* Memory Read */
#define LCD_PLTAR               0x30   /* Partial Area */
#define LCD_VSCRDEF             0x33   /* Vertical Scrolling Definition */
#define LCD_TEOFF               0x34   /* Tearing Effect Line OFF */
#define LCD_TEON                0x35   /* Tearing Effect Line ON */
#define LCD_MAC                 0x36   /* Memory Access Control register*/
#define LCD_VSCRSADD            0x37   /* Vertical Scrolling Start Address */
#define LCD_IDMOFF              0x38   /* Idle Mode OFF */
#define LCD_IDMON               0x39   /* Idle Mode ON */
#define LCD_PIXEL_FORMAT        0x3A   /* Pixel Format register */
#define LCD_WRITE_MEM_CONTINUE  0x3C   /* Write Memory Continue */
#define LCD_READ_MEM_CONTINUE   0x3E   /* Read Memory Continue */
#define LCD_SET_TEAR_SCANLINE   0x44   /* Set Tear Scanline */
#define LCD_GET_SCANLINE        0x45   /* Get Scanline */
#define LCD_WDB                 0x51   /* Write Brightness Display register */
#define LCD_RDDISBV             0x52   /* Read Display Brightness */
#define LCD_WCD                 0x53   /* Write Control Display register*/
#define LCD_RDCTRLD             0x54   /* Read CTRL Display */
#define LCD_WRCABC              0x55   /* Write Content Adaptive Brightness Control */
#define LCD_RDCABC              0x56   /* Read Content Adaptive Brightness Control */
#define LCD_WRITE_CABC          0x5E   /* Write CABC Minimum Brightness */
#define LCD_READ_CABC           0x5F   /* Read CABC Minimum Brightness */
#define LCD_READ_ID1            0xDA   /* Read ID1 */
#define LCD_READ_ID2            0xDB   /* Read ID2 */
#define LCD_READ_ID3            0xDC   /* Read ID3 */

/* Level 2 Commands */
#define LCD_RGB_INTERFACE       0xB0   /* RGB Interface Signal Control */
#define LCD_FRMCTR1             0xB1   /* Frame Rate Control (In Normal Mode) */
#define LCD_FRMCTR2             0xB2   /* Frame Rate Control (In Idle Mode) */
#define LCD_FRMCTR3             0xB3   /* Frame Rate Control (In Partial Mode) */
#define LCD_INVTR               0xB4   /* Display Inversion Control */
#define LCD_BPC                 0xB5   /* Blanking Porch Control register */
#define LCD_DFC                 0xB6   /* Display Function Control register */
#define LCD_ETMOD               0xB7   /* Entry Mode Set */
#define LCD_BACKLIGHT1          0xB8   /* Backlight Control 1 */
#define LCD_BACKLIGHT2          0xB9   /* Backlight Control 2 */
#define LCD_BACKLIGHT3          0xBA   /* Backlight Control 3 */
#define LCD_BACKLIGHT4          0xBB   /* Backlight Control 4 */
#define LCD_BACKLIGHT5          0xBC   /* Backlight Control 5 */
#define LCD_BACKLIGHT7          0xBE   /* Backlight Control 7 */
#define LCD_BACKLIGHT8          0xBF   /* Backlight Control 8 */
#define LCD_POWER1              0xC0   /* Power Control 1 register */
#define LCD_POWER2              0xC1   /* Power Control 2 register */
#define LCD_VCOM1               0xC5   /* VCOM Control 1 register */
#define LCD_VCOM2               0xC7   /* VCOM Control 2 register */
#define LCD_NVMWR               0xD0   /* NV Memory Write */
#define LCD_NVMPKEY             0xD1   /* NV Memory Protection Key */
#define LCD_RDNVM               0xD2   /* NV Memory Status Read */
#define LCD_READ_ID4            0xD3   /* Read ID4 */
#define LCD_PGAMMA              0xE0   /* Positive Gamma Correction register */
#define LCD_NGAMMA              0xE1   /* Negative Gamma Correction register */
#define LCD_DGAMCTRL1           0xE2   /* Digital Gamma Control 1 */
#define LCD_DGAMCTRL2           0xE3   /* Digital Gamma Control 2 */
#define LCD_INTERFACE           0xF6   /* Interface control register */

/* Extend register commands */
#define LCD_POWERA               0xCB   /* Power control A register */
#define LCD_POWERB               0xCF   /* Power control B register */
#define LCD_DTCA                 0xE8   /* Driver timing control A */
#define LCD_DTCB                 0xEA   /* Driver timing control B */
#define LCD_POWER_SEQ            0xED   /* Power on sequence register */
#define LCD_3GAMMA_EN            0xF2   /* 3 Gamma enable register */
#define LCD_PRC                  0xF7   /* Pump ratio control register */


/* Includes ------------------------------------------------------------------*/
#include "H750_ILI9341_RGB.h"
#include "spi.h"
#include "dma2d.h"
#include "ltdc.h"
#include "main.h"

/* Global Variables ------------------------------------------------------------------*/
volatile uint16_t LCD_HEIGHT = ILI9341_SCREEN_HEIGHT;
volatile uint16_t LCD_WIDTH	 = ILI9341_SCREEN_WIDTH;

#define LtdcHandler		hltdc
#define ABS(X)  			((X) > 0 ? (X) : -(X))
#define POLY_X(Z)     ((int32_t)((Points + Z)->X))
#define POLY_Y(Z)     ((int32_t)((Points + Z)->Y))

static uint32_t ActiveLayer = 0;

/*Send data (char) to LCD*/
void ILI9341_SPI_Send(unsigned char SPI_Data)
{
	HAL_SPI_Transmit(HSPI_INSTANCE, &SPI_Data, 1, 1);
}

/* Send command (char) to LCD */
void ILI9341_Write_Command(uint8_t Command)
{
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
	ILI9341_SPI_Send(Command);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

/* Send Data (char) to LCD */
void ILI9341_Write_Data(uint8_t Data)
{
	HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	ILI9341_SPI_Send(Data);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

/* Set Address - Location block - to draw into */
void ILI9341_Set_Address(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2)
{
	ILI9341_Write_Command(0x2A);
	ILI9341_Write_Data(X1>>8);
	ILI9341_Write_Data(X1);
	ILI9341_Write_Data(X2>>8);
	ILI9341_Write_Data(X2);

	ILI9341_Write_Command(0x2B);
	ILI9341_Write_Data(Y1>>8);
	ILI9341_Write_Data(Y1);
	ILI9341_Write_Data(Y2>>8);
	ILI9341_Write_Data(Y2);

	ILI9341_Write_Command(0x2C);
}

/*HARDWARE RESET*/
void ILI9341_Reset(void)
{
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(20);
	HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
}

/*Initialize LCD display*/
void ILI9341_Init(void)
{
	ILI9341_Reset();

	//SOFTWARE RESET
	ILI9341_Write_Command(0x01);
	HAL_Delay(100);

  ILI9341_Write_Command(0xCA);
  ILI9341_Write_Data(0xC3);
  ILI9341_Write_Data(0x08);
  ILI9341_Write_Data(0x50);
  ILI9341_Write_Command(LCD_POWERB);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Data(0xC1);
  ILI9341_Write_Data(0x30);
  ILI9341_Write_Command(LCD_POWER_SEQ);
  ILI9341_Write_Data(0x64);
  ILI9341_Write_Data(0x03);
  ILI9341_Write_Data(0x12);
  ILI9341_Write_Data(0x81);
  ILI9341_Write_Command(LCD_DTCA);
  ILI9341_Write_Data(0x85);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Data(0x78);
  ILI9341_Write_Command(LCD_POWERA);
  ILI9341_Write_Data(0x39);
  ILI9341_Write_Data(0x2C);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Data(0x34);
  ILI9341_Write_Data(0x02);
  ILI9341_Write_Command(LCD_PRC);
  ILI9341_Write_Data(0x20);
  ILI9341_Write_Command(LCD_DTCB);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Command(LCD_FRMCTR1);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Data(0x1B);
  ILI9341_Write_Command(LCD_DFC);
  ILI9341_Write_Data(0x0A);
  ILI9341_Write_Data(0xA2);
  ILI9341_Write_Command(LCD_POWER1);
  ILI9341_Write_Data(0x10);
  ILI9341_Write_Command(LCD_POWER2);
  ILI9341_Write_Data(0x10);
  ILI9341_Write_Command(LCD_VCOM1);
  ILI9341_Write_Data(0x45);
  ILI9341_Write_Data(0x15);
  ILI9341_Write_Command(LCD_VCOM2);
  ILI9341_Write_Data(0x90);
  ILI9341_Write_Command(LCD_MAC);
  ILI9341_Write_Data(0xC8);
  ILI9341_Write_Command(LCD_3GAMMA_EN);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Command(LCD_RGB_INTERFACE);
  ILI9341_Write_Data(0xC2);
  ILI9341_Write_Command(LCD_DFC);
  ILI9341_Write_Data(0x0A);
  ILI9341_Write_Data(0xA7);
  ILI9341_Write_Data(0x27);
  ILI9341_Write_Data(0x04);

  /* Colomn address set */
  ILI9341_Write_Command(LCD_COLUMN_ADDR);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Data(0xEF);
  /* Page address set */
  ILI9341_Write_Command(LCD_PAGE_ADDR);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Data(0x01);
  ILI9341_Write_Data(0x3F);
  ILI9341_Write_Command(LCD_INTERFACE);
  ILI9341_Write_Data(0x01);
  ILI9341_Write_Data(0x00);
  ILI9341_Write_Data(0x06);

  ILI9341_Write_Command(LCD_GRAM);
  HAL_Delay(200);

	//3GAMMA FUNCTION DISABLE
	ILI9341_Write_Command(0xF2);
	ILI9341_Write_Data(0x00);

	//GAMMA CURVE SELECTED
	ILI9341_Write_Command(0x26);
	ILI9341_Write_Data(0x01);

	//POSITIVE GAMMA CORRECTION
	ILI9341_Write_Command(0xE0);
	ILI9341_Write_Data(0x0F);
	ILI9341_Write_Data(0x31);
	ILI9341_Write_Data(0x2B);
	ILI9341_Write_Data(0x0C);
	ILI9341_Write_Data(0x0E);
	ILI9341_Write_Data(0x08);
	ILI9341_Write_Data(0x4E);
	ILI9341_Write_Data(0xF1);
	ILI9341_Write_Data(0x37);
	ILI9341_Write_Data(0x07);
	ILI9341_Write_Data(0x10);
	ILI9341_Write_Data(0x03);
	ILI9341_Write_Data(0x0E);
	ILI9341_Write_Data(0x09);
	ILI9341_Write_Data(0x00);

	//NEGATIVE GAMMA CORRECTION
	ILI9341_Write_Command(0xE1);
	ILI9341_Write_Data(0x00);
	ILI9341_Write_Data(0x0E);
	ILI9341_Write_Data(0x14);
	ILI9341_Write_Data(0x03);
	ILI9341_Write_Data(0x11);
	ILI9341_Write_Data(0x07);
	ILI9341_Write_Data(0x31);
	ILI9341_Write_Data(0xC1);
	ILI9341_Write_Data(0x48);
	ILI9341_Write_Data(0x08);
	ILI9341_Write_Data(0x0F);
	ILI9341_Write_Data(0x0C);
	ILI9341_Write_Data(0x31);
	ILI9341_Write_Data(0x36);
	ILI9341_Write_Data(0x0F);

	//EXIT SLEEP
	ILI9341_Write_Command(0x11);
	HAL_Delay(120);

	//TURN ON DISPLAY
	ILI9341_Write_Command(0x29);
}

/**
  * @brief  Enables the Display.
  * @param  None
  * @retval None
  */
void ILI9341_DisplayOn(void)
{
  /* Display On */
	ILI9341_Write_Command(LCD_DISPLAY_ON);
}

/**
  * @brief  Disables the Display.
  * @param  None
  * @retval None
  */
void ILI9341_DisplayOff(void)
{
  /* Display Off */
	ILI9341_Write_Command(LCD_DISPLAY_OFF);
}

/**
  * @brief  Get LCD PIXEL WIDTH.
  * @param  None
  * @retval LCD PIXEL WIDTH.
  */
uint16_t ILI9341_GetLcdPixelWidth(void)
{
  /* Return LCD PIXEL WIDTH */
  return ILI9341_SCREEN_WIDTH;
}

/**
  * @brief  Get LCD PIXEL HEIGHT.
  * @param  None
  * @retval LCD PIXEL HEIGHT.
  */
uint16_t ILI9341_GetLcdPixelHeight(void)
{
  /* Return LCD PIXEL HEIGHT */
  return ILI9341_SCREEN_HEIGHT;
}

/**
  * @brief  Fills buffer.
  * @param  LayerIndex: layer index
  * @param  pDst: output color
  * @param  xSize: buffer width
  * @param  ySize: buffer height
  * @param  OffLine: offset
  * @param  ColorIndex: color Index
  */
static void FillBuffer(uint32_t LayerIndex, void * pDst, uint32_t xSize, uint32_t ySize, uint32_t OffLine, uint32_t ColorIndex)
{

  /* Register to memory mode with ARGB8888 as color Mode */
	hdma2d.Init.Mode         = DMA2D_R2M;
  if(LtdcHandler.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB565)
  { /* RGB565 format */
    hdma2d.Init.ColorMode    = DMA2D_OUTPUT_RGB565;
		ColorIndex = ((ColorIndex & RED)<<8) | ((ColorIndex & GREEN )<<5) | ((ColorIndex & BLUE) << 3);
  }
  else
  { /* ARGB8888 format */
    hdma2d.Init.ColorMode    = DMA2D_OUTPUT_RGB888;
  }
  hdma2d.Init.OutputOffset = OffLine;

  hdma2d.Instance = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&hdma2d) == HAL_OK)
  {
    if(HAL_DMA2D_ConfigLayer(&hdma2d, LayerIndex) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d, ColorIndex, (uint32_t)pDst, xSize, ySize) == HAL_OK)
      {
        /* Polling For DMA transfer */
        HAL_DMA2D_PollForTransfer(&hdma2d, 10);
      }
    }
  }
}

uint16_t ILI9341_GetXSize(void)
{
	return ILI9341_GetLcdPixelWidth();
}

uint16_t ILI9341_GetYSize(void)
{
	return ILI9341_GetLcdPixelHeight();
}

/**
  * @brief  Writes Pixel.
  * @param  Xpos: the X position
  * @param  Ypos: the Y position
  * @param  RGB_Code: the pixel color in ARGB mode (8-8-8-8)
  */
void ILI9341_DrawPixel(uint16_t Xpos, uint16_t Ypos, uint32_t RGB_Code)
{
  if((Xpos >= ILI9341_GetXSize()) || (Ypos >= ILI9341_GetYSize())) return;

  /* Write data value to all SDRAM memory */
  //*(__IO uint32_t*) (LtdcHandler.LayerCfg[ActiveLayer].FBStartAdress + (4*(Ypos*ILI9341_GetXSize() + Xpos))) = RGB_Code;		// 32bit color
	*(__IO uint16_t*) (LtdcHandler.LayerCfg[ActiveLayer].FBStartAdress + (2*(Ypos*ILI9341_GetXSize() + Xpos))) = RGB_Code;			// 16bit color

//	uint16_t * LCD_BUFF;
//	LCD_BUFF = (uint16_t *)0x24025800;
//	LCD_BUFF[Ypos * 240L + Xpos] = RGB_Code;

}

/**
  * @brief  Reads Pixel.
  * @param  Xpos: the X position
  * @param  Ypos: the Y position
  * @retval RGB pixel color
  */
uint32_t ILI9341_ReadPixel(uint16_t Xpos, uint16_t Ypos)
{
  uint32_t ret = 0;

  if(hltdc.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB8888)
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint32_t*) (hltdc.LayerCfg[ActiveLayer].FBStartAdress + (4*(Ypos*ILI9341_GetXSize() + Xpos)));
  }
  else if(hltdc.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB888)
  {
    /* Read data value from SDRAM memory */
    ret = (*(__IO uint32_t*) (hltdc.LayerCfg[ActiveLayer].FBStartAdress + (4*(Ypos*ILI9341_GetXSize() + Xpos))) & 0x00FFFFFF);
  }
  else if((hltdc.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_RGB565) || \
          (hltdc.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_ARGB4444) || \
          (hltdc.LayerCfg[ActiveLayer].PixelFormat == LTDC_PIXEL_FORMAT_AL88))
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint16_t*) (hltdc.LayerCfg[ActiveLayer].FBStartAdress + (2*(Ypos*ILI9341_GetXSize() + Xpos)));
  }
  else
  {
    /* Read data value from SDRAM memory */
    ret = *(__IO uint8_t*) (hltdc.LayerCfg[ActiveLayer].FBStartAdress + (2*(Ypos*ILI9341_GetXSize() + Xpos)));
  }

  return ret;
}

/**
  * @brief  Clears the hole LCD.
  * @param  Color: the color of the background
  */
void ILI9341_Clear(uint32_t Color)
{
  /* Clear the LCD */
  FillBuffer(ActiveLayer, (uint32_t *)(hltdc.LayerCfg[ActiveLayer].FBStartAdress), ILI9341_GetXSize(), ILI9341_GetYSize(), 0, Color);
}

/**
  * @brief  Selects the LCD Layer.
  * @param  LayerIndex: the Layer foreground or background.
  */
void ILI9341_SelectLayer(uint32_t LayerIndex)
{
  ActiveLayer = LayerIndex;
}

/**
  * @brief  Sets a LCD Layer visible.
  * @param  LayerIndex: the visible Layer.
  * @param  state: new state of the specified layer.
  *    This parameter can be: ENABLE or DISABLE.
  */
void ILI9341_SetLayerVisible(uint32_t LayerIndex, FunctionalState state)
{
  if(state == ENABLE)
  {
    __HAL_LTDC_LAYER_ENABLE(&LtdcHandler, LayerIndex);
  }
  else
  {
    __HAL_LTDC_LAYER_DISABLE(&LtdcHandler, LayerIndex);
  }
  __HAL_LTDC_RELOAD_CONFIG(&LtdcHandler);
}

/**
  * @brief  Sets an LCD Layer visible without reloading.
  * @param  LayerIndex: Visible Layer
  * @param  State: New state of the specified layer
  *          This parameter can be one of the following values:
  *            @arg  ENABLE
  *            @arg  DISABLE
  * @retval None
  */
void ILI9341_SetLayerVisible_NoReload(uint32_t LayerIndex, FunctionalState State)
{
  if(State == ENABLE)
  {
    __HAL_LTDC_LAYER_ENABLE(&LtdcHandler, LayerIndex);
  }
  else
  {
    __HAL_LTDC_LAYER_DISABLE(&LtdcHandler, LayerIndex);
  }
  /* Do not Sets the Reload  */
}

/**
  * @brief  Configures the Transparency.
  * @param  LayerIndex: the Layer foreground or background.
  * @param  Transparency: the Transparency,
  *    This parameter must range from 0x00 to 0xFF.
  */
void ILI9341_SetTransparency(uint32_t LayerIndex, uint8_t Transparency)
{
  HAL_LTDC_SetAlpha(&LtdcHandler, Transparency, LayerIndex);
}

/**
  * @brief  Configures the transparency without reloading.
  * @param  LayerIndex: Layer foreground or background.
  * @param  Transparency: Transparency
  *           This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF
  * @retval None
  */
void ILI9341_SetTransparency_NoReload(uint32_t LayerIndex, uint8_t Transparency)
{
  HAL_LTDC_SetAlpha_NoReload(&LtdcHandler, Transparency, LayerIndex);
}

/**
  * @brief  Sets a LCD layer frame buffer address.
  * @param  LayerIndex: specifies the Layer foreground or background
  * @param  Address: new LCD frame buffer value
  */
void ILI9341_SetLayerAddress(uint32_t LayerIndex, uint32_t Address)
{
  HAL_LTDC_SetAddress(&LtdcHandler, Address, LayerIndex);
}

/**
  * @brief  Sets an LCD layer frame buffer address without reloading.
  * @param  LayerIndex: Layer foreground or background
  * @param  Address: New LCD frame buffer value
  * @retval None
  */
void ILI9341_SetLayerAddress_NoReload(uint32_t LayerIndex, uint32_t Address)
{
  HAL_LTDC_SetAddress_NoReload(&LtdcHandler, Address, LayerIndex);
}

/**
  * @brief  Sets the Display window.
  * @param  LayerIndex: layer index
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @param  Width: LCD window width
  * @param  Height: LCD window height
  */
void ILI9341_SetLayerWindow(uint16_t LayerIndex, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* reconfigure the layer size */
  HAL_LTDC_SetWindowSize(&LtdcHandler, Width, Height, LayerIndex);

  /* reconfigure the layer position */
  HAL_LTDC_SetWindowPosition(&LtdcHandler, Xpos, Ypos, LayerIndex);
}

/**
  * @brief  Sets display window without reloading.
  * @param  LayerIndex: Layer index
  * @param  Xpos: LCD X position
  * @param  Ypos: LCD Y position
  * @param  Width: LCD window width
  * @param  Height: LCD window height
  * @retval None
  */
void ILI9341_SetLayerWindow_NoReload(uint16_t LayerIndex, uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
  /* Reconfigure the layer size */
  HAL_LTDC_SetWindowSize_NoReload(&LtdcHandler, Width, Height, LayerIndex);

  /* Reconfigure the layer position */
  HAL_LTDC_SetWindowPosition_NoReload(&LtdcHandler, Xpos, Ypos, LayerIndex);
}

/**
  * @brief  Configures and sets the color Keying.
  * @param  LayerIndex: the Layer foreground or background
  * @param  RGBValue: the Color reference
  */
void ILI9341_SetColorKeying(uint32_t LayerIndex, uint32_t RGBValue)
{
  /* Configure and Enable the color Keying for LCD Layer */
  HAL_LTDC_ConfigColorKeying(&LtdcHandler, RGBValue, LayerIndex);
  HAL_LTDC_EnableColorKeying(&LtdcHandler, LayerIndex);
}

/**
  * @brief  Configures and sets the color keying without reloading.
  * @param  LayerIndex: Layer foreground or background
  * @param  RGBValue: Color reference
  * @retval None
  */
void ILI9341_SetColorKeying_NoReload(uint32_t LayerIndex, uint32_t RGBValue)
{
  /* Configure and Enable the color Keying for LCD Layer */
  HAL_LTDC_ConfigColorKeying_NoReload(&LtdcHandler, RGBValue, LayerIndex);
  HAL_LTDC_EnableColorKeying_NoReload(&LtdcHandler, LayerIndex);
}

/**
  * @brief  Disables the color Keying.
  * @param  LayerIndex: the Layer foreground or background
  */
void ILI9341_ResetColorKeying(uint32_t LayerIndex)
{
  /* Disable the color Keying for LCD Layer */
  HAL_LTDC_DisableColorKeying(&LtdcHandler, LayerIndex);
}

/**
  * @brief  Disables the color keying without reloading.
  * @param  LayerIndex: Layer foreground or background
  * @retval None
  */
void ILI9341_ResetColorKeying_NoReload(uint32_t LayerIndex)
{
  /* Disable the color Keying for LCD Layer */
  HAL_LTDC_DisableColorKeying_NoReload(&LtdcHandler, LayerIndex);
}

/**
  * @brief  Disables the color keying without reloading.
  * @param  ReloadType: can be one of the following values
  *         - LCD_RELOAD_IMMEDIATE
  *         - LCD_RELOAD_VERTICAL_BLANKING
  * @retval None
  */
void ILI9341_Relaod(uint32_t ReloadType)
{
  HAL_LTDC_Relaod (&LtdcHandler, ReloadType);
}


/**
  * @brief  Displays an horizontal line.
  * @param  Xpos: the X position
  * @param  Ypos: the Y position
  * @param  Length: line length
  */
void ILI9341_DrawHLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint16_t Color)
{
  uint32_t xaddress = 0;

  /* Get the line address */
  xaddress = (LtdcHandler.LayerCfg[ActiveLayer].FBStartAdress) + 2 * (ILI9341_GetXSize()*Ypos + Xpos);

  /* Write line */
  FillBuffer(ActiveLayer, (uint32_t *)xaddress, Length, 1, 0, Color);
}

/**
  * @brief  Displays a vertical line.
  * @param  Xpos: the X position
  * @param  Ypos: the Y position
  * @param  Length: line length
  */
void ILI9341_DrawVLine(uint16_t Xpos, uint16_t Ypos, uint16_t Length, uint16_t Color)
{
  uint32_t xaddress = 0;

  /* Get the line address */
  xaddress = (LtdcHandler.LayerCfg[ActiveLayer].FBStartAdress) + 2 * (ILI9341_GetXSize()*Ypos + Xpos);

  /* Write line */
  FillBuffer(ActiveLayer, (uint32_t *)xaddress, 1, Length, (ILI9341_GetXSize() - 1), Color);
}

/**
  * @brief  Displays an uni-line (between two points).
  * @param  X1: the point 1 X position
  * @param  Y1: the point 1 Y position
  * @param  X2: the point 2 X position
  * @param  Y2: the point 2 Y position
  */
void ILI9341_DrawLine(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2, uint16_t Color)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = ABS(X2 - X1);        /* The difference between the x's */
  deltay = ABS(Y2 - Y1);        /* The difference between the y's */
  x = X1;                       /* Start x off at the first pixel */
  y = Y1;                       /* Start y off at the first pixel */

  if (X2 >= X1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (Y2 >= Y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    ILI9341_DrawPixel(x, y, Color);   /* Draw the current pixel */
    num += numadd;                            /* Increase the numerator by the top of the fraction */
    if (num >= den)                           /* Check if numerator >= denominator */
    {
      num -= den;                             /* Calculate the new numerator value */
      x += xinc1;                             /* Change the x as appropriate */
      y += yinc1;                             /* Change the y as appropriate */
    }
    x += xinc2;                               /* Change the x as appropriate */
    y += yinc2;                               /* Change the y as appropriate */
  }
}

/**
  * @brief  Displays a rectangle.
  * @param  Xpos: the X position
  * @param  Ypos: the Y position
  * @param  Height: display rectangle height
  * @param  Width: display rectangle width
  */
void ILI9341_DrawRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height, uint16_t Color)
{
  /* Draw horizontal lines */
  ILI9341_DrawHLine(Xpos, Ypos, Width,Color);
  ILI9341_DrawHLine(Xpos, (Ypos+ Height), Width, Color);

  /* Draw vertical lines */
  ILI9341_DrawVLine(Xpos, Ypos, Height, Color);
  ILI9341_DrawVLine((Xpos + Width), Ypos, Height, Color);
}

/**
  * @brief  Displays a circle.
  * @param  Xpos: the X position
  * @param  Ypos: the Y position
  * @param  Radius: the circle radius
  */
void ILI9341_DrawCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius, uint16_t Color)
{
	if(Radius == 0) return;

  int32_t  d;/* Decision Variable */
  uint32_t  curx;/* Current X Value */
  uint32_t  cury;/* Current Y Value */

  d = 3 - (Radius << 1);
  curx = 0;
  cury = Radius;

  while (curx <= cury)
  {
    ILI9341_DrawPixel((Xpos + curx), (Ypos - cury), Color);
    ILI9341_DrawPixel((Xpos - curx), (Ypos - cury), Color);
    ILI9341_DrawPixel((Xpos + cury), (Ypos - curx), Color);
    ILI9341_DrawPixel((Xpos - cury), (Ypos - curx), Color);
    ILI9341_DrawPixel((Xpos + curx), (Ypos + cury), Color);
    ILI9341_DrawPixel((Xpos - curx), (Ypos + cury), Color);
    ILI9341_DrawPixel((Xpos + cury), (Ypos + curx), Color);
    ILI9341_DrawPixel((Xpos - cury), (Ypos + curx), Color);

    if (d < 0)
    {
      d += (curx << 2) + 6;
    }
    else
    {
      d += ((curx - cury) << 2) + 10;
      cury--;
    }
    curx++;
  }
}

/**
  * @brief  Displays an poly-line (between many points).
  * @param  Points: pointer to the points array
  * @param  PointCount: Number of points
  */
void ILI9341_DrawPolygon(pPoint Points, uint16_t PointCount, uint16_t Color)
{
  int16_t x = 0, y = 0;

  if(PointCount < 2)
  {
    return;
  }

  ILI9341_DrawLine(Points->X, Points->Y, (Points+PointCount-1)->X, (Points+PointCount-1)->Y, Color);

  while(--PointCount)
  {
    x = Points->X;
    y = Points->Y;
    Points++;
    ILI9341_DrawLine(x, y, Points->X, Points->Y, Color);
  }
}

/**
  * @brief  Displays an Ellipse.
  * @param  Xpos: the X position
  * @param  Ypos: the Y position
  * @param  XRadius: the X radius of ellipse
  * @param  YRadius: the Y radius of ellipse
  */
void ILI9341_DrawEllipse(int Xpos, int Ypos, int XRadius, int YRadius, uint16_t Color)
{
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float k = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;

  k = (float)(rad2/rad1);

  do {
    ILI9341_DrawPixel((Xpos-(uint16_t)(x/k)), (Ypos+y), Color);
    ILI9341_DrawPixel((Xpos+(uint16_t)(x/k)), (Ypos+y), Color);
    ILI9341_DrawPixel((Xpos+(uint16_t)(x/k)), (Ypos-y), Color);
    ILI9341_DrawPixel((Xpos-(uint16_t)(x/k)), (Ypos-y), Color);

    e2 = err;
    if (e2 <= x) {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
}

/**
  * @brief  Displays a bitmap picture loaded in the internal Flash (32 bpp).
  * @param  X: the bmp x position in the LCD
  * @param  Y: the bmp Y position in the LCD
  * @param  pBmp: Bmp picture address in the internal Flash
  */
void ILI9341_DrawBitmap(uint32_t X, uint32_t Y, uint8_t *pBmp)
{
  uint32_t index = 0, width = 0, height = 0, bitpixel = 0;
  uint32_t address;
  uint32_t inputcolormode = 0;

  /* Get bitmap data address offset */
  index = pBmp[10] + (pBmp[11] << 8) + (pBmp[12] << 16)  + (pBmp[13] << 24);

  /* Read bitmap width */
  width = pBmp[18] + (pBmp[19] << 8) + (pBmp[20] << 16)  + (pBmp[21] << 24);

  /* Read bitmap height */
  height = pBmp[22] + (pBmp[23] << 8) + (pBmp[24] << 16)  + (pBmp[25] << 24);

  /* Read bit/pixel */
  bitpixel = pBmp[28] + (pBmp[29] << 8);

  /* Set Address */
  address = LtdcHandler.LayerCfg[ActiveLayer].FBStartAdress + (((ILI9341_GetXSize()*Y) + X)*(4));

  /* Get the Layer pixel format */
  if ((bitpixel/8) == 4)
  {
    inputcolormode = CM_ARGB8888;
  }
  else if ((bitpixel/8) == 2)
  {
    inputcolormode = CM_RGB565;
  }
  else
  {
    inputcolormode = CM_RGB888;
  }

  (void)inputcolormode;

  /* bypass the bitmap header */
  pBmp += (index + (width * (height - 1) * (bitpixel/8)));

  /* Convert picture to ARGB8888 pixel format */
  for(index=0; index < height; index++)
  {
  /* Pixel format conversion */
//  ConvertLineToARGB8888((uint32_t *)pBmp, (uint32_t *)address, width, inputcolormode);

  /* Increment the source and destination buffers */
  	address+=  ((ILI9341_GetXSize() - width + width)*4);
  	pBmp -= width*(bitpixel/8);
  }
}

/**
  * @brief  Displays a full rectangle.
  * @param  Xpos: the X position
  * @param  Ypos: the Y position
  * @param  Height: rectangle height
  * @param  Width: rectangle width
  */
void ILI9341_FillRect(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height, uint16_t Color)
{
  uint32_t xaddress = 0;

  if((Xpos >= ILI9341_GetXSize()) || (Ypos >= ILI9341_GetYSize())) return;
  if(Width == 0 || Height == 0) return;

  /* Get the rectangle start address */
  xaddress = (LtdcHandler.LayerCfg[ActiveLayer].FBStartAdress) + 2 * (ILI9341_GetXSize()*Ypos + Xpos);

  /* Fill the rectangle */
  FillBuffer(ActiveLayer, (uint32_t *)xaddress, Width, Height, (ILI9341_GetXSize() - Width), Color);
}

/**
  * @brief  Displays a full circle.
  * @param  Xpos: the X position
  * @param  Ypos: the Y position
  * @param  Radius: the circle radius
  */
void ILI9341_FillCircle(uint16_t Xpos, uint16_t Ypos, uint16_t Radius, uint16_t Color)
{
  int32_t  d;    /* Decision Variable */
  uint32_t  curx;/* Current X Value */
  uint32_t  cury;/* Current Y Value */

  if(Radius == 0) return;

  d = 3 - (Radius << 1);

  curx = 0;
  cury = Radius;

  while (curx <= cury)
  {
    if(cury > 0)
    {
      ILI9341_DrawHLine(Xpos - cury, Ypos + curx, 2*cury, Color);
      ILI9341_DrawHLine(Xpos - cury, Ypos - curx, 2*cury, Color);
    }

    if(curx > 0)
    {
      ILI9341_DrawHLine(Xpos - curx, Ypos - cury, 2*curx, Color);
      ILI9341_DrawHLine(Xpos - curx, Ypos + cury, 2*curx, Color);
    }
    if (d < 0)
    {
      d += (curx << 2) + 6;
    }
    else
    {
      d += ((curx - cury) << 2) + 10;
      cury--;
    }
    curx++;
  }

  ILI9341_DrawCircle(Xpos, Ypos, Radius, Color);
}

/**
  * @brief  Fill triangle.
  * @param  X1: the point 1 x position
  * @param  Y1: the point 1 y position
  * @param  X2: the point 2 x position
  * @param  Y2: the point 2 y position
  * @param  X3: the point 3 x position
  * @param  Y3: the point 3 y position
  */
void ILI9341_FillTriangle(uint16_t X1, uint16_t X2, uint16_t X3, uint16_t Y1, uint16_t Y2, uint16_t Y3, uint16_t Color)
{
  int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
  yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
  curpixel = 0;

  deltax = ABS(X2 - X1);        /* The difference between the x's */
  deltay = ABS(Y2 - Y1);        /* The difference between the y's */
  x = X1;                       /* Start x off at the first pixel */
  y = Y1;                       /* Start y off at the first pixel */

  if (X2 >= X1)                 /* The x-values are increasing */
  {
    xinc1 = 1;
    xinc2 = 1;
  }
  else                          /* The x-values are decreasing */
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (Y2 >= Y1)                 /* The y-values are increasing */
  {
    yinc1 = 1;
    yinc2 = 1;
  }
  else                          /* The y-values are decreasing */
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)         /* There is at least one x-value for every y-value */
  {
    xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
    yinc2 = 0;                  /* Don't change the y for every iteration */
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;         /* There are more x-values than y-values */
  }
  else                          /* There is at least one y-value for every x-value */
  {
    xinc2 = 0;                  /* Don't change the x for every iteration */
    yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;         /* There are more y-values than x-values */
  }

  for (curpixel = 0; curpixel <= numpixels; curpixel++)
  {
    ILI9341_DrawLine(x, y, X3, Y3, Color);

    num += numadd;              /* Increase the numerator by the top of the fraction */
    if (num >= den)             /* Check if numerator >= denominator */
    {
      num -= den;               /* Calculate the new numerator value */
      x += xinc1;               /* Change the x as appropriate */
      y += yinc1;               /* Change the y as appropriate */
    }
    x += xinc2;                 /* Change the x as appropriate */
    y += yinc2;                 /* Change the y as appropriate */
  }
}

/**
  * @brief  Displays a full poly-line (between many points).
  * @param  Points: pointer to the points array
  * @param  PointCount: Number of points
  */
void ILI9341_FillPolygon(pPoint Points, uint16_t PointCount, uint16_t Color)
{

  int16_t x = 0, y = 0, x2 = 0, y2 = 0, xcenter = 0, ycenter = 0, xfirst = 0, yfirst = 0, pixelx = 0, pixely = 0, counter = 0;
  uint16_t  imageleft = 0, imageright = 0, imagetop = 0, imagebottom = 0;

  imageleft = imageright = Points->X;
  imagetop= imagebottom = Points->Y;

  for(counter = 1; counter < PointCount; counter++)
  {
    pixelx = POLY_X(counter);
    if(pixelx < imageleft)
    {
      imageleft = pixelx;
    }
    if(pixelx > imageright)
    {
      imageright = pixelx;
    }

    pixely = POLY_Y(counter);
    if(pixely < imagetop)
    {
      imagetop = pixely;
    }
    if(pixely > imagebottom)
    {
      imagebottom = pixely;
    }
  }

  if(PointCount < 2)
  {
    return;
  }

  xcenter = (imageleft + imageright)/2;
  ycenter = (imagebottom + imagetop)/2;

  xfirst = Points->X;
  yfirst = Points->Y;

  while(--PointCount)
  {
    x = Points->X;
    y = Points->Y;
    Points++;
    x2 = Points->X;
    y2 = Points->Y;

    ILI9341_FillTriangle(x, x2, xcenter, y, y2, ycenter, Color);
    ILI9341_FillTriangle(x, xcenter, x2, y, ycenter, y2, Color);
    ILI9341_FillTriangle(xcenter, x2, x, ycenter, y2, y, Color);
  }

  ILI9341_FillTriangle(xfirst, x2, xcenter, yfirst, y2, ycenter, Color);
  ILI9341_FillTriangle(xfirst, xcenter, x2, yfirst, ycenter, y2, Color);
  ILI9341_FillTriangle(xcenter, x2, xfirst, ycenter, y2, yfirst, Color);
}

/**
  * @brief  Draw a full ellipse.
  * @param  Xpos: the X position
  * @param  Ypos: the Y position
  * @param  XRadius: X radius of ellipse
  * @param  YRadius: Y radius of ellipse.
  */
void ILI9341_FillEllipse(int Xpos, int Ypos, int XRadius, int YRadius, uint16_t Color)
{
  int x = 0, y = -YRadius, err = 2-2*XRadius, e2;
  float K = 0, rad1 = 0, rad2 = 0;

  rad1 = XRadius;
  rad2 = YRadius;
  K = (float)(rad2/rad1);

  do
  {
    ILI9341_DrawHLine((Xpos-(uint16_t)(x/K)), (Ypos+y), (2*(uint16_t)(x/K) + 1), Color);
    ILI9341_DrawHLine((Xpos-(uint16_t)(x/K)), (Ypos-y), (2*(uint16_t)(x/K) + 1), Color);

    e2 = err;
    if (e2 <= x)
    {
      err += ++x*2+1;
      if (-y == x && e2 <= y) e2 = 0;
    }
    if (e2 > y) err += ++y*2+1;
  }
  while (y <= 0);
}
