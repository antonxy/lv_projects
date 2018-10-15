/**
 * @file disp.c
 * 
 */

/*********************
 *      INCLUDES
 *********************/
#include "tft.h"
#include "lv_conf.h"
#include "lvgl/lv_core/lv_vdb.h"
#include "lvgl/lv_hal/lv_hal.h"

#include "stm32f7xx.h"
#include "stm32f769i_discovery.h"
#include "stm32f769i_discovery_lcd.h"
#include "stm32f769i_discovery_sdram.h"
/*********************
 *      DEFINES
 *********************/

#if TFT_EXT_FB != 0

#define SDRAM_BANK_ADDR SDRAM_DEVICE_ADDR /* Set in stm32f769i_discovery_sdram.h (0xC0000000) */
#define SDRAM_TIMEOUT ((uint32_t)0xFFFF)

#endif

/* DMA Stream parameters definitions. You can modify these parameters to select
   a different DMA Stream and/or channel.
   But note that only DMA2 Streams are capable of Memory to Memory transfers. */
#define DMA_STREAM               DMA2_Stream0
#define DMA_CHANNEL              DMA_CHANNEL_0
#define DMA_STREAM_IRQ           DMA2_Stream0_IRQn
#define DMA_STREAM_IRQHANDLER    DMA2_Stream0_IRQHandler


#define VSYNC               1 
#define VBP                 1 
#define VFP                 1
#define VACT                480
#define HSYNC               1
#define HBP                 1
#define HFP                 1
#define HACT                800

#define LAYER0_ADDRESS               (LCD_FB_START_ADDRESS)


/**********************
 *      TYPEDEFS
 **********************/

/**********************
 *  STATIC PROTOTYPES
 **********************/

/*These 3 functions are needed by LittlevGL*/
static void tft_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color);
static void tft_map(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p);
static void tft_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p);
#if TFT_USE_GPU != 0
static void gpu_mem_blend(lv_color_t * dest, const lv_color_t * src, uint32_t length, lv_opa_t opa);
static void gpu_mem_fill(lv_color_t * dest, uint32_t length, lv_color_t color);
#endif

/*DMA to flush to frame buffer*/
static void DMA_Config(void);
static void DMA_TransferComplete(DMA_HandleTypeDef *han);
static void DMA_TransferError(DMA_HandleTypeDef *han);

static void Error_Handler(void);
/**********************
 *  STATIC VARIABLES
 **********************/


DSI_HandleTypeDef hdsi_eval;
LTDC_HandleTypeDef hltdc_eval;

#if TFT_USE_GPU != 0
static DMA2D_HandleTypeDef     Dma2dHandle;
static void DMA2D_Config(void);
#endif

#if TFT_EXT_FB != 0
SDRAM_HandleTypeDef hsdram;
FMC_SDRAM_TimingTypeDef SDRAM_Timing;
FMC_SDRAM_CommandTypeDef command;
static __IO uint32_t * my_fb = (__IO uint32_t*) (SDRAM_BANK_ADDR);
#else
static uint32_t my_fb[TFT_HOR_RES * TFT_VER_RES];
#endif


DMA_HandleTypeDef     DmaHandle;
static int32_t x1_flush;
static int32_t y1_flush;
static int32_t x2_flush;
static int32_t y2_fill;
static int32_t y_fill_act;
static const lv_color_t * buf_to_flush;


extern LTDC_HandleTypeDef hltdc_discovery;
extern DSI_HandleTypeDef hdsi_discovery;
DSI_CmdCfgTypeDef CmdCfg;
DSI_LPCmdTypeDef LPCmd;
DSI_PLLInitTypeDef dsiPllInit;

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/

/**
 * Initialize your display here
 */
void tft_init(void)
{
	lv_disp_drv_t disp_drv;
	lv_disp_drv_init(&disp_drv);

#if TFT_EXT_FB != 0
	BSP_SDRAM_Init();
#endif

	uint8_t lcd_status = BSP_LCD_Init();
	if (lcd_status != LCD_OK) {
		Error_Handler();
	}

	BSP_LCD_LayerDefaultInit(0, LAYER0_ADDRESS);     
	BSP_LCD_SelectLayer(0); 

	DMA_Config();

	disp_drv.disp_fill = tft_fill;
	disp_drv.disp_map = tft_map;
	disp_drv.disp_flush = tft_flush;
#if TFT_USE_GPU != 0
	DMA2D_Config();
	disp_drv.mem_blend = gpu_mem_blend;
	disp_drv.mem_fill = gpu_mem_fill;
#endif
	lv_disp_drv_register(&disp_drv);
}

/**********************
 *   STATIC FUNCTIONS
 **********************/

/**
 * Flush a color buffer
 * @param x1 left coordinate of the rectangle
 * @param x2 right coordinate of the rectangle
 * @param y1 top coordinate of the rectangle
 * @param y2 bottom coordinate of the rectangle
 * @param color_p pointer to an array of colors
 */
static void tft_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p)
{
	/*Return if the area is out the screen*/
	if(x2 < 0) return;
	if(y2 < 0) return;
	if(x1 > TFT_HOR_RES - 1) return;
	if(y1 > TFT_VER_RES - 1) return;

	/*Truncate the area to the screen*/
	int32_t act_x1 = x1 < 0 ? 0 : x1;
	int32_t act_y1 = y1 < 0 ? 0 : y1;
	int32_t act_x2 = x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : x2;
	int32_t act_y2 = y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : y2;

	x1_flush = act_x1;
	y1_flush = act_y1;
	x2_flush = act_x2;
	y2_fill = act_y2;
	y_fill_act = act_y1;
	buf_to_flush = color_p;


	  /*##-7- Start the DMA transfer using the interrupt mode #*/
	  /* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
	  /* Enable All the DMA interrupts */
	HAL_StatusTypeDef err;
	err = HAL_DMA_Start_IT(&DmaHandle,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_fill_act * TFT_HOR_RES + x1_flush],
			  (x2_flush - x1_flush + 1));
	if(err != HAL_OK)
	{
		while(1);	/*Halt on error*/
	}
}

/**
 * Fill a rectangular area with a color
 * @param x1 left coordinate of the rectangle
 * @param x2 right coordinate of the rectangle
 * @param y1 top coordinate of the rectangle
 * @param y2 bottom coordinate of the rectangle
 * @param color fill color
 */
static void tft_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color)
{
    /*Return if the area is out the screen*/
    if(x2 < 0) return;
    if(y2 < 0) return;
    if(x1 > TFT_HOR_RES - 1) return;
    if(y1 > TFT_VER_RES - 1) return;

    /*Truncate the area to the screen*/
    int32_t act_x1 = x1 < 0 ? 0 : x1;
    int32_t act_y1 = y1 < 0 ? 0 : y1;
    int32_t act_x2 = x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : x2;
    int32_t act_y2 = y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : y2;

	uint32_t x;
	uint32_t y;

	/*Fill the remaining area*/
	for(x = act_x1; x <= act_x2; x++) {
		for(y = act_y1; y <= act_y2; y++) {
			my_fb[y * TFT_HOR_RES + x] = color.full;
		}
	}
}


/**
 * Put a color map to a rectangular area
 * @param x1 left coordinate of the rectangle
 * @param x2 right coordinate of the rectangle
 * @param y1 top coordinate of the rectangle
 * @param y2 bottom coordinate of the rectangle
 * @param color_p pointer to an array of colors
 */
static void tft_map(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_p)
{
	/*Return if the area is out the screen*/
	if(x2 < 0) return;
	if(y2 < 0) return;
	if(x1 > TFT_HOR_RES - 1) return;
	if(y1 > TFT_VER_RES - 1) return;

	/*Truncate the area to the screen*/
	int32_t act_x1 = x1 < 0 ? 0 : x1;
	int32_t act_y1 = y1 < 0 ? 0 : y1;
	int32_t act_x2 = x2 > TFT_HOR_RES - 1 ? TFT_HOR_RES - 1 : x2;
	int32_t act_y2 = y2 > TFT_VER_RES - 1 ? TFT_VER_RES - 1 : y2;

#if LV_VDB_DOUBLE == 0
	uint32_t y;
	for(y = act_y1; y <= act_y2; y++) {
		memcpy((void*)&my_fb[y * TFT_HOR_RES + act_x1],
				color_p,
				(act_x2 - act_x1 + 1) * sizeof(my_fb[0]));
		color_p += x2 - x1 + 1;    /*Skip the parts out of the screen*/
	}
#else

	x1_flush = act_x1;
	y1_flush = act_y1;
	x2_flush = act_x2;
	y2_fill = act_y2;
	y_fill_act = act_y1;
	buf_to_flush = color_p;


	  /*##-7- Start the DMA transfer using the interrupt mode #*/
	  /* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
	  /* Enable All the DMA interrupts */
	  if(HAL_DMA_Start_IT(&DmaHandle,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_fill_act * TFT_HOR_RES + x1_flush],
						  (x2_flush - x1_flush + 1)) != HAL_OK)
	  {
	    while(1)
	    {
	    }
	  }

#endif
}


#if TFT_USE_GPU != 0

/**
 * Copy pixels to destination memory using opacity
 * @param dest a memory address. Copy 'src' here.
 * @param src pointer to pixel map. Copy it to 'dest'.
 * @param length number of pixels in 'src'
 * @param opa opacity (0, OPA_TRANSP: transparent ... 255, OPA_COVER, fully cover)
 */
static void gpu_mem_blend(lv_color_t * dest, const lv_color_t * src, uint32_t length, lv_opa_t opa)
{
	/*Wait for the previous operation*/
	HAL_DMA2D_PollForTransfer(&Dma2dHandle, 100);
	Dma2dHandle.Init.Mode         = DMA2D_M2M_BLEND;
	/* DMA2D Initialization */
	if(HAL_DMA2D_Init(&Dma2dHandle) != HAL_OK)
	{
		/* Initialization Error */
		while(1);
	}

	Dma2dHandle.LayerCfg[1].InputAlpha = opa;
    HAL_DMA2D_ConfigLayer(&Dma2dHandle, 1);
	HAL_DMA2D_BlendingStart(&Dma2dHandle, (uint32_t) src, (uint32_t) dest, (uint32_t)dest, length, 1);
}

/**
 * Copy pixels to destination memory using opacity
 * @param dest a memory address. Copy 'src' here.
 * @param src pointer to pixel map. Copy it to 'dest'.
 * @param length number of pixels in 'src'
 * @param opa opacity (0, OPA_TRANSP: transparent ... 255, OPA_COVER, fully cover)
 */
static void gpu_mem_fill(lv_color_t * dest, uint32_t length, lv_color_t color)
{
	/*Wait for the previous operation*/
	HAL_DMA2D_PollForTransfer(&Dma2dHandle, 100);

   Dma2dHandle.Init.Mode         = DMA2D_R2M;
   /* DMA2D Initialization */
   if(HAL_DMA2D_Init(&Dma2dHandle) != HAL_OK)
   {
     /* Initialization Error */
     while(1);
   }


	Dma2dHandle.LayerCfg[1].InputAlpha = 0xff;
    HAL_DMA2D_ConfigLayer(&Dma2dHandle, 1);
	HAL_DMA2D_BlendingStart(&Dma2dHandle, (uint32_t) lv_color_to24(color), (uint32_t) dest, (uint32_t)dest, length, 1);
}

#endif

/**
  * @brief  Configure the DMA controller according to the Stream parameters
  *         defined in main.h file
  * @note  This function is used to :
  *        -1- Enable DMA2 clock
  *        -2- Select the DMA functional Parameters
  *        -3- Select the DMA instance to be used for the transfer
  *        -4- Select Callbacks functions called after Transfer complete and
               Transfer error interrupt detection
  *        -5- Initialize the DMA stream
  *        -6- Configure NVIC for DMA transfer complete/error interrupts
  * @param  None
  * @retval None
  */
static void DMA_Config(void)
{
  /*## -1- Enable DMA2 clock #################################################*/
  __HAL_RCC_DMA2_CLK_ENABLE();

  /*##-2- Select the DMA functional Parameters ###############################*/
  DmaHandle.Init.Channel = DMA_CHANNEL;                     /* DMA_CHANNEL_0                    */
  DmaHandle.Init.Direction = DMA_MEMORY_TO_MEMORY;          /* M2M transfer mode                */
  DmaHandle.Init.PeriphInc = DMA_PINC_ENABLE;               /* Peripheral increment mode Enable */
  DmaHandle.Init.MemInc = DMA_MINC_ENABLE;                  /* Memory increment mode Enable     */
  DmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD; /* Peripheral data alignment : 16bit */
  DmaHandle.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;    /* memory data alignment : 16bit     */
  DmaHandle.Init.Mode = DMA_NORMAL;                         /* Normal DMA mode                  */
  DmaHandle.Init.Priority = DMA_PRIORITY_HIGH;              /* priority level : high            */
  DmaHandle.Init.FIFOMode = DMA_FIFOMODE_ENABLE;            /* FIFO mode enabled                */
  DmaHandle.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL; /* FIFO threshold: 1/4 full   */
  DmaHandle.Init.MemBurst = DMA_MBURST_SINGLE;              /* Memory burst                     */
  DmaHandle.Init.PeriphBurst = DMA_PBURST_SINGLE;           /* Peripheral burst                 */

  /*##-3- Select the DMA instance to be used for the transfer : DMA2_Stream0 #*/
  DmaHandle.Instance = DMA_STREAM;

  /*##-4- Initialize the DMA stream ##########################################*/
  if(HAL_DMA_Init(&DmaHandle) != HAL_OK)
  {
    /* Turn LED4 on: in case of Initialization Error */
    BSP_LED_On(LED2);
    while(1)
    {
    }
}

  /*##-5- Select Callbacks functions called after Transfer complete and Transfer error */
  HAL_DMA_RegisterCallback(&DmaHandle, HAL_DMA_XFER_CPLT_CB_ID, DMA_TransferComplete);
  HAL_DMA_RegisterCallback(&DmaHandle, HAL_DMA_XFER_ERROR_CB_ID, DMA_TransferError);

  /*##-6- Configure NVIC for DMA transfer complete/error interrupts ##########*/
  HAL_NVIC_SetPriority(DMA_STREAM_IRQ, 0, 0);
  HAL_NVIC_EnableIRQ(DMA_STREAM_IRQ);
}

/**
  * @brief  DMA conversion complete callback
  * @note   This function is executed when the transfer complete interrupt
  *         is generated
  * @retval None
  */
static void DMA_TransferComplete(DMA_HandleTypeDef *han)
{
	y_fill_act ++;

	if(y_fill_act > y2_fill) {
		  lv_flush_ready();
	} else {
	  buf_to_flush += x2_flush - x1_flush + 1;
	  /*##-7- Start the DMA transfer using the interrupt mode ####################*/
	  /* Configure the source, destination and buffer size DMA fields and Start DMA Stream transfer */
	  /* Enable All the DMA interrupts */
	  if(HAL_DMA_Start_IT(han,(uint32_t)buf_to_flush, (uint32_t)&my_fb[y_fill_act * TFT_HOR_RES + x1_flush],
						  (x2_flush - x1_flush + 1)) != HAL_OK)
	  {
	    while(1);	/*Halt on error*/
	  }
	}
}

/**
  * @brief  DMA conversion error callback
  * @note   This function is executed when the transfer error interrupt
  *         is generated during DMA transfer
  * @retval None
  */
static void DMA_TransferError(DMA_HandleTypeDef *han)
{

}



/**
  * @brief  This function handles DMA Stream interrupt request.
  * @param  None
  * @retval None
  */
void DMA_STREAM_IRQHANDLER(void)
{
    /* Check the interrupt and clear flag */
    HAL_DMA_IRQHandler(&DmaHandle);
}

#if TFT_USE_GPU != 0
/**
  * @brief  DMA2D Transfer completed callback
  * @param  hdma2d: DMA2D handle.
  * @note   This example shows a simple way to report end of DMA2D transfer, and
  *         you can add your own implementation.
  * @retval None
  */
static void DMA2D_TransferComplete(DMA2D_HandleTypeDef *hdma2d)
{

}

/**
  * @brief  DMA2D error callbacks
  * @param  hdma2d: DMA2D handle
  * @note   This example shows a simple way to report DMA2D transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
static void DMA2D_TransferError(DMA2D_HandleTypeDef *hdma2d)
{

}

/**
  * @brief DMA2D configuration.
  * @note  This function Configure the DMA2D peripheral :
  *        1) Configure the Transfer mode as memory to memory with blending.
  *        2) Configure the output color mode as RGB565 pixel format.
  *        3) Configure the foreground
  *          - first image loaded from FLASH memory
  *          - constant alpha value (decreased to see the background)
  *          - color mode as RGB565 pixel format
  *        4) Configure the background
  *          - second image loaded from FLASH memory
  *          - color mode as RGB565 pixel format
  * @retval None
  */
static void DMA2D_Config(void)
{
  /* Configure the DMA2D Mode, Color Mode and output offset */
  Dma2dHandle.Init.Mode         = DMA2D_M2M_BLEND;
  Dma2dHandle.Init.ColorMode    = DMA2D_ARGB8888;
  Dma2dHandle.Init.OutputOffset = 0x0;

  /* DMA2D Callbacks Configuration */
  Dma2dHandle.XferCpltCallback  = DMA2D_TransferComplete;
  Dma2dHandle.XferErrorCallback = DMA2D_TransferError;

  /* Foreground Configuration */
  Dma2dHandle.LayerCfg[1].AlphaMode = DMA2D_REPLACE_ALPHA;
  Dma2dHandle.LayerCfg[1].InputAlpha = 0xFF;
  Dma2dHandle.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  Dma2dHandle.LayerCfg[1].InputOffset = 0x0;

  /* Background Configuration */
  Dma2dHandle.LayerCfg[0].AlphaMode = DMA2D_REPLACE_ALPHA;
  Dma2dHandle.LayerCfg[0].InputAlpha = 0xFF;
  Dma2dHandle.LayerCfg[0].InputColorMode = DMA2D_INPUT_ARGB8888;
  Dma2dHandle.LayerCfg[0].InputOffset = 0x0;

  Dma2dHandle.Instance   = DMA2D;

  /* DMA2D Initialization */
  if(HAL_DMA2D_Init(&Dma2dHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  HAL_DMA2D_ConfigLayer(&Dma2dHandle, 0);
  HAL_DMA2D_ConfigLayer(&Dma2dHandle, 1);
}
#endif



/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(1)
  {
		BSP_LED_On(LED1);
		HAL_Delay(400);
		BSP_LED_Off(LED1);
		HAL_Delay(100);
  }
}
