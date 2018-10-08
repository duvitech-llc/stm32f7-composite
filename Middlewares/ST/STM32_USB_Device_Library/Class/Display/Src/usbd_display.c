/**
  ******************************************************************************
  * @file    usbd_display.c
  * @author  MCD Application Team
  * @brief   This file provides the HID core functions.
  *
  * @verbatim
  *
  *          ===================================================================
  *                                DISPLAY Class  Description
  *          ===================================================================
  *
  *
  *
  *
  *
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_display.h"
#include "usbd_ctlreq.h"
#include "main.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_DISPLAY
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_DISPLAY_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_DISPLAY_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_DISPLAY_Private_Macros
  * @{
  */

/**
  * @}
  */




/** @defgroup USBD_DISPLAY_Private_FunctionPrototypes
  * @{
  */

extern uint32_t LCD_X_Size;
extern uint32_t LCD_Y_Size;

static uint8_t  USBD_DISPLAY_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx);

static uint8_t  USBD_DISPLAY_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx);

static uint8_t  USBD_DISPLAY_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);

static uint8_t  *USBD_DISPLAY_GetCfgDesc (uint16_t *length);

static uint8_t  *USBD_DISPLAY_GetDeviceQualifierDesc (uint16_t *length);


static uint8_t  USBD_DISPLAY_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);


/* located in SDRAM */
static struct sBufferStruct* pFillBuffer = 0;
static struct sBufferStruct* pTempBuffer = 0;
static uint32_t frame_count = 0;
/**
  * @}
  */

/** @defgroup USBD_DISPLAY_Private_Variables
  * @{
  */

USBD_ClassTypeDef  USBD_DISPLAY_ClassDriver =
{
  USBD_DISPLAY_Init,
  USBD_DISPLAY_DeInit,
  USBD_DISPLAY_Setup,
  NULL,
  NULL,
  NULL,
  USBD_DISPLAY_DataOut,
  NULL,
  NULL,
  NULL,
  USBD_DISPLAY_GetCfgDesc,
  USBD_DISPLAY_GetCfgDesc,
  USBD_DISPLAY_GetCfgDesc,
  USBD_DISPLAY_GetDeviceQualifierDesc,
};


#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
/* USB DISPLAY device Configuration Descriptor */
__ALIGN_BEGIN  uint8_t USBD_DISPLAY_CfgDesc[USB_DISPLAY_CONFIG_DESC_SIZ] __ALIGN_END =
{
  0x09, /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_DISPLAY_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x02,         /*iConfiguration: Index of string descriptor describing the configuration*/
  USB_CONFIG_BUS_POWERED ,                   // bmAttributes          0x80 Bus Powered
  USB_CONFIG_POWER_MA(500),                  // bMaxPower              100 mA
  
  /* 09 */

  /**********  Descriptor of DISPLAY interface 0 Alternate setting 0 **************/
#if 0	
	/*************************************************************/
	/* Interface Association Descriptor - For Custom Display Class */
	/*************************************************************/
  DISPLAY_INTERFACE_ASSOCIATION_DESC_SIZE,     /* bLength                  8 */
  USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE,  /* bDescriptorType         11 */
	0x00,        																/* bFirstInterface */
	0x01,                                       /* bInterface Count */
	0xFF,                                       /* bInterfaceClass */
	0xFF,                                       /* bInterfaceSubClass */
	0xFF,                                       /* bInterfaceProtocol */
	0x00,             													/* iFunction- string index */
#endif

	/* Standard Interface descriptor */
	0x09,                                 /* Descriptor size */
	USB_DESC_TYPE_INTERFACE,              /* Interface Descriptor type */
	0x00,  																/* Interface number */
	0x00,                                 /* Alternate setting number */
	0x01,                                 /* Number of end points */
	0xFF,                                 /* Interface class */
	0xFF,                                 /* Interface sub class */
	0xFF,                                 /* Interface protocol code */
	0x00,       													/* Interface descriptor string index */
	
	/// Endpoint for Display interface
	0x07,                                      /* bLength */
	USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType - EP */
	USB_ENDPOINT_OUT(USB_DISPLAY_ENDPOINT),      																	/* bEndpointAddress - OUT EP */
	USBD_EP_TYPE_BULK,                         /* bmAttributes - Bulk */
	LOBYTE(USB_DISPLAY_MAX_SIZE),                                      
	HIBYTE(USB_DISPLAY_MAX_SIZE),   /* wMaxPacketSize */
	0x00,                                      /* bInterval */	
	
};

#if defined ( __ICCARM__ ) /*!< IAR Compiler */
  #pragma data_alignment=4
#endif
/* USB Standard Device Descriptor */
__ALIGN_BEGIN uint8_t USBD_DISPLAY_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

static void rtc_SetFirmwareUpdateRegister(void){
	__HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_BKPSRAM_CLK_ENABLE();	
	HAL_PWREx_EnableBkUpReg();
	
	HAL_PWR_EnableBkUpAccess();
	
	*(__IO uint8_t *) (BKPSRAM_BASE) = 0x4A; // A4ABBA;
	
	if((*(__IO uint8_t *) (BKPSRAM_BASE)) != 0x4A){
		printf("Failure\r\n");
	}
	
	HAL_PWREx_DisableBkUpReg();
}


/**
  * @}
  */

/** @defgroup USBD_DISPLAY_Private_Functions
  * @{
  */

/**
  * @brief  USBD_DISPLAY_Init
  *         Initialize the DISPLAY interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_DISPLAY_Init (USBD_HandleTypeDef *pdev,
                               uint8_t cfgidx)
{
  uint8_t ret = USBD_FAIL; 

	printf("%s\r\n", __func__);
	// DMA_Config();
  /* Open EP IN */
  USBD_LL_OpenEP(pdev,
		      USB_ENDPOINT_OUT(USB_DISPLAY_ENDPOINT),		      
          USBD_EP_TYPE_BULK,
					USB_DISPLAY_MAX_SIZE);
 // pdev->ep_out[USB_ENDPOINT_OUT(USB_DISPLAY_ENDPOINT) & 0xFU].is_used = 1U;
	
	/* continuous buffer clears both */
	pFillBuffer = deQueue(empty_disp_buffers);
	if(pFillBuffer == 0){
			Error_Handler();
	}
	
	/* just in case */
	pFillBuffer->pos = pFillBuffer->address;
	pFillBuffer->length=0;
	
	/* Prepare Out endpoint to receive next packet */
	ret = USBD_LL_PrepareReceive(pdev,
												 USB_ENDPOINT_OUT(USB_DISPLAY_ENDPOINT),
												 (uint8_t*)pFillBuffer->pos,
												 USB_DISPLAY_MAX_SIZE);
	if(ret != USBD_OK)
		printf("Failed to prepare receive for BULK\r\n");
	
	ret = USBD_OK; 
	
	return ret;
}

/**
  * @brief  USBD_DISPLAY_Init
  *         DeInitialize the DISPLAY layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_DISPLAY_DeInit (USBD_HandleTypeDef *pdev,
                                 uint8_t cfgidx)
{
	printf("%s\r\n",__func__);
  USBD_LL_FlushEP(pdev, USB_ENDPOINT_OUT(USB_DISPLAY_ENDPOINT));
  // pdev->ep_out[USB_ENDPOINT_OUT(USB_DISPLAY_ENDPOINT) & 0xFU].is_used = 0U;
	/* Open EP OUT */
  USBD_LL_CloseEP(pdev,
		      USB_ENDPOINT_OUT(USB_DISPLAY_ENDPOINT));
   
  return USBD_OK;
}

/**
  * @brief  USBD_DISPLAY_Setup
  *         Handle the DISPLAY specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_DISPLAY_Setup (USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
	printf("%s\r\n",__func__);

  return USBD_OK;
}


/**
  * @brief  USBD_DISPLAY_GetCfgDesc
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_DISPLAY_GetCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_DISPLAY_CfgDesc);
  return USBD_DISPLAY_CfgDesc;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_DISPLAY_DeviceQualifierDescriptor (uint16_t *length)
{
  *length = sizeof (USBD_DISPLAY_DeviceQualifierDesc);
  return USBD_DISPLAY_DeviceQualifierDesc;
}

/**
  * @brief  USBD_DISPLAY_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t  USBD_DISPLAY_DataOut (USBD_HandleTypeDef *pdev,
                              uint8_t epnum)
{
	
	// printf("%s\r\n",__func__);
	uint32_t len = USBD_LL_GetRxDataSize (pdev, epnum);

	// Check the first word for start of jpeg frame
	if(*(uint16_t*)pFillBuffer->pos == 0xd8ff){
		// start of frame
		
		if(*(uint32_t*)pFillBuffer->pos == 0xb00bd8ff){					
			printf("bootloader requested\r\n");
			rtc_SetFirmwareUpdateRegister();
			NVIC_SystemReset();		
		}
		frame_count++;
		pFillBuffer->frame_id = frame_count;
	}
	
	// move pointer
	if(len<USB_DISPLAY_MAX_SIZE){
		pFillBuffer->pos+=(len-2);
	}else{
		pFillBuffer->pos+=(USB_DISPLAY_MAX_SIZE-2);
	}

	// Check the last word for end of jpeg frame	
	if ((*(uint8_t*)pFillBuffer->pos == 0xFF) && (*(uint8_t*)(pFillBuffer->pos + 1) == 0xD9)){
		// end of frame
		pFillBuffer->pos+=2;
		
		/* check for empty buffer before we send this to be shown */
		pTempBuffer = deQueue(empty_disp_buffers);
		if(pTempBuffer != 0){
			pFillBuffer->length = pFillBuffer->pos - pFillBuffer->address;
			enQueue(process_disp_buffers, pFillBuffer);			
			
			pFillBuffer = pTempBuffer;
			pTempBuffer = 0;
		}else{
			printf("no empty frames\r\n");
		}
		
		/* just in case set length and position */
		pFillBuffer->length = 0;
		pFillBuffer->pos = pFillBuffer->address;
		
	}else{	
		// move remainder of the pointer
		pFillBuffer->pos+=2;
		if( pFillBuffer->pos - pFillBuffer->address > pFillBuffer->maxlen ){
			/* data overflow */
			printf("overflow\r\n");
		}			
	}
	
	USBD_LL_PrepareReceive(pdev,
													 USB_ENDPOINT_OUT(USB_DISPLAY_ENDPOINT),
													 (uint8_t*)pFillBuffer->pos,
													 USB_DISPLAY_MAX_SIZE);

  return USBD_OK;
}

/**
* @brief  DeviceQualifierDescriptor
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_DISPLAY_GetDeviceQualifierDesc (uint16_t *length)
{
  *length = sizeof (USBD_DISPLAY_DeviceQualifierDesc);
  return USBD_DISPLAY_DeviceQualifierDesc;
}





/**
  * @}
  */


/**
  * @}
  */


/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
