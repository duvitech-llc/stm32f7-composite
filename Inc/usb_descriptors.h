#ifndef __USB_DESCRIPTORS_H
#define __USB_DESCRIPTORS_H

#include "usbd_composite.h"
#include "usbd_desc.h"
#include "usbd_video_core.h"
#include "usbd_display.h"
#include "usbd_ctlreq.h"

#define  USBD_IDX_CAM_INTERFACE_STR                   0x03 
#define  USBD_IDX_DISPLAY_INTERFACE_STR               0x04 

#define USBD_CAMERA_INTERFACE_STRING       "Duvitech Camera Interface"
#define USBD_DISPLAY_INTERFACE_STRING     "Duvitech Display Interface"

uint16_t get_Composite_Descriptor_Length(void);
uint8_t* get_Composite_Descriptor(void);

#endif
