/* Define to prevent recursive inclusion -------------------------------------*/ 
#ifndef __USB_COMPOSITE_CORE_H
#define __USB_COMPOSITE_CORE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

#define MAX_CLASSES 4
#define MAX_ENDPOINTS 16

extern USBD_ClassTypeDef USBD_Composite;
extern int in_endpoint_to_class[MAX_ENDPOINTS];
extern int out_endpoint_to_class[MAX_ENDPOINTS];

void USBD_Composite_Set_Descriptor(const uint8_t *descriptor, uint16_t size);
void USBD_Composite_Set_Classes(USBD_ClassTypeDef *class0, USBD_ClassTypeDef *class1);
	 
#ifdef __cplusplus
}
#endif

#endif  /* __USB_COMPOSITE_CORE_H */
