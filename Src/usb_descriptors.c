#include "usb_descriptors.h"
#include "uvc.h"

__ALIGN_BEGIN static uint8_t COMPOSITE_UVC_DISPLAY_DESCRIPTOR[] __ALIGN_END =
{
  /*     */
  /* UVC */
  /*     */
    /* Configuration 1 */
  USB_CONFIGUARTION_DESC_SIZE,               // bLength                  9
  USB_CONFIGURATION_DESCRIPTOR_TYPE,         // bDescriptorType          2
  WBVAL((USB_VIDEO_DESC_SIZ + USB_DISPLAY_CONFIG_DESC_SIZ - 0x09)),
  0x03,                                      // bNumInterfaces           2
  0x01,                                      // bConfigurationValue      1 ID of this configuration
  0x00,                                      // iConfiguration           0 no description available
  USB_CONFIG_BUS_POWERED ,                   // bmAttributes          0x80 Bus Powered
  USB_CONFIG_POWER_MA(500),                  // bMaxPower              100 mA
  
  
  /* Interface Association Descriptor */
  UVC_INTERFACE_ASSOCIATION_DESC_SIZE,       // bLength                  8
  USB_INTERFACE_ASSOCIATION_DESCRIPTOR_TYPE, // bDescriptorType         11
  0x00,                                      // bFirstInterface          0
  0x02,                                      // bInterfaceCount          2
  CC_VIDEO,                                  // bFunctionClass          14 Video
  SC_VIDEO_INTERFACE_COLLECTION,             // bFunctionSubClass        3 Video Interface Collection
  PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
  0x02,                                      // iFunction                2
  
  
  /* VideoControl Interface Descriptor */
  
  
  /* Standard VC Interface Descriptor  = interface 0 */
  USB_INTERFACE_DESC_SIZE,                   // bLength                  9
  USB_INTERFACE_DESCRIPTOR_TYPE,             // bDescriptorType          4
  USB_UVC_VCIF_NUM,                          // bInterfaceNumber         0 index of this interface (VC)
  0x00,                                      // bAlternateSetting        0 index of this setting
  0x00,                                      // bNumEndpoints            0 no endpoints
  CC_VIDEO,                                  // bInterfaceClass         14 Video
  SC_VIDEOCONTROL,                           // bInterfaceSubClass       1 Video Control
  PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
  0x02,                                      // iFunction                2
  
  
  /* Class-specific VC Interface Descriptor */
  UVC_VC_INTERFACE_HEADER_DESC_SIZE(1),      // bLength                 13 12 + 1 (header + 1*interface
  CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
  VC_HEADER,                                 // bDescriptorSubtype      1 (HEADER)
  WBVAL(UVC_VERSION),                        // bcdUVC                  1.10 or 1.00
  WBVAL(VC_TERMINAL_SIZ),                    // wTotalLength            header+units+terminals
  DBVAL(48000000),                         // dwClockFrequency  			48.000000 MHz
  0x01,                                      // bInCollection            1 one streaming interface
  0x01,                                      // baInterfaceNr( 0)        1 VS interface 1 belongs to this VC interface
  
  
  /* Input Terminal Descriptor (Camera) */
  UVC_CAMERA_TERMINAL_DESC_SIZE(2),          // bLength                 17 15 + 2 controls
  CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
  VC_INPUT_TERMINAL,                         // bDescriptorSubtype       2 (INPUT_TERMINAL)
  0x01,                                      // bTerminalID              1 ID of this Terminal
  WBVAL(ITT_CAMERA),                         // wTerminalType       0x0201 Camera Sensor
  0x00,                                      // bAssocTerminal           0 no Terminal associated
  0x00,                                      // iTerminal                0 no description available
  WBVAL(0x0000),                             // wObjectiveFocalLengthMin 0
  WBVAL(0x0000),                             // wObjectiveFocalLengthMax 0
  WBVAL(0x0000),                             // wOcularFocalLength       0
  0x02,                                      // bControlSize             2
  0x00, 0x00,                                // bmControls          0x0000 no controls supported
  
  /* Output Terminal Descriptor */
  UVC_OUTPUT_TERMINAL_DESC_SIZE(0),          // bLength                  9
  CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
  VC_OUTPUT_TERMINAL,                        // bDescriptorSubtype       3 (OUTPUT_TERMINAL)
  0x02,                                      // bTerminalID              2 ID of this Terminal
  WBVAL(TT_STREAMING),                       // wTerminalType       0x0101 USB streaming terminal
  0x00,                                      // bAssocTerminal           0 no Terminal assiciated
  0x01,                                      // bSourceID                1 input pin connected to output pin unit 1
  0x00,                                      // iTerminal                0 no description available
  
  
  /* Video Streaming (VS) Interface Descriptor */
  
  
  /* Standard VS Interface Descriptor  = interface 1 */
  // alternate setting 0 = Zero Bandwidth
  USB_INTERFACE_DESC_SIZE,                   // bLength                  9
  USB_INTERFACE_DESCRIPTOR_TYPE,             // bDescriptorType          4
  USB_UVC_VSIF_NUM,                          // bInterfaceNumber         1 index of this interface
  0x00,                                      // bAlternateSetting        0 index of this setting
  0x00,                                      // bNumEndpoints            0 no EP used
  CC_VIDEO,                                  // bInterfaceClass         14 Video
  SC_VIDEOSTREAMING,                         // bInterfaceSubClass       2 Video Streaming
  PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
  0x00,                                      // iInterface               0 no description available
  
  
  /* Class-specific VS Header Descriptor (Input) */
  UVC_VS_INTERFACE_INPUT_HEADER_DESC_SIZE(1,1),// bLength               14 13 + (1*1) (no specific controls used)
  CS_INTERFACE,                              // bDescriptorType         36 (INTERFACE)
  VS_INPUT_HEADER,                           // bDescriptorSubtype       1 (INPUT_HEADER)
  0x01,                                      // bNumFormats              1 one format descriptor follows
  WBVAL(VC_HEADER_SIZ),
  USB_ENDPOINT_IN(USB_UVC_ENDPOINT),         // bEndPointAddress      0x83 EP 3 IN
  0x00,                                      // bmInfo                   0 no dynamic format change supported
  0x02,                                      // bTerminalLink            2 supplies terminal ID 2 (Output terminal)
  0x02,                                      // bStillCaptureMethod      0 NO supports still image capture
  0x01,                                      // bTriggerSupport          0 HW trigger supported for still image capture
  0x00,                                      // bTriggerUsage            0 HW trigger initiate a still image capture
  0x01,                                      // bControlSize             1 one byte bmaControls field size
  0x00,                                      // bmaControls(0)           0 no VS specific controls
  
  /* Class-specific VS Format Descriptor  */
  VS_FORMAT_UNCOMPRESSED_DESC_SIZE,     /* bLength 27*/
  CS_INTERFACE,                         /* bDescriptorType : CS_INTERFACE */
  VS_FORMAT_MJPEG,                      /* bDescriptorSubType : VS_FORMAT_MJPEG subtype */
  0x01,                                 /* bFormatIndex : First (and only) format descriptor */
  0x01,                                 /* bNumFrameDescriptors : One frame descriptor for this format follows. */
  0x01,                                 /* bmFlags : Uses fixed size samples.. */
  0x01,                                 /* bDefaultFrameIndex : Default frame index is 1. */
  0x00,                                 /* bAspectRatioX : Non-interlaced stream not required. */
  0x00,                                 /* bAspectRatioY : Non-interlaced stream not required. */
  0x00,                                 /* bmInterlaceFlags : Non-interlaced stream */
  0x00,                                 /* bCopyProtect : No restrictions imposed on the duplication of this video stream. */
  
  /* Class-specific VS Frame Descriptor */
  VS_FRAME_COMPRESSED_DESC_SIZE,      	/* bLength 2A */
  CS_INTERFACE,                         /* bDescriptorType : CS_INTERFACE */
  VS_FRAME_MJPEG,                				/* bDescriptorSubType : VS_FRAME_UNCOMPRESSED */
  0x01,                                 /* bFrameIndex : First (and only) frame descriptor */
  0x00,                                 /* bmCapabilities : Still images using capture method 0 are supported at this frame setting.D1: Fixed frame-rate. */
  WBVAL(WIDTH),                         /* wWidth (2bytes): Width of frame is 128 pixels. */
  WBVAL(HEIGHT),                        /* wHeight (2bytes): Height of frame is 64 pixels. */
  DBVAL(MIN_BIT_RATE),                  /* dwMinBitRate (4bytes): Min bit rate in bits/s  */ // 128*64*16*5 = 655360 = 0x000A0000 //5fps
  DBVAL(MAX_BIT_RATE),                  /* dwMaxBitRate (4bytes): Max bit rate in bits/s  */ // 128*64*16*5 = 655360 = 0x000A0000
  DBVAL(MAX_FRAME_SIZE),                /* dwMaxVideoFrameBufSize (4bytes): Maximum video or still frame size, in bytes. */ // 128*64*2 = 16384 = 0x00004000
  DBVAL(INTERVAL),				        			/* dwDefaultFrameInterval : 1,000,000 * 100ns -> 10 FPS */ // 5 FPS -> 200ms -> 200,000 us -> 2,000,000 X 100ns = 0x001e8480
  0x00,                                 /* bFrameIntervalType : Continuous frame interval */
  DBVAL(INTERVAL),                      /* dwMaxFrameInterval : 1,000,000 ns  *100ns -> 10 FPS */
  DBVAL(INTERVAL),                      /* dwMFrameInterval : 1,000,000 ns  *100ns -> 10 FPS */
  0x00, 0x00, 0x00, 0x00,               /* dwFrameIntervalStep : No frame interval step supported. */
  
  /* Color Matching Descriptor */
  VS_COLOR_MATCHING_DESC_SIZE,          /* bLength */
  CS_INTERFACE,                         /* bDescriptorType : CS_INTERFACE */
  0x0D,                                 /* bDescriptorSubType : VS_COLORFORMAT */
  0x00,                                 /* bColorPrimarie : 1: BT.709, sRGB (default) */
  0x00,                                 /* bTransferCharacteristics : 1: BT.709 (default) */
  0x00,                                 /* bMatrixCoefficients : 1: BT. 709. */
  
  /* Standard VS Interface Descriptor  = interface 1 */
  // alternate setting 1 = operational setting
  USB_INTERFACE_DESC_SIZE,                   // bLength                  9
  USB_INTERFACE_DESCRIPTOR_TYPE,             // bDescriptorType          4
  USB_UVC_VSIF_NUM,                          // bInterfaceNumber         1 index of this interface
  0x01,                                      // bAlternateSetting        1 index of this setting
  0x01,                                      // bNumEndpoints            1 one EP used
  CC_VIDEO,                                  // bInterfaceClass         14 Video
  SC_VIDEOSTREAMING,                         // bInterfaceSubClass       2 Video Streaming
  PC_PROTOCOL_UNDEFINED,                     // bInterfaceProtocol       0 (protocol undefined)
  0x00,                                      // iInterface               0 no description available
  
  /* Standard VS Isochronous Video data Endpoint Descriptor */
  USB_ENDPOINT_DESC_SIZE,                   // bLength                  7
  USB_ENDPOINT_DESCRIPTOR_TYPE,             // bDescriptorType          5 (ENDPOINT)
  USB_ENDPOINT_IN(USB_UVC_ENDPOINT),        // bEndpointAddress      0x83 EP 3 IN
  USB_ENDPOINT_TYPE_ISOCHRONOUS | USB_ENDPOINT_SYNC_ASYNCHRONOUS,            // bmAttributes             1 isochronous transfer type
  WBVAL(VIDEO_PACKET_SIZE),                 // wMaxPacketSize
  0x01,                                      // bInterval                1 one frame interval
  
  /*     			 */
  /*  DISPLAY  */
  /*     			 */
  
	/* Standard Interface descriptor */
	0x09,                                 /* Descriptor size */
	USB_DESC_TYPE_INTERFACE,              /* Interface Descriptor type */
	0x02,  																/* Interface number */
	0x00,                                 /* Alternate setting number */
	0x01,                                 /* Number of end points */
	0xFF,                                 /* Interface class */
	0xFF,                                 /* Interface sub class */
	0xFF,                                 /* Interface protocol code */
	0x00,       													/* Interface descriptor string index */
	
	/// Endpoint for Display interface
	0x07,                                      /* bLength */
	USB_DESC_TYPE_ENDPOINT,                    /* bDescriptorType - EP */
	USB_ENDPOINT_OUT(USB_DISPLAY_ENDPOINT),    /* bEndpointAddress - OUT EP */
	USBD_EP_TYPE_BULK,                         /* bmAttributes - Bulk */
	LOBYTE(USB_DISPLAY_MAX_SIZE),                                      
	HIBYTE(USB_DISPLAY_MAX_SIZE),   /* wMaxPacketSize */
	0x00,                                      /* bInterval */	
	
};


uint16_t get_Composite_Descriptor_Length(void){
	return sizeof(COMPOSITE_UVC_DISPLAY_DESCRIPTOR);
}


uint8_t* get_Composite_Descriptor(void){
	return COMPOSITE_UVC_DISPLAY_DESCRIPTOR;
}
