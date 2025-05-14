#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbd_def.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_hid.h"

/* USER CODE BEGIN INCLUDE */


extern USBD_HandleTypeDef hUsbDeviceFS;

extern uint8_t UserTxBuffer[512];
extern uint8_t UserRxBuffer[512];
extern uint8_t CDC_EpAdd_Inst[3]; 
extern uint8_t HID_EpAdd_Inst;    
extern uint8_t HID_InstID;
extern uint8_t CDC_InstID;



/** USB Device initialization function. */
void MX_USB_DEVICE_Init(void);



#ifdef __cplusplus
}
#endif
