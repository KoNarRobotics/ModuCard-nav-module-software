#include "usb_device.h"
#include "usbd_desc.h"
#include "usbd_def.h"


USBD_HandleTypeDef hUsbDeviceFS;

// extern USBD_HandleTypeDef hUsbDeviceFS;
uint8_t UserTxBuffer[512] = "MY CDC is Working!\r\n";
uint8_t UserRxBuffer[512];

// USBD_CDC_ItfTypeDef USBD_CDC_Template_fops;
uint8_t CDC_EpAdd_Inst[3] = { CDC_IN_EP, CDC_OUT_EP, CDC_CMD_EP }; /* CDC Endpoint Addresses array */
uint8_t HID_EpAdd_Inst    = HID_EPIN_ADDR;                         /* HID Endpoint Address array */
// uint8_t hid_report_buffer[4];
uint8_t HID_InstID = 0;
uint8_t CDC_InstID = 0;


void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &Class_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_CDC_Template_fops) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

