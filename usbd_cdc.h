/**
  ******************************************************************************
  * @file    usbd_cdc.h
  * @author  MCD Application Team
  * @brief   header file for the usbd_cdc.c file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

#ifndef __USB_CDC_H
#define __USB_CDC_H

#ifdef __cplusplus
extern "C" {
#endif

#include  "usbd_def.h"

/* CDC Endpoints parameters: you can fine tune these values depending on the 
needed baudrates and performance. */

enum
{
    CDC_IN_EP = 0x81U,  /* EP1 for data IN */
    CDC_OUT_EP = 0x01U,  /* EP1 for data OUT */
    CDC_CMD_EP = 0x82U,  /* EP2 for CDC commands */
    CDC_DATA_HS_MAX_PACKET_SIZE = 512U,  /* Endpoint IN & OUT Packet size */
    CDC_DATA_FS_MAX_PACKET_SIZE = 64U,  /* Endpoint IN & OUT Packet size */
    CDC_CMD_PACKET_SIZE = 8U,  /* Control Endpoint Packet size */
    USB_CDC_CONFIG_DESC_SIZ = 67U,
    CDC_DATA_HS_IN_PACKET_SIZE = CDC_DATA_HS_MAX_PACKET_SIZE,
    CDC_DATA_HS_OUT_PACKET_SIZE = CDC_DATA_HS_MAX_PACKET_SIZE,
    CDC_DATA_FS_IN_PACKET_SIZE = CDC_DATA_FS_MAX_PACKET_SIZE,
    CDC_DATA_FS_OUT_PACKET_SIZE = CDC_DATA_FS_MAX_PACKET_SIZE,
    CDC_SEND_ENCAPSULATED_COMMAND = 0x00U,
    CDC_GET_ENCAPSULATED_RESPONSE = 0x01U,
    CDC_SET_COMM_FEATURE = 0x02U,
    CDC_GET_COMM_FEATURE = 0x03U,
    CDC_CLEAR_COMM_FEATURE = 0x04U,
    CDC_SET_LINE_CODING = 0x20U,
    CDC_GET_LINE_CODING = 0x21U,
    CDC_SET_CONTROL_LINE_STATE = 0x22U,
    CDC_SEND_BREAK = 0x23U,
    CDC_HS_BINTERVAL = 0x10U,
    CDC_FS_BINTERVAL = 0x10U,

    HL_RX_BUFFER_SIZE = 256,
    APP_RX_DATA_SIZE = 256,
};

typedef enum
{
    USB_CDC_RX_BUFFER_OK   = 0U,
    USB_CDC_RX_BUFFER_NO_DATA
} 
USB_CDC_RX_BUFFER_StatusTypeDef;

uint8_t CDC_Transmit(USBD_HandleTypeDef* pdev, const uint8_t* Buf, uint16_t sz);
uint8_t CDC_ReadRxBuffer(USBD_HandleTypeDef* pdev, uint8_t* Buf, uint16_t sz);
uint16_t CDC_GetRxBufferBytesAvailable(USBD_HandleTypeDef* pdev);
void CDC_FlushRxBuffer(USBD_HandleTypeDef* pdev);

extern const USBD_ClassTypeDef USBD_CDC;

#ifdef __cplusplus
}
#endif

#endif  /* __USB_CDC_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
