/**
  ******************************************************************************
  * @file    usbd_cdc.c
  * @author  MCD Application Team
  * @brief   This file provides the high layer firmware functions to manage the
  * following functionalities of the USB CDC Class:
  * - Initialization and Configuration of high and low layer
  * - Enumeration as CDC Device (and enumeration for each implemented 
  *   memory interface)
  * - OUT/IN data transfer
  * - Command IN transfer (class requests management)
  * - Error management
  *
  *  @verbatim
  *
  * ===================================================================
  *  CDC Class Driver Description
  * ===================================================================
  * This driver manages the "Universal Serial Bus Class Definitions for 
  * Communications Devices Revision 1.2 November 16, 2007" and the sub-protocol 
  * specification of "Universal Serial Bus Communications Class Subclass 
  * Specification for PSTN Devices Revision 1.2 February 9, 2007"
  * This driver implements the following aspects of the specification:
  * - Device descriptor management
  * - Configuration descriptor management
  * - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command
  *   endpoint (IN)
  * - Requests management (as described in section 6.2 in specification)
  * - Abstract Control Model compliant
  * - Union Functional collection (using 1 IN endpoint for control)
  * - Data interface class
  *
  * These aspects may be enriched or modified for a specific user application.
  *
  * This driver doesn't implement the following aspects of the specification
  * (but it is possible to manage these features with some modifications on this
  * driver):
  * - Any class-specific aspect relative to communication classes should be 
  *   managed by user application.
  * - All communication classes other than PSTN are not managed
  *
  *  @endverbatim
  *
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

#include <stdalign.h>
#include "usbd_def.h"
#include "usbd_cdc.h"

alignas(sizeof(uint32_t)) static const uint8_t USBD_CDC_DeviceQualifierDesc[] =
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

alignas(sizeof(uint32_t)) uint8_t const 
USBD_CDC_CfgHSDesc[USB_CDC_CONFIG_DESC_SIZ] =
{
    /*Configuration Descriptor*/
    0x09,                           /* bLength: Configuration Descriptor size */
    USB_DESC_TYPE_CONFIGURATION,    /* bDescriptorType: Configuration */
    USB_CDC_CONFIG_DESC_SIZ,        /* wTotalLength:no of returned bytes */
    0x00,
    0x02,   /* bNumInterfaces: 2 interface */
    0x01,   /* bConfigurationValue: Configuration value */
    0x00,   /* iConfiguration: Index of string descriptor describing the cfg */
    0xC0,   /* bmAttributes: self powered */
    0x32,   /* MaxPower 0 mA */

    /*------------------------------------------------------------------------*/

    /*Interface Descriptor */
    0x09,                       /* bLength: Interface Descriptor size */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType: Interface */
    /* Interface descriptor type */
    0x00,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: One endpoints used */
    0x02,   /* bInterfaceClass: Communication Interface Class */
    0x02,   /* bInterfaceSubClass: Abstract Control Model */
    0x01,   /* bInterfaceProtocol: Common AT commands */
    0x00,   /* iInterface: */

    /*Header Functional Descriptor*/
    0x05,   /* bLength: Endpoint Descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: Header Func Desc */
    0x10,   /* bcdCDC: spec release number */
    0x01,

    /*Call Management Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0+D1 */
    0x01,   /* bDataInterface: 1 */

    /*ACM Functional Descriptor*/
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,   /* bmCapabilities */

    /*Union Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x00,   /* bMasterInterface: Communication class interface */
    0x01,   /* bSlaveInterface0: Data Class Interface */

    /*Endpoint 2 Descriptor*/
    0x07,                           /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,         /* bDescriptorType: Endpoint */
    CDC_CMD_EP,                     /* bEndpointAddress */
    0x03,                           /* bmAttributes: Interrupt */
    LOBYTE(CDC_CMD_PACKET_SIZE),    /* wMaxPacketSize: */
    HIBYTE(CDC_CMD_PACKET_SIZE),
    CDC_HS_BINTERVAL,               /* bInterval: */
    /*------------------------------------------------------------------------*/

    /*Data class interface descriptor*/
    0x09,                       /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType: */
    0x01,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: Two endpoints used */
    0x0A,   /* bInterfaceClass: CDC */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */

    /*Endpoint OUT Descriptor*/
    0x07,                               /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,             /* bDescriptorType: Endpoint */
    CDC_OUT_EP,                         /* bEndpointAddress */
    0x02,                               /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),/* wMaxPacketSize: */
    HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
    0x00,                               /* bInterval ignore for Bulk transfer */

    /*Endpoint IN Descriptor*/
    0x07,                               /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,             /* bDescriptorType: Endpoint */
    CDC_IN_EP,                          /* bEndpointAddress */
    0x02,                               /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),/* wMaxPacketSize: */
    HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
    0x00                                /* bInterval ignore for Bulk transfer */
};

alignas(sizeof(uint32_t)) const uint8_t 
USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZ] =
{
    /*Configuration Descriptor*/
    0x09,                           /* bLength: Configuration Descriptor size */
    USB_DESC_TYPE_CONFIGURATION,    /* bDescriptorType: Configuration */
    USB_CDC_CONFIG_DESC_SIZ,        /* wTotalLength:no of returned bytes */
    0x00,
    0x02,   /* bNumInterfaces: 2 interface */
    0x01,   /* bConfigurationValue: Configuration value */
    0x00,   /* iConfiguration: Index of string descriptor describing the cfg */
    0xC0,   /* bmAttributes: self powered */
    0x32,   /* MaxPower 0 mA */

    /*------------------------------------------------------------------------*/

    /*Interface Descriptor */
    0x09,                       /* bLength: Interface Descriptor size */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType: Interface */

    /* Interface descriptor type */
    0x00,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: One endpoints used */
    0x02,   /* bInterfaceClass: Communication Interface Class */
    0x02,   /* bInterfaceSubClass: Abstract Control Model */
    0x01,   /* bInterfaceProtocol: Common AT commands */
    0x00,   /* iInterface: */

    /*Header Functional Descriptor*/
    0x05,   /* bLength: Endpoint Descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: Header Func Desc */
    0x10,   /* bcdCDC: spec release number */
    0x01,

    /*Call Management Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0+D1 */
    0x01,   /* bDataInterface: 1 */

    /*ACM Functional Descriptor*/
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,   /* bmCapabilities */

    /*Union Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x00,   /* bMasterInterface: Communication class interface */
    0x01,   /* bSlaveInterface0: Data Class Interface */

    /*Endpoint 2 Descriptor*/
    0x07,                           /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,         /* bDescriptorType: Endpoint */
    CDC_CMD_EP,                     /* bEndpointAddress */
    0x03,                           /* bmAttributes: Interrupt */
    LOBYTE(CDC_CMD_PACKET_SIZE),    /* wMaxPacketSize: */
    HIBYTE(CDC_CMD_PACKET_SIZE),
    CDC_FS_BINTERVAL,               /* bInterval: */

    /*------------------------------------------------------------------------*/

    /*Data class interface descriptor*/
    0x09,                       /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType: */
    0x01,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: Two endpoints used */
    0x0A,   /* bInterfaceClass: CDC */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */

    /*Endpoint OUT Descriptor*/
    0x07,                               /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,             /* bDescriptorType: Endpoint */
    CDC_OUT_EP,                         /* bEndpointAddress */
    0x02,                               /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),/* wMaxPacketSize: */
    HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
    0x00,                               /* bInterval ignore for Bulk transfer */

    /*Endpoint IN Descriptor*/
    0x07,                               /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,             /* bDescriptorType: Endpoint */
    CDC_IN_EP,                          /* bEndpointAddress */
    0x02,                               /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),/* wMaxPacketSize: */
    HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
    0x00                                /* bInterval ignore for Bulk transfer */
};

alignas(sizeof(uint32_t)) uint8_t const 
USBD_CDC_OtherSpeedCfgDesc[USB_CDC_CONFIG_DESC_SIZ] =
{
    0x09,   /* bLength: Configuation Descriptor size */
    USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION,
    USB_CDC_CONFIG_DESC_SIZ,
    0x00,
    0x02,   /* bNumInterfaces: 2 interfaces */
    0x01,   /* bConfigurationValue: */
    0x04,   /* iConfiguration: */
    0xC0,   /* bmAttributes: */
    0x32,   /* MaxPower 100 mA */

    /*Interface Descriptor */
    0x09,                       /* bLength: Interface Descriptor size */
    USB_DESC_TYPE_INTERFACE,    /* bDescriptorType: Interface */
    /* Interface descriptor type */
    0x00,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x01,   /* bNumEndpoints: One endpoints used */
    0x02,   /* bInterfaceClass: Communication Interface Class */
    0x02,   /* bInterfaceSubClass: Abstract Control Model */
    0x01,   /* bInterfaceProtocol: Common AT commands */
    0x00,   /* iInterface: */

    /*Header Functional Descriptor*/
    0x05,   /* bLength: Endpoint Descriptor size */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x00,   /* bDescriptorSubtype: Header Func Desc */
    0x10,   /* bcdCDC: spec release number */
    0x01,

    /*Call Management Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0+D1 */
    0x01,   /* bDataInterface: 1 */

    /*ACM Functional Descriptor*/
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,   /* bmCapabilities */

    /*Union Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x00,   /* bMasterInterface: Communication class interface */
    0x01,   /* bSlaveInterface0: Data Class Interface */

    /*Endpoint 2 Descriptor*/
    0x07,                           /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,         /* bDescriptorType: Endpoint */
    CDC_CMD_EP,                     /* bEndpointAddress */
    0x03,                           /* bmAttributes: Interrupt */
    LOBYTE(CDC_CMD_PACKET_SIZE),    /* wMaxPacketSize: */
    HIBYTE(CDC_CMD_PACKET_SIZE),
    CDC_FS_BINTERVAL,               /* bInterval: */

    /*------------------------------------------------------------------------*/

    /*Data class interface descriptor*/
    0x09,                           /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_INTERFACE,        /* bDescriptorType: */
    0x01,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: Two endpoints used */
    0x0A,   /* bInterfaceClass: CDC */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */

    /*Endpoint OUT Descriptor*/
    0x07,                           /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,         /* bDescriptorType: Endpoint */
    CDC_OUT_EP,                     /* bEndpointAddress */
    0x02,                           /* bmAttributes: Bulk */
    0x40,                           /* wMaxPacketSize: */
    0x00,
    0x00,                           /* bInterval: ignore for Bulk transfer */

    /*Endpoint IN Descriptor*/
    0x07,                           /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,         /* bDescriptorType: Endpoint */
    CDC_IN_EP,                      /* bEndpointAddress */
    0x02,                           /* bmAttributes: Bulk */
    0x40,                           /* wMaxPacketSize: */
    0x00,
    0x00                            /* bInterval */
};

typedef struct
{
    uint32_t data[CDC_DATA_HS_MAX_PACKET_SIZE / 4U];  /* Force 32bits align */
    uint8_t CmdOpCode;
    uint8_t CmdLength;
    uint32_t TxState;
    uint8_t lcBuffer[7]; // Line coding buffer
    uint8_t UserRxBuffer[APP_RX_DATA_SIZE];
    volatile uint8_t rxBuffer[HL_RX_BUFFER_SIZE]; // Receive buffer
    volatile uint16_t rxBufferHeadPos; // Receive buffer write position
    volatile uint16_t rxBufferTailPos; // Receive buffer read position
}
USBD_CDC_HandleTypeDef;

static USBD_CDC_HandleTypeDef USBD_CDC_Data;

/******************************************************************************/
/* Line Coding Structure                                                      */
/*----------------------------------------------------------------------------*/
/* Offset | Field       | Size | Value  | Description                         */
/* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per sec  */
/* 4      | bCharFormat |   1  | Number | Stop bits                           */
/*                                        0 - 1 Stop bit                      */
/*                                        1 - 1.5 Stop bits                   */
/*                                        2 - 2 Stop bits                     */
/* 5      | bParityType |  1   | Number | Parity                              */
/*                                        0 - None                            */
/*                                        1 - Odd                             */
/*                                        2 - Even                            */
/*                                        3 - Mark                            */
/*                                        4 - Space                           */
/* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).         */
/******************************************************************************/
/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK
  */
static void CDC_Control(
    USBD_HandleTypeDef *pdev, 
    uint8_t cmd, 
    uint8_t* pbuf, 
    uint16_t length)
{
    USBD_CDC_HandleTypeDef *hcdc = pdev->pClassData;

    switch(cmd)
    {
        case CDC_SEND_ENCAPSULATED_COMMAND: break;
        case CDC_GET_ENCAPSULATED_RESPONSE: break;
        case CDC_SET_COMM_FEATURE: break;
        case CDC_GET_COMM_FEATURE: break;
        case CDC_CLEAR_COMM_FEATURE: break;
        case CDC_SET_CONTROL_LINE_STATE: break;
        case CDC_SEND_BREAK: break;

        case CDC_SET_LINE_CODING:
            hcdc->lcBuffer[0] = pbuf[0];
            hcdc->lcBuffer[1] = pbuf[1];
            hcdc->lcBuffer[2] = pbuf[2];
            hcdc->lcBuffer[3] = pbuf[3];
            hcdc->lcBuffer[4] = pbuf[4];
            hcdc->lcBuffer[5] = pbuf[5];
            hcdc->lcBuffer[6] = pbuf[6];
        break;

        case CDC_GET_LINE_CODING:
            pbuf[0] = hcdc->lcBuffer[0];
            pbuf[1] = hcdc->lcBuffer[1];
            pbuf[2] = hcdc->lcBuffer[2];
            pbuf[3] = hcdc->lcBuffer[3];
            pbuf[4] = hcdc->lcBuffer[4];
            pbuf[5] = hcdc->lcBuffer[5];
            pbuf[6] = hcdc->lcBuffer[6];

            CDC_FlushRxBuffer(pdev);

        break;

        default: break;
    }
}

/**
  * @brief  USBD_CDC_Init
  *         Initialize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    uint8_t ret = 0U;
    USBD_CDC_HandleTypeDef *hcdc;
    size_t in_packet_size = 0;
    size_t out_packet_size = 0;

    if (pdev->dev_speed == USBD_SPEED_HIGH)
    {   
        in_packet_size = CDC_DATA_HS_IN_PACKET_SIZE;
        out_packet_size = CDC_DATA_HS_OUT_PACKET_SIZE;
    }
    else
    {
        in_packet_size = CDC_DATA_FS_IN_PACKET_SIZE;
        out_packet_size = CDC_DATA_FS_OUT_PACKET_SIZE;        
    }

    USBD_EP_Open(pdev, CDC_IN_EP, in_packet_size, USBD_EP_TYPE_BULK);
    USBD_EP_Open(pdev, CDC_OUT_EP, out_packet_size, USBD_EP_TYPE_BULK);
    USBD_EP_Open(pdev, CDC_CMD_EP, CDC_CMD_PACKET_SIZE, USBD_EP_TYPE_INTR);

    pdev->pClassData = &USBD_CDC_Data;
    hcdc = (USBD_CDC_HandleTypeDef *) pdev->pClassData;

    // https://stackoverflow.com/a/26925578
    const uint32_t baudrate = 9600;
    hcdc->lcBuffer[0] = (uint8_t)(baudrate);
    hcdc->lcBuffer[1] = (uint8_t)(baudrate >> 8);
    hcdc->lcBuffer[2] = (uint8_t)(baudrate >> 16);
    hcdc->lcBuffer[3] = (uint8_t)(baudrate >> 24);
    hcdc->lcBuffer[4] = 0; // 1 Stop bit
    hcdc->lcBuffer[5] = 0; // No parity
    hcdc->lcBuffer[6] = 8; // 8 data bits

    hcdc->TxState = 0U;

    /* Prepare Out endpoint to receive next packet */
    USBD_EP_Receive(pdev, CDC_OUT_EP, hcdc->UserRxBuffer, out_packet_size);
    return ret;
}

/**
  * @brief  USBD_CDC_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    USBD_EP_Close(pdev, CDC_IN_EP);
    USBD_EP_Close(pdev, CDC_OUT_EP);
    USBD_EP_Close(pdev, CDC_CMD_EP);

    if (pdev->pClassData != NULL)
    {
        pdev->pClassData = NULL;
    }

    return 0;
}

/**
  * @brief  USBD_CDC_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t 
USBD_CDC_Setup(USBD_HandleTypeDef *pdev, const USBD_SetupReqTypedef *req)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *) pdev->pClassData;
    const uint16_t zeros = 0U;
    uint8_t ret = USBD_OK;
    int replysize = -1;

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_CLASS:
        if (req->wLength)
        {
            if (req->bmRequest & 0x80U)
            {
                CDC_Control(pdev, req->bRequest, (void*) hcdc->data, req->wLength);
                USBD_CtlSendData(pdev, (void*) hcdc->data, req->wLength);
            }
            else
            {
                hcdc->CmdOpCode = req->bRequest;
                hcdc->CmdLength = (uint8_t)req->wLength;
                USBD_CtlPrepareRx(pdev, (void*) hcdc->data, req->wLength);
            }
        }
        else
        {
            CDC_Control(pdev, req->bRequest, (void *)req, 0U);
        }
        replysize = 0;
        break;

    case USB_REQ_TYPE_STANDARD:
        if (pdev->dev_state == USBD_STATE_CONFIGURED)
        {
            switch (req->bRequest)
            {
                case USB_REQ_GET_STATUS: replysize = 2; break;
                case USB_REQ_GET_INTERFACE: replysize = 1; break;
                case USB_REQ_SET_INTERFACE: replysize = 0; break;
                default: break;
            }
        }
        break;

    default: break;
    }

    if (replysize != 0)
    {
        if (replysize > 0)
        {
            ret = USBD_CtlSendData(pdev, (const uint8_t*) &zeros, replysize);
        }
        else
        {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;            
        }
    }

    return ret;
}

/**
  * @brief  USBD_CDC_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

    /*&& (pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0*/

    if ((pdev->ep_in[epnum].total_length > 0))
    {
        pdev->ep_in[epnum].total_length = 0;

        /* Send ZLP */
        USBD_EP_Transmit(pdev, epnum, NULL, 0);
    }
    else
    {
        hcdc->TxState = 0U;
    }

    return USBD_OK;
}

/**
  * @brief  USBD_CDC_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *) pdev->pClassData;
    const uint16_t len = USBD_GetRxCount(pdev, epnum);
    uint16_t tempHeadPos = hcdc->rxBufferHeadPos;
    const uint32_t max_packet_size = (pdev->dev_speed == USBD_SPEED_HIGH) ?
        CDC_DATA_HS_OUT_PACKET_SIZE :
        CDC_DATA_FS_OUT_PACKET_SIZE;

    /* USB data will be immediately processed, this allow next USB traffic being
    NAKed till the end of the application Xfer */

    for (uint32_t i = 0; i < len; i++) 
    {
        hcdc->rxBuffer[tempHeadPos] = hcdc->UserRxBuffer[i];
        tempHeadPos = (uint16_t)(
            (uint16_t)(tempHeadPos + 1) % HL_RX_BUFFER_SIZE
        );

        if (tempHeadPos == hcdc->rxBufferTailPos) 
        {
            return USBD_FAIL;
        }
    }

    hcdc->rxBufferHeadPos = tempHeadPos;
    USBD_EP_Receive(pdev, CDC_OUT_EP, hcdc->UserRxBuffer, max_packet_size);
    return USBD_OK;
}

/**
  * @brief  USBD_CDC_EP0_RxReady
  *         Handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *) pdev->pClassData;

    if (hcdc->CmdOpCode != 0xFFU)
    {
        CDC_Control(pdev, hcdc->CmdOpCode, (void *)hcdc->data, hcdc->CmdLength);
        hcdc->CmdOpCode = 0xFFU;
    }

    return USBD_OK;
}

/**
  * @brief  CDC_Transmit
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit(USBD_HandleTypeDef* pdev, const uint8_t* Buf, uint16_t Len)
{
    USBD_CDC_HandleTypeDef *hcdc = pdev->pClassData;
    uint8_t status = USBD_BUSY;

    if (hcdc->TxState == 0)
    {
        hcdc->TxState = 1U;
        pdev->ep_in[CDC_IN_EP & 0xFU].total_length = Len;
        USBD_EP_Transmit(pdev, CDC_IN_EP, Buf, Len);
        status = USBD_OK;        
    }

    return status;
}

uint8_t CDC_ReadRxBuffer(USBD_HandleTypeDef* pdev, uint8_t* Buf, uint16_t Len) 
{
	uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable(pdev);
    USBD_CDC_HandleTypeDef *hcdc = pdev->pClassData;
    uint8_t status = USB_CDC_RX_BUFFER_NO_DATA;

	if (bytesAvailable >= Len)
    {
        for (uint8_t i = 0; i < Len; i++) 
        {
            Buf[i] = hcdc->rxBuffer[hcdc->rxBufferTailPos];
            hcdc->rxBufferTailPos = (uint16_t)(
                (uint16_t)(hcdc->rxBufferTailPos + 1) % HL_RX_BUFFER_SIZE
            );
        }

        status = USB_CDC_RX_BUFFER_OK;
    }

	return status;
}

uint16_t CDC_GetRxBufferBytesAvailable(USBD_HandleTypeDef* pdev) 
{
    USBD_CDC_HandleTypeDef *hcdc = pdev->pClassData;
    return (uint16_t)
        (hcdc->rxBufferHeadPos - hcdc->rxBufferTailPos) % HL_RX_BUFFER_SIZE;
}

void CDC_FlushRxBuffer(USBD_HandleTypeDef* pdev) 
{
    USBD_CDC_HandleTypeDef *hcdc = pdev->pClassData;
    hcdc->rxBufferHeadPos = 0;
    hcdc->rxBufferTailPos = 0;
}

const USBD_ClassTypeDef USBD_CDC =
{
    USBD_CDC_Init,
    USBD_CDC_DeInit,
    USBD_CDC_Setup,
    NULL,
    USBD_CDC_EP0_RxReady,
    USBD_CDC_DataIn,
    USBD_CDC_DataOut,
    NULL,
    NULL,
    NULL,
    { USBD_CDC_CfgHSDesc, sizeof(USBD_CDC_CfgHSDesc) },
    { USBD_CDC_CfgFSDesc, sizeof(USBD_CDC_CfgFSDesc) },
    { USBD_CDC_OtherSpeedCfgDesc, sizeof(USBD_CDC_OtherSpeedCfgDesc) },
    { USBD_CDC_DeviceQualifierDesc, sizeof(USBD_CDC_DeviceQualifierDesc) },
};
