/**
  ******************************************************************************
  * @file    usbd_def.h
  * @author  MCD Application Team
  * @brief   General defines for the usb device library
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_DEF_H
#define __USBD_DEF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "usbd_drv.h"

#define USBD_SELF_POWERED                               1
#ifndef USBD_LPM_ENABLED
#define USBD_LPM_ENABLED                                0U
#endif /* USBD_LPM_ENABLED */

enum
{
    USB_LEN_DEV_QUALIFIER_DESC = 0x0AU,
    USB_LEN_DEV_DESC = 0x12U,
    USB_LEN_CFG_DESC = 0x09U,
    USB_LEN_IF_DESC = 0x09U,
    USB_LEN_EP_DESC = 0x07U,
    USB_LEN_OTG_DESC = 0x03U,
    USB_LEN_LANGID_STR_DESC = 0x04U,
    USB_LEN_OTHER_SPEED_DESC_SIZ = 0x09U,

    USBD_IDX_LANGID_STR = 0x00U,
    USBD_IDX_MFC_STR = 0x01U,
    USBD_IDX_PRODUCT_STR = 0x02U,
    USBD_IDX_SERIAL_STR = 0x03U,
    USBD_IDX_CONFIG_STR = 0x04U,
    USBD_IDX_INTERFACE_STR = 0x05U,

    USB_REQ_TYPE_STANDARD = 0x00U,
    USB_REQ_TYPE_CLASS = 0x20U,
    USB_REQ_TYPE_VENDOR = 0x40U,
    USB_REQ_TYPE_MASK = 0x60U,

    USB_REQ_RECIPIENT_DEVICE = 0x00U,
    USB_REQ_RECIPIENT_INTERFACE = 0x01U,
    USB_REQ_RECIPIENT_ENDPOINT = 0x02U,
    USB_REQ_RECIPIENT_MASK = 0x03U,

    USB_REQ_GET_STATUS = 0x00U,
    USB_REQ_CLEAR_FEATURE = 0x01U,
    USB_REQ_SET_FEATURE = 0x03U,
    USB_REQ_SET_ADDRESS = 0x05U,
    USB_REQ_GET_DESCRIPTOR = 0x06U,
    USB_REQ_SET_DESCRIPTOR = 0x07U,
    USB_REQ_GET_CONFIGURATION = 0x08U,
    USB_REQ_SET_CONFIGURATION = 0x09U,
    USB_REQ_GET_INTERFACE = 0x0AU,
    USB_REQ_SET_INTERFACE = 0x0BU,
    USB_REQ_SYNCH_FRAME = 0x0CU,

    USB_DESC_TYPE_DEVICE = 0x01U,
    USB_DESC_TYPE_CONFIGURATION = 0x02U,
    USB_DESC_TYPE_STRING = 0x03U,
    USB_DESC_TYPE_INTERFACE = 0x04U,
    USB_DESC_TYPE_ENDPOINT = 0x05U,
    USB_DESC_TYPE_DEVICE_QUALIFIER = 0x06U,
    USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION = 0x07U,
    USB_DESC_TYPE_BOS = 0x0FU,

    USB_CONFIG_REMOTE_WAKEUP = 0x02U,
    USB_CONFIG_SELF_POWERED = 0x01U,

    USB_FEATURE_EP_HALT = 0x00U,
    USB_FEATURE_REMOTE_WAKEUP = 0x01U,
    USB_FEATURE_TEST_MODE = 0x02U,

    USB_DEVICE_CAPABITY_TYPE = 0x10U,

    USB_HS_MAX_PACKET_SIZE = 512U,
    USB_FS_MAX_PACKET_SIZE = 64U,
    USB_MAX_EP0_SIZE = 64U,

    /* Device Status */
    USBD_STATE_DEFAULT = 0x01U,
    USBD_STATE_ADDRESSED = 0x02U,
    USBD_STATE_CONFIGURED = 0x03U,
    USBD_STATE_SUSPENDED = 0x04U,

    /* EP0 State*/
    USBD_EP0_IDLE = 0x00U,
    USBD_EP0_SETUP = 0x01U,
    USBD_EP0_DATA_IN = 0x02U,
    USBD_EP0_DATA_OUT = 0x03U,
    USBD_EP0_STATUS_IN = 0x04U,
    USBD_EP0_STATUS_OUT = 0x05U,
    USBD_EP0_STALL = 0x06U,

    USBD_EP_TYPE_CTRL = 0x00U,
    USBD_EP_TYPE_ISOC = 0x01U,
    USBD_EP_TYPE_BULK = 0x02U,
    USBD_EP_TYPE_INTR = 0x03U,

    USBD_OK   = 0U,
    USBD_BUSY = EBUSY,
    USBD_FAIL = EINVAL,  
};

typedef struct usb_setup_req
{
    uint8_t   bmRequest;
    uint8_t   bRequest;
    uint16_t  wValue;
    uint16_t  wIndex;
    uint16_t  wLength;
} 
USBD_SetupReqTypedef;

struct _USBD_HandleTypeDef;

typedef struct 
{
    const void* ptr;
    size_t size;
} 
USBD_BufferTypeDef;

typedef struct _Device_cb
{
    uint8_t (*Init)(struct _USBD_HandleTypeDef *pdev, uint8_t cfgidx);
    uint8_t (*DeInit)(struct _USBD_HandleTypeDef *pdev, uint8_t cfgidx);
    /* Control Endpoints*/
    uint8_t (*Setup)(
        struct _USBD_HandleTypeDef *pdev, const USBD_SetupReqTypedef *req
    );
    uint8_t (*EP0_TxSent)(struct _USBD_HandleTypeDef *pdev);
    uint8_t (*EP0_RxReady)(struct _USBD_HandleTypeDef *pdev);
    /* Class Specific Endpoints*/
    uint8_t (*DataIn)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
    uint8_t (*DataOut)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
    uint8_t (*SOF)(struct _USBD_HandleTypeDef *pdev);
    uint8_t (*IsoINIncomplete)(struct _USBD_HandleTypeDef *pdev, uint8_t epnum);
    uint8_t (*IsoOUTIncomplete)(struct _USBD_HandleTypeDef *pdev, uint8_t epn);

    USBD_BufferTypeDef HSConfigDescriptor;
    USBD_BufferTypeDef FSConfigDescriptor;
    USBD_BufferTypeDef OtherSpeedConfigDescriptor;
    USBD_BufferTypeDef DeviceQualifierDescriptor;
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)  
    USBD_BufferTypeDef UsrStrDescriptor;
#endif

} USBD_ClassTypeDef;

/* Following USB Device Speed */
typedef enum
{
    USBD_SPEED_HIGH  = 0U,
    USBD_SPEED_FULL  = 1U,
    USBD_SPEED_LOW   = 2U,
} USBD_SpeedTypeDef;

typedef int USBD_StatusTypeDef;

/* USB Device descriptors structure */
typedef struct
{
    USBD_BufferTypeDef DeviceDescriptor;
    USBD_BufferTypeDef LangIDStrDescriptor;
    USBD_BufferTypeDef ManufacturerStrDescriptor;
    USBD_BufferTypeDef ProductStrDescriptor;
    USBD_BufferTypeDef SerialStrDescriptor;
    USBD_BufferTypeDef ConfigurationStrDescriptor;
    USBD_BufferTypeDef InterfaceStrDescriptor;
#if (USBD_LPM_ENABLED == 1U)
    USBD_BufferTypeDef BOSDescriptor;
#endif
} 
USBD_DescriptorsTypeDef;

/* USB Device handle structure */
typedef struct
{
    uint32_t                status;
    uint32_t                is_used;
    uint32_t                total_length;
    uint32_t                rem_length;
    uint32_t                maxpacket;
} 
USBD_EndpointTypeDef;

/* USB Device handle structure */
typedef struct _USBD_HandleTypeDef
{
    uint32_t                dev_config;
    uint32_t                dev_default_config;
    uint32_t                dev_config_status;
    USBD_SpeedTypeDef       dev_speed;
    USBD_EndpointTypeDef    ep_in[16];
    USBD_EndpointTypeDef    ep_out[16];
    uint32_t                ep0_state;
    uint32_t                ep0_data_len;
    uint8_t                 dev_state;
    uint8_t                 dev_old_state;
    uint8_t                 dev_address;
    uint32_t                dev_remote_wakeup;
    USBD_SetupReqTypedef    request;
    const USBD_DescriptorsTypeDef *pDesc;
    const USBD_ClassTypeDef *pClass;
    void                    *pClassData;
    struct _PCD_HandleTypeDef *pData;
} 
USBD_HandleTypeDef;

#define SWAPBYTE(addr) (((uint16_t)(*((uint8_t *)(addr)))) + \
    (((uint16_t)(*(((uint8_t *)(addr)) + 1U))) << 8U))

#define LOBYTE(x)  ((uint8_t)((x) & 0x00FFU))
#define HIBYTE(x)  ((uint8_t)(((x) & 0xFF00U) >> 8U))
#define MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define MAX(a, b)  (((a) > (b)) ? (a) : (b))

USBD_StatusTypeDef  USBD_CtlSendData(
    USBD_HandleTypeDef *pdev,
    const uint8_t *pbuf,
    uint16_t len
);

USBD_StatusTypeDef USBD_CtlPrepareRx(
    USBD_HandleTypeDef  *pdev,
    uint8_t *pbuf,
    uint16_t len
);

void USBD_CtlError(USBD_HandleTypeDef  *pdev, const USBD_SetupReqTypedef *req);

/**
* @brief  USBD_Init
*         Initializes the device stack
* @param  pdev: device instance
* @param  pdev: driver instance
* @param  pdesc: Descriptor structure address
* @param  id: Low level core index
* @param  pclass: class descriptor
* @retval status
*/
static inline USBD_StatusTypeDef USBD_Init(
    USBD_HandleTypeDef *pdev, 
    struct _PCD_HandleTypeDef* pdrv, 
    const USBD_DescriptorsTypeDef *pdesc, 
    const USBD_ClassTypeDef *pclass)
{
    pdev->pDesc = pdesc;
    pdev->pClass = pclass;
    pdev->dev_state = USBD_STATE_DEFAULT;
    pdev->pData = pdrv;
    return (USBD_Drv_Init(pdrv, pdev) != 0) ? USBD_FAIL : USBD_OK;
}

/**
* @brief  USBD_DeInit
*         Re-Initialize th device library
* @param  pdev: device instance
* @retval status: status
*/
static inline USBD_StatusTypeDef USBD_DeInit(USBD_HandleTypeDef *pdev)
{
    pdev->dev_state = USBD_STATE_DEFAULT;
    pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
    USBD_Drv_Stop(pdev->pData);
    USBD_Drv_DeInit(pdev->pData);
    return USBD_OK;
}

/**
  * @brief  USBD_Start
  *         Start the USB Device Core.
  * @param  pdev: Device Handle
  * @retval USBD Status
  */
static inline USBD_StatusTypeDef USBD_Start(USBD_HandleTypeDef *pdev)
{
    USBD_Drv_Start(pdev->pData);
    return USBD_OK;
}

/**
  * @brief  USBD_Stop
  *         Stop the USB Device Core.
  * @param  pdev: Device Handle
  * @retval USBD Status
  */
static inline USBD_StatusTypeDef USBD_Stop(USBD_HandleTypeDef *pdev)
{
    pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
    USBD_Drv_Stop(pdev->pData);
    return USBD_OK;
}

static inline int USBD_EP_Open(
    USBD_HandleTypeDef *pdev, 
    uint8_t ep_addr, 
    uint16_t ep_max_packet_sz, 
    uint8_t ep_type)
{
    pdev->ep_in[ep_addr & 0xFU].is_used = 1U;
    return USBD_Drv_EP_Open(pdev->pData, ep_addr, ep_max_packet_sz, ep_type);
}

static inline int USBD_EP_Close(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    pdev->ep_in[ep_addr & 0xFU].is_used = 0U;
    return USBD_Drv_EP_Close(pdev->pData, ep_addr);
}

static inline int USBD_EP_Transmit(
    USBD_HandleTypeDef *pdev, 
    uint8_t ep_addr, 
    const uint8_t *buf, 
    uint32_t len)
{
    return USBD_Drv_EP_Transmit(pdev->pData, ep_addr, buf, len);
}

static inline int USBD_EP_Receive(
    USBD_HandleTypeDef *pdev, 
    uint8_t ep_addr, 
    uint8_t *buf, 
    uint32_t len)
{
    return USBD_Drv_EP_Receive(pdev->pData, ep_addr, buf, len);
}

/**
* @brief  USBD_GetRxCount
*         returns the received data length
* @param  pdev: device instance
* @param  ep_addr: endpoint address
* @retval Rx Data blength
*/
static inline size_t USBD_GetRxCount(USBD_HandleTypeDef *pdev, uint8_t ep_addr)
{
    return USBD_Drv_EP_GetRxCount(pdev->pData, ep_addr);
}

#ifdef __cplusplus
}
#endif

#endif /* __USBD_DEF_H */
