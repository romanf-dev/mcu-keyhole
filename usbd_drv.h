/**
  ******************************************************************************
  * @file    usbd_drv.h
  * @brief   Interface between USB driver and USB stack.
  ******************************************************************************
  */

#ifndef __USB_DRIVER_H
#define __USB_DRIVER_H

#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

struct _USBD_HandleTypeDef;
struct _PCD_HandleTypeDef;

/* Functions implemented by driver, called by USBD library. */

int USBD_Drv_Init(struct _PCD_HandleTypeDef *hpcd, void* context);
int USBD_Drv_DeInit(struct _PCD_HandleTypeDef *hpcd);
int USBD_Drv_Start(struct _PCD_HandleTypeDef *hpcd);
int USBD_Drv_Stop(struct _PCD_HandleTypeDef *hpcd);
int USBD_Drv_SetAddress(struct _PCD_HandleTypeDef *hpcd, uint8_t address);

int USBD_Drv_EP_SetStall(struct _PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
int USBD_Drv_EP_IsStall(struct _PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
int USBD_Drv_EP_ClrStall(struct _PCD_HandleTypeDef *hpcd, uint8_t ep_addr);

int USBD_Drv_EP_Open(
    struct _PCD_HandleTypeDef *hpcd, 
    uint8_t ep_addr, 
    uint16_t ep_max_packet_sz, 
    uint8_t ep_type
);

int USBD_Drv_EP_Close(struct _PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
int USBD_Drv_EP_Flush(struct _PCD_HandleTypeDef *hpcd, uint8_t ep_addr);

int USBD_Drv_EP_Receive(
    struct _PCD_HandleTypeDef *hpcd, 
    uint8_t ep_addr, 
    uint8_t *pBuf, 
    uint32_t len
);

int USBD_Drv_EP_Transmit(
    struct _PCD_HandleTypeDef *hpcd, 
    uint8_t ep_addr, 
    const uint8_t *pBuf, 
    uint32_t len
);

size_t USBD_Drv_EP_GetRxCount(struct _PCD_HandleTypeDef *hpcd, uint8_t ep_addr);

/* Functions implemented by the library, called by USB driver. */

int USBD_Lib_Reset(struct _USBD_HandleTypeDef  *pdev);
int USBD_Lib_Suspend(struct _USBD_HandleTypeDef  *pdev);
int USBD_Lib_Resume(struct _USBD_HandleTypeDef  *pdev);
int USBD_Lib_SOF(struct _USBD_HandleTypeDef  *pdev);
int USBD_Lib_SetupStage(struct _USBD_HandleTypeDef *pdev, const uint8_t *pdata);

int USBD_Lib_DataOutStage(
    struct _USBD_HandleTypeDef *pdev, 
    uint8_t epnum, 
    uint8_t *pdata
);

int USBD_Lib_DataInStage(
    struct _USBD_HandleTypeDef *pdev, 
    uint8_t epnum, 
    const uint8_t *pdata
);

#ifdef __cplusplus
}
#endif

#endif  /* __USB_CDC_H */
