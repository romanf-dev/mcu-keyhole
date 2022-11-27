/**
  ******************************************************************************
  * @file    usbd_req.c
  * @author  MCD Application Team
  * @brief   This file provides the standard USB requests following chapter 9.
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

#include "usbd_def.h"

/**
* @brief  USBD_CtlSendData
*         send data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be sent
* @retval status
*/
USBD_StatusTypeDef 
USBD_CtlSendData(USBD_HandleTypeDef *pdev, const uint8_t *pbuf, uint16_t len)
{
    pdev->ep0_state = USBD_EP0_DATA_IN;
    pdev->ep_in[0].total_length = len;
    pdev->ep_in[0].rem_length   = len;
    USBD_Drv_EP_Transmit(pdev->pData, 0x00U, pbuf, len);
    return USBD_OK;
}

/**
* @brief  USBD_CtlPrepareRx
*         receive data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be received
* @retval status
*/
USBD_StatusTypeDef 
USBD_CtlPrepareRx(USBD_HandleTypeDef *pdev, uint8_t *pbuf, uint16_t len)
{
    pdev->ep0_state = USBD_EP0_DATA_OUT;
    pdev->ep_out[0].total_length = len;
    pdev->ep_out[0].rem_length   = len;
    USBD_Drv_EP_Receive(pdev->pData, 0U, pbuf, len);
    return USBD_OK;
}

/**
* @brief  USBD_CtlSendStatus
*         send zero length packet on the ctl pipe
* @param  pdev: device instance
* @retval status
*/
static USBD_StatusTypeDef USBD_CtlSendStatus(USBD_HandleTypeDef *pdev)
{
    pdev->ep0_state = USBD_EP0_STATUS_IN;
    USBD_Drv_EP_Transmit(pdev->pData, 0x00U, NULL, 0U);
    return USBD_OK;
}

/**
* @brief  USBD_CtlReceiveStatus
*         receive zero length packet on the ctl pipe
* @param  pdev: device instance
* @retval status
*/
static USBD_StatusTypeDef USBD_CtlReceiveStatus(USBD_HandleTypeDef *pdev)
{
    pdev->ep0_state = USBD_EP0_STATUS_OUT;
    USBD_Drv_EP_Receive(pdev->pData, 0U, NULL, 0U);
    return USBD_OK;
}

/**
* @brief  USBD_GetDescriptor
*         Handle Get Descriptor requests
* @param  pdev: device instance
* @param  req: usb request
*/
static void 
USBD_GetDescriptor(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    const USBD_BufferTypeDef* descr = NULL;
    const uint8_t id = (uint8_t)(req->wValue);
    const USBD_BufferTypeDef* const string_descr[] =
    {
        &pdev->pDesc->LangIDStrDescriptor,
        &pdev->pDesc->ManufacturerStrDescriptor,
        &pdev->pDesc->ProductStrDescriptor,
        &pdev->pDesc->SerialStrDescriptor,
        &pdev->pDesc->ConfigurationStrDescriptor,
        &pdev->pDesc->InterfaceStrDescriptor
    };

    switch (req->wValue >> 8)
    {
    case USB_DESC_TYPE_DEVICE:
        descr = &pdev->pDesc->DeviceDescriptor;
        break;

    case USB_DESC_TYPE_CONFIGURATION:
        descr = pdev->dev_speed == USBD_SPEED_HIGH ?
            &pdev->pClass->HSConfigDescriptor :
            &pdev->pClass->FSConfigDescriptor;
        break;

    case USB_DESC_TYPE_STRING:
        if (id < (sizeof(string_descr) / sizeof(string_descr[0])))
        {
            descr = string_descr[id];
        }
        break;

    case USB_DESC_TYPE_DEVICE_QUALIFIER:
        descr = &pdev->pClass->DeviceQualifierDescriptor;
        break;

    case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
        descr = &pdev->pClass->OtherSpeedConfigDescriptor;
        break;

    default: break;
    }

    if (descr != NULL)
    {
        if ((descr->size != 0U) && (req->wLength != 0U))
        {
            USBD_CtlSendData(pdev, descr->ptr, MIN(descr->size, req->wLength));
        }

        if (req->wLength == 0U)
        {
            USBD_CtlSendStatus(pdev);
        }
    }
    else
    {
        USBD_CtlError(pdev, req);
    }
}

/**
* @brief  USBD_SetAddress
*         Set device address
* @param  pdev: device instance
* @param  req: usb request
*/
static void USBD_SetAddress(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    if ((req->wIndex == 0U) && (req->wLength == 0U) && (req->wValue < 128U))
    {
        const uint8_t dev_addr = (uint8_t)(req->wValue) & 0x7FU;

        if (pdev->dev_state == USBD_STATE_CONFIGURED)
        {
            USBD_CtlError(pdev, req);
        }
        else
        {
            pdev->dev_address = dev_addr;
            USBD_Drv_SetAddress(pdev->pData, dev_addr);
            USBD_CtlSendStatus(pdev);
            pdev->dev_state = (dev_addr != 0U) ? 
                USBD_STATE_ADDRESSED :
                USBD_STATE_DEFAULT;
        }
    }
    else
    {
        USBD_CtlError(pdev, req);
    }
}

/**
* @brief  USBD_SetConfig
*         Handle Set device configuration request
* @param  pdev: device instance
* @param  req: usb request
*/
static void USBD_SetConfig(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    const uint8_t cfgidx = (uint8_t)(req->wValue);
    const uint8_t* const devdescr = pdev->pDesc->DeviceDescriptor.ptr;

    if (cfgidx <= devdescr[17])
    {
        switch (pdev->dev_state)
        {
        case USBD_STATE_ADDRESSED:
            if (cfgidx)
            {
                if (pdev->pClass->Init(pdev, cfgidx) != 0U)
                {
                    USBD_CtlError(pdev, req);
                    return;
                }

                pdev->dev_config = cfgidx;
                pdev->dev_state = USBD_STATE_CONFIGURED;
                USBD_CtlSendStatus(pdev);
            }
            else
            {
                USBD_CtlSendStatus(pdev);
            }
            break;

        case USBD_STATE_CONFIGURED:
            if (cfgidx == 0U)
            {
                pdev->dev_state = USBD_STATE_ADDRESSED;
                pdev->dev_config = cfgidx;
                pdev->pClass->DeInit(pdev, cfgidx);
                USBD_CtlSendStatus(pdev);
            }
            else if (cfgidx != pdev->dev_config)
            {
                /* Clear old configuration */
                pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);

                /* set new configuration */
                pdev->dev_config = cfgidx;
                if (pdev->pClass->Init(pdev, cfgidx) != 0U)
                {
                    USBD_CtlError(pdev, req);
                    return;
                }

                USBD_CtlSendStatus(pdev);
            }
            else
            {
                USBD_CtlSendStatus(pdev);
            }
            break;

        default:
            USBD_CtlError(pdev, req);
            pdev->pClass->DeInit(pdev, cfgidx);
            break;
        }
    }
    else
    {
        USBD_CtlError(pdev, req);
    }
}

/**
* @brief  USBD_GetConfig
*         Handle Get device configuration request
* @param  pdev: device instance
* @param  req: usb request
*/
static void USBD_GetConfig(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    if (req->wLength != 1U)
    {
        USBD_CtlError(pdev, req);
    }
    else
    {
        switch (pdev->dev_state)
        {
        case USBD_STATE_DEFAULT:
        case USBD_STATE_ADDRESSED:
            pdev->dev_default_config = 0U;
            USBD_CtlSendData(pdev, (void *)&pdev->dev_default_config, 1U);
            break;

        case USBD_STATE_CONFIGURED:
            USBD_CtlSendData(pdev, (void *)&pdev->dev_config, 1U);
            break;

        default:
            USBD_CtlError(pdev, req);
            break;
        }
    }
}

/**
* @brief  USBD_GetStatus
*         Handle Get Status request
* @param  pdev: device instance
* @param  req: usb request
*/
static void USBD_GetStatus(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    switch (pdev->dev_state)
    {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
    case USBD_STATE_CONFIGURED:
        if (req->wLength != 0x2U)
        {
            USBD_CtlError(pdev, req);
            break;
        }

#if (USBD_SELF_POWERED == 1U)
        pdev->dev_config_status = USB_CONFIG_SELF_POWERED;
#else
        pdev->dev_config_status = 0U;
#endif

        if (pdev->dev_remote_wakeup)
        {
            pdev->dev_config_status |= USB_CONFIG_REMOTE_WAKEUP;
        }

        USBD_CtlSendData(pdev, (void *)&pdev->dev_config_status, 2U);
        break;

    default:
        USBD_CtlError(pdev, req);
        break;
    }
}

/**
* @brief  USBD_SetFeature
*         Handle Set device feature request
* @param  pdev: device instance
* @param  req: usb request
*/
static void USBD_SetFeature(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
    {
        pdev->dev_remote_wakeup = 1U;
        USBD_CtlSendStatus(pdev);
    }
}

/**
* @brief  USBD_ClrFeature
*         Handle clear device feature request
* @param  pdev: device instance
* @param  req: usb request
*/
static void USBD_ClrFeature(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    switch (pdev->dev_state)
    {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
    case USBD_STATE_CONFIGURED:
        if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
        {
            pdev->dev_remote_wakeup = 0U;
            USBD_CtlSendStatus(pdev);
        }
        break;

    default:
        USBD_CtlError(pdev, req);
        break;
    }
}

/**
* @brief  USBD_StdDevReq
*         Handle standard usb device requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static USBD_StatusTypeDef 
USBD_StdDevReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    USBD_StatusTypeDef ret = USBD_OK;

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
        pdev->pClass->Setup(pdev, req);
        break;

    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest)
        {
        case USB_REQ_GET_DESCRIPTOR: USBD_GetDescriptor(pdev, req); break;
        case USB_REQ_SET_ADDRESS: USBD_SetAddress(pdev, req); break;
        case USB_REQ_SET_CONFIGURATION: USBD_SetConfig(pdev, req); break;
        case USB_REQ_GET_CONFIGURATION: USBD_GetConfig(pdev, req); break;
        case USB_REQ_GET_STATUS: USBD_GetStatus(pdev, req); break;
        case USB_REQ_SET_FEATURE: USBD_SetFeature(pdev, req); break;
        case USB_REQ_CLEAR_FEATURE: USBD_ClrFeature(pdev, req); break;
        default: USBD_CtlError(pdev, req); break;
        }
        break;

    default:
        USBD_CtlError(pdev, req);
        break;
    }

    return ret;
}

/**
* @brief  USBD_StdItfReq
*         Handle standard usb interface requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static USBD_StatusTypeDef 
USBD_StdItfReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef  *req)
{
    USBD_StatusTypeDef ret = USBD_OK;
    const uint8_t* const cfgdesc = (pdev->dev_speed == USBD_SPEED_HIGH) ?
        pdev->pClass->HSConfigDescriptor.ptr :
        pdev->pClass->FSConfigDescriptor.ptr;

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
    case USB_REQ_TYPE_STANDARD:
        switch (pdev->dev_state)
        {
        case USBD_STATE_DEFAULT:
        case USBD_STATE_ADDRESSED:
        case USBD_STATE_CONFIGURED:
            if (LOBYTE(req->wIndex) < cfgdesc[4])
            {
                ret = (USBD_StatusTypeDef)pdev->pClass->Setup(pdev, req);

                if ((req->wLength == 0U) && (ret == USBD_OK))
                {
                    USBD_CtlSendStatus(pdev);
                }
            }
            else
            {
                USBD_CtlError(pdev, req);
            }
            break;

        default:
            USBD_CtlError(pdev, req);
            break;
        }
        break;

    default:
        USBD_CtlError(pdev, req);
        break;
    }

    return USBD_OK;
}

/**
* @brief  USBD_StdEPReq
*         Handle standard usb endpoint requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static USBD_StatusTypeDef 
USBD_StdEPReq(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef  *req)
{
    USBD_EndpointTypeDef *pep;
    USBD_StatusTypeDef ret = USBD_OK;
    const uint8_t ep_addr = LOBYTE(req->wIndex);
    const uint8_t non_ctl_ep = ((ep_addr != 0x00U) && (ep_addr != 0x80U));

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
    case USB_REQ_TYPE_CLASS:
    case USB_REQ_TYPE_VENDOR:
        pdev->pClass->Setup(pdev, req);
        break;

    case USB_REQ_TYPE_STANDARD:
        switch (req->bRequest)
        {
        case USB_REQ_SET_FEATURE:
            switch (pdev->dev_state)
            {
            case USBD_STATE_ADDRESSED:
                if (non_ctl_ep)
                {
                    USBD_Drv_EP_SetStall(pdev->pData, ep_addr);
                    USBD_Drv_EP_SetStall(pdev->pData, 0x80U);
                }
                else
                {
                    USBD_CtlError(pdev, req);
                }
                break;

            case USBD_STATE_CONFIGURED:
                if (req->wValue == USB_FEATURE_EP_HALT)
                {
                    if (non_ctl_ep && (req->wLength == 0x00U))
                    {
                        USBD_Drv_EP_SetStall(pdev->pData, ep_addr);
                    }
                }
                USBD_CtlSendStatus(pdev);
                break;

            default:
                USBD_CtlError(pdev, req);
                break;
            }
            break;

        case USB_REQ_CLEAR_FEATURE:
            switch (pdev->dev_state)
            {
            case USBD_STATE_ADDRESSED:
                if (non_ctl_ep)
                {
                    USBD_Drv_EP_SetStall(pdev->pData, ep_addr);
                    USBD_Drv_EP_SetStall(pdev->pData, 0x80U);
                }
                else
                {
                    USBD_CtlError(pdev, req);
                }
                break;

            case USBD_STATE_CONFIGURED:
                if (req->wValue == USB_FEATURE_EP_HALT)
                {
                    if ((ep_addr & 0x7FU) != 0x00U)
                    {
                        USBD_Drv_EP_ClrStall(pdev->pData, ep_addr);
                    }

                    USBD_CtlSendStatus(pdev);
                }
                break;

            default:
                USBD_CtlError(pdev, req);
                break;
            }
            break;

        case USB_REQ_GET_STATUS:
            pep = ((ep_addr & 0x80U) == 0x80U) ? 
                &pdev->ep_in[ep_addr & 0x7FU] :
                &pdev->ep_out[ep_addr & 0x7FU];

            switch (pdev->dev_state)
            {
            case USBD_STATE_ADDRESSED:
                if (non_ctl_ep)
                {
                    USBD_CtlError(pdev, req);
                    break;
                }

                pep->status = 0x0000U;
                USBD_CtlSendData(pdev, (uint8_t *)(void *)&pep->status, 2U);
                break;

            case USBD_STATE_CONFIGURED:
                if (pep->is_used == 0U)
                {
                    USBD_CtlError(pdev, req);
                    break;
                }

                pep->status = 
                    (non_ctl_ep && USBD_Drv_EP_IsStall(pdev->pData, ep_addr)) ?
                        0x0001U :
                        0x0000U;

                USBD_CtlSendData(pdev, (uint8_t *)(void *)&pep->status, 2U);
                break;

            default:
                USBD_CtlError(pdev, req);
                break;
            }
            break;

        default:
            USBD_CtlError(pdev, req);
            break;
        }
        break;

    default:
        USBD_CtlError(pdev, req);
        break;
    }

    return ret;
}

/**
* @brief  USBD_ParseSetupRequest
*         Copy buffer into setup structure
* @param  pdev: device instance
* @param  req: usb request
* @retval None
*/
static void USBD_ParseSetupRequest(USBD_SetupReqTypedef *req, const uint8_t *pdata)
{
    req->bmRequest = *(uint8_t *)(pdata);
    req->bRequest = *(uint8_t *)(pdata + 1U);
    req->wValue = SWAPBYTE(pdata + 2U);
    req->wIndex = SWAPBYTE(pdata + 4U);
    req->wLength = SWAPBYTE(pdata + 6U);
}

/**
* @brief  USBD_SetupStage
*         Handle the setup stage
* @param  pdev: device instance
* @retval status
*/
int USBD_Lib_SetupStage(USBD_HandleTypeDef *pdev, const uint8_t *psetup)
{
    USBD_ParseSetupRequest(&pdev->request, psetup);

    pdev->ep0_state = USBD_EP0_SETUP;
    pdev->ep0_data_len = pdev->request.wLength;

    switch (pdev->request.bmRequest & 0x1FU)
    {
    case USB_REQ_RECIPIENT_DEVICE:
        USBD_StdDevReq(pdev, &pdev->request);
        break;

    case USB_REQ_RECIPIENT_INTERFACE:
        USBD_StdItfReq(pdev, &pdev->request);
        break;

    case USB_REQ_RECIPIENT_ENDPOINT:
        USBD_StdEPReq(pdev, &pdev->request);
        break;

    default:
        USBD_Drv_EP_SetStall(pdev->pData, (pdev->request.bmRequest & 0x80U));
        break;
    }

    return USBD_OK;
}

/**
* @brief  USBD_DataOutStage
*         Handle data OUT stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
int 
USBD_Lib_DataOutStage(USBD_HandleTypeDef *pdev, uint8_t epnum, uint8_t *pdata)
{
    USBD_EndpointTypeDef *pep;

    if (epnum == 0U)
    {
        pep = &pdev->ep_out[0];

        if (pdev->ep0_state == USBD_EP0_DATA_OUT)
        {
            if (pep->rem_length > pep->maxpacket)
            {
                pep->rem_length -= pep->maxpacket;

                USBD_Drv_EP_Receive(
                    pdev->pData, 
                    0U, 
                    pdata, 
                    (uint16_t) MIN(pep->rem_length, pep->maxpacket)
                );
            }
            else
            {
                if ((pdev->pClass->EP0_RxReady != NULL) &&
                    (pdev->dev_state == USBD_STATE_CONFIGURED))
                {
                    pdev->pClass->EP0_RxReady(pdev);
                }

                USBD_CtlSendStatus(pdev);
            }
        }
        else
        {
            if (pdev->ep0_state == USBD_EP0_STATUS_OUT)
            {
                /*
                * STATUS PHASE completed, update ep0_state to idle
                */
                pdev->ep0_state = USBD_EP0_IDLE;
                USBD_Drv_EP_SetStall(pdev->pData, 0U);
            }
        }
    }
    else if ((pdev->pClass->DataOut != NULL) &&
            (pdev->dev_state == USBD_STATE_CONFIGURED))
    {
        pdev->pClass->DataOut(pdev, epnum);
    }
    else
    {
        /* should never be in this condition */
        return USBD_FAIL;
    }

    return USBD_OK;
}

/**
* @brief  USBD_DataInStage
*         Handle data in stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
int 
USBD_Lib_DataInStage(USBD_HandleTypeDef *pdev, uint8_t epnum, const uint8_t *pdata)
{
    USBD_EndpointTypeDef *pep;

    if (epnum == 0U)
    {
        pep = &pdev->ep_in[0];

        if (pdev->ep0_state == USBD_EP0_DATA_IN)
        {
            if (pep->rem_length > pep->maxpacket)
            {
                pep->rem_length -= pep->maxpacket;

                USBD_Drv_EP_Transmit(
                    pdev->pData, 
                    0U, 
                    pdata, 
                    (uint16_t)pep->rem_length
                );

                /* Prepare endpoint for premature end of transfer */
                USBD_Drv_EP_Receive(pdev->pData, 0U, NULL, 0U);
            }
            else
            {
                /* last packet is MPS multiple, so send ZLP packet */
                if ((pep->total_length % pep->maxpacket == 0U) &&
                    (pep->total_length >= pep->maxpacket) &&
                    (pep->total_length < pdev->ep0_data_len))
                {
                    USBD_Drv_EP_Transmit(pdev->pData, 0x00U, NULL, 0);
                    pdev->ep0_data_len = 0U;

                    /* Prepare endpoint for premature end of transfer */
                    USBD_Drv_EP_Receive(pdev->pData, 0U, NULL, 0U);
                }
                else
                {
                    if ((pdev->pClass->EP0_TxSent != NULL) &&
                        (pdev->dev_state == USBD_STATE_CONFIGURED))
                    {
                        pdev->pClass->EP0_TxSent(pdev);
                    }

                    USBD_Drv_EP_SetStall(pdev->pData, 0x80U);
                    USBD_CtlReceiveStatus(pdev);
                }
            }
        }
        else
        {
            if ((pdev->ep0_state == USBD_EP0_STATUS_IN) ||
                (pdev->ep0_state == USBD_EP0_IDLE))
            {
                USBD_Drv_EP_SetStall(pdev->pData, 0x80U);
            }
        }
    }
    else if ((pdev->pClass->DataIn != NULL) &&
            (pdev->dev_state == USBD_STATE_CONFIGURED))
    {
        pdev->pClass->DataIn(pdev, epnum);
    }
    else
    {
        /* should never be in this condition */
        return USBD_FAIL;
    }

    return USBD_OK;
}

/**
* @brief  USBD_Lib_Reset
*         Handle Reset event
* @param  pdev: device instance
* @retval status
*/
int USBD_Lib_Reset(USBD_HandleTypeDef *pdev)
{
    pdev->dev_speed = USBD_SPEED_FULL;

    /* Open EP0 OUT */
    USBD_Drv_EP_Open(pdev->pData, 0x00U, USB_MAX_EP0_SIZE, USBD_EP_TYPE_CTRL);
    pdev->ep_out[0x00U & 0xFU].is_used = 1U;
    pdev->ep_out[0].maxpacket = USB_MAX_EP0_SIZE;

    /* Open EP0 IN */
    USBD_Drv_EP_Open(pdev->pData, 0x80U, USB_MAX_EP0_SIZE, USBD_EP_TYPE_CTRL);
    pdev->ep_in[0x80U & 0xFU].is_used = 1U;
    pdev->ep_in[0].maxpacket = USB_MAX_EP0_SIZE;

    /* Upon Reset call user call back */
    pdev->dev_state = USBD_STATE_DEFAULT;
    pdev->ep0_state = USBD_EP0_IDLE;
    pdev->dev_config = 0U;
    pdev->dev_remote_wakeup = 0U;

    if (pdev->pClassData)
    {
        pdev->pClass->DeInit(pdev, (uint8_t)pdev->dev_config);
    }

    return USBD_OK;
}

/**
* @brief  USBD_Suspend
*         Handle Suspend event
* @param  pdev: device instance
* @retval status
*/
int USBD_Lib_Suspend(USBD_HandleTypeDef *pdev)
{
    pdev->dev_old_state =  pdev->dev_state;
    pdev->dev_state  = USBD_STATE_SUSPENDED;
    return USBD_OK;
}

/**
* @brief  USBD_Resume
*         Handle Resume event
* @param  pdev: device instance
* @retval status
*/
int USBD_Lib_Resume(USBD_HandleTypeDef *pdev)
{
    if (pdev->dev_state == USBD_STATE_SUSPENDED)
    {
        pdev->dev_state = pdev->dev_old_state;
    }

    return USBD_OK;
}

/**
* @brief  USBD_SOF
*         Handle SOF event
* @param  pdev: device instance
* @retval status
*/
int USBD_Lib_SOF(USBD_HandleTypeDef *pdev)
{
    if (pdev->dev_state == USBD_STATE_CONFIGURED)
    {
        if (pdev->pClass->SOF != NULL)
        {
            pdev->pClass->SOF(pdev);
        }
    }

    return USBD_OK;
}

/**
* @brief  USBD_CtlError
*         Handle USB low level Error
* @param  pdev: device instance
* @param  req: usb request
* @retval None
*/
void USBD_CtlError(USBD_HandleTypeDef *pdev, const USBD_SetupReqTypedef *req)
{
    USBD_Drv_EP_SetStall(pdev->pData, 0x80U);
    USBD_Drv_EP_SetStall(pdev->pData, 0U);
}
