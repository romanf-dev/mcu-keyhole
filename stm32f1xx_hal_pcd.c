/**
  ******************************************************************************
  * @file    stm32f1xx_hal_pcd.c
  * @author  MCD Application Team
  * @brief   PCD HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the USB Peripheral Controller:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_pcd.h"
#include "usbd_drv.h"

#define PCD_MIN(a, b)  (((a) < (b)) ? (a) : (b))
#define PCD_MAX(a, b)  (((a) > (b)) ? (a) : (b))

static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd);
static HAL_StatusTypeDef HAL_PCD_EP_DB_Transmit(PCD_HandleTypeDef *hpcd, PCD_EPTypeDef *ep, uint16_t wEPVal);
static uint16_t HAL_PCD_EP_DB_Receive(PCD_HandleTypeDef *hpcd, PCD_EPTypeDef *ep, uint16_t wEPVal);

/**
  * @brief  Initializes the PCD according to the specified
  *         parameters in the PCD_InitTypeDef and initialize the associated handle.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
int USBD_Drv_Init(PCD_HandleTypeDef *hpcd, void* context)
{
  uint8_t i;

  /* Check the PCD handle allocation */
  if (hpcd == NULL)
  {
    return HAL_ERROR;
  }

  /* Check the parameters */
  assert_param(IS_PCD_ALL_INSTANCE(hpcd->Instance));

  if (hpcd->State == HAL_PCD_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    hpcd->Lock = HAL_UNLOCKED;

    /* Init the low level hardware : GPIO, CLOCK, NVIC... */
    //HAL_PCD_MspInit(hpcd);
  }

  hpcd->State = HAL_PCD_STATE_BUSY;

  /* Disable the Interrupts */
  __HAL_PCD_DISABLE(hpcd);

  /* Init endpoints structures */
  for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
  {
    /* Init ep structure */
    hpcd->IN_ep[i].is_in = 1U;
    hpcd->IN_ep[i].num = i;
    hpcd->IN_ep[i].tx_fifo_num = i;
    /* Control until ep is activated */
    hpcd->IN_ep[i].type = EP_TYPE_CTRL;
    hpcd->IN_ep[i].maxpacket = 0U;
    hpcd->IN_ep[i].u.xfer_buff = 0U;
    hpcd->IN_ep[i].xfer_len = 0U;
  }

  for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
  {
    hpcd->OUT_ep[i].is_in = 0U;
    hpcd->OUT_ep[i].num = i;
    /* Control until ep is activated */
    hpcd->OUT_ep[i].type = EP_TYPE_CTRL;
    hpcd->OUT_ep[i].maxpacket = 0U;
    hpcd->OUT_ep[i].u.xfer_buff = 0U;
    hpcd->OUT_ep[i].xfer_len = 0U;
  }

  /* Init Device */
  if (USB_DevInit(hpcd->Instance, hpcd->Init) != HAL_OK)
  {
    hpcd->State = HAL_PCD_STATE_ERROR;
    return HAL_ERROR;
  }

  hpcd->pData = context;
  hpcd->USB_Address = 0U;
  hpcd->State = HAL_PCD_STATE_READY;

  return HAL_OK;
}

/**
  * @brief  DeInitializes the PCD peripheral.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
int USBD_Drv_DeInit(PCD_HandleTypeDef *hpcd)
{
  /* Check the PCD handle allocation */
  if (hpcd == NULL)
  {
    return HAL_ERROR;
  }

  hpcd->State = HAL_PCD_STATE_BUSY;

  /* Stop Device */
  if (USB_StopDevice(hpcd->Instance) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* DeInit the low level hardware: CLOCK, NVIC.*/
  //HAL_PCD_MspDeInit(hpcd);

  hpcd->State = HAL_PCD_STATE_RESET;

  return HAL_OK;
}

/**
  * @brief  Start the USB device
  * @param  hpcd PCD handle
  * @retval HAL status
  */
int USBD_Drv_Start(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd);
  __HAL_PCD_ENABLE(hpcd);

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Stop the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
int USBD_Drv_Stop(PCD_HandleTypeDef *hpcd)
{
  __HAL_LOCK(hpcd);
  __HAL_PCD_DISABLE(hpcd);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  This function handles PCD interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd)
{
  uint16_t store_ep[8];
  uint8_t i;

  if (__HAL_PCD_GET_FLAG(hpcd, USB_ISTR_CTR))
  {
    /* servicing of the endpoint correct transfer interrupt */
    /* clear of the CTR flag into the sub */
    (void)PCD_EP_ISR_Handler(hpcd);
  }

  if (__HAL_PCD_GET_FLAG(hpcd, USB_ISTR_RESET))
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_RESET);

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->ResetCallback(hpcd);
#else
    USBD_Lib_Reset(hpcd->pData);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    (void)USBD_Drv_SetAddress(hpcd, 0U);
  }

  if (__HAL_PCD_GET_FLAG(hpcd, USB_ISTR_PMAOVR))
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_PMAOVR);
  }

  if (__HAL_PCD_GET_FLAG(hpcd, USB_ISTR_ERR))
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_ERR);
  }

  if (__HAL_PCD_GET_FLAG(hpcd, USB_ISTR_WKUP))
  {
    hpcd->Instance->CNTR &= (uint16_t) ~(USB_CNTR_LP_MODE);
    hpcd->Instance->CNTR &= (uint16_t) ~(USB_CNTR_FSUSP);

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->ResumeCallback(hpcd);
#else
    USBD_Lib_Resume(hpcd->pData);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_WKUP);
  }

  if (__HAL_PCD_GET_FLAG(hpcd, USB_ISTR_SUSP))
  {
    /* WA: To Clear Wakeup flag if raised with suspend signal */

    /* Store Endpoint register */
    for (i = 0U; i < 8U; i++)
    {
      store_ep[i] = PCD_GET_ENDPOINT(hpcd->Instance, i);
    }

    /* FORCE RESET */
    hpcd->Instance->CNTR |= (uint16_t)(USB_CNTR_FRES);

    /* CLEAR RESET */
    hpcd->Instance->CNTR &= (uint16_t)(~USB_CNTR_FRES);

    /* wait for reset flag in ISTR */
    while ((hpcd->Instance->ISTR & USB_ISTR_RESET) == 0U)
    {
    }

    /* Clear Reset Flag */
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_RESET);

    /* Restore Registre */
    for (i = 0U; i < 8U; i++)
    {
      PCD_SET_ENDPOINT(hpcd->Instance, i, store_ep[i]);
    }

    /* Force low-power mode in the macrocell */
    hpcd->Instance->CNTR |= (uint16_t)USB_CNTR_FSUSP;

    /* clear of the ISTR bit must be done after setting of CNTR_FSUSP */
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_SUSP);

    hpcd->Instance->CNTR |= (uint16_t)USB_CNTR_LP_MODE;

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->SuspendCallback(hpcd);
#else
    USBD_Lib_Suspend(hpcd->pData);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
  }

  if (__HAL_PCD_GET_FLAG(hpcd, USB_ISTR_SOF))
  {
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_SOF);

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd->SOFCallback(hpcd);
#else
    USBD_Lib_SOF(hpcd->pData);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
  }

  if (__HAL_PCD_GET_FLAG(hpcd, USB_ISTR_ESOF))
  {
    /* clear ESOF flag in ISTR */
    __HAL_PCD_CLEAR_FLAG(hpcd, USB_ISTR_ESOF);
  }
}

/**
  * @brief  Handles PCD Wakeup interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
void HAL_PCD_WKUP_IRQHandler(PCD_HandleTypeDef *hpcd)
{
  /* Clear EXTI pending Bit */
  __HAL_USB_WAKEUP_EXTI_CLEAR_FLAG();
}

/**
  * @brief  Set the USB Device address.
  * @param  hpcd PCD handle
  * @param  address new device address
  * @retval HAL status
  */
int USBD_Drv_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address)
{
  __HAL_LOCK(hpcd);
  hpcd->USB_Address = address;
  (void)USB_SetDevAddress(hpcd->Instance, address);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}
/**
  * @brief  Open and configure an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  ep_mps endpoint max packet size
  * @param  ep_type endpoint type
  * @retval HAL status
  */
int USBD_Drv_EP_Open(
    PCD_HandleTypeDef *hpcd, 
    uint8_t ep_addr, 
    uint16_t ep_mps, 
    uint8_t ep_type)
{
  HAL_StatusTypeDef  ret = HAL_OK;
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->num = ep_addr & EP_ADDR_MSK;
  ep->maxpacket = ep_mps;
  ep->type = ep_type;

  if (ep->is_in != 0U)
  {
    /* Assign a Tx FIFO */
    ep->tx_fifo_num = ep->num;
  }
  /* Set initial data PID. */
  if (ep_type == EP_TYPE_BULK)
  {
    ep->data_pid_start = 0U;
  }

  __HAL_LOCK(hpcd);
  (void)USB_ActivateEndpoint(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);

  return ret;
}

/**
  * @brief  Deactivate an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
int USBD_Drv_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }
  ep->num   = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);
  (void)USB_DeactivateEndpoint(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);
  return HAL_OK;
}

/**
  * @brief  Receive an amount of data.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the reception buffer
  * @param  len amount of data to be received
  * @retval HAL status
  */
int USBD_Drv_EP_Receive(
    PCD_HandleTypeDef *hpcd, 
    uint8_t ep_addr, 
    uint8_t *pBuf, 
    uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->u.xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0StartXfer(hpcd->Instance, ep);
  }
  else
  {
    (void)USB_EPStartXfer(hpcd->Instance, ep);
  }

  return HAL_OK;
}

/**
  * @brief  Get Received Data Size
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval Data Size
  */
size_t USBD_Drv_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  return hpcd->OUT_ep[ep_addr & EP_ADDR_MSK].xfer_count;
}
/**
  * @brief  Send an amount of data
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the transmission buffer
  * @param  len amount of data to be sent
  * @retval HAL status
  */
int USBD_Drv_EP_Transmit(
    PCD_HandleTypeDef *hpcd, 
    uint8_t ep_addr, 
    const uint8_t *pBuf, 
    uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->u.xfer_buff_ro = pBuf;
  ep->xfer_len = len;
#if defined (USB)
  ep->xfer_fill_db = 1U;
  ep->xfer_len_db = len;
#endif /* defined (USB) */
  ep->xfer_count = 0U;
  ep->is_in = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0StartXfer(hpcd->Instance, ep);
  }
  else
  {
    (void)USB_EPStartXfer(hpcd->Instance, ep);
  }

  return HAL_OK;
}

/**
  * @brief  Set a STALL condition over an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
int USBD_Drv_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & EP_ADDR_MSK) > hpcd->Init.dev_endpoints)
  {
    return HAL_ERROR;
  }

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr];
    ep->is_in = 0U;
  }

  ep->is_stall = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);

  (void)USB_EPSetStall(hpcd->Instance, ep);

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Clear a STALL condition over in an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
int USBD_Drv_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if (((uint32_t)ep_addr & 0x0FU) > hpcd->Init.dev_endpoints)
  {
    return HAL_ERROR;
  }

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->is_stall = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(hpcd);
  (void)USB_EPClearStall(hpcd->Instance, ep);
  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Returns Stall condition.
  * @param  hpcd PCD handle
  * @param  ep_addr: Endpoint number
  * @retval Stall (1: Yes, 0: No)
  */
int USBD_Drv_EP_IsStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
    const PCD_EPTypeDef* pep = ((ep_addr & 0x80) == 0x80) ? 
        hpcd->IN_ep : 
        hpcd->OUT_ep;

    return pep[ep_addr & 0x7F].is_stall;
}

/**
  * @brief  Flush an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
int USBD_Drv_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr)
{
  __HAL_LOCK(hpcd);

  if ((ep_addr & 0x80U) == 0x80U)
  {
    (void)USB_FlushTxFifo(hpcd->Instance, (uint32_t)ep_addr & EP_ADDR_MSK);
  }
  else
  {
    (void)USB_FlushRxFifo(hpcd->Instance);
  }

  __HAL_UNLOCK(hpcd);

  return HAL_OK;
}

/**
  * @brief  Activate remote wakeup signalling
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd)
{
  return (USB_ActivateRemoteWakeup(hpcd->Instance));
}

/**
  * @brief  De-activate remote wakeup signalling.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd)
{
  return (USB_DeActivateRemoteWakeup(hpcd->Instance));
}

/**
  * @brief  Return the PCD handle state.
  * @param  hpcd PCD handle
  * @retval HAL state
  */
PCD_StateTypeDef HAL_PCD_GetState(PCD_HandleTypeDef *hpcd)
{
  return hpcd->State;
}

/**
  * @brief  This function handles PCD Endpoint interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
static HAL_StatusTypeDef PCD_EP_ISR_Handler(PCD_HandleTypeDef *hpcd)
{
  PCD_EPTypeDef *ep;
  uint16_t count, wIstr, wEPVal, TxByteNbre;
  uint8_t epindex;

  /* stay in loop while pending interrupts */
  while ((hpcd->Instance->ISTR & USB_ISTR_CTR) != 0U)
  {
    wIstr = hpcd->Instance->ISTR;

    /* extract highest priority endpoint number */
    epindex = (uint8_t)(wIstr & USB_ISTR_EP_ID);

    if (epindex == 0U)
    {
      /* Decode and service control endpoint interrupt */

      /* DIR bit = origin of the interrupt */
      if ((wIstr & USB_ISTR_DIR) == 0U)
      {
        /* DIR = 0 */

        /* DIR = 0 => IN  int */
        /* DIR = 0 implies that (EP_CTR_TX = 1) always */
        PCD_CLEAR_TX_EP_CTR(hpcd->Instance, PCD_ENDP0);
        ep = &hpcd->IN_ep[0];

        ep->xfer_count = PCD_GET_EP_TX_CNT(hpcd->Instance, ep->num);
        ep->u.xfer_buff_ro += ep->xfer_count;

        /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd->DataInStageCallback(hpcd, 0U);
#else
        USBD_Lib_DataInStage(hpcd->pData, 0, hpcd->IN_ep[0].u.xfer_buff_ro);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

        if ((hpcd->USB_Address > 0U) && (ep->xfer_len == 0U))
        {
          hpcd->Instance->DADDR = ((uint16_t)hpcd->USB_Address | USB_DADDR_EF);
          hpcd->USB_Address = 0U;
        }
      }
      else
      {
        /* DIR = 1 */

        /* DIR = 1 & CTR_RX => SETUP or OUT int */
        /* DIR = 1 & (CTR_TX | CTR_RX) => 2 int pending */
        ep = &hpcd->OUT_ep[0];
        wEPVal = PCD_GET_ENDPOINT(hpcd->Instance, PCD_ENDP0);

        if ((wEPVal & USB_EP_SETUP) != 0U)
        {
          /* Get SETUP Packet */
          ep->xfer_count = PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

          USB_ReadPMA(hpcd->Instance, (uint8_t *)hpcd->Setup,
                      ep->pmaadress, (uint16_t)ep->xfer_count);

          /* SETUP bit kept frozen while CTR_RX = 1 */
          PCD_CLEAR_RX_EP_CTR(hpcd->Instance, PCD_ENDP0);

          /* Process SETUP Packet*/
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
          hpcd->SetupStageCallback(hpcd);
#else
          USBD_Lib_SetupStage(hpcd->pData, (uint8_t *)hpcd->Setup);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
        }
        else if ((wEPVal & USB_EP_CTR_RX) != 0U)
        {
          PCD_CLEAR_RX_EP_CTR(hpcd->Instance, PCD_ENDP0);

          /* Get Control Data OUT Packet */
          ep->xfer_count = PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

          if ((ep->xfer_count != 0U) && (ep->u.xfer_buff != 0U))
          {
            USB_ReadPMA(hpcd->Instance, ep->u.xfer_buff,
                        ep->pmaadress, (uint16_t)ep->xfer_count);

            ep->u.xfer_buff += ep->xfer_count;

            /* Process Control Data OUT Packet */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
            hpcd->DataOutStageCallback(hpcd, 0U);
#else
            USBD_Lib_DataOutStage(hpcd->pData, 0, hpcd->OUT_ep[0].u.xfer_buff);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
          }

          PCD_SET_EP_RX_CNT(hpcd->Instance, PCD_ENDP0, ep->maxpacket);
          PCD_SET_EP_RX_STATUS(hpcd->Instance, PCD_ENDP0, USB_EP_RX_VALID);
        }
      }
    }
    else
    {
      /* Decode and service non control endpoints interrupt */
      /* process related endpoint register */
      wEPVal = PCD_GET_ENDPOINT(hpcd->Instance, epindex);

      if ((wEPVal & USB_EP_CTR_RX) != 0U)
      {
        /* clear int flag */
        PCD_CLEAR_RX_EP_CTR(hpcd->Instance, epindex);
        ep = &hpcd->OUT_ep[epindex];

        /* OUT Single Buffering */
        if (ep->doublebuffer == 0U)
        {
          count = (uint16_t)PCD_GET_EP_RX_CNT(hpcd->Instance, ep->num);

          if (count != 0U)
          {
            USB_ReadPMA(hpcd->Instance, ep->u.xfer_buff, ep->pmaadress, count);
          }
        }
        else
        {
          /* manage double buffer bulk out */
          if (ep->type == EP_TYPE_BULK)
          {
            count = HAL_PCD_EP_DB_Receive(hpcd, ep, wEPVal);
          }
          else /* manage double buffer iso out */
          {
            /* free EP OUT Buffer */
            PCD_FreeUserBuffer(hpcd->Instance, ep->num, 0U);

            if ((PCD_GET_ENDPOINT(hpcd->Instance, ep->num) & USB_EP_DTOG_RX) != 0U)
            {
              /* read from endpoint BUF0Addr buffer */
              count = (uint16_t)PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);

              if (count != 0U)
              {
                USB_ReadPMA(hpcd->Instance, ep->u.xfer_buff, ep->pmaaddr0, count);
              }
            }
            else
            {
              /* read from endpoint BUF1Addr buffer */
              count = (uint16_t)PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);

              if (count != 0U)
              {
                USB_ReadPMA(hpcd->Instance, ep->u.xfer_buff, ep->pmaaddr1, count);
              }
            }
          }
        }
        /* multi-packet on the NON control OUT endpoint */
        ep->xfer_count += count;
        ep->u.xfer_buff += count;

        if ((ep->xfer_len == 0U) || (count < ep->maxpacket))
        {
          /* RX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
          hpcd->DataOutStageCallback(hpcd, ep->num);
#else
          USBD_Lib_DataOutStage(hpcd->pData, ep->num, hpcd->OUT_ep[ep->num].u.xfer_buff);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
        }
        else
        {
          (void) USB_EPStartXfer(hpcd->Instance, ep);
        }

      }

      if ((wEPVal & USB_EP_CTR_TX) != 0U)
      {
        ep = &hpcd->IN_ep[epindex];

        /* clear int flag */
        PCD_CLEAR_TX_EP_CTR(hpcd->Instance, epindex);

        /* Manage all non bulk transaction or Bulk Single Buffer Transaction */
        if ((ep->type != EP_TYPE_BULK) ||
            ((ep->type == EP_TYPE_BULK) && ((wEPVal & USB_EP_KIND) == 0U)))
        {
          /* multi-packet on the NON control IN endpoint */
          TxByteNbre = (uint16_t)PCD_GET_EP_TX_CNT(hpcd->Instance, ep->num);

          if (ep->xfer_len > TxByteNbre)
          {
            ep->xfer_len -= TxByteNbre;
          }
          else
          {
            ep->xfer_len = 0U;
          }

          /* Zero Length Packet? */
          if (ep->xfer_len == 0U)
          {
            /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
            hpcd->DataInStageCallback(hpcd, ep->num);
#else
            USBD_Lib_DataInStage(hpcd->pData, ep->num, hpcd->IN_ep[ep->num].u.xfer_buff_ro);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
          }
          else
          {
            /* Transfer is not yet Done */
            ep->u.xfer_buff_ro += TxByteNbre;
            ep->xfer_count += TxByteNbre;
            (void)USB_EPStartXfer(hpcd->Instance, ep);
          }
        }
        /* bulk in double buffer enable in case of transferLen> Ep_Mps */
        else
        {
          (void)HAL_PCD_EP_DB_Transmit(hpcd, ep, wEPVal);
        }
      }
    }
  }

  return HAL_OK;
}

/**
  * @brief  Manage double buffer bulk out transaction from ISR
  * @param  hpcd PCD handle
  * @param  ep current endpoint handle
  * @param  wEPVal Last snapshot of EPRx register value taken in ISR
  * @retval HAL status
  */
static uint16_t HAL_PCD_EP_DB_Receive(PCD_HandleTypeDef *hpcd,
                                      PCD_EPTypeDef *ep, uint16_t wEPVal)
{
  uint16_t count;

  /* Manage Buffer0 OUT */
  if ((wEPVal & USB_EP_DTOG_RX) != 0U)
  {
    /* Get count of received Data on buffer0 */
    count = (uint16_t)PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);

    if (ep->xfer_len >= count)
    {
      ep->xfer_len -= count;
    }
    else
    {
      ep->xfer_len = 0U;
    }

    if (ep->xfer_len == 0U)
    {
      /* set NAK to OUT endpoint since double buffer is enabled */
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_NAK);
    }

    /* Check if Buffer1 is in blocked sate which requires to toggle */
    if ((wEPVal & USB_EP_DTOG_TX) != 0U)
    {
      PCD_FreeUserBuffer(hpcd->Instance, ep->num, 0U);
    }

    if (count != 0U)
    {
      USB_ReadPMA(hpcd->Instance, ep->u.xfer_buff, ep->pmaaddr0, count);
    }
  }
  /* Manage Buffer 1 DTOG_RX=0 */
  else
  {
    /* Get count of received data */
    count = (uint16_t)PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);

    if (ep->xfer_len >= count)
    {
      ep->xfer_len -= count;
    }
    else
    {
      ep->xfer_len = 0U;
    }

    if (ep->xfer_len == 0U)
    {
      /* set NAK on the current endpoint */
      PCD_SET_EP_RX_STATUS(hpcd->Instance, ep->num, USB_EP_RX_NAK);
    }

    /*Need to FreeUser Buffer*/
    if ((wEPVal & USB_EP_DTOG_TX) == 0U)
    {
      PCD_FreeUserBuffer(hpcd->Instance, ep->num, 0U);
    }

    if (count != 0U)
    {
      USB_ReadPMA(hpcd->Instance, ep->u.xfer_buff, ep->pmaaddr1, count);
    }
  }

  return count;
}

/**
  * @brief  Manage double buffer bulk IN transaction from ISR
  * @param  hpcd PCD handle
  * @param  ep current endpoint handle
  * @param  wEPVal Last snapshot of EPRx register value taken in ISR
  * @retval HAL status
  */
static HAL_StatusTypeDef HAL_PCD_EP_DB_Transmit(PCD_HandleTypeDef *hpcd,
                                                PCD_EPTypeDef *ep, uint16_t wEPVal)
{
  uint32_t len;
  uint16_t TxByteNbre;

  /* Data Buffer0 ACK received */
  if ((wEPVal & USB_EP_DTOG_TX) != 0U)
  {
    /* multi-packet on the NON control IN endpoint */
    TxByteNbre = (uint16_t)PCD_GET_EP_DBUF0_CNT(hpcd->Instance, ep->num);

    if (ep->xfer_len > TxByteNbre)
    {
      ep->xfer_len -= TxByteNbre;
    }
    else
    {
      ep->xfer_len = 0U;
    }
    /* Transfer is completed */
    if (ep->xfer_len == 0U)
    {
      /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->DataInStageCallback(hpcd, ep->num);
#else
      USBD_Lib_DataInStage(hpcd->pData, ep->num, hpcd->IN_ep[ep->num].u.xfer_buff_ro);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      if ((wEPVal & USB_EP_DTOG_RX) != 0U)
      {
        PCD_FreeUserBuffer(hpcd->Instance, ep->num, 1U);
      }
    }
    else /* Transfer is not yet Done */
    {
      /* need to Free USB Buff */
      if ((wEPVal & USB_EP_DTOG_RX) != 0U)
      {
        PCD_FreeUserBuffer(hpcd->Instance, ep->num, 1U);
      }

      /* Still there is data to Fill in the next Buffer */
      if (ep->xfer_fill_db == 1U)
      {
        ep->u.xfer_buff_ro += TxByteNbre;
        ep->xfer_count += TxByteNbre;

        /* Calculate the len of the new buffer to fill */
        if (ep->xfer_len_db >= ep->maxpacket)
        {
          len = ep->maxpacket;
          ep->xfer_len_db -= len;
        }
        else if (ep->xfer_len_db == 0U)
        {
          len = TxByteNbre;
          ep->xfer_fill_db = 0U;
        }
        else
        {
          ep->xfer_fill_db = 0U;
          len = ep->xfer_len_db;
          ep->xfer_len_db = 0U;
        }

        /* Write remaining Data to Buffer */
        /* Set the Double buffer counter for pma buffer1 */
        PCD_SET_EP_DBUF0_CNT(hpcd->Instance, ep->num, ep->is_in, len);

        /* Copy user buffer to USB PMA */
        USB_WritePMA(hpcd->Instance, ep->u.xfer_buff_ro,  ep->pmaaddr0, (uint16_t)len);
      }
    }
  }
  else /* Data Buffer1 ACK received */
  {
    /* multi-packet on the NON control IN endpoint */
    TxByteNbre = (uint16_t)PCD_GET_EP_DBUF1_CNT(hpcd->Instance, ep->num);

    if (ep->xfer_len >= TxByteNbre)
    {
      ep->xfer_len -= TxByteNbre;
    }
    else
    {
      ep->xfer_len = 0U;
    }

    /* Transfer is completed */
    if (ep->xfer_len == 0U)
    {
      /* TX COMPLETE */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd->DataInStageCallback(hpcd, ep->num);
#else
      USBD_Lib_DataInStage(hpcd->pData, ep->num, hpcd->IN_ep[ep->num].u.xfer_buff_ro);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      /*need to Free USB Buff*/
      if ((wEPVal & USB_EP_DTOG_RX) == 0U)
      {
        PCD_FreeUserBuffer(hpcd->Instance, ep->num, 1U);
      }
    }
    else /* Transfer is not yet Done */
    {
      /* need to Free USB Buff */
      if ((wEPVal & USB_EP_DTOG_RX) == 0U)
      {
        PCD_FreeUserBuffer(hpcd->Instance, ep->num, 1U);
      }

      /* Still there is data to Fill in the next Buffer */
      if (ep->xfer_fill_db == 1U)
      {
        ep->u.xfer_buff_ro += TxByteNbre;
        ep->xfer_count += TxByteNbre;

        /* Calculate the len of the new buffer to fill */
        if (ep->xfer_len_db >= ep->maxpacket)
        {
          len = ep->maxpacket;
          ep->xfer_len_db -= len;
        }
        else if (ep->xfer_len_db == 0U)
        {
          len = TxByteNbre;
          ep->xfer_fill_db = 0U;
        }
        else
        {
          len = ep->xfer_len_db;
          ep->xfer_len_db = 0U;
          ep->xfer_fill_db = 0;
        }

        /* Set the Double buffer counter for pmabuffer1 */
        PCD_SET_EP_DBUF1_CNT(hpcd->Instance, ep->num, ep->is_in, len);

        /* Copy the user buffer to USB PMA */
        USB_WritePMA(hpcd->Instance, ep->u.xfer_buff_ro,  ep->pmaaddr1, (uint16_t)len);
      }
    }
  }

  /*enable endpoint IN*/
  PCD_SET_EP_TX_STATUS(hpcd->Instance, ep->num, USB_EP_TX_VALID);

  return HAL_OK;
}

/**
  * @brief  Configure PMA for EP
  * @param  hpcd  Device instance
  * @param  ep_addr endpoint address
  * @param  ep_kind endpoint Kind
  *                  USB_SNG_BUF: Single Buffer used
  *                  USB_DBL_BUF: Double Buffer used
  * @param  pmaadress: EP address in The PMA: In case of single buffer endpoint
  *                   this parameter is 16-bit value providing the address
  *                   in PMA allocated to endpoint.
  *                   In case of double buffer endpoint this parameter
  *                   is a 32-bit value providing the endpoint buffer 0 address
  *                   in the LSB part of 32-bit value and endpoint buffer 1 address
  *                   in the MSB part of 32-bit value.
  * @retval HAL status
  */

HAL_StatusTypeDef  HAL_PCDEx_PMAConfig(PCD_HandleTypeDef *hpcd, uint16_t ep_addr,
                                       uint16_t ep_kind, uint32_t pmaadress)
{
  PCD_EPTypeDef *ep;

  /* initialize ep structure*/
  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd->IN_ep[ep_addr & EP_ADDR_MSK];
  }
  else
  {
    ep = &hpcd->OUT_ep[ep_addr];
  }

  /* Here we check if the endpoint is single or double Buffer*/
  if (ep_kind == PCD_SNG_BUF)
  {
    /* Single Buffer */
    ep->doublebuffer = 0U;
    /* Configure the PMA */
    ep->pmaadress = (uint16_t)pmaadress;
  }
  else /* USB_DBL_BUF */
  {
    /* Double Buffer Endpoint */
    ep->doublebuffer = 1U;
    /* Configure the PMA */
    ep->pmaaddr0 = (uint16_t)(pmaadress & 0xFFFFU);
    ep->pmaaddr1 = (uint16_t)((pmaadress & 0xFFFF0000U) >> 16);
  }

  return HAL_OK;
}
