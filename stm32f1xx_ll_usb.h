/**
  ******************************************************************************
  * @file    stm32f1xx_ll_usb.h
  * @author  MCD Application Team
  * @brief   Header file of USB Low Layer HAL module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32F1xx_LL_USB_H
#define STM32F1xx_LL_USB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal_def.h"

#if defined (USB) || defined (USB_OTG_FS)
/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */

/** @addtogroup USB_LL
  * @{
  */

/* Exported types ------------------------------------------------------------*/

#if defined (USB)

typedef enum
{
  USB_DEVICE_MODE  = 0
} USB_ModeTypeDef;

/**
  * @brief  USB Initialization Structure definition
  */
typedef struct
{
  uint32_t dev_endpoints;           /*!< Device Endpoints number.
                                         This parameter depends on the used USB core.
                                         This parameter must be a number between Min_Data = 1 and Max_Data = 15 */

  uint32_t speed;                   /*!< USB Core speed.
                                         This parameter can be any value of @ref USB_Core_Speed                 */

  uint32_t ep0_mps;                 /*!< Set the Endpoint 0 Max Packet size.                                    */

  uint32_t phy_itface;              /*!< Select the used PHY interface.
                                         This parameter can be any value of @ref USB_Core_PHY                   */

  uint32_t Sof_enable;              /*!< Enable or disable the output of the SOF signal.                        */

  uint32_t low_power_enable;        /*!< Enable or disable Low Power mode                                       */

  uint32_t lpm_enable;              /*!< Enable or disable Battery charging.                                    */

  uint32_t battery_charging_enable; /*!< Enable or disable Battery charging.                                    */
} USB_CfgTypeDef;

typedef struct
{
  uint8_t   num;             /*!< Endpoint number
                                  This parameter must be a number between Min_Data = 1 and Max_Data = 15    */

  uint8_t   is_in;           /*!< Endpoint direction
                                  This parameter must be a number between Min_Data = 0 and Max_Data = 1     */

  uint8_t   is_stall;        /*!< Endpoint stall condition
                                  This parameter must be a number between Min_Data = 0 and Max_Data = 1     */

  uint8_t   type;            /*!< Endpoint type
                                  This parameter can be any value of @ref USB_EP_Type                       */

  uint8_t   data_pid_start;  /*!< Initial data PID
                                  This parameter must be a number between Min_Data = 0 and Max_Data = 1     */

  uint16_t  pmaadress;       /*!< PMA Address
                                  This parameter can be any value between Min_addr = 0 and Max_addr = 1K    */

  uint16_t  pmaaddr0;        /*!< PMA Address0
                                  This parameter can be any value between Min_addr = 0 and Max_addr = 1K    */

  uint16_t  pmaaddr1;        /*!< PMA Address1
                                  This parameter can be any value between Min_addr = 0 and Max_addr = 1K    */

  uint8_t   doublebuffer;    /*!< Double buffer enable
                                  This parameter can be 0 or 1                                              */

  uint16_t  tx_fifo_num;     /*!< This parameter is not required by USB Device FS peripheral, it is used
                                  only by USB OTG FS peripheral
                                  This parameter is added to ensure compatibility across USB peripherals    */

  uint32_t  maxpacket;       /*!< Endpoint Max packet size
                                  This parameter must be a number between Min_Data = 0 and Max_Data = 64KB  */

  volatile union
  {
    uint8_t *xfer_buff;        /*!< Pointer to transfer buffer                                                */
    const uint8_t* xfer_buff_ro;
  } u;

  uint32_t  xfer_len;        /*!< Current transfer length                                                   */

  uint32_t  xfer_count;      /*!< Partial transfer length in case of multi packet transfer                  */

  uint32_t  xfer_len_db;      /*!< double buffer transfer length used with bulk double buffer in           */

  uint8_t   xfer_fill_db;     /*!< double buffer Need to Fill new buffer  used with bulk_in                */

} USB_EPTypeDef;
#endif /* defined (USB) */

/* Exported constants --------------------------------------------------------*/

/** @defgroup PCD_Exported_Constants PCD Exported Constants
  * @{
  */

#if defined (USB_OTG_FS)
/** @defgroup USB_OTG_CORE VERSION ID
  * @{
  */
#define USB_OTG_CORE_ID_300A          0x4F54300AU
#define USB_OTG_CORE_ID_310A          0x4F54310AU
/**
  * @}
  */

/** @defgroup USB_Core_Mode_ USB Core Mode
  * @{
  */
#define USB_OTG_MODE_DEVICE                    0U
#define USB_OTG_MODE_HOST                      1U
#define USB_OTG_MODE_DRD                       2U
/**
  * @}
  */

/** @defgroup USB_LL Device Speed
  * @{
  */
#define USBD_FS_SPEED                          2U
#define USBH_FSLS_SPEED                        1U
/**
  * @}
  */

/** @defgroup USB_LL_Core_Speed USB Low Layer Core Speed
  * @{
  */
#define USB_OTG_SPEED_FULL                     3U
/**
  * @}
  */

/** @defgroup USB_LL_Core_PHY USB Low Layer Core PHY
  * @{
  */
#define USB_OTG_ULPI_PHY                       1U
#define USB_OTG_EMBEDDED_PHY                   2U
/**
  * @}
  */

/** @defgroup USB_LL_Turnaround_Timeout Turnaround Timeout Value
  * @{
  */
#ifndef USBD_FS_TRDT_VALUE
#define USBD_FS_TRDT_VALUE                     5U
#define USBD_DEFAULT_TRDT_VALUE                9U
#endif /* USBD_HS_TRDT_VALUE */
/**
  * @}
  */

/** @defgroup USB_LL_Core_MPS USB Low Layer Core MPS
  * @{
  */
#define USB_OTG_FS_MAX_PACKET_SIZE            64U
#define USB_OTG_MAX_EP0_SIZE                  64U
/**
  * @}
  */

/** @defgroup USB_LL_Core_PHY_Frequency USB Low Layer Core PHY Frequency
  * @{
  */
#define DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ     (0U << 1)
#define DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ     (1U << 1)
#define DSTS_ENUMSPD_FS_PHY_48MHZ              (3U << 1)
/**
  * @}
  */

/** @defgroup USB_LL_CORE_Frame_Interval USB Low Layer Core Frame Interval
  * @{
  */
#define DCFG_FRAME_INTERVAL_80                 0U
#define DCFG_FRAME_INTERVAL_85                 1U
#define DCFG_FRAME_INTERVAL_90                 2U
#define DCFG_FRAME_INTERVAL_95                 3U
/**
  * @}
  */

/** @defgroup USB_LL_EP0_MPS USB Low Layer EP0 MPS
  * @{
  */
#define EP_MPS_64                        0U
#define EP_MPS_32                        1U
#define EP_MPS_16                        2U
#define EP_MPS_8                         3U
/**
  * @}
  */

/** @defgroup USB_LL_EP_Speed USB Low Layer EP Speed
  * @{
  */
#define EP_SPEED_LOW                           0U
#define EP_SPEED_FULL                          1U
#define EP_SPEED_HIGH                          2U
/**
  * @}
  */

/** @defgroup USB_LL_EP_Type USB Low Layer EP Type
  * @{
  */
#define EP_TYPE_CTRL                           0U
#define EP_TYPE_ISOC                           1U
#define EP_TYPE_BULK                           2U
#define EP_TYPE_INTR                           3U
#define EP_TYPE_MSK                            3U
/**
  * @}
  */

/** @defgroup USB_LL_STS_Defines USB Low Layer STS Defines
  * @{
  */
#define STS_GOUT_NAK                           1U
#define STS_DATA_UPDT                          2U
#define STS_XFER_COMP                          3U
#define STS_SETUP_COMP                         4U
#define STS_SETUP_UPDT                         6U
/**
  * @}
  */

/** @defgroup USB_LL_HCFG_SPEED_Defines USB Low Layer HCFG Speed Defines
  * @{
  */
#define HCFG_30_60_MHZ                         0U
#define HCFG_48_MHZ                            1U
#define HCFG_6_MHZ                             2U
/**
  * @}
  */

/** @defgroup USB_LL_HPRT0_PRTSPD_SPEED_Defines USB Low Layer HPRT0 PRTSPD Speed Defines
  * @{
  */
#define HPRT0_PRTSPD_HIGH_SPEED                0U
#define HPRT0_PRTSPD_FULL_SPEED                1U
#define HPRT0_PRTSPD_LOW_SPEED                 2U
/**
  * @}
  */

#define HCCHAR_CTRL                            0U
#define HCCHAR_ISOC                            1U
#define HCCHAR_BULK                            2U
#define HCCHAR_INTR                            3U

#define HC_PID_DATA0                           0U
#define HC_PID_DATA2                           1U
#define HC_PID_DATA1                           2U
#define HC_PID_SETUP                           3U

#define GRXSTS_PKTSTS_IN                       2U
#define GRXSTS_PKTSTS_IN_XFER_COMP             3U
#define GRXSTS_PKTSTS_DATA_TOGGLE_ERR          5U
#define GRXSTS_PKTSTS_CH_HALTED                7U

#define USBx_PCGCCTL    *(__IO uint32_t *)((uint32_t)USBx_BASE + USB_OTG_PCGCCTL_BASE)
#define USBx_HPRT0      *(__IO uint32_t *)((uint32_t)USBx_BASE + USB_OTG_HOST_PORT_BASE)

#define USBx_DEVICE     ((USB_OTG_DeviceTypeDef *)(USBx_BASE + USB_OTG_DEVICE_BASE))
#define USBx_INEP(i)    ((USB_OTG_INEndpointTypeDef *)(USBx_BASE + USB_OTG_IN_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USBx_OUTEP(i)   ((USB_OTG_OUTEndpointTypeDef *)(USBx_BASE + USB_OTG_OUT_ENDPOINT_BASE + ((i) * USB_OTG_EP_REG_SIZE)))
#define USBx_DFIFO(i)   *(__IO uint32_t *)(USBx_BASE + USB_OTG_FIFO_BASE + ((i) * USB_OTG_FIFO_SIZE))

#define USBx_HOST       ((USB_OTG_HostTypeDef *)(USBx_BASE + USB_OTG_HOST_BASE))
#define USBx_HC(i)      ((USB_OTG_HostChannelTypeDef *)(USBx_BASE + USB_OTG_HOST_CHANNEL_BASE + ((i) * USB_OTG_HOST_CHANNEL_SIZE)))
#endif /* defined (USB_OTG_FS) */

#if defined (USB)
/** @defgroup USB_LL_EP0_MPS USB Low Layer EP0 MPS
  * @{
  */
#define EP_MPS_64                              0U
#define EP_MPS_32                              1U
#define EP_MPS_16                              2U
#define EP_MPS_8                               3U
/**
  * @}
  */

/** @defgroup USB_LL_EP_Type USB Low Layer EP Type
  * @{
  */
#define EP_TYPE_CTRL                           0U
#define EP_TYPE_ISOC                           1U
#define EP_TYPE_BULK                           2U
#define EP_TYPE_INTR                           3U
#define EP_TYPE_MSK                            3U
/**
  * @}
  */

/** @defgroup USB_LL Device Speed
  * @{
  */
#define USBD_FS_SPEED                          2U
/**
  * @}
  */

#define BTABLE_ADDRESS                         0x000U
#define PMA_ACCESS                             2U
#endif /* defined (USB) */
#if defined (USB_OTG_FS)
#define EP_ADDR_MSK                            0xFU
#endif /* defined (USB_OTG_FS) */
#if defined (USB)
#define EP_ADDR_MSK                            0x7U
#endif /* defined (USB) */
/**
  * @}
  */

/* Exported macro ------------------------------------------------------------*/
/** @defgroup USB_LL_Exported_Macros USB Low Layer Exported Macros
  * @{
  */
#if defined (USB_OTG_FS)
#define USB_MASK_INTERRUPT(__INSTANCE__, __INTERRUPT__)     ((__INSTANCE__)->GINTMSK &= ~(__INTERRUPT__))
#define USB_UNMASK_INTERRUPT(__INSTANCE__, __INTERRUPT__)   ((__INSTANCE__)->GINTMSK |= (__INTERRUPT__))

#define CLEAR_IN_EP_INTR(__EPNUM__, __INTERRUPT__)          (USBx_INEP(__EPNUM__)->DIEPINT = (__INTERRUPT__))
#define CLEAR_OUT_EP_INTR(__EPNUM__, __INTERRUPT__)         (USBx_OUTEP(__EPNUM__)->DOEPINT = (__INTERRUPT__))
#endif /* defined (USB_OTG_FS) */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup USB_LL_Exported_Functions USB Low Layer Exported Functions
  * @{
  */
#if defined (USB)
HAL_StatusTypeDef USB_DevInit(USB_TypeDef *USBx, USB_CfgTypeDef cfg);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_SetDevSpeed(USB_TypeDef *USBx, uint8_t speed);
HAL_StatusTypeDef USB_FlushRxFifo(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_FlushTxFifo(USB_TypeDef *USBx, uint32_t num);
HAL_StatusTypeDef USB_ActivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPSetStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(USB_TypeDef *USBx, USB_EPTypeDef *ep);
HAL_StatusTypeDef USB_SetDevAddress(USB_TypeDef *USBx, uint8_t address);
HAL_StatusTypeDef USB_StopDevice(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_WritePacket(USB_TypeDef *USBx, uint8_t *src,
                                  uint8_t ch_ep_num, uint16_t len);

void             *USB_ReadPacket(USB_TypeDef *USBx, uint8_t *dest, uint16_t len);

uint32_t          USB_ReadInterrupts(USB_TypeDef *USBx);
uint32_t          USB_ReadDevAllOutEpInterrupt(USB_TypeDef *USBx);
uint32_t          USB_ReadDevOutEPInterrupt(USB_TypeDef *USBx, uint8_t epnum);
uint32_t          USB_ReadDevAllInEpInterrupt(USB_TypeDef *USBx);
uint32_t          USB_ReadDevInEPInterrupt(USB_TypeDef *USBx, uint8_t epnum);
void              USB_ClearInterrupts(USB_TypeDef *USBx, uint32_t interrupt);
HAL_StatusTypeDef USB_ActivateRemoteWakeup(USB_TypeDef *USBx);
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(USB_TypeDef *USBx);

void              USB_WritePMA(USB_TypeDef *USBx, const uint8_t *pbUsrBuf,
                               uint16_t wPMABufAddr, uint16_t wNBytes);

void              USB_ReadPMA(USB_TypeDef *USBx, uint8_t *pbUsrBuf,
                              uint16_t wPMABufAddr, uint16_t wNBytes);
#endif /* defined (USB) */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* defined (USB) || defined (USB_OTG_FS) */

#ifdef __cplusplus
}
#endif


#endif /* STM32F1xx_LL_USB_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
