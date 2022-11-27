/**
  ******************************************************************************
  * @file    stm32f1xx_hal_def.h
  * @author  MCD Application Team
  * @brief   This file contains HAL common defines, enumeration, macros and
  *          structures definitions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics.
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
#ifndef __STM32F1xx_HAL_DEF
#define __STM32F1xx_HAL_DEF

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"
#include <stddef.h>
#include <errno.h>

/* Exported types ------------------------------------------------------------*/

/**
  * @brief  HAL Status structures definition
  */
enum
{
  HAL_OK       = 0,
  HAL_ERROR    = EINVAL,
  HAL_BUSY     = EBUSY,
  HAL_TIMEOUT  = ETIMEDOUT
};

typedef int HAL_StatusTypeDef;

/**
  * @brief  HAL Lock structures definition
  */
typedef enum
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U
} HAL_LockTypeDef;

#define HAL_IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) != 0U)
#define HAL_IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == 0U)

#define UNUSED(X) (void)X      /* To avoid gcc/g++ warnings */

#if (USE_RTOS == 1U)
/* Reserved for future use */
#error "USE_RTOS should be 0 in the current HAL release"
#else
#define __HAL_LOCK(__HANDLE__)                                           \
                                do{                                        \
                                    if((__HANDLE__)->Lock == HAL_LOCKED)   \
                                    {                                      \
                                       return HAL_BUSY;                    \
                                    }                                      \
                                    else                                   \
                                    {                                      \
                                       (__HANDLE__)->Lock = HAL_LOCKED;    \
                                    }                                      \
                                  }while (0U)

#define __HAL_UNLOCK(__HANDLE__)                                          \
                                  do{                                       \
                                      (__HANDLE__)->Lock = HAL_UNLOCKED;    \
                                    }while (0U)
#endif /* USE_RTOS */

#define assert_param(expr) ((void)0U)

#ifdef __cplusplus
}
#endif

#endif /* ___STM32F1xx_HAL_DEF */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
