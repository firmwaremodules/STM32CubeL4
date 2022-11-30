/**
  ******************************************************************************
  * @file    FreeRTOS\FreeRTOS_LowPower\Inc\main.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the main.c
  *          file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4r9i_discovery.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Use after variable allocation to have linker allocate to the OSPI flash segment.
 * This requires cooperation with the linker script (having defined and
 * assigned the ExtFlashSection section to OSPI region).
 *
 * E.g.
 *   const uint8_t image[] LOCATION_EXTFLASH_ATTRIBUTE = { 0xAA, 0xBB, ... image data ... };
 */
#define LOCATION_EXTFLASH_ATTRIBUTE __attribute__ ((section ("ExtFlashSection"))) __attribute__ ((aligned(4))) 

 /* Select from 1 or 2 to change the application so that a difference exists for a patch to be generated
 (Same application binary between versions still works in full-image update mode (.sfb) but no patch is generated (.sfbp)).
 */
#define FW_UPDATE_VERSION                       1

/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


