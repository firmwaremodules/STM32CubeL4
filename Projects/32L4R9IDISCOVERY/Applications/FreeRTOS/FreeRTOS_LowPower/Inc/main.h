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

/* Constants used by Serial Command Line Mode */
#define TX_TIMEOUT                              ((uint32_t)100U)
#define RX_TIMEOUT                              ((uint32_t)1000U)

#define BOARD_STATUS_OK                         0
#define BOARD_STATUS_ERROR                      -1

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

/* CRC
* Hardware or SW (depending on device support) used for ymodem 16-bit CRC.
* Hardware must always be powered up on Init.
*/
typedef enum
{
    BOARD_CRC_CONFIG_NONE = 0U,   /*!< None */
    BOARD_CRC_CONFIG_32BIT,       /*!< Default configuration */
    BOARD_CRC_CONFIG_16BIT        /*!< 16 bit configuration */
} BOARD_CRC_ConfigTypeDef;
/* APIs return 0 on success, < 0 on error. */
int board_crc_init(void);
int board_crc_deinit(void);
int board_crc_config(BOARD_CRC_ConfigTypeDef eCRCConfg);
uint32_t board_crc_calculate(uint32_t pBuffer[], uint32_t BufferLength);


/* Bindings for the YMODEM firmware updater */
int Board_COM_Transmit(uint8_t* Data, uint16_t uDataLength, uint32_t uTimeout);
int Board_COM_Receive(uint8_t* Data, uint16_t uDataLength, uint32_t uTimeout);
int Board_COM_Flush(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */


