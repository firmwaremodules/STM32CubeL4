/**
  ******************************************************************************
  * @file    FreeRTOS\FreeRTOS_LowPower\Src\main.c
  * @author  MCD Application Team
  * @brief   Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stm32_secure_patching_bootloader_interface_v1.3.0.h"
#include "stm32l4r9i_discovery_ospi_nor.h"
#include "fw_update_ymodem.h"
#include <string.h>
#include <stdio.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define blckqSTACK_SIZE   configMINIMAL_STACK_SIZE
#define QUEUE_SIZE        (uint32_t) 1
/* Private variables ---------------------------------------------------------*/
osMessageQId osQueue;

/* Handle to STLINK VCOM UART for printf redirection */
static UART_HandleTypeDef hUart;

#define QSPI_MEMORY_MAPPED_NAME "OSPI"

/* Content placed into OSPI flash in the "SEG1" area.  This emulates scheme that TouchGFx uses to
place GUI assets in external flash while bundling them with the main application manufacturing and update binaries. */
#if FW_UPDATE_VERSION == 1
const char qspi_string[] LOCATION_EXTFLASH_ATTRIBUTE = "This is a version 1 message from " QSPI_MEMORY_MAPPED_NAME "!";
#else
const char qspi_string[] LOCATION_EXTFLASH_ATTRIBUTE = "This is a version 2 message from " QSPI_MEMORY_MAPPED_NAME "!";
#endif

/* The number of items the queue can hold.  This is 1 as the Rx task will
remove items as they are added so the Tx task should always find the queue
empty. */
#define QUEUE_LENGTH             (1)

/* The rate at which the Tx task sends to the queue. */
#define TX_DELAY                 (500)

/* The value that is sent from the Tx task to the Rx task on the queue. */
#define QUEUED_VALUE             (100)

/* The length of time the LED will remain on for.  It is on just long enough
to be able to see with the human eye so as not to distort the power readings too
much. */
#define LED_TOGGLE_DELAY         (20)

/* Private function prototypes -----------------------------------------------*/
static void QueueReceiveThread(const void *argument);
static void QueueSendThread(const void *argument);
static void FirmwareUpdateThread(const void* argument);
static void GPIO_ConfigAN(void);
void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the System clock to 120 MHz */
  SystemClock_Config();

  /* Configure GPIO's to AN to reduce power consumption */
  GPIO_ConfigAN();

  /* Initialize LED */
  BSP_LED_Init(LED1);

  /* Initialize OSPI flash in memory mapped mode for direct access to flash read through memory bus */
  BSP_OSPI_NOR_Init();
  BSP_OSPI_NOR_EnableMemoryMappedMode();

  /* Initialize STLINK VCOM UART */
  hUart.Init.BaudRate = 115200U;
  hUart.Init.WordLength = UART_WORDLENGTH_8B;
  hUart.Init.StopBits = UART_STOPBITS_1;
  hUart.Init.Parity = UART_PARITY_NONE;
  hUart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hUart.Init.Mode = UART_MODE_RX | UART_MODE_TX;
  hUart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  hUart.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  BSP_COM_Init(COM1, &hUart);

  printf("Starting FreeRTOS_LowPower stm32-secure-patching-bootloader demo\n");

  /* Get the firmware version embedded in the active slot header using the
   * Secure Engine services.
   */
  SE_StatusTypeDef se_Status = SE_KO;
  SE_APP_ActiveFwInfo sl_FwInfo;
  memset(&sl_FwInfo, 0xFF, sizeof(SE_APP_ActiveFwInfo));
  SE_APP_GetActiveFwInfo(&se_Status, &sl_FwInfo);
  printf("Firmware Version: %ld.%ld.%ld\n",
      FW_VERSION_MAJOR(sl_FwInfo.ActiveFwVersion),
      FW_VERSION_MINOR(sl_FwInfo.ActiveFwVersion),
      FW_VERSION_PATCH(sl_FwInfo.ActiveFwVersion));
  char bootver[32];
  memset(bootver, 0, sizeof(bootver));
  SE_APP_GetBootVer(&se_Status, bootver, sizeof(bootver));
  printf("Bootloader Version: %s\n", bootver);

  /* If OSPI is working in memory mapped mode, should see the "This is a version..." message */
  printf(QSPI_MEMORY_MAPPED_NAME " string: %s\n", qspi_string);

  /* Create the queue used by the two threads */
  osMessageQDef(osqueue, QUEUE_LENGTH, uint16_t);
  osQueue = osMessageCreate(osMessageQ(osqueue), NULL);

  /* Note the Tx has a lower priority than the Rx when the threads are
  spawned. */
  osThreadDef(RxThread, QueueReceiveThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate(osThread(RxThread), NULL);

  osThreadDef(TxThread, QueueSendThread, osPriorityBelowNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate(osThread(TxThread), NULL);

  /* Thread to manage firmware update over YMODEM/UART.
   This must be lower priority than other threads because of the way we're using HAL_UART's blocking timing mechanism. 
  */
  osThreadDef(FwUpdateThread, FirmwareUpdateThread, osPriorityLow, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate(osThread(FwUpdateThread), NULL);

  printf("Starting scheduler\n");

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  for (;;);

}

/**
  * @brief  Message Queue Producer Thread.
  * @param  argument: Not used
  * @retval None
  */
static void QueueSendThread(const void *argument)
{
  for (;;)
  {
    /* Place this thread into the blocked state until it is time to run again.
       The kernel will place the MCU into the Retention low power sleep state
       when the idle thread next runs. */
    osDelay(TX_DELAY);

    /* Send to the queue - causing the queue receive thread to flash its LED.
       It should not be necessary to block on the queue send because the Rx
       thread will already have removed the last queued item. */
    osMessagePut(osQueue, (uint32_t)QUEUED_VALUE, 0);
  }
}

/**
  * @brief  Message Queue Consumer Thread.
  * @param  argument: Not used
  * @retval None
  */
static void QueueReceiveThread(const void *argument)
{
  osEvent event;

  for (;;)
  {
    /* Wait until something arrives in the queue. */
    event = osMessageGet(osQueue, osWaitForever);

    /*  To get here something must have arrived, but is it the expected
    value?  If it is, turn the LED on for a short while. */
    if (event.status == osEventMessage)
    {
      if (event.value.v == QUEUED_VALUE)
      {
        BSP_LED_On(LED1);
        osDelay(LED_TOGGLE_DELAY);
        BSP_LED_Off(LED1);
      }
    }
  }
}

/**
  * @brief  Pre Sleep Processing
  * @param  ulExpectedIdleTime: Expected time in idle state
  * @retval None
  */
void PreSleepProcessing(uint32_t ulExpectedIdleTime)
{
  /* Called by the kernel before it places the MCU into a sleep mode because
  configPRE_SLEEP_PROCESSING() is #defined to PreSleepProcessing().

  NOTE:  Additional actions can be taken here to get the power consumption
  even lower.  For example, peripherals can be turned off here, and then back
  on again in the post sleep processing function.  For maximum power saving
  ensure all unused pins are in their lowest power state. */

  /* 
    (ulExpectedIdleTime) is set to 0 to indicate that PreSleepProcessing contains
    its own wait for interrupt or wait for event instruction and so the kernel vPortSuppressTicksAndSleep 
    function does not need to execute the wfi instruction  
  */
  (void) ulExpectedIdleTime;


  /* Suspend the HAL Tick */
  HAL_SuspendTick();

  /*Enter to sleep Mode using the HAL function HAL_PWR_EnterSLEEPMode with WFI instruction*/
  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);  
}

/**
  * @brief  Post Sleep Processing
  * @param  ulExpectedIdleTime: Not used
  * @retval None
  */
void PostSleepProcessing(uint32_t ulExpectedIdleTime)
{
  /* Called by the kernel when the MCU exits a sleep mode because
  configPOST_SLEEP_PROCESSING is #defined to PostSleepProcessing(). */

  /* Avoid compiler warnings about the unused parameter. */
  (void) ulExpectedIdleTime;

  /* resume the HAL tick */
  HAL_ResumeTick();

}


static void PrintMainMenu(void)
{
    printf("\n------------------- Main Menu ----------------------------\n\n");
    printf("  Download a new firmware image or patch via YMODEM ----- 1\n\n");
}

/* This is a quick and dirty port of ST's YMODEM updater code 
It uses blocking delays which we'd ideally avoid in an RTOS environment. 
*/
static void FirmwareUpdateThread(const void* argument)
{
    uint8_t key = 0U;

    PrintMainMenu();

    for (;;)
    {
        while (1U)
        {
            /* Clean the input path */
            Board_COM_Flush();

            /* Receive input from UART.  Note that this timeout defines the
             * loop period and the LED blink rate.  *Blocking delay*
             * The RTOS works around this by preemptive task switching to higher priority
             * threads.  However, during firmware update operations that access flash or
             * the secure engine, there must be no other tasks that can use these resources.
             * If this cannot be guaranteed, then task switching must be suspended around the
             * APP_UPDATE_XXX APIs.
             */
            if (Board_COM_Receive(&key, 1U, RX_TIMEOUT) == BOARD_STATUS_OK)
            {
                switch (key)
                {
                case '1':
                    FW_UPDATE_YMODEM_Run();
                    break;
                default:
                    printf("Invalid option\n");
                    break;
                }

                /*Print Main Menu message*/
                PrintMainMenu();
            }
        }
    }
}

/**
  * @brief  Configure all GPIO's to AN to reduce the power consumption
  * @param  None
  * @retval None
  */
static void GPIO_ConfigAN(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Configure all GPIO as analog to reduce current consumption on non used IOs */
  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Pin = GPIO_PIN_All;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /* Disable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOD_CLK_DISABLE();
  __HAL_RCC_GPIOE_CLK_DISABLE();
  __HAL_RCC_GPIOF_CLK_DISABLE();
  __HAL_RCC_GPIOG_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();
  __HAL_RCC_GPIOI_CLK_DISABLE();
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 120000000
  *            HCLK(Hz)                       = 120000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 60
  *            PLL_Q                          = 2
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* Enable voltage range 1 boost mode for frequency above 80 Mhz */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  __HAL_RCC_PWR_CLK_DISABLE();

  /* Enable MSI Oscillator and activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
  
  /* To avoid undershoot due to maximum frequency, select PLL as system clock source */
  /* with AHB prescaler divider 2 as first step */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* AHB prescaler divider at 1 as second step */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

/* Adaptation for printf direction to STLINK VCOM UART */

#if defined(__ICCARM__)
#error not supported
#elif defined(__CC_ARM)
#error not supported
#elif defined(__GNUC__)
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define WRITE_PROTOTYPE int _write(int file, char* ptr, int len)
#endif

WRITE_PROTOTYPE
{
    HAL_UART_Transmit(&hUart, (uint8_t*)ptr, len, 1000);
    return len;
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval ch
  */
PUTCHAR_PROTOTYPE
{
    /* e.g. write a character to the USART and Loop until the end of transmission */
    HAL_UART_Transmit(&hUart, (uint8_t*)&ch, 1U, 1000);

    return ch;
}

/* Bindings for the YMODEM firmware updater */
int Board_COM_Transmit(uint8_t* Data, uint16_t uDataLength, uint32_t uTimeout)
{
    return HAL_UART_Transmit(&hUart, (uint8_t*)Data, uDataLength, uTimeout);
}

/**
  * @brief Receive Data.
  * @param uDataLength: Data pointer to the Data to receive.
  * @param uTimeout: Timeout duration.
  * @retval Status of the Receive operation.
  */
int Board_COM_Receive(uint8_t* Data, uint16_t uDataLength, uint32_t uTimeout)
{
    return HAL_UART_Receive(&hUart, (uint8_t*)Data, uDataLength, uTimeout);
}

int Board_COM_Flush(void)
{
    /* Clean the input path */
    __HAL_UART_FLUSH_DRREGISTER(&hUart);
    return HAL_OK;
}

/* Bindings for YMODEM CRC */

static CRC_HandleTypeDef    CrcHandle;

int board_crc_init(void)
{
    /* Configure the peripheral clock */
    __HAL_RCC_CRC_CLK_ENABLE();

    /* Configure CRC with default polynomial - standard configuration */
    return board_crc_config(BOARD_CRC_CONFIG_32BIT);
}

int board_crc_deinit(void)
{
    /* Disable the peripheral clock */
    __HAL_RCC_CRC_CLK_DISABLE();
    return BOARD_STATUS_OK;
}

int board_crc_config(BOARD_CRC_ConfigTypeDef eCRCConfg)
{
    int status = BOARD_STATUS_OK;

    /* Switch to the selected configuration */
    CrcHandle.Instance = CRC;

    /* The input data are not inverted */
    CrcHandle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;

    /* The output data are not inverted */
    CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;

    switch (eCRCConfg)
    {
    case BOARD_CRC_CONFIG_32BIT:
        /* The Default polynomial is used */
        CrcHandle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
        /* The default init value is used */
        CrcHandle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
        /* The input data are 32-bit long words */
        CrcHandle.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
        /* Valid parameter*/
        break;

    case BOARD_CRC_CONFIG_16BIT:
        /* The CRC-16-CCIT polynomial is used */
        CrcHandle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
        CrcHandle.Init.GeneratingPolynomial = 0x1021U;
        CrcHandle.Init.CRCLength = CRC_POLYLENGTH_16B;
        /* The zero init value is used */
        CrcHandle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
        CrcHandle.Init.InitValue = 0U;
        /* The input data are 8-bit long */
        CrcHandle.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
        /* Valid parameter*/
        break;

    default:
        /* Invalid parameter */
        status = BOARD_STATUS_ERROR;
        break;
    }

    if (status == BOARD_STATUS_OK)
    {
        if (HAL_CRC_Init(&CrcHandle) != HAL_OK)
        {
            status = BOARD_STATUS_ERROR;
        }
    }
    return status;
}

uint32_t board_crc_calculate(uint32_t pBuffer[], uint32_t BufferLength)
{
    return HAL_CRC_Calculate(&CrcHandle, pBuffer, BufferLength);
}




#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif

