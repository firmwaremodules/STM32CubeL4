/**
  @page FreeRTOS_LowPower FreeRTOS Low Power example
 
  @verbatim
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_LowPower/readme.txt
  * @author  MCD Application Team
  * @brief   Description of the FreeRTOS low power example.
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
  @endverbatim

/*
 * Copyright (c) 2021-2022 Firmware Modules Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files(the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions :
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS IN THE SOFTWARE.
 */
 
Modifications and requirements to support demonstrating the https://github.com/firmwaremodules/stm32-secure-patching-bootloader

- Enabled STLINK VCOM UART and bound to printf
- Prints firmware and bootloader versions at application start (confirming access to bootloader secure engine)
Since bootloader is multi-segment capable and knows about OSPI flash:
- Added BSP_OSPI_NOR driver and initialization
- Stores a string to the ExtFlashSection allocated to OSPI flash start area in application SLOT0 SEG1 area.
- Enables OSPI memory mapped mode
- Prints stored string to confirm memory mapped mode operation.
- Added YMODEM UART firmware update capability and wrapped into thread (ported from DemoApp and from ST's YMODEM updater)
- Adapted YMODEM UART firmware update to use non-blocking RX interrupt mode with FreeRTOS message queues.


Requirements:
- Load the combined .hex file produced in Projects\Binary  e.g. BOOT_FreeRTOS-LowPower_STM32L4R9I-Discovery_<version>.hex
The <version> is fixed in the post-build command-line to v1.0.0 so it is easy to test firmware update by changing this 'to' as well as 'from' versions for patch testing.
[The <version> is auto generated from the git repo information (git decribe --tags --always --dirty).]
Only the semantic version digits, if found, will be embedded in the firmware image header and reported through the API.
This repository is tagged so that semantic version digits are found.
- Use STM32CubeProgrammer to the load the .hex file.  **Select the correct external flash loader (STM32L4R9I-DISCO) before programming**
- Use TeraTerm or similar tool to connect to STLINK COM port and observe output messages.
- Power cycle board to start bootloader and application.

Firmware update testing:
- Load .sfb or .sfbp (patch) files through YMODEM UART updater by accessing menu option (1)
1. Open project in STM32CubeIDE 
2. Build version v1.0.0 (out of box configuration in repository)
3. Load BOOT_FreeRTOS-LowPower_STM32L4R9I-Discovery_v1.0.0.hex to target (see above for instructions)
4. Open project properties, go to settings->Build Steps->Post-build steps
    Change Command: ... "v1.0.0" "v1.0.0" ...   -> "v1.1.0" "v1.0.0"   ( "to" and "from" versions)
5. Apply and Close.
6. Open main.h.  Change FW_UPDATE_VERSION to 2.
7. Build project.
8. You now have FreeRTOS-LowPower_STM32L4R9I-Discovery_v1.1.0.sfb and FreeRTOS-LowPower_STM32L4R9I-Discovery_v1.0.0_v1.1.0.sfbp.
9. In terminal program select option (1). With teraterm, File->Transfer->Ymodem->Send
10. Choose to send either of the .sfb or .sfbp files.  Both generate the same result: version v1.1.0 becoming the active application.


After loading BOOT_FreeRTOS-LowPower_STM32L4R9I-Discovery_v1.0.0.hex on a DISCO-L4R9I:

[  0.000] Bootloader starting up.
STM32 Secure Patching Bootloader. github.com/firmwaremodules/stm32-secure-patching-bootloader
Build: v1.3.0
Registered to: unregistered
Target: DISCO-L4R9I
UID: 0450474d7e346720
Clock:MSI,40,120,HSI48 Crypto:SW UART:2,115200,N81
SLOT0:08015000 SLOT1:90400000 SIZE:5EB000
APP RAMSTART:20001700 VTOR:08015200
[  0.030] Target check: CPUID:410FC241 IDCODE:10036470 FLASHSZ:0800 .. OK.
[  0.105] OSPI init success.
[  0.362] Verify bootloader.
[  0.413] SHA256: f9aab1bb8d49ba8fa0a48054bb8bb330509184cb29e2e31b3fc5740f9eb70870 Valid
[  0.421] UART loader check trigger.
[  1.426] Check USB flash media.
[  2.429] No valid firmware found on flash media, status=1
[  2.435] Verify slot 1 header.
[  2.438] Slot 1 is empty.
[  2.440] Verify slot 0.
[  2.443] Verify slot 0 header.
[  2.652] Verify slot 0 signature.
[  2.676] Verify slot 0 ready.
[  2.679] Verify slot 0 fw ver.
[  2.889] Slot 0 has valid active firmware version 1.1.0
[  2.894] Preparing to launch application in slot 0.
Starting FreeRTOS_LowPower stm32-secure-patching-bootloader demo - RX interrupt mode
Firmware Version: 1.0.0
Bootloader Version: v1.3.0
OSPI string: This is a version 1 message from OSPI!
Starting scheduler

------------------- Main Menu ----------------------------

  Download a new firmware image or patch via YMODEM ----- 1


Quick-test binaries in bin\ directory.


@par Application Description

How to enter and exit low-power mode with CMSIS RTOS API.

This application creates two threads.

   + An Rx thread that blocks on a queue to wait for data, blinking the LED1 each 
     time data is received (turning it on and then off again) before returning 
     to block on the queue once more.

   + A Tx thread that repeatedly enters the Blocked state for 500ms.  
     On exiting the blocked state the Tx thread sends a value through the queue 
     to the Rx thread (causing the Rx thread to exit the blocked state and blink 
     the LED1).

Blocking for a finite period allows the kernel to stop the tick interrupt
and place the STM32 into sleep mode - the lowest power mode possible
that allows the CPU registers and RAM to retain their state.

In this example, non-used GPIOs are configured to analog, thus helping to reduce 
the power consumption of the device.

Observed behaviour:

Every 500ms the MCU will come out of the low power state to turn the LED1 on,
then return to the low power state for 20ms before leaving the low power
state again to turn the LED1 off.  This will be observed as a fast blinking
on the LED1.

The RTOS tick is suppressed while the MCU is in its low power state.

@note Care must be taken when using HAL_Delay(), this function provides accurate
      delay (in milliseconds) based on variable incremented in HAL time base ISR.
      This implies that if HAL_Delay() is called from a peripheral ISR process, then
      the HAL time base interrupt must have higher priority (numerically lower) than
      the peripheral interrupt. Otherwise the caller ISR process will be blocked.
      To change the HAL time base interrupt priority you have to use HAL_NVIC_SetPriority()
      function.
 
@note The application needs to ensure that the HAL time base is always set to 1 millisecond
      to have correct HAL operation.

@note The FreeRTOS heap size configTOTAL_HEAP_SIZE defined in FreeRTOSConfig.h is set accordingly to the 
      OS resources memory requirements of the application with +10% margin and rounded to the upper Kbyte boundary.

For more details about FreeRTOS implementation on STM32Cube, please refer to UM1722 "Developing Applications 
on STM32Cube with RTOS".


@par Directory contents
    - FreeRTOS/FreeRTOS_LowPower/Src/main.c                       Main program
    - FreeRTOS/FreeRTOS_LowPower/Src/stm32l4xx_hal_timebase_tim.c HAL timebase file
    - FreeRTOS/FreeRTOS_LowPower/Src/stm32l4xx_it.c               Interrupt handlers
    - FreeRTOS/FreeRTOS_LowPower/Src/system_stm32l4xx.c           STM32L4xx system clock configuration file
    - FreeRTOS/FreeRTOS_LowPower/Inc/main.h                       Main program header file
    - FreeRTOS/FreeRTOS_LowPower/Inc/stm32l4xx_hal_conf.h         HAL Library Configuration file
    - FreeRTOS/FreeRTOS_LowPower/Inc/stm32l4xx_it.h               Interrupt handlers header file
    - FreeRTOS/FreeRTOS_LowPower/Inc/FreeRTOSConfig.h             FreeRTOS Configuration file

@par Hardware and Software environment

  - This example runs on STM32L4R9xx devices.
    
  - This example has been tested with STM32L4R9I-Discovery board and can be
    easily tailored to any other supported device and development board.
    

@par How to use it ?

In order to make the program work, you must do the following:
 - Open your preferred toolchain 
 - Rebuild all files and load your image into target memory
 - Run the example
  
 */
