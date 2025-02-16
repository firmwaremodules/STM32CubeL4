# STM32 Secure Patching Bootloader Reference Project Integrations for STM32L4

[stm32-secure-patching-bootloader](https://github.com/firmwaremodules/stm32-secure-patching-bootloader)

This repository is a fork of the main STM32CubeL4 repository.  We have integrated our bootloader with the projects listed below.  You can use these projects as references or starting points for your own bootloader integrations.

The bootloader is incorporated as a *subrepo* `Bootloader`.  To build the reference projects you must init and checkout the bootloader subrepo as well:

* `git clone --recursive https://github.com/firmwaremodules/STM32CubeL4`

or for already cloned repository without --recursive:

* `git submodule update --init`

| Board | Project | 
| --- | --- | 
| [DISCO-L4R9I](https://github.com/firmwaremodules/stm32-secure-patching-bootloader/tree/main/Libs/DISCO-L4R9I)| [Applications/FreeRTOS_LowPower](https://github.com/firmwaremodules/STM32CubeL4/tree/master/Projects/32L4R9IDISCOVERY/Applications/FreeRTOS/FreeRTOS_LowPower) |

Outputs are in

* `Project/Binary`

<hr>

## STM32CubeL4 MCU Firmware Package

![latest tag](https://img.shields.io/github/v/tag/STMicroelectronics/STM32CubeL4.svg?color=brightgreen)

**STM32Cube** is an STMicroelectronics original initiative to ease developers' life by reducing efforts, time and cost.

**STM32Cube** covers the overall STM32 products portfolio. It includes a comprehensive embedded software platform delivered for each STM32 series.
   * The CMSIS modules (core and device) corresponding to the ARM(tm) core implemented in this STM32 product.
   * The STM32 HAL-LL drivers, an abstraction layer offering a set of APIs ensuring maximized portability across the STM32 portfolio.
   * The BSP drivers of each evaluation, demonstration or nucleo board provided for this STM32 series.
   * A consistent set of middleware libraries such as RTOS, USB, FatFS, graphics, touch sensing library...
   * A full set of software projects (basic examples, applications, and demonstrations) for each board provided for this STM32 series.

The **STM32CubeL4 MCU Package** projects are directly running on the STM32L4 series boards. You can find in each Projects/*Board name* directories a set of software projects (Applications/Demonstration/Examples).

In this FW Package, the modules **Middlewares/ST/TouchGFX** and **Middlewares/ST/STemWin** are not directly accessible. They must be downloaded from a ST server. The respective URL are available in a readme.txt file inside each module.

## Release note

Details about the content of this release are available in the release note [here](https://htmlpreview.github.io/?https://github.com/STMicroelectronics/STM32CubeL4/blob/master/Release_Notes.html).

## Boards available

  * STM32L4
    * [32L4P5GDISCOVERY](https://www.st.com/en/evaluation-tools/stm32l4p5g-dk.html)
    * [32L4R9IDISCOVERY](https://www.st.com/en/evaluation-tools/32l4r9idiscovery.html)
    * 32L476GDISCOVERY (obsolete)
    * [32L496GDISCOVERY](https://www.st.com/en/evaluation-tools/32l496gdiscovery.html)
    * [B-L475E-IOT01A](https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html)
    * [B-L4S5I-IOT01A](https://www.st.com/en/evaluation-tools/b-l4s5i-iot01a.html)
    * [NUCLEO-L4P5ZG](https://www.st.com/en/evaluation-tools/nucleo-l4p5zg.html)
    * [NUCLEO-L4R5ZI](https://www.st.com/en/evaluation-tools/nucleo-l4r5zi.html)
    * [NUCLEO-L4R5ZI-P](https://www.st.com/en/evaluation-tools/nucleo-l4r5zi-p.html)
    * [NUCLEO-L412KB](https://www.st.com/en/evaluation-tools/nucleo-l412kb.html)
    * [NUCLEO-L412RB-P](https://www.st.com/en/evaluation-tools/nucleo-l412rb-p.html)
    * [NUCLEO-L432KC](https://www.st.com/en/evaluation-tools/nucleo-l432kc.html)
    * [NUCLEO-L433RC-P](https://www.st.com/en/evaluation-tools/nucleo-l433rc-p.html)
    * [NUCLEO-L452RE](https://www.st.com/en/evaluation-tools/nucleo-l452re.html)
    * [NUCLEO-L452RE-P](https://www.st.com/en/evaluation-tools/nucleo-l452re-p.html)
    * [NUCLEO-L476RG](https://www.st.com/en/evaluation-tools/nucleo-l476rg.html)
    * [NUCLEO-L496ZG](https://www.st.com/en/evaluation-tools/nucleo-l496zg.html)
    * [NUCLEO-L496ZG-P](https://www.st.com/en/evaluation-tools/nucleo-l496zg-p.html)
    * [STM32L4R9I-EVAL](https://www.st.com/en/evaluation-tools/stm32l4r9i-eval.html)
    * [STM32L476G-EVAL](https://www.st.com/en/evaluation-tools/stm32l476g-eval.html)

## Troubleshooting

Please refer to the [CONTRIBUTING.md](CONTRIBUTING.md) guide.
