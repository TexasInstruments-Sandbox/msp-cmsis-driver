<div align="center">

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="./TI-Logo-White.svg" width="300">
  <img alt="Texas Instruments Logo" src="./TI-Logo-Color.svg" width="300">
</picture>

# MSP-CMSIS-DRIVER
This repository contains TI's MSP-CMSIS-DRIVER package.

[Summary](#summary) | [Features](#features) | [Background](#background) | [Related Repos](#related-repos) | [Setup Instructions](#setup-instructions) | [Build Instructions](#build-instructions) | [Supported Devices](#supported-devices) | [Specification](#specification) | [Known Issues](#known-issues) | [Licensing](#licensing) | [Contributions](#contributions) | [Developer Resources](developer-resources)
</div>

## Summary

This package provides a CMSIS-DRIVER inteface to peripherals on [TI MSPM0 microcontrollers](https://www.ti.com/microcontrollers-mcus-processors/arm-based-microcontrollers/arm-cortex-m0-mcus/overview.html) based on Arm® Cortex®-M0+ processors.

## Features

This package currently supports the following CMSIS-Driver modules:
- Driver_USART (configurable with DMA or interrupt driven I/O)
- Driver_I2C (configurable with DMA or interrupt driven I/O)
- Driver_SPI (configurable with DMA or interrupt driven I/O)
- Driver_GPIO

The package is developed to be compatible with [CMSIS-DRIVER 2.10.0](https://arm-software.github.io/CMSIS_6/main/Driver/index.html).

## Background
This package is currently released to TI's GitHub sandbox for evaluation.

### Related Repos

[MSPM0-SDK repository (software development kit for MSPM0 microcontrollers)](https://github.com/TexasInstruments/mspm0-sdk)

[CMSIS-Driver repository](https://github.com/ARM-software/CMSIS-Driver)

## Setup Instructions

This software package was developed & tested against the following dependent components:
- TI Code Composer Studio v20 IDE or greater ([view on TI.com](https://www.ti.com/ccstudio))
- TI Arm Clang 4.0.x (or greater), included in Code Composer Studio or available from TI separately ([view on TI.com](https://www.ti.com/tool/download/ARM-CGT-CLANG/)) 
- TI MSPM0-SDK 2.04.00.06 (or greater) ([view on TI.com](https://www.ti.com/tool/MSPM0-SDK) or [view on GitHub](https://github.com/TexasInstruments/mspm0-sdk))

Currently, CMSIS-Pack delivery is not supported.  This is planned to be added in future verions of this package to ease integration.

### Package Contents

The repository contains the following directories:

- /source
    - /CMSIS/Drivers (These are the Arm standard interface files)
        - Driver_Common.h
        - Driver_USART.h
        - Driver_I2C.h
        - Driver_SPI.h
        - Driver_GPIO.h
    - /Drivers (These are the TI implementation back end source files)
        - /V0
            - Driver_USART_MSP_Priv.h
            - Driver_USART_MSP_Priv.c
            - Driver_I2C_MSP_Priv.h
            - Driver_I2C_MSP_Priv.c
            - Driver_SPI_MSP_Priv.h
            - Driver_SPI_MSP_Priv.c
            - Driver_GPIO_MSP_Priv.h
            - Driver_GPIO_MSP_Priv.c
    - /Drivers_Interface (These are the TI implementation front end source files)
        - Driver_Common_MSP.h
        - Driver_USART_MSP.h
        - Driver_USART_MSP.c
        - Driver_I2C_MSP.h
        - Driver_I2C_MSP.c
        - Driver_SPI_MSP.h
        - Driver_SPI_MSP.c
        - Driver_GPIO_MSP.h
        - Driver_GPIO_MSP.c
- /examples (These are the code examples for the LaunchPad EVMs and Code Composer Studio)
    - /nortos
        - /LP_MSPM0G3507
            - /cmsis_driver_uart
                - /cmsis-driver-usart-echo (This is the USART echo code example for MSPM0G3507)
                - /cmsis-driver-usart-loopback-dma (This is the USART dual-driver / DMA code example for MSPM0G3507)
            - /cmsis_driver_i2c
                - /cmsis-driver-i2c-transmit-receive (This is the I2C transmit and receieve code example for MSPM0G3507)
                - /cmsis-driver-i2c-transmit-receive-dma (This is the I2C transmit and receieve w/ DMA code example for MSPM0G3507)
            - /cmsis_driver_spi
                - /cmsis-driver-spi-controller (This is the SPI controller code example for MSPM0G3507)
                - /cmsis-driver-spi-controller-dma (This is the SPI controller w/ DMA code example for MSPM0G3507)
                - /cmsis-driver-spi-target (This is the SPI target code example for MSPM0G3507)
                - /cmsis-driver-spi-target-dma (This is the SPI target w/ DMA code example for MSPM0G3507)
            -/cmsis_driver_gpio
                -/cmsis-driver-gpio-button-toggle (Example LED toggle on each button press)
        - /LP_MSPM0G3519
            - /cmsis_driver_uart
                - /cmsis-driver-usart-echo (This is the USART echo code example for MSPM0G3507)
                - /cmsis-driver-usart-loopback-dma (This is the USART dual-driver / DMA code example for MSPM0G3519)
            - /cmsis_driver_i2c
                - /cmsis-driver-i2c-transmit-receive (This is the I2C transmit and receieve code example for MSPM0G3519)
                - /cmsis-driver-i2c-transmit-receive-dma (This is the I2C transmit and receieve w/ DMA code example for MSPM0G3519)
            - /cmsis_driver_spi
                - /cmsis-driver-spi-controller (This is the SPI controller code example for MSPM0G3519)
                - /cmsis-driver-spi-controller-dma (This is the SPI controller w/ DMA code example for MSPM0G3519)
                - /cmsis-driver-spi-target (This is the SPI target code example for MSPM0G3519)
                - /cmsis-driver-spi-target-dma (This is the SPI target w/ DMA code example for MSPM0G3519)
            -/cmsis_driver_gpio
                -/cmsis-driver-gpio-button-toggle (Example LED toggle on each button press)

### Quick Start

Follow the steps below to set up an environment to import, build, and run one of the included code examples.

1. Download and install TI Code Composer Studio with support for MSPM0 microcontrollers.
2. Download and install (or clone from GitHub) the MSPM0-SDK (software development kit for MSPM0 microcontrollers).
3. In TI Code Composer Studio, import a code example such as the CMSIS-DRIVER USART Echo code example by selecting "File -> Import Project" in Code Composer Studio and browsing to the example project for the desired target hardware platform.
4. Build the code example in Code Composer Studio.
5. Connect a serial terminal to the LaunchPad XDS-110 UART back channel COM port.  The device will communicate with the host computer using its UART0 peripheral on PA10/PA11, which by default connects to the TI XDS-110 emulator (virtual COM port).  A configuration of 115200B 8N1 is used.
6. Launch a debug session in Code Composer Studio.
7. The device will send a welcome banner string to the serial communication terminal.  It will then echo characters received in the terminal back to the terminal (in the case of the USART echo example).

Quick start guides for using MSPM0 with Code Composer Studio are [available on TI.com](https://dev.ti.com/tirex/explore/node?node=A__AN.wVFo67g0a4GchDji01A__MSPM0-SDK__a3PaaoK__LATEST).

## Build Instructions

The MSP-CMSIS-DRIVER package is delivered as source only.  Build of the drivers takes place as a part of the targeted project build overall.  A static configuration file is required to be set up to define static parameters.  All other parameters are configured through the CMSIS-DRIVER application programming interface.

### CMSIS-Driver Static Configuration Procedures

The MSP-CMSIS-DRIVER modules do not require any configuration in the SysConfig visual configuration tool.  The static properties of each driver instance must be configured in a file named *Driver_Config_MSP.h*"* and subsequently included in the build include path.

#### Driver_USART Static Configuration

The Driver_USART module requires the definitions below to be placed in the *Driver_Config_MSP.h* file for each instance.  The code examples already include this file.
These definitions specify the static driver configuration parameters which are not exposed through runtime APIs.

- TX pin mux index and port function selection (set unused functions to DRIVER_IO_MSP_NONE value).
- RX pin mux index and port function selection (set unused functions to DRIVER_IO_MSP_NONE value)
- RTS pin mux index and port function selection (set unused functions to DRIVER_IO_MSP_NONE value)
- CTS pin mux index and port function selection (set unused functions to DRIVER_IO_MSP_NONE value)
- Clock selection and frequency
- TX DMA hardware controller instance, channel, trigger (optional)
- RX DMA hardware controller instance, channel, trigger (optional)

Two configurations are shown in an example *Driver_Config_MSP.h* below, one for USART0 and a second for USART1.  The USART0 configuration does not configure DMA channels for TX/RX (Note that the TX_DMA_xx and RX_DMA_xx values are set to NONE labels).  As such, USART0 will use interrupt driven input/output.  The USART1 configuration is configured to use the DMA for TX/RX (note that the TX_DMA_xx and RX_DMA_xx values are set to valid parameters).  As such, USART1 will use DMA driven input/output.  The code examples only send data on USART0, but USART1 static configuration is also included and the code examples can be easily modified to initialize and use USART1 without needing to modify the static configuration.

To add more instances, simply copy and past the configuration block and update it for the parameters used by USART2, USART3, and so forth as required.  The device datasheet may be referred to for identifying pin configurations (IOMUX PINCM, PF).  If the peripheral clock sources/frequencies are updated, note that the CLOCK_FREQ parameter must too be updated to ensure correct baud rate calculations.

```
#include <ti/devices/msp/msp.h>
#include <Driver_USART_MSP.h>

/* USART Driver Configuration Options */

/* Driver_USART0 Configuration (Maps to MSP hardware UART0 peripheral) */
#define DRIVER_CONFIG_HAS_USART0 (1)
#if (DRIVER_CONFIG_HAS_USART0==1) && defined(UART0_BASE)
#define DRIVER_USART0_TX_PINCM         (IOMUX_PINCM21)
#define DRIVER_USART0_TX_PF            (IOMUX_PINCM21_PF_UART0_TX)
#define DRIVER_USART0_RX_PINCM         (IOMUX_PINCM22)
#define DRIVER_USART0_RX_PF            (IOMUX_PINCM22_PF_UART0_RX)
#define DRIVER_USART0_RTS_PINCM        (IOMUX_PINCM19)
#define DRIVER_USART0_RTS_PF           (IOMUX_PINCM19_PF_UART0_RTS)
#define DRIVER_USART0_CTS_PINCM        (IOMUX_PINCM20)
#define DRIVER_USART0_CTS_PF           (IOMUX_PINCM20_PF_UART0_CTS)
#define DRIVER_USART0_CLOCK_SEL        (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_USART0_CLOCK_FREQ       (32000000)
#define DRIVER_USART0_TX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART0_TX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART0_TX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#define DRIVER_USART0_RX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART0_RX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART0_RX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#endif

/* Driver_USART1 Configuration (Maps to MSP hardware UART1 peripheral) */
#define DRIVER_CONFIG_HAS_USART1 (1)
#if (DRIVER_CONFIG_HAS_USART1==1) && defined(UART1_BASE)
#define DRIVER_USART1_TX_PINCM         (IOMUX_PINCM23)
#define DRIVER_USART1_TX_PF            (IOMUX_PINCM23_PF_UART1_TX)
#define DRIVER_USART1_RX_PINCM         (IOMUX_PINCM24)
#define DRIVER_USART1_RX_PF            (IOMUX_PINCM24_PF_UART1_RX)
#define DRIVER_USART1_RTS_PINCM        (IOMUX_PINCM16)
#define DRIVER_USART1_RTS_PF           (IOMUX_PINCM16_PF_UART1_RTS)
#define DRIVER_USART1_CTS_PINCM        (IOMUX_PINCM15)
#define DRIVER_USART1_CTS_PF           (IOMUX_PINCM15_PF_UART1_CTS)
#define DRIVER_USART1_CLOCK_SEL        (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_USART1_CLOCK_FREQ       (32000000)
#define DRIVER_USART1_TX_DMA_HW        (DMA)
#define DRIVER_USART1_TX_DMA_CH        (0U)
#define DRIVER_USART1_TX_DMA_TRIG      (DMA_UART1_TX_TRIG)
#define DRIVER_USART1_RX_DMA_HW        (DMA)
#define DRIVER_USART1_RX_DMA_CH        (1U)
#define DRIVER_USART1_RX_DMA_TRIG      (DMA_UART1_RX_TRIG)
#endif
```
To include MSP-CMSIS-DRIVER software in another project, all that is required is to bring in the /msp-cmsis-driver directory to the project, and the corresponding include paths, and add a Driver_Config_MSP.h to the project (example shown above) to provide the static configurations for the driver instances.

#### Driver_I2C Static Configuration

The Driver_I2C module requires the definitions below to be placed in the *Driver_Config_MSP.h* file for each instance.  The code examples already include this file.
These definitions specify the static driver configuration parameters which are not exposed through runtime APIs.

- SCL pin mux index and port function selection 
- SDA pin mux index and port function selection
- Clock selection and frequency
- Transmit DMA hardware instance, channel, trigger (optional)
- Receive DMA hardware instance, channel, trigger (optional)

Two configurations are shown in an example *Driver_Config_MSP.h* below, one for I2C0 and a second for I2C1.  The I2C0 configuration does not configure DMA channels for transmit and receive.  As such, I2C0 will use interrupt driven input/output.  The I2C1 configuration is configured to use the DMA for transmit and receive. As such, I2C1 will use DMA driven input/output.  

To add more instances, simply copy and past the configuration block and update it for the parameters used by the desired I2C instance.  The device datasheet may be referred to for identifying pin configurations (IOMUX PINCM, PF).  If the peripheral clock sources/frequencies are updated, note that the CLOCK_FREQ parameter must too be updated to ensure correct bus speed calculations.

```
/* I2C Driver Configuration Options */

/* Driver_I2C0 Configuration (Maps to MSP hardware I2C0 peripheral) */
#define DRIVER_CONFIG_HAS_I2C0 (1)
#if (DRIVER_CONFIG_HAS_I2C0==1) && defined(I2C0_BASE)
#define DRIVER_I2C0_SDA_PINCM               (IOMUX_PINCM1)
#define DRIVER_I2C0_SDA_PF                  (IOMUX_PINCM1_PF_I2C0_SDA)
#define DRIVER_I2C0_SCL_PINCM               (IOMUX_PINCM2)
#define DRIVER_I2C0_SCL_PF                  (IOMUX_PINCM2_PF_I2C0_SCL)
#define DRIVER_I2C0_CLOCK_SEL               (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_I2C0_CLOCK_FREQ              (32000000U)
#define DRIVER_I2C0_TRANSMIT_DMA_HW         (DRIVER_DMA_HW_NONE)
#define DRIVER_I2C0_TRANSMIT_DMA_CH         (DRIVER_DMA_CH_NONE)
#define DRIVER_I2C0_TRANSMIT_DMA_TRIG       (DRIVER_DMA_TRIG_NONE)
#define DRIVER_I2C0_RECEIVE_DMA_HW          (DRIVER_DMA_HW_NONE)
#define DRIVER_I2C0_RECEIVE_DMA_CH          (DRIVER_DMA_CH_NONE)
#define DRIVER_I2C0_RECEIVE_DMA_TRIG        (DRIVER_DMA_TRIG_NONE)
#endif

/* Driver_I2C1 Configuration (Maps to MSP hardware I2C1 peripheral) */
#define DRIVER_CONFIG_HAS_I2C1 (1)
#if (DRIVER_CONFIG_HAS_I2C1==1) && defined(I2C1_BASE)
#define DRIVER_I2C1_SDA_PINCM               (IOMUX_PINCM16)
#define DRIVER_I2C1_SDA_PF                  (IOMUX_PINCM16_PF_I2C1_SDA)
#define DRIVER_I2C1_SCL_PINCM               (IOMUX_PINCM15)
#define DRIVER_I2C1_SCL_PF                  (IOMUX_PINCM15_PF_I2C1_SCL)
#define DRIVER_I2C1_CLOCK_SEL               (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_I2C1_CLOCK_FREQ              (32000000U)
#define DRIVER_I2C1_TRANSMIT_DMA_HW         (DMA)
#define DRIVER_I2C1_TRANSMIT_DMA_CH         (2U)
#define DRIVER_I2C1_TRANSMIT_DMA_TRIG       (DMA_I2C1_TX_TRIG)
#define DRIVER_I2C1_RECEIVE_DMA_HW          (DMA)
#define DRIVER_I2C1_RECEIVE_DMA_CH          (3U)
#define DRIVER_I2C1_RECEIVE_DMA_TRIG        (DMA_I2C1_RX_TRIG)
#endif
```
To include MSP-CMSIS-DRIVER software in another project, all that is required is to bring in the /msp-cmsis-driver directory to the project, and the corresponding include paths, and add a Driver_Config_MSP.h to the project (example shown above) to provide the static configurations for the driver instances.

#### Driver_SPI Static Configuration

The Driver_SPI module requires the definitions below to be placed in the *Driver_Config_MSP.h* file for each instance.  The code examples already include this file.
These definitions specify the static driver configuration parameters which are not exposed through runtime APIs.

- MOSI pin mux index and port function selection 
- MISO pin mux index and port function selection
- SS pin mux index and port function selection 
- SS GPIO Port location (required for SW controlled SS)
- SS GPIO Pin Number (required for SW controlled SS)
- SCLK pin mux index and port function selection
- Clock selection and frequency
- Transmit DMA hardware instance, channel, trigger (optional)
- Receive DMA hardware instance, channel, trigger (optional)
    - Note that for a Transfer with DMA, it is required that both the transmit and receive DMA are available. 

When calling initialize for a given driver, since it is not yet known what mode the driver will be configured in (master or slave), no pin configuration will occur and will instead be left in the default MCU state. Pin configuration is deferred to Control with control parameter ARM_SPI_MODE_MASTER or AMR_SPI_MODE_SLAVE. 

Two configurations are shown in an example *Driver_Config_MSP.h* below, one for SPI0 and a second for SPI1.  The SPI0 configuration does not configure DMA channels for transmit and receive.  As such, SPI0 will use interrupt driven input/output.  The SPI1 configuration is configured to use the DMA for transmit and receive. As such, SPI1 will use DMA driven input/output.

To add more instances, simply copy and past the configuration block and update it for the parameters used by the desired SPI instance.  The device datasheet may be referred to for identifying pin configurations (IOMUX PINCM, PF).  If the peripheral clock sources/frequencies are updated, note that the CLOCK_FREQ parameter must too be updated to ensure correct bus speed calculations.

```
/* I2C Driver Configuration Options */

/* Driver_SPI0 Configuration (Maps to MSP hardware SPI0 peripheral) */
#define DRIVER_CONFIG_HAS_SPI0 (1)
#if (DRIVER_CONFIG_HAS_SPI0==1) && defined(SPI0_BASE)
#define DRIVER_SPI0_MOSI_PINCM              (IOMUX_PINCM43)
#define DRIVER_SPI0_MOSI_PF                 (IOMUX_PINCM43_PF_SPI0_PICO)
#define DRIVER_SPI0_MISO_PINCM              (IOMUX_PINCM21)
#define DRIVER_SPI0_MISO_PF                 (IOMUX_PINCM21_PF_SPI0_POCI)
#define DRIVER_SPI0_SS                      (DL_SPI_CHIP_SELECT_0)
#define DRIVER_SPI0_SS_GPIO_PORT            (GPIOA)
#define DRIVER_SPI0_SS_PIN_NUMBER           (DL_GPIO_PIN_8)
#define DRIVER_SPI0_SS_PINCM                (IOMUX_PINCM19)
#define DRIVER_SPI0_SS_PF                   (IOMUX_PINCM19_PF_SPI0_CS0)
#define DRIVER_SPI0_SCLK_PINCM              (IOMUX_PINCM22)
#define DRIVER_SPI0_SCLK_PF                 (IOMUX_PINCM22_PF_SPI0_SCLK)
#define DRIVER_SPI0_CLOCK_SEL               (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_SPI0_CLOCK_FREQ              (32000000U)
#define DRIVER_SPI1_TRANSMIT_DMA_HW         (DRIVER_DMA_HW_NONE)
#define DRIVER_SPI1_TRANSMIT_DMA_CH         (DRIVER_DMA_CH_NONE)
#define DRIVER_SPI1_TRANSMIT_DMA_TRIG       (DRIVER_DMA_TRIG_NONE)
#define DRIVER_SPI1_RECEIVE_DMA_HW          (DRIVER_DMA_HW_NONE)
#define DRIVER_SPI1_RECEIVE_DMA_CH          (DRIVER_DMA_CH_NONE)
#define DRIVER_SPI1_RECEIVE_DMA_TRIG        (DRIVER_DMA_TRIG_NONE)
#endif

/* Driver_spi1 Configuration (Maps to MSP hardware SPI1 peripheral) */
#define DRIVER_CONFIG_HAS_SPI1 (1)
#if (DRIVER_CONFIG_HAS_SPI1==1) && defined(SPI1_BASE)
#define DRIVER_SPI1_MOSI_PINCM              (IOMUX_PINCM25)
#define DRIVER_SPI1_MOSI_PF                 (IOMUX_PINCM25_PF_SPI1_PICO)
#define DRIVER_SPI1_MISO_PINCM              (IOMUX_PINCM24)
#define DRIVER_SPI1_MISO_PF                 (IOMUX_PINCM24_PF_SPI1_POCI)
#define DRIVER_SPI1_SS                      (DL_SPI_CHIP_SELECT_0)
#define DRIVER_SPI1_SS_GPIO_PORT            (GPIOA)
#define DRIVER_SPI1_SS_PIN_NUMBER           (DL_GPIO_PIN_2)
#define DRIVER_SPI1_SS_PINCM                (IOMUX_PINCM7)
#define DRIVER_SPI1_SS_PF                   (IOMUX_PINCM7_PF_SPI1_CS0)
#define DRIVER_SPI1_SCLK_PINCM              (IOMUX_PINCM26)
#define DRIVER_SPI1_SCLK_PF                 (IOMUX_PINCM26_PF_SPI1_SCLK)
#define DRIVER_SPI1_CLOCK_SEL               (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_SPI1_CLOCK_FREQ              (32000000U)
#define DRIVER_SPI1_TRANSMIT_DMA_HW         (DMA)
#define DRIVER_SPI1_TRANSMIT_DMA_CH         (0U)
#define DRIVER_SPI1_TRANSMIT_DMA_TRIG       (DMA_SPI0_TX_TRIG)
#define DRIVER_SPI1_RECEIVE_DMA_HW          (DRIVER_DMA_HW_NONE)
#define DRIVER_SPI1_RECEIVE_DMA_CH          (1U)
#define DRIVER_SPI1_RECEIVE_DMA_TRIG        (DMA_SPI0_RX_TRIG)
#endif
```

#### Driver_GPIO Static Configuration

The Driver_GPIO only requires specifying which GPIO ports will be used in the *Driver_Config_MSP.h* file. An example is shown below. The code examples already include this file.
These definitions specify the static driver configuration parameters which are not exposed through runtime APIs. To see what GPIO ports and pins are available for a certain device,
refer to the datasheet.

All IOMUX values are automatically generated in the driver and configuration is handled within the APIs. To interact with an I/O pin, pass a value for ARM_GPIO_Pin_t corresponding
to the pin number within the specific GPIO port. For instance, to use PA10, pass in 10 to all CMSIS-GPIO APIs.

To use a pin as an input, first call Setup() followed by SetDirection() with ARM_GPIO_INPUT. After this, optionally configure pull resistors or event triggers.

To use a pin as an output, first call Setup() followed by SetDirection() with ARM_GPIO_OUTPUT. Calling SetOutputMode() is optional unless open-drain configuration is specifically desired.

```
/* GPIO Driver Configuration Options */

#define DRIVER_CONFIG_HAS_GPIOA (0)
#define DRIVER_CONFIG_HAS_GPIOB (1)
#define DRIVER_CONFIG_HAS_GPIOC (0)
```

To include MSP-CMSIS-DRIVER software in another project, all that is required is to bring in the /msp-cmsis-driver directory to the project, and the corresponding include paths, and add a Driver_Config_MSP.h to the project (example shown above) to provide the static configurations for the driver instances.

## Supported Devices

The following LaunchPad evaluation kits are currently supported with code examples in this package:
- LP-MSPM0G3507 LaunchPad Evaluation Kit ([view on TI.com](https://www.ti.com/tool/lp-MSPM0G3519))
    - Uses the [MSPM0G3507](https://www.ti.com/product/MSPM0G3507) 80MHz Arm® Cortex®-M0+ MCU with 128KB flash 32KB SRAM 2x4Msps ADC, DAC, 3xCOMP, 2xOPA, CAN-FD, MATHA
- LP-MSPM0G3519 LaunchPad Evaluation Kit ([view on TI.com](https://www.ti.com/tool/lp-MSPM0G3507))
    - Uses the [MSPM0G3519](https://www.ti.com/product/MSPM0G3519) 80 MHz ARM® Cortex®-M0+ MCU with dual-bank 512kB flash, 128kB SRAM, 2xCAN-FD, 2xADC, DAC, COMP

The following LaunchPad evaluation kits are planned to be supported with code examples in future versions of this package:
- LP-MSPM0L1117 LaunchPad Evaluation Kit ([view on TI.com](https://www.ti.com/tool/lp-MSPM0L1117))
    - Uses the [MSPM0L1117](https://www.ti.com/product/MSPM0L1117) 32MHz Arm® Cortex®-M0+ MCU with 128KB dual-bank flash, 16KB SRAM, 12-bit 1.68Msps ADC
- LP-MSPM0L2228 LaunchPad Evaluation Kit ([view on TI.com](https://www.ti.com/tool/lp-MSPM0L2228))
    - Uses the [MSPM0L2228](https://www.ti.com/product/MSPM0L2228) 32MHz Arm® Cortex®-M0+ MCU with 256KB dual-bank flash, 32KB SRAM, 12bit ADC, COMP, LCD, VBAT, PSA

The software is currently tested against the following device variants:
- MSPM0G110x, MSPM0G150x, MSPM0G310x, MSPM0G350x, MSPM0G151x, MSPM0G351x 80MHz Arm® Cortex®-M0+ MCUs

The drivers are expected to work without major modifications on additional MSPM0 devices.

## Specification

The MSP-CMSIS-DRIVER package specifications are provided below for reference.

### Driver Package Specifications
The overall package supports the following capabilities:

| Req ID        | Description             | Summary                                              | Status |
| --------------|-------------------------|------------------------------------------------------|--------|
| MSPSWSDK-5132 | [SPS] CMSIS-Driver peripheral access structure | The MSP-CMSIS-DRIVER modules shall provide a CMSIS-Driver access structure for each hardware instance of the supported peripheral type. | Implemented |
| MSPSWSDK-5133 | [SPS] CMSIS-Driver CMSIS headers | The MSP-CMSIS-DRIVER modules shall use the CMSIS-Driver defined interface header files for the respective modules (ex: Driver_USART.h) as-is without modification. | Implemented |
| MSPSWSDK-5134 | [SPS] CMSIS-Driver peripheral instance pin selection and configuration | The MSP-CMSIS-DRIVER modules shall support configuration of the MSP IOMUX during driver init/deinit. | Implemented |
| MSPSWSDK-5135 | [SPS] CMSIS-Driver peripheral instance clock configuration | The MSP-CMSIS-DRIVER modules shall support configuration of the peripheral module functional clock during driver power control configuration. | Implemented |
| MSPSWSDK-5136 | [SPS] CMSIS-Driver dependencies | The MSP-CMSIS-DRIVER modules shall be based upon DriverLib and not have other dependencies outside of DriverLib, Arm CMSIS includes, and MSP MCU header support files. | Implemented |

### Driver_USART Specifications
The Driver_USART module supports the following capabilities:

| Req ID        | Description             | Summary                                              | Status |
| --------------|-------------------------|------------------------------------------------------|--------|
| MSPSWSDK-5137 | [SPS] CMSIS-Driver UART module interface | The MSP-CMSIS-DRIVER UART module shall implement the USART application interface defined in the CMSIS-Driver USART interface specification. | Implemented |
| MSPSWSDK-5138 | [SPS] CMSIS-Driver USART module feature: CTS | The MSP-CMSIS-DRIVER UART module shall provide an option for using a clear-to-send (CTS) line with CMSIS-Driver defined functionality. | Implemented |
| MSPSWSDK-5139| [SPS] CMSIS-Driver USART module feature: RTS | The MSP-CMSIS-DRIVER UART module shall provide an option for using a ready-to-send (RTS) line with CMSIS-Driver defined functionality. | Implemented |
| MSPSWSDK-5140 | [SPS] CMSIS-Driver USART module feature: baud rate configuration | The MSP-CMSIS-DRIVER UART module shall support baud rate configuration taking a desired baud rate and functional clock frequency as inputs. | Implemented |
| MSPSWSDK-5141 | [SPS] CMSIS-Driver USART module feature: word length configuration | The MSP-CMSIS-DRIVER UART module shall support configurable word lengths of 5, 6, 7, or 8 data bits. Other bit configurations shall return an error. | Implemented |
| MSPSWSDK-5142 | [SPS] CMSIS-Driver USART module feature: parity bits | The MSP-CMSIS-DRIVER UART module shall support configurable parity: no parity, even parity, or odd parity. | Implemented |
| MSPSWSDK-5143 | [SPS] CMSIS-Driver USART module feature: stop bits | The MSP-CMSIS-DRIVER UART module shall support configurable stop bits: one or two stop bits. Other conditions shall return an error. | Implemented |
| MSPSWSDK-5144 | [SPS] CMSIS-Driver USART module feature: transmit completed event | The MSP-CMSIS-DRIVER UART module shall support generation of a transmit completed event which indicates when data has completed transmission on the line. | Implemented |
| MSPSWSDK-5145 | [SPS] CMSIS-Driver USART module feature: receive timeout error event | The MSP-CMSIS-DRIVER UART module shall support generation of a receive error timeout event, using the maximum available hardware timeout interval by default (as a hard coded value). | Implemented |
| MSPSWSDK-5146 | [SPS] CMSIS-Driver USART module feature: framing error event | The MSP-CMSIS-DRIVER UART module shall support generation of a framing error event. | Implemented |
| MSPSWSDK-5147 | [SPS] CMSIS-Driver USART module feature: parity error event | The MSP-CMSIS-DRIVER UART module shall support generation of a parity error event. | Implemented |
| MSPSWSDK-5148 | [SPS] CMSIS-Driver USART module feature: receive overflow error event | The MSP-CMSIS-DRIVER UART module shall support generation of a receive overflow error event. | Implemented |
| MSPSWSDK-5149 | [SPS] CMSIS-Driver USART module feature: receive complete event | The MSP-CMSIS-DRIVER UART module shall support generation of a receive complete event, indicating when the requested number of words has been received. | Implemented |
| MSPSWSDK-5150 | [SPS] CMSIS-Driver USART module feature: send complete event | The MSP-CMSIS-DRIVER UART module shall support generation of a transmit complete event, indicating when the requested number of words has been loaded to the UART hardware for subsequent transmission. | Implemented |
| MSPSWSDK-5151 | [SPS] CMSIS-Driver UART module feature: asynchronous mode | The MSP-CMSIS-DRIVER UART module shall support asynchronous serial communication in RX, TX, or RX/TX modes. | Implemented |
| MSPSWSDK-5152 | [SPS] CMSIS-Driver UART module interrupt driven I/O | The MSP-CMSIS-DRIVER UART module shall provide an option for using interrupt driven non-blocking input/output for transmit and receive operations, configured statically on a per-instance basis. | Implemented |
| MSPSWSDK-5153 | [SPS] CMSIS-Driver USART module DMA driven I/O | The MSP-CMSIS-DRIVER UART module shall provide an option for using DMA driven non-blocking input/output for transmit and receive operations, configured statically on a per-instance basis. | Implemented |
| MSPSWSDK-5154 | [SPS] CMSIS-Driver USART module loopback code example with DMA | The MSP-CMSIS-Driver product shall include a code example demonstrating USART in a loopback configuration with an external wire for loopback, demonstrating setup and teardown of the driver and use of DMA. | Implemented |
| MSPSWSDK-5245 | [SPS] CMSIS-Driver USART module back channel UART example | The MSP-CMSIS-DRIVER product shall include a code example for USART transmit / receive in a back channel configuration with the XDS-110. | Implemented |

### Driver_I2C Specifications
The Driver_I2C module supports the following capabilities:

| Req ID        | Description             | Summary                                              | Status |
| --------------|-------------------------|------------------------------------------------------|--------|
| MSPSWSDK-5427 | [SPS] CMSIS-Driver I2C module feature: Controller mode | The MSP-CMSIS-DRIVER I2C module shall provide an option for operating in controller mode. | Implemented |
| MSPSWSDK-5428 | [SPS] CMSIS-Driver I2C module feature: Target mode | The MSP-CMSIS-DRIVER I2C module shall provide an option for operating in target mode. | Implemented |
| MSPSWSDK-5429 | [SPS] CMSIS-Driver I2C module feature: Transfer done event | The MSP-CMSIS-DRIVER I2C module shall provide a transfer done event fired when a transfer completes. | Implemented |
| MSPSWSDK-5430 | [SPS] CMSIS-Driver I2C module feature: Transfer Incomplete event | The MSP-CMSIS-DRIVER I2C module shall provide a transfer imcompleted event fired when a transfer does not complete successfully. | Implemented |
| MSPSWSDK-5431 | [SPS] CMSIS-Driver I2C module feature: Arbitration loss event | The MSP-CMSIS-DRIVER I2C module shall provide an arbitration lost event fired when bus abitration is lost. | Implemented |
| MSPSWSDK-5432 | [SPS] CMSIS-Driver I2C module feature: Bus error event | The MSP-CMSIS-DRIVER I2C module shall provide a bus error event fired when there is an I2C bus error.| Implemented |
| MSPSWSDK-5433 | [SPS] CMSIS-Driver I2C module feature: General call event | The MSP-CMSIS-DRIVER I2C module shall provide a general call event fired when a target is addressed via general call. | Implemented |
| MSPSWSDK-5434 | [SPS] CMSIS-Driver I2C module feature: Bus clear event| The MSP-CMSIS-DRIVER I2C module shall provide a bus clear event fired when the I2C bus is cleared via 9 clock pulses. | Implemented |
| MSPSWSDK-5435 | [SPS] CMSIS-Driver I2C module feature: Address NACK event | The MSP-CMSIS-DRIVER I2C module shall provide a NACK event fired when the controller receives a NACK response.| Implemented |
| MSPSWSDK-5436 | [SPS] CMSIS-Driver I2C module feature: Slave transmit event | The MSP-CMSIS-DRIVER I2C module shall provide a slave transmit event fired when the target is addressed to write but SlaveTransmit was not called. | Implemented |
| MSPSWSDK-5437 | [SPS] CMSIS-Driver I2C module feature: Slave receive event | The MSP-CMSIS-DRIVER I2C module shall provide a slave receive event fired when the target is addressed to read but SlaveReceive was not called.| Implemented |
| MSPSWSDK-5438 | [SPS] CMSIS-Driver I2C module feature: Bus speed selection | The MSP-CMSIS-DRIVER I2C module shall provide an option of selecting different bus speeds for the I2C driver. | Implemented |
| MSPSWSDK-5439 | [SPS] CMSIS-Driver I2C module feature: I2C own address selection | The MSP-CMSIS-DRIVER I2C module shall provide an option for a target to set its own address.| Implemented |
| MSPSWSDK-5440 | [SPS] CMSIS-Driver I2C module feature: 10-bit address mode | The MSP-CMSIS-DRIVER I2C module shall provide an option for a target to use a 10-bit addressing mode. | Implemented |
| MSPSWSDK-5441 | [SPS] CMSIS-Driver I2C module feature: DMA driven transfers | The MSP-CMSIS-DRIVER I2C module shall provide an option for using DMA-driven non-blocking transfers. | Implemented |
| MSPSWSDK-5442 | [SPS] CMSIS-Driver I2C Controller and Target transfer example | The MSP-CMSIS-DRIVER product shall include an example showcasing transfers between controller and target. | Implemented |
| MSPSWSDK-5446 | [SPS] CMSIS-Driver I2C Controller and Target w/ DMA transfer example | The MSP-CMSIS-DRIVER product shall include an example showcasing transfers with DMA between controller and target. | Implemented |

## Driver_SPI Specifications

The Driver_SPI module supports the following capabilities:

| Req ID        | Description             | Summary                                              | Status |
| --------------|-------------------------|------------------------------------------------------|--------|
| MSPSWSDK-5677 | [SPS] CMSIS-Driver SPI module feature: Controller mode | The MSP-CMSIS-DRIVER SPI module shall provide an option for operating in controller mode. | Implemented |
| MSPSWSDK-5678 | [SPS] CMSIS-Driver SPI module feature: Target mode | The MSP-CMSIS-DRIVER SPI module shall provide an option for operating in target mode. | Implemented |
| MSPSWSDK-5679 | [SPS] CMSIS-Driver SPI module feature: Transfer done event | The MSP-CMSIS-DRIVER SPI module shall provide a transfer done event fired when a transfer completes. | Implemented |
| MSPSWSDK-5680 | [SPS] CMSIS-Driver SPI module feature: Data lost event | The MSP-CMSIS-DRIVER SPI module shall provide a data lost event fired when a slave device receives data but no operation was started. | Implemented |
| MSPSWSDK-5681 | [SPS] CMSIS-Driver SPI module feature: Frame format selection | The MSP-CMSIS-DRIVER SPI module shall provide an option to select the frame format to be used in transmission.| Implemented |
| MSPSWSDK-5682 | [SPS] CMSIS-Driver SPI module feature: Bus speed selection | The MSP-CMSIS-DRIVER SPI module shall provide an option to select the bus speed. | Implemented |
| MSPSWSDK-5683 | [SPS] CMSIS-Driver SPI module feature: Get Bus Speed | The MSP-CMSIS-DRIVER SPI module shall provide an option to read the current bus speed. | Implemented |
| MSPSWSDK-5684 | [SPS] CMSIS-Driver SPI module feature: Controller Chip Select Behavior | The MSP-CMSIS-DRIVER SPI module shall provide options to control how the controller uses the chip select line (software, hardware, none). | Implemented |
| MSPSWSDK-5685 | [SPS] CMSIS-Driver SPI module feature: Target Chip Select Behavior | The MSP-CMSIS-DRIVER SPI module shall provide to control how the target uses the chip select line (software, hardware, none). | Implemented |
| MSPSWSDK-5686 | [SPS] CMSIS-Driver SPI module feature: Data size | The MSP-CMSIS-DRIVER SPI module shall provide to control how many bits are in one frame. | Implemented |
| MSPSWSDK-5687 | [SPS] CMSIS-Driver SPI module feature: Default TX Value | The MSP-CMSIS-DRIVER SPI module shall provide to control what default TX value is used. | Implemented |
| MSPSWSDK-5688 | [SPS] CMSIS-Driver SPI module feature: DMA Driver Transfer | The MSP-CMSIS-DRIVER SPI module shall provide an option to use DMA driven transfers. | Implemented |
| MSPSWSDK-5689 | [SPS] CMSIS-Driver SPI Controller and Target transfer example | The MSP-CMSIS-DRIVER product shall include an example showcasing transfers between controller and target. | Implemented |
| MSPSWSDK-5690 | [SPS] CMSIS-Driver SPI Controller and Target w/ DMA transfer example | The MSP-CMSIS-DRIVER product shall include an example showcasing transfers with DMA between controller and target. | Implemented |

## Driver_GPIO Specifications

The Driver_GPIO module supports the following capabilities:

| Req ID        | Description             | Summary                                              | Status |
| --------------|-------------------------|------------------------------------------------------|--------|
| MSPSWSDK-5791 | [SPS] CMSIS-Driver GPIO module feature: Direction Control | The MSP-CMSIS-DRIVER GPIO module shall provide an option for configuring the direction of a pin. | Implemented |
| MSPSWSDK-5792 | [SPS] CMSIS-Driver GPIO module feature: Output Mode Control | The MSP-CMSIS-DRIVER GPIO module shall allow configuration of output mode on a pin (push-pull or open-drain). | Implemented |
| MSPSWSDK-5793 | [SPS] CMSIS-Driver GPIO module feature: Resistor Control | The MSP-CMSIS-DRIVER GPIO module shall allow configuration of the internal resistor on a pin for both input and output. | Implemented |
| MSPSWSDK-5794 | [SPS] CMSIS-Driver GPIO module feature: Event Trigger Control | The MSP-CMSIS-DRIVER GPIO module shall allow configuration of edge-based interrupts on a pin. | Implemented |
| MSPSWSDK-5795 | [SPS] CMSIS-Driver GPIO module feature: Output Control | The MSP-CMSIS-DRIVER GPIO module shall allow control of output on a pin. | Implemented |
| MSPSWSDK-5796 | [SPS] CMSIS-Driver GPIO module feature: Read Input | The MSP-CMSIS-DRIVER GPIO shall allow reading the input of a pin. | Implemented |
| MSPSWSDK-5797 | [SPS] CMSIS-Driver GPIO module feature: Trigger Event | The MSP-CMSIS-DRIVER GPIO shall provide a trigger edge event on a gpio pin via callback. | Implemented |

## Licensing
[License information](./LICENSE.md)

## Contributions

This repository is not currently accepting community contributions.

---
## Developer Resources
[TI E2E™ design support forums](https://e2e.ti.com) | [Learn about software development at TI](https://www.ti.com/design-development/software-development.html) | [Training Academies](https://www.ti.com/design-development/ti-developer-zone.html#ti-developer-zone-tab-1) | [TI Developer Zone](https://dev.ti.com/)
