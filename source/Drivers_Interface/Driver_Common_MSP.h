/*
 * Copyright (C) 2024-2025 Texas Instruments Incorporated
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and  
 * limitations under the License.
 */

/*!****************************************************************************
 *  @file       Driver_Common_MSP.h
 *  @brief      CMSIS-Drivers common definitions file for Texas Instruments 
 *              MSP MCUs based on Arm Cortex-M CPUs
 *
 *  TODO: Brief description
 *
 *  <hr>
 ******************************************************************************
 */

#ifndef DRIVER_COMMON_MSP_H_
#define DRIVER_COMMON_MSP_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/*!
 * @brief Definition to use when IO function is not used in the configuration.
 *        Example: USART used for TX only (RX MSP_IO values shall be set 
 *        to this definition to inform the driver that no IO for RX is to
 *        be configured.)
 */
#define DRIVER_IO_MSP_NONE (0xFFU)

/*!
 * @brief Channel definition to use when DMA is not used in the configuration.
 */
#define DRIVER_DMA_HW_NONE (0x00000000U)

/*!
 * @brief Channel definition to use when DMA is not used in the configuration.
 */
#define DRIVER_DMA_CH_NONE (0xFFU)

/*!
 * @brief Trigger definition to use when DMA is not used in the configuration.
 */
#define DRIVER_DMA_TRIG_NONE (0xFFU)

/*!
 * @brief Definitions for an MSP MCU digital input/output 
 *        pin used by a CMSIS-DRIVER.  Specifies the IOMUX
 *        pin control management (PINCM) register index
 *        and the pin function (PF) for the configured function.
 */
typedef struct
{
    /*! IOMUX PINCM register offset (see datasheet for PINCM index for
     *  a given digital input/output pin). */
    uint8_t iomuxPinCtlMgmtRegIndex; 
    /*! IOMUX pin function value (see datasheet for PF value for a 
     *  given peripheral function selection on a given PINCM). */
    uint8_t iomuxPinFunction;
} DRIVER_IO_MSP;

/*! @enum DRIVER_CLK_MSP presents the 3 peripheral clock selections 
 *        which are available to peripherals managed by CMSIS-Drivers. */
typedef enum
{
    /*! Selects BUSCLK as the clock source with BUSCLK sourced from MCLK */
    DRIVER_CLK_MSP_BUSCLK,
    /*! Selects constant 4MHz MFCLK as the clock source */
    DRIVER_CLK_MSP_MFCLK,
    /*! Selects constant 32kHz LFCLK as the clock source */
    DRIVER_CLK_MSP_LFCLK
} DRIVER_CLK_MSP;

/**
 * @brief Stores a pointer to the address of the MSP UART hardware registers.
 *        for the corresponding UART hardware peripheral instance.
 */
typedef DMA_Regs DRIVER_DMA_MSP_HW;

/*!
 * @brief Definitions for an MSP MCU DMA channel used by a CMSIS-DRIVER.
 *        Specifies the DMA channel and trigger source to use.
 */
typedef struct
{
    /*! Driver DMA controller selection */
    DRIVER_DMA_MSP_HW *hw;
    /*! Driver DMA channel selection */
    uint8_t ch;
    /*! Driver DMA trigger selection */
    uint8_t trig;
} DRIVER_DMA_MSP;

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_COMMON_MSP_H_ */
