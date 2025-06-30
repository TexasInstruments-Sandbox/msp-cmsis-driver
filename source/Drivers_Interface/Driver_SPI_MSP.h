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
 *  @file       Driver_SPI_MSP.h
 *  @brief      MSP SPI CMSIS-Driver for Cortex-M based devices with
 *              SPI peripheral instances.
 * 
 *  The CMSIS-Drivers SPI driver allows for simple, non-blocking SPI
 *  send, receive and transfer communication via a high-level, 
 *  Arm-standardized API. This driver is developed to be compatible with 
 *  the Arm CMSIS-Driver specification. 
 *
 ******************************************************************************
 */

#ifndef DRIVER_SPI_MSP_H_
#define DRIVER_SPI_MSP_H_
 
#ifdef  __cplusplus
extern "C"
{
#endif

#include <ti/devices/msp/msp.h>
#include <Driver_SPI.h>
#include "Driver_Config_MSP.h"
 
/*
 * The structure declarations below are the interfaces to
 * the driver for each respective SPI instance.
 */

#if defined(SPI0_BASE) && defined(DRIVER_CONFIG_HAS_SPI0)
extern ARM_DRIVER_SPI Driver_SPI0;
#endif

#if defined(SPI1_BASE) && defined(DRIVER_CONFIG_HAS_SPI1)
extern ARM_DRIVER_SPI Driver_SPI1;
#endif

#if defined(SPI2_BASE) && defined(DRIVER_CONFIG_HAS_SPI2)
extern ARM_DRIVER_SPI Driver_SPI2;
#endif

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_SPI_MSP_H_ */
