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
 *  @file       Driver_I2C_MSP.h
 *  @brief      MSP I2C CMSIS-Driver for Cortex-M based devices with
 *              I2C peripheral instances.
 * 
 *  The CMSIS-Drivers I2C driver allows for simple, non-blocking I2C
 *  transmit and receive communication via a high-level, Arm-standardized API.
 *  This driver is developed to be compatible with the Arm CMSIS-Driver
 *  specification.
 * 
 *
 ******************************************************************************
 */

#ifndef DRIVER_I2C_MSP_H_
#define DRIVER_I2C_MSP_H_
 
#ifdef  __cplusplus
extern "C"
{
#endif

#include <ti/devices/msp/msp.h>
#include <Driver_I2C.h>
#include "Driver_Config_MSP.h"
 
/*
 * The structure declarations below are the interfaces to
 * the driver for each respective I2C instance.
 */

#if defined(I2C0_BASE) && defined(DRIVER_CONFIG_HAS_I2C0)
extern ARM_DRIVER_I2C Driver_I2C0;
#endif

#if defined(I2C1_BASE) && defined(DRIVER_CONFIG_HAS_I2C1)
extern ARM_DRIVER_I2C Driver_I2C1;
#endif

#if defined(I2C2_BASE) && defined(DRIVER_CONFIG_HAS_I2C2)
extern ARM_DRIVER_I2C Driver_I2C2;
#endif

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_I2C_MSP_H_ */
