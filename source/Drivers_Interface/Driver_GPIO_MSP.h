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
 *  @file       Driver_GPIO_MSP.h
 *  @brief      MSP GPIO CMSIS-Driver for Cortex-M based devices with
 *              GPIO peripheral instances.
 * 
 *  The CMSIS-Drivers GPIO allows for input and output operation on a pin level.
 *  This driver is developed to be compatible with the Arm CMSIS-Driver
 *  specification.
 * 
 *
 ******************************************************************************
 */

#ifndef DRIVER_GPIO_MSP_H_
#define DRIVER_GPIO_MSP_H_
 
#ifdef  __cplusplus
extern "C"
{
#endif

#include <ti/devices/msp/msp.h>
#include <Driver_GPIO.h>
#include "Driver_Config_MSP.h"
 
/*
 * The structure declarations below are the interfaces to
 * the driver for each respective GPIO instance.
 */

#if defined(GPIOA_BASE) && defined(DRIVER_CONFIG_HAS_GPIOA)
extern ARM_DRIVER_GPIO Driver_GPIOA;
#endif

#if defined(GPIOB_BASE) && defined(DRIVER_CONFIG_HAS_GPIOB)
extern ARM_DRIVER_GPIO Driver_GPIOB;
#endif

#if defined(GPIOC_BASE) && defined(DRIVER_CONFIG_HAS_GPIOC)
extern ARM_DRIVER_GPIO Driver_GPIOC;
#endif

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_GPIO_MSP_H_ */
