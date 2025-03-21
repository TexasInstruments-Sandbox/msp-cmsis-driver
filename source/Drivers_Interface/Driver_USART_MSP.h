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
 *  @file       Driver_USART_MSP.h
 *  @brief      MSP USART CMSIS-Driver for Cortex-M based devices with
 *              UART main or UART extend peripheral instances.
 * 
 *  The CMSIS-Drivers USART driver allows for simple, non-blocking UART
 *  send and receive communication via a high-level, Arm-standardized API.
 *  This driver is developed to be compatible with the Arm CMSIS-Driver
 *  specification.
 * 
 *  This driver implements asynchronous communication mode via
 *  interrupt-driven input/output.  It does not implement synchronous mode
 *  communication, as the underlying hardware peripheral is asynchronous only.
 *
 ******************************************************************************
 */

#ifndef DRIVER_USART_MSP_H_
#define DRIVER_USART_MSP_H_
 
#ifdef  __cplusplus
extern "C"
{
#endif

#include <ti/devices/msp/msp.h>
#include <Driver_USART.h>
#include "Driver_Config_MSP.h"
 
/*
 * The structure declarations below are the interfaces to
 * the driver for each respective USART instance.
 */
 
#if defined(UART0_BASE) && defined(DRIVER_CONFIG_HAS_USART0)
extern ARM_DRIVER_USART Driver_USART0;
#endif

#if defined(UART1_BASE) && defined(DRIVER_CONFIG_HAS_USART1)
extern ARM_DRIVER_USART Driver_USART1;
#endif

#if defined(UART2_BASE) && defined(DRIVER_CONFIG_HAS_USART2)
extern ARM_DRIVER_USART Driver_USART2;
#endif

#if defined(UART3_BASE) && defined(DRIVER_CONFIG_HAS_USART3)
extern ARM_DRIVER_USART Driver_USART3;
#endif

#if defined(UART4_BASE) && defined(DRIVER_CONFIG_HAS_USART4)
extern ARM_DRIVER_USART Driver_USART4;
#endif

#if defined(UART5_BASE) && defined(DRIVER_CONFIG_HAS_USART5)
extern ARM_DRIVER_USART Driver_USART5;
#endif

#if defined(UART6_BASE) && defined(DRIVER_CONFIG_HAS_USART6)
extern ARM_DRIVER_USART Driver_USART6;
#endif

#if defined(UART7_BASE) && defined(DRIVER_CONFIG_HAS_USART7)
extern ARM_DRIVER_USART Driver_USART7;
#endif

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_USART_MSP_H_ */
