/*
 * Copyright (c) 2024, Texas Instruments Incorporated
 * All rights reserved.
 *
 * TODO: Add license
 */
/*!****************************************************************************
 *  @file       Driver_Config_MSP.h
 *  @brief      CMSIS-Drivers board configuration file for Texas Instruments 
 *              MSP MCUs based on Arm Cortex-M CPUs
 *
 *  This configuration file must be tailored for each board port.
 *
 *  <hr>
 ******************************************************************************
 */

#ifndef DRIVER_CONFIG_MSP_H_
#define DRIVER_CONFIG_MSP_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include <ti/devices/msp/msp.h>
#include <Driver_GPIO_MSP.h>

/* GPIO Driver Configuration Options */

#define DRIVER_CONFIG_HAS_GPIOA (0)
#define DRIVER_CONFIG_HAS_GPIOB (1)
#define DRIVER_CONFIG_HAS_GPIOC (0)

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_CONFIG_MSP_H_ */
