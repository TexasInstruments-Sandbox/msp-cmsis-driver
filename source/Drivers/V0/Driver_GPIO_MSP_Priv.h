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
 *  @file       Driver_GPIO_MSP_Priv.h
 *  @brief      MSP GPIO CMSIS-Driver for Cortex-M devices (V0 HW version)
 * 
 *  The CMSIS-Drivers GPIO driver allows for simple, non-blocking GPIO
 *  control via Arm-standardized API. This driver is developed to be compatible 
 *  with the Arm CMSIS-Driver specification.
 *
 *  This module is the private (internal) driver layer for GPIO.  Applications
 *  are expected to use the driver through the Driver_GPIO_MSP.h layer.
 *
 *  Known limitations:
 *
 ******************************************************************************
 */

#ifndef DRIVER_GPIO_MSP_PRIV_H_
#define DRIVER_GPIO_MSP_PRIV_H_

#include <Driver_GPIO.h>
#include <Driver_Common_MSP.h>

#define MAX_PIN_NUM (31)

/**
 * @brief Stores a pointer to the address of the MSP GPIO hardware registers.
 *        for the corresponding GPIO hardware peripheral instance.
 */
typedef GPIO_Regs DRIVER_GPIO_MSP_HW;

/**
 * @brief Driver instance state data structure to hold run-time modified
 *        state information for the driver.  One structure is used per module
 *        instance deployed in an application.  This structure must be stored
 *        in SRAM and be read/write accessible by the driver.
 */
typedef struct
{
    /*! Pointer to callback function for event handling. */
    ARM_GPIO_SignalEvent_t callbacks[32];
    /*! Flag indicating if the GPIO port has been powered with NVIC enabled */
    uint8_t enabled : 1;
} DRIVER_GPIO_MSP_STATE;

/**
 * @brief Driver instance configuration data structure to hold the configuration
 *        information for each driver instance, including hw module access.
 *        This is the primary data structure used by the driver and a pointer to 
 *        this structure is passed to most APIs in the driver.
 *        This is a static data structure and may be stored in
 *        read-only memory for better memory layout optimization.
 */
typedef struct
{
    /*! Pointer to GPIO module base address used by this driver instance. */
    DRIVER_GPIO_MSP_HW *hw;
    /*! Pointer to MSP GPIO driver state data structure. */
    DRIVER_GPIO_MSP_STATE *state;
    /*! Pointer to array of IOMUX_PINCMs for corresponding GPIO module pins */
    uint8_t *pinCMs;
    /*! GPIO module NVIC interrupt port */
    uint8_t irq;
} DRIVER_GPIO_MSP;


/**
 *  @brief      Sets up the specific pin as GPIO with default configuration.
 *              Pin is configured as input without pull-resistor and no event
 *              triggers. Also specifies the callbuck function to register for
 *              a particular pin.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 *  @param[in]  pin GPIO pin to configure.
 *  @param[in]  cb_event Pointer to the application callback function.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK
 *
 *  @post       The pin is configured as input without a pull-up resistor.
 * 
 */
extern int32_t MSP_ARM_GPIO_Setup(DRIVER_GPIO_MSP *module, ARM_GPIO_Pin_t pin,
    ARM_GPIO_SignalEvent_t cb_event);

/**
 *  @brief      Set the GPIO direction of specified pin.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 *  @param[in]  pin GPIO pin to configure.
 *  @param[in]  direction Desired direction: either ARM_GPIO_INPUT or 
 *                        ARM_GPIO_OUTPUT.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK
 * 
 *  @post       The pin is configured with specified direction.
 * 
 */
extern int32_t MSP_ARM_GPIO_SetDirection(DRIVER_GPIO_MSP *module,
    ARM_GPIO_Pin_t pin, ARM_GPIO_DIRECTION direction);

/**
 *  @brief      Set the GPIO output mode of specified pin.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 *  @param[in]  pin GPIO pin to configure.
 *  @param[in]  mode Desired output mode: either ARM_GPIO_PUSH_PULL or 
 *                        ARM_GPIO_OPEN_DRAIN.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK
 * 
 *  @pre        The pin has been configured as an output by calling 
 *              MSP_ARM_GPIO_SetDirection with ARM_GPIO_OUTPUT.
 *
 *  @post       The pin is configured with specified output mode.
 * 
 */
extern int32_t MSP_ARM_GPIO_SetOutputMode(DRIVER_GPIO_MSP *module, 
    ARM_GPIO_Pin_t pin, ARM_GPIO_OUTPUT_MODE mode);

/**
 *  @brief      Set the GPIO pull resistor of specified pin.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 *  @param[in]  pin GPIO pin to configure.
 *  @param[in]  resistor Desired resistor configuration: one of  
 *                       ARM_GPIO_PULL_NONE, ARM_GPIO_PULL_UP, or 
 *                       ARM_GPIO_PULL_DOWN.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK
 * 
 *  @post       The pin is configured with specified pull resistor configuration.
 * 
 */
extern int32_t MSP_ARM_GPIO_SetPullResistor(DRIVER_GPIO_MSP *module, 
    ARM_GPIO_Pin_t pin, ARM_GPIO_PULL_RESISTOR resistor);

/**
 *  @brief      Set the GPIO event trigger of specified pin.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 *  @param[in]  pin GPIO pin to configure.
 *  @param[in]  trigger Desired event trigger: one of ARM_GPIO_TRIGGER_NONE,  
 *                      ARM_GPIO_TRIGGER_RISING_EDGE, 
 *                      ARM_GPIO_TRIGGER_FALLING_EDGE, or  
 *                      ARM_GPIO_TRIGGER_EITHER_EDGE.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK
 * 
 *  @post       The pin is configured with specified event trigger.
 * 
 */
extern int32_t MSP_ARM_GPIO_SetEventTrigger(DRIVER_GPIO_MSP *module, 
    ARM_GPIO_Pin_t pin, ARM_GPIO_EVENT_TRIGGER trigger);

/**
 *  @brief      Set the GPIO event trigger of specified pin.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 *  @param[in]  pin GPIO pin to configure.
 *  @param[in]  val Pin level (0 or 1)
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK
 * 
 *  @post       The pin is set with specified level. If the pin is currently 
 *              configured as an input, the level is latched and will driven 
 *              once the pin is configured as output.
 * 
 */
extern int32_t MSP_ARM_GPIO_SetOutput(DRIVER_GPIO_MSP *module, 
    ARM_GPIO_Pin_t pin, uint32_t val);

/**
 *  @brief      Read the level of the specified pin.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 *  @param[in]  pin GPIO pin to read.
 * 
 *  @return     GPIO Pin Level (0 or 1)
 * 
 */
extern uint32_t MSP_ARM_GPIO_GetInput(DRIVER_GPIO_MSP *module, 
    ARM_GPIO_Pin_t pin);

/**
 *  @brief      This is the GPIO driver interrupt handler to be called by the
 *              wrapping interrupt service routine for each HW instance.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @return     None.
 * 
 */
extern void MSP_ARM_GPIO_IRQHandler(DRIVER_GPIO_MSP *module);

#endif /* DRIVER_GPIO_MSP_PRIV_H_ */
