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
 *  @file       Driver_I2C_MSP.c
 ******************************************************************************
 */

#include <ti/driverlib/driverlib.h>
#include <Driver_I2C_MSP_Priv.h>
#include <Driver_I2C_MSP.h>

/* The I2Cx driver instance expansion generators are given below.
 * These generators are used to create the corresponding data structures and 
 * function linkage implementations that are required by the Arm 
 * CMSIS-Drivers specification for each individual I2C driver instance used 
 * in an application.
 */

#define GEN_DRIVER_I2C_MSP_STATE(inst) \
DRIVER_I2C_MSP_STATE DRIVER_I2C##inst##_MSP_STATE;

#define GEN_DRIVER_I2C_MSP(inst) \
DRIVER_I2C_MSP DRIVER_I2C##inst##_MSP = \
{ \
    .hw = I2C##inst, \
    .state = &DRIVER_I2C##inst##_MSP_STATE, \
    .sdaPin.iomuxPinCtlMgmtRegIndex = DRIVER_I2C##inst##_SDA_PINCM, \
    .sdaPin.iomuxPinFunction = DRIVER_I2C##inst##_SDA_PF, \
    .sclPin.iomuxPinCtlMgmtRegIndex = DRIVER_I2C##inst##_SCL_PINCM, \
    .sclPin.iomuxPinFunction =  DRIVER_I2C##inst##_SCL_PF, \
    .transmitDMA.hw = DRIVER_I2C##inst##_TRANSMIT_DMA_HW, \
    .transmitDMA.ch = DRIVER_I2C##inst##_TRANSMIT_DMA_CH, \
    .transmitDMA.trig = DRIVER_I2C##inst##_TRANSMIT_DMA_TRIG, \
    .receiveDMA.hw = DRIVER_I2C##inst##_RECEIVE_DMA_HW, \
    .receiveDMA.ch = DRIVER_I2C##inst##_RECEIVE_DMA_CH, \
    .receiveDMA.trig = DRIVER_I2C##inst##_RECEIVE_DMA_TRIG, \
    .clock = DRIVER_I2C##inst##_CLOCK_SEL, \
    .clockFreq = DRIVER_I2C##inst##_CLOCK_FREQ, \
    .irq = I2C##inst##_INT_IRQn \
};

#define GEN_DRIVER_I2C_MSP_FXNS(inst) \
static ARM_I2C_CAPABILITIES MSP_ARM_I2C##inst##_GetCapabilities(void) \
{ \
return MSP_ARM_I2C_GetCapabilities(&DRIVER_I2C##inst##_MSP); \
} \
static int32_t MSP_ARM_I2C##inst##_Initialize(ARM_I2C_SignalEvent_t cb_event) \
{ \
return MSP_ARM_I2C_Initialize(&DRIVER_I2C##inst##_MSP, cb_event); \
} \
static int32_t MSP_ARM_I2C##inst##_Uninitialize(void) \
{ \
return MSP_ARM_I2C_Uninitialize(&DRIVER_I2C##inst##_MSP); \
} \
static int32_t MSP_ARM_I2C##inst##_PowerControl(ARM_POWER_STATE state) \
{ \
return MSP_ARM_I2C_PowerControl(&DRIVER_I2C##inst##_MSP, state); \
} \
static int32_t MSP_ARM_I2C##inst##_MasterTransmit(uint32_t addr, \
    const uint8_t *data, uint32_t num, bool xfer_pending) \
{ \
return MSP_ARM_I2C_MasterTransmit(&DRIVER_I2C##inst##_MSP, addr, data, num, \
    xfer_pending); \
} \
static int32_t MSP_ARM_I2C##inst##_MasterReceive(uint32_t addr, uint8_t *data, \
    uint32_t num, bool xfer_pending) \
{ \
return MSP_ARM_I2C_MasterReceive(&DRIVER_I2C##inst##_MSP, addr, data, num, \
    xfer_pending); \
} \
static int32_t MSP_ARM_I2C##inst##_SlaveTransmit(const uint8_t *data, \
    uint32_t num) \
{ \
return MSP_ARM_I2C_SlaveTransmit(&DRIVER_I2C##inst##_MSP, data, num); \
} \
static int32_t MSP_ARM_I2C##inst##_SlaveReceive(uint8_t *data, uint32_t num) \
{ \
return MSP_ARM_I2C_SlaveReceive(&DRIVER_I2C##inst##_MSP, data, num); \
} \
static int32_t MSP_ARM_I2C##inst##_GetDataCount(void) \
{ \
return MSP_ARM_I2C_GetDataCount(&DRIVER_I2C##inst##_MSP); \
} \
static int32_t MSP_ARM_I2C##inst##_Control(uint32_t control, uint32_t arg) \
{ \
return MSP_ARM_I2C_Control(&DRIVER_I2C##inst##_MSP, control, arg); \
} \
static ARM_I2C_STATUS MSP_ARM_I2C##inst##_GetStatus(void) \
{ \
return MSP_ARM_I2C_GetStatus(&DRIVER_I2C##inst##_MSP); \
} \

#define GEN_DRIVER_I2C_MSP_IF(inst) \
ARM_DRIVER_I2C Driver_I2C##inst = \
{ \
MSP_ARM_I2C_GetVersion, \
MSP_ARM_I2C##inst##_GetCapabilities, \
MSP_ARM_I2C##inst##_Initialize, \
MSP_ARM_I2C##inst##_Uninitialize, \
MSP_ARM_I2C##inst##_PowerControl, \
MSP_ARM_I2C##inst##_MasterTransmit, \
MSP_ARM_I2C##inst##_MasterReceive, \
MSP_ARM_I2C##inst##_SlaveTransmit, \
MSP_ARM_I2C##inst##_SlaveReceive, \
MSP_ARM_I2C##inst##_GetDataCount, \
MSP_ARM_I2C##inst##_Control, \
MSP_ARM_I2C##inst##_GetStatus, \
};

#define GEN_DRIVER_I2C_MSP_ISR(inst) \
void I2C##inst##_IRQHandler(void) \
{ \
MSP_ARM_I2C_IRQHandler(&DRIVER_I2C##inst##_MSP); \
}

/* I2Cx Definitions */

#if defined(I2C0_BASE) && (DRIVER_CONFIG_HAS_I2C0==1)
GEN_DRIVER_I2C_MSP_STATE(0)
GEN_DRIVER_I2C_MSP(0)
GEN_DRIVER_I2C_MSP_FXNS(0)
GEN_DRIVER_I2C_MSP_IF(0)
GEN_DRIVER_I2C_MSP_ISR(0)
#endif

#if defined(I2C1_BASE) && (DRIVER_CONFIG_HAS_I2C1==1)
GEN_DRIVER_I2C_MSP_STATE(1)
GEN_DRIVER_I2C_MSP(1)
GEN_DRIVER_I2C_MSP_FXNS(1)
GEN_DRIVER_I2C_MSP_IF(1)
GEN_DRIVER_I2C_MSP_ISR(1)
#endif

#if defined(I2C2_BASE) && (DRIVER_CONFIG_HAS_I2C2==1)
GEN_DRIVER_I2C_MSP_STATE(2)
GEN_DRIVER_I2C_MSP(2)
GEN_DRIVER_I2C_MSP_FXNS(2)
GEN_DRIVER_I2C_MSP_IF(2)
GEN_DRIVER_I2C_MSP_ISR(2)
#endif