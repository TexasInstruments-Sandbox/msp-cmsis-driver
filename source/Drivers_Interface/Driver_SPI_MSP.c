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
 *  @file       Driver_SPI_MSP.c
 ******************************************************************************
 */

#include <ti/driverlib/driverlib.h>
#include <Driver_SPI_MSP_Priv.h>
#include <Driver_SPI_MSP.h>

/* The SPIx driver instance expansion generators are given below.
 * These generators are used to create the corresponding data structures and 
 * function linkage implementations that are required by the Arm 
 * CMSIS-Drivers specification for each individual SPI driver instance used 
 * in an application.
 */

#define GEN_DRIVER_SPI_MSP_STATE(inst) \
DRIVER_SPI_MSP_STATE DRIVER_SPI##inst##_MSP_STATE;

#define GEN_DRIVER_SPI_MSP(inst) \
DRIVER_SPI_MSP DRIVER_SPI##inst##_MSP = \
{ \
    .hw = SPI##inst, \
    .state = &DRIVER_SPI##inst##_MSP_STATE, \
    .mosiPin.iomuxPinCtlMgmtRegIndex = DRIVER_SPI##inst##_MOSI_PINCM, \
    .mosiPin.iomuxPinFunction = DRIVER_SPI##inst##_MOSI_PF, \
    .misoPin.iomuxPinCtlMgmtRegIndex = DRIVER_SPI##inst##_MISO_PINCM, \
    .misoPin.iomuxPinFunction =  DRIVER_SPI##inst##_MISO_PF, \
    .ss = DRIVER_SPI##inst##_SS, \
    .ssPin.iomuxPinCtlMgmtRegIndex = DRIVER_SPI##inst##_SS_PINCM, \
    .ssPin.iomuxPinFunction =  DRIVER_SPI##inst##_SS_PF, \
    .ssGPIOPort = DRIVER_SPI##inst##_SS_GPIO_PORT, \
    .ssPinNumber = DRIVER_SPI##inst##_SS_PIN_NUMBER, \
    .sclkPin.iomuxPinCtlMgmtRegIndex = DRIVER_SPI##inst##_SCLK_PINCM, \
    .sclkPin.iomuxPinFunction =  DRIVER_SPI##inst##_SCLK_PF, \
    .transmitDMA.hw = DRIVER_SPI##inst##_TRANSMIT_DMA_HW, \
    .transmitDMA.ch = DRIVER_SPI##inst##_TRANSMIT_DMA_CH, \
    .transmitDMA.trig = DRIVER_SPI##inst##_TRANSMIT_DMA_TRIG, \
    .receiveDMA.hw = DRIVER_SPI##inst##_RECEIVE_DMA_HW, \
    .receiveDMA.ch = DRIVER_SPI##inst##_RECEIVE_DMA_CH, \
    .receiveDMA.trig = DRIVER_SPI##inst##_RECEIVE_DMA_TRIG, \
    .clock = DRIVER_SPI##inst##_CLOCK_SEL, \
    .clockFreq = DRIVER_SPI##inst##_CLOCK_FREQ, \
    .irq = SPI##inst##_INT_IRQn \
};

#define GEN_DRIVER_SPI_MSP_FXNS(inst) \
static ARM_SPI_CAPABILITIES MSP_ARM_SPI##inst##_GetCapabilities(void) \
{ \
return MSP_ARM_SPI_GetCapabilities(&DRIVER_SPI##inst##_MSP); \
} \
static int32_t MSP_ARM_SPI##inst##_Initialize(ARM_SPI_SignalEvent_t cb_event) \
{ \
return MSP_ARM_SPI_Initialize(&DRIVER_SPI##inst##_MSP, cb_event); \
} \
static int32_t MSP_ARM_SPI##inst##_Uninitialize(void) \
{ \
return MSP_ARM_SPI_Uninitialize(&DRIVER_SPI##inst##_MSP); \
} \
static int32_t MSP_ARM_SPI##inst##_PowerControl(ARM_POWER_STATE state) \
{ \
return MSP_ARM_SPI_PowerControl(&DRIVER_SPI##inst##_MSP, state); \
} \
static int32_t MSP_ARM_SPI##inst##_Send(const void *data, uint32_t num) \
{ \
return MSP_ARM_SPI_Send(&DRIVER_SPI##inst##_MSP, data, num); \
} \
static int32_t MSP_ARM_SPI##inst##_Receive(void *data, uint32_t num) \
{ \
return MSP_ARM_SPI_Receive(&DRIVER_SPI##inst##_MSP, data, num); \
} \
static int32_t MSP_ARM_SPI##inst##_Transfer(const void *data_out, \
    void *data_in, uint32_t num) \
{ \
return MSP_ARM_SPI_Transfer(&DRIVER_SPI##inst##_MSP, data_out, data_in, num); \
} \
static uint32_t MSP_ARM_SPI##inst##_GetDataCount(void) \
{ \
return MSP_ARM_SPI_GetDataCount(&DRIVER_SPI##inst##_MSP); \
} \
static int32_t MSP_ARM_SPI##inst##_Control(uint32_t control, uint32_t arg) \
{ \
return MSP_ARM_SPI_Control(&DRIVER_SPI##inst##_MSP, control, arg); \
} \
static ARM_SPI_STATUS MSP_ARM_SPI##inst##_GetStatus(void) \
{ \
return MSP_ARM_SPI_GetStatus(&DRIVER_SPI##inst##_MSP); \
} \

#define GEN_DRIVER_SPI_MSP_IF(inst) \
ARM_DRIVER_SPI Driver_SPI##inst = \
{ \
MSP_ARM_SPI_GetVersion, \
MSP_ARM_SPI##inst##_GetCapabilities, \
MSP_ARM_SPI##inst##_Initialize, \
MSP_ARM_SPI##inst##_Uninitialize, \
MSP_ARM_SPI##inst##_PowerControl, \
MSP_ARM_SPI##inst##_Send, \
MSP_ARM_SPI##inst##_Receive, \
MSP_ARM_SPI##inst##_Transfer, \
MSP_ARM_SPI##inst##_GetDataCount, \
MSP_ARM_SPI##inst##_Control, \
MSP_ARM_SPI##inst##_GetStatus, \
};

#define GEN_DRIVER_SPI_MSP_ISR(inst) \
void SPI##inst##_IRQHandler(void) \
{ \
MSP_ARM_SPI_IRQHandler(&DRIVER_SPI##inst##_MSP); \
}

/* SPIx Definitions */

#if defined(SPI0_BASE) && (DRIVER_CONFIG_HAS_SPI0==1)
GEN_DRIVER_SPI_MSP_STATE(0)
GEN_DRIVER_SPI_MSP(0)
GEN_DRIVER_SPI_MSP_FXNS(0)
GEN_DRIVER_SPI_MSP_IF(0)
GEN_DRIVER_SPI_MSP_ISR(0)
#endif

#if defined(SPI1_BASE) && (DRIVER_CONFIG_HAS_SPI1==1)
GEN_DRIVER_SPI_MSP_STATE(1)
GEN_DRIVER_SPI_MSP(1)
GEN_DRIVER_SPI_MSP_FXNS(1)
GEN_DRIVER_SPI_MSP_IF(1)
GEN_DRIVER_SPI_MSP_ISR(1)
#endif

#if defined(SPI2_BASE) && (DRIVER_CONFIG_HAS_SPI2==1)
GEN_DRIVER_SPI_MSP_STATE(2)
GEN_DRIVER_SPI_MSP(2)
GEN_DRIVER_SPI_MSP_FXNS(2)
GEN_DRIVER_SPI_MSP_IF(2)
GEN_DRIVER_SPI_MSP_ISR(2)
#endif