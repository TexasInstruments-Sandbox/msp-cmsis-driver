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
 *  @file       Driver_USART_MSP.c
 ******************************************************************************
 */

#include <ti/driverlib/driverlib.h>
#include <Driver_USART_MSP_Priv.h>
#include <Driver_USART_MSP.h>

/* The USARTx driver instance expansion generators are given below.
 * These generators are used to create the corresponding data structures and function
 * linkage implementations that are required by the Arm CMSIS-Drivers specification
 * for each individual USART driver instance used in an application.
 */

#define GEN_DRIVER_USART_MSP_STATE(inst) \
DRIVER_USART_MSP_STATE DRIVER_USART##inst##_MSP_STATE;

#define GEN_DRIVER_USART_MSP(inst) \
DRIVER_USART_MSP DRIVER_USART##inst##_MSP = \
{ \
    .hw = UART##inst, \
    .state = &DRIVER_USART##inst##_MSP_STATE, \
    .txPin.iomuxPinCtlMgmtRegIndex = DRIVER_USART##inst##_TX_PINCM, \
    .txPin.iomuxPinFunction = DRIVER_USART##inst##_TX_PF, \
    .rxPin.iomuxPinCtlMgmtRegIndex = DRIVER_USART##inst##_RX_PINCM, \
    .rxPin.iomuxPinFunction =  DRIVER_USART##inst##_RX_PF, \
    .rtsPin.iomuxPinCtlMgmtRegIndex = DRIVER_USART##inst##_RTS_PINCM, \
    .rtsPin.iomuxPinFunction =  DRIVER_USART##inst##_RTS_PF, \
    .ctsPin.iomuxPinCtlMgmtRegIndex = DRIVER_USART##inst##_CTS_PINCM, \
    .ctsPin.iomuxPinFunction =  DRIVER_USART##inst##_CTS_PF, \
    .txDMA.hw = DRIVER_USART##inst##_TX_DMA_HW, \
    .txDMA.ch = DRIVER_USART##inst##_TX_DMA_CH, \
    .txDMA.trig = DRIVER_USART##inst##_TX_DMA_TRIG, \
    .rxDMA.hw = DRIVER_USART##inst##_RX_DMA_HW, \
    .rxDMA.ch = DRIVER_USART##inst##_RX_DMA_CH, \
    .rxDMA.trig = DRIVER_USART##inst##_RX_DMA_TRIG, \
    .clock = DRIVER_USART##inst##_CLOCK_SEL, \
    .clockFreq = DRIVER_USART##inst##_CLOCK_FREQ, \
    .irq = UART##inst##_INT_IRQn \
};

#define GEN_DRIVER_USART_MSP_FXNS(inst) \
static ARM_USART_CAPABILITIES MSP_ARM_USART##inst##_GetCapabilities(void) \
{ \
return MSP_ARM_USART_GetCapabilities(&DRIVER_USART##inst##_MSP); \
} \
static int32_t MSP_ARM_USART##inst##_Initialize(ARM_USART_SignalEvent_t cb_event) \
{ \
return MSP_ARM_USART_Initialize(&DRIVER_USART##inst##_MSP, cb_event); \
} \
static int32_t MSP_ARM_USART##inst##_Uninitialize(void) \
{ \
return MSP_ARM_USART_Uninitialize(&DRIVER_USART##inst##_MSP); \
} \
static int32_t MSP_ARM_USART##inst##_PowerControl(ARM_POWER_STATE state) \
{ \
return MSP_ARM_USART_PowerControl(&DRIVER_USART##inst##_MSP, state); \
} \
static int32_t MSP_ARM_USART##inst##_Send(const void *data, uint32_t num) \
{ \
return MSP_ARM_USART_Send(&DRIVER_USART##inst##_MSP, data, num); \
} \
static int32_t MSP_ARM_USART##inst##_Receive(void *data, uint32_t num) \
{ \
return MSP_ARM_USART_Receive(&DRIVER_USART##inst##_MSP, data, num); \
} \
static int32_t MSP_ARM_USART##inst##_Transfer(const void *data_out, \
                                        void *data_in, uint32_t num) \
{ \
return MSP_ARM_USART_Transfer(&DRIVER_USART##inst##_MSP, data_out, \
                                data_in, num); \
} \
static uint32_t MSP_ARM_USART##inst##_GetTxCount(void) \
{ \
return MSP_ARM_USART_GetTxCount(&DRIVER_USART##inst##_MSP); \
} \
static uint32_t MSP_ARM_USART##inst##_GetRxCount(void) \
{ \
return MSP_ARM_USART_GetRxCount(&DRIVER_USART##inst##_MSP); \
} \
static int32_t MSP_ARM_USART##inst##_Control(uint32_t control, uint32_t arg) \
{ \
return MSP_ARM_USART_Control(&DRIVER_USART##inst##_MSP, control, arg); \
} \
static ARM_USART_STATUS MSP_ARM_USART##inst##_GetStatus(void) \
{ \
return MSP_ARM_USART_GetStatus(&DRIVER_USART##inst##_MSP); \
} \
static int32_t MSP_ARM_USART##inst##_SetModemControl(ARM_USART_MODEM_CONTROL control) \
{ \
return MSP_ARM_USART_SetModemControl(&DRIVER_USART##inst##_MSP, control); \
} \
static ARM_USART_MODEM_STATUS MSP_ARM_USART##inst##_GetModemStatus(void) \
{ \
return MSP_ARM_USART_GetModemStatus(&DRIVER_USART##inst##_MSP); \
}

#define GEN_DRIVER_USART_MSP_IF(inst) \
ARM_DRIVER_USART Driver_USART##inst = \
{ \
MSP_ARM_USART_GetVersion, \
MSP_ARM_USART##inst##_GetCapabilities, \
MSP_ARM_USART##inst##_Initialize, \
MSP_ARM_USART##inst##_Uninitialize, \
MSP_ARM_USART##inst##_PowerControl, \
MSP_ARM_USART##inst##_Send, \
MSP_ARM_USART##inst##_Receive, \
MSP_ARM_USART##inst##_Transfer, \
MSP_ARM_USART##inst##_GetTxCount, \
MSP_ARM_USART##inst##_GetRxCount, \
MSP_ARM_USART##inst##_Control, \
MSP_ARM_USART##inst##_GetStatus, \
MSP_ARM_USART##inst##_SetModemControl, \
MSP_ARM_USART##inst##_GetModemStatus \
};

#define GEN_DRIVER_USART_MSP_ISR(inst) \
void UART##inst##_IRQHandler(void) \
{ \
MSP_ARM_USART_IRQHandler(&DRIVER_USART##inst##_MSP); \
}

/* USARTx Definitions */

#if defined(UART0_BASE) && (DRIVER_CONFIG_HAS_USART0==1)
GEN_DRIVER_USART_MSP_STATE(0)
GEN_DRIVER_USART_MSP(0)
GEN_DRIVER_USART_MSP_FXNS(0)
GEN_DRIVER_USART_MSP_IF(0)
GEN_DRIVER_USART_MSP_ISR(0)
#endif

#if defined(UART1_BASE) && (DRIVER_CONFIG_HAS_USART1==1)
GEN_DRIVER_USART_MSP_STATE(1)
GEN_DRIVER_USART_MSP(1)
GEN_DRIVER_USART_MSP_FXNS(1)
GEN_DRIVER_USART_MSP_IF(1)
GEN_DRIVER_USART_MSP_ISR(1)
#endif

#if defined(UART2_BASE) && (DRIVER_CONFIG_HAS_USART2==1)
GEN_DRIVER_USART_MSP_STATE(2)
GEN_DRIVER_USART_MSP(2)
GEN_DRIVER_USART_MSP_FXNS(2)
GEN_DRIVER_USART_MSP_IF(2)
GEN_DRIVER_USART_MSP_ISR(2)
#endif

#if defined(UART3_BASE) && (DRIVER_CONFIG_HAS_USART3==1)
GEN_DRIVER_USART_MSP_STATE(3)
GEN_DRIVER_USART_MSP(3)
GEN_DRIVER_USART_MSP_FXNS(3)
GEN_DRIVER_USART_MSP_IF(3)
GEN_DRIVER_USART_MSP_ISR(3)
#endif

#if defined(UART4_BASE) && (DRIVER_CONFIG_HAS_USART4==1)
GEN_DRIVER_USART_MSP_STATE(4)
GEN_DRIVER_USART_MSP(4)
GEN_DRIVER_USART_MSP_FXNS(4)
GEN_DRIVER_USART_MSP_IF(4)
GEN_DRIVER_USART_MSP_ISR(4)
#endif

#if defined(UART5_BASE) && (DRIVER_CONFIG_HAS_USART5==1)
GEN_DRIVER_USART_MSP_STATE(5)
GEN_DRIVER_USART_MSP(5)
GEN_DRIVER_USART_MSP_FXNS(5)
GEN_DRIVER_USART_MSP_IF(5)
GEN_DRIVER_USART_MSP_ISR(5)
#endif

#if defined(UART6_BASE) && (DRIVER_CONFIG_HAS_USART6==1)
GEN_DRIVER_USART_MSP_STATE(6)
GEN_DRIVER_USART_MSP(6)
GEN_DRIVER_USART_MSP_FXNS(6)
GEN_DRIVER_USART_MSP_IF(6)
GEN_DRIVER_USART_MSP_ISR(6)
#endif

#if defined(UART7_BASE) && (DRIVER_CONFIG_HAS_USART7==1)
GEN_DRIVER_USART_MSP_STATE(7)
GEN_DRIVER_USART_MSP(7)
GEN_DRIVER_USART_MSP_FXNS(7)
GEN_DRIVER_USART_MSP_IF(7)
GEN_DRIVER_USART_MSP_ISR(7)
#endif
