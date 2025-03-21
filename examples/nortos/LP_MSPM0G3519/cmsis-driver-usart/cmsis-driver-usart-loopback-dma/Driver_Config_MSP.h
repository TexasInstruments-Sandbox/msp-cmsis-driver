/*
 * Copyright (c) 2025, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*!****************************************************************************
 *  @file       Driver_Config_MSP.h
 *  @brief      CMSIS-Drivers board configuration file for Texas Instruments 
 *              MSP MCUs based on Arm Cortex-M CPUs
 *
 *  This configuration file must be tailwored for each board port.
 *
 ******************************************************************************
 */

 #ifndef DRIVER_CONFIG_MSP_H_
 #define DRIVER_CONFIG_MSP_H_
 
 #ifdef  __cplusplus
 extern "C"
 {
 #endif
 
 #include <ti/devices/msp/msp.h>
 #include <Driver_USART_MSP.h>
 
 /* USART Driver Configuration Options */
 
 /* Driver_USART0 Configuration (Maps to MSP hardware UART0 peripheral) */
 #define DRIVER_CONFIG_HAS_USART0 (1)
 #if (DRIVER_CONFIG_HAS_USART0==1) && defined(UART0_BASE)
 #define DRIVER_USART0_TX_PINCM         (IOMUX_PINCM21)
 #define DRIVER_USART0_TX_PF            (IOMUX_PINCM21_PF_UART0_TX)
 #define DRIVER_USART0_RX_PINCM         (DRIVER_IO_MSP_NONE)
 #define DRIVER_USART0_RX_PF            (0U)
 #define DRIVER_USART0_RTS_PINCM        (DRIVER_IO_MSP_NONE)
 #define DRIVER_USART0_RTS_PF           (0U)
 #define DRIVER_USART0_CTS_PINCM        (DRIVER_IO_MSP_NONE)
 #define DRIVER_USART0_CTS_PF           (0U)
 #define DRIVER_USART0_CLOCK_SEL        (DRIVER_CLK_MSP_BUSCLK)
 #define DRIVER_USART0_CLOCK_FREQ       (32000000U)
 #define DRIVER_USART0_TX_DMA_HW        (DRIVER_DMA_HW_NONE)
 #define DRIVER_USART0_TX_DMA_CH        (DRIVER_DMA_CH_NONE)
 #define DRIVER_USART0_TX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
 #define DRIVER_USART0_RX_DMA_HW        (DRIVER_DMA_HW_NONE)
 #define DRIVER_USART0_RX_DMA_CH        (DRIVER_DMA_CH_NONE)
 #define DRIVER_USART0_RX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
 #endif
 
 /* Driver_USART1 Configuration (Maps to MSP hardware UART1 peripheral) */
 #define DRIVER_CONFIG_HAS_USART1 (1)
 #if (DRIVER_CONFIG_HAS_USART1==1) && defined(UART1_BASE)
 #define DRIVER_USART1_TX_PINCM         (IOMUX_PINCM23)
 #define DRIVER_USART1_TX_PF            (IOMUX_PINCM23_PF_UART1_TX)
 #define DRIVER_USART1_RX_PINCM         (IOMUX_PINCM24)
 #define DRIVER_USART1_RX_PF            (IOMUX_PINCM24_PF_UART1_RX)
 #define DRIVER_USART1_RTS_PINCM        (DRIVER_IO_MSP_NONE)
 #define DRIVER_USART1_RTS_PF           (0U)
 #define DRIVER_USART1_CTS_PINCM        (DRIVER_IO_MSP_NONE)
 #define DRIVER_USART1_CTS_PF           (0U)
 #define DRIVER_USART1_CLOCK_SEL        (DRIVER_CLK_MSP_BUSCLK)
 #define DRIVER_USART1_CLOCK_FREQ       (32000000U)
 #define DRIVER_USART1_TX_DMA_HW        (DMA)
 #define DRIVER_USART1_TX_DMA_CH        (0U)
 #define DRIVER_USART1_TX_DMA_TRIG      (DMA_UART1_TX_TRIG)
 #define DRIVER_USART1_RX_DMA_HW        (DMA)
 #define DRIVER_USART1_RX_DMA_CH        (1U)
 #define DRIVER_USART1_RX_DMA_TRIG      (DMA_UART1_RX_TRIG)
 #endif
 
/* Driver_USART2 Configuration (Maps to MSP hardware UART2 peripheral) */
#define DRIVER_CONFIG_HAS_USART2 (0)
#if (DRIVER_CONFIG_HAS_USART2==1) && defined(UART2_BASE)
#define DRIVER_USART2_TX_PINCM         (DRIVER_IO_MSP_NONE)
#define DRIVER_USART2_TX_PF            (0U)
#define DRIVER_USART2_RX_PINCM         (DRIVER_IO_MSP_NONE)
#define DRIVER_USART2_RX_PF            (0U)
#define DRIVER_USART2_RTS_PINCM        (DRIVER_IO_MSP_NONE)
#define DRIVER_USART2_RTS_PF           (0U)
#define DRIVER_USART2_CTS_PINCM        (DRIVER_IO_MSP_NONE)
#define DRIVER_USART2_CTS_PF           (0U)
#define DRIVER_USART2_CLOCK_SEL        (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_USART2_CLOCK_FREQ       (32000000U)
#define DRIVER_USART2_TX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART2_TX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART2_TX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#define DRIVER_USART2_RX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART2_RX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART2_RX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#endif

/* Driver_USART3 Configuration (Maps to MSP hardware UART3 peripheral) */
#define DRIVER_CONFIG_HAS_USART3 (0)
#if (DRIVER_CONFIG_HAS_USART3==1) && defined(UART3_BASE)
#define DRIVER_USART3_TX_PINCM         (DRIVER_IO_MSP_NONE)
#define DRIVER_USART3_TX_PF            (0U)
#define DRIVER_USART3_RX_PINCM         (DRIVER_IO_MSP_NONE)
#define DRIVER_USART3_RX_PF            (0U)
#define DRIVER_USART3_RTS_PINCM        (DRIVER_IO_MSP_NONE)
#define DRIVER_USART3_RTS_PF           (0U)
#define DRIVER_USART3_CTS_PINCM        (DRIVER_IO_MSP_NONE)
#define DRIVER_USART3_CTS_PF           (0U)
#define DRIVER_USART3_CLOCK_SEL        (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_USART3_CLOCK_FREQ       (32000000U)
#define DRIVER_USART3_TX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART3_TX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART3_TX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#define DRIVER_USART3_RX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART3_RX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART3_RX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#endif

/* Driver_USART4 Configuration (Maps to MSP hardware UART4 peripheral) */
#define DRIVER_CONFIG_HAS_USART4 (0)
#if (DRIVER_CONFIG_HAS_USART4==1) && defined(UART4_BASE)
#define DRIVER_USART4_TX_PINCM         (DRIVER_IO_MSP_NONE)
#define DRIVER_USART4_TX_PF            (0U)
#define DRIVER_USART4_RX_PINCM         (DRIVER_IO_MSP_NONE)
#define DRIVER_USART4_RX_PF            (0U)
#define DRIVER_USART4_RTS_PINCM        (DRIVER_IO_MSP_NONE)
#define DRIVER_USART4_RTS_PF           (0U)
#define DRIVER_USART4_CTS_PINCM        (DRIVER_IO_MSP_NONE)
#define DRIVER_USART4_CTS_PF           (0U)
#define DRIVER_USART4_CLOCK_SEL        (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_USART4_CLOCK_FREQ       (32000000U)
#define DRIVER_USART4_TX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART4_TX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART4_TX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#define DRIVER_USART4_RX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART4_RX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART4_RX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#endif

/* Driver_USART5 Configuration (Maps to MSP hardware UART5 peripheral) */
#define DRIVER_CONFIG_HAS_USART5 (0)
#if (DRIVER_CONFIG_HAS_USART5==1) && defined(UART5_BASE)
#define DRIVER_USART5_TX_PINCM         (DRIVER_IO_MSP_NONE)
#define DRIVER_USART5_TX_PF            (0U)
#define DRIVER_USART5_RX_PINCM         (DRIVER_IO_MSP_NONE)
#define DRIVER_USART5_RX_PF            (0U)
#define DRIVER_USART5_RTS_PINCM        (DRIVER_IO_MSP_NONE)
#define DRIVER_USART5_RTS_PF           (0U)
#define DRIVER_USART5_CTS_PINCM        (DRIVER_IO_MSP_NONE)
#define DRIVER_USART5_CTS_PF           (0U)
#define DRIVER_USART5_CLOCK_SEL        (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_USART5_CLOCK_FREQ       (32000000U)
#define DRIVER_USART5_TX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART5_TX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART5_TX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#define DRIVER_USART5_RX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART5_RX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART5_RX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#endif

/* Driver_USART6 Configuration (Maps to MSP hardware UART6 peripheral) */
#define DRIVER_CONFIG_HAS_USART6 (0)
#if (DRIVER_CONFIG_HAS_USART6==1) && defined(UART6_BASE)
#define DRIVER_USART6_TX_PINCM         (DRIVER_IO_MSP_NONE)
#define DRIVER_USART6_TX_PF            (0U)
#define DRIVER_USART6_RX_PINCM         (DRIVER_IO_MSP_NONE)
#define DRIVER_USART6_RX_PF            (0U)
#define DRIVER_USART6_RTS_PINCM        (DRIVER_IO_MSP_NONE)
#define DRIVER_USART6_RTS_PF           (0U)
#define DRIVER_USART6_CTS_PINCM        (DRIVER_IO_MSP_NONE)
#define DRIVER_USART6_CTS_PF           (0U)
#define DRIVER_USART6_CLOCK_SEL        (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_USART6_CLOCK_FREQ       (32000000U)
#define DRIVER_USART6_TX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART6_TX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART6_TX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#define DRIVER_USART6_RX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART6_RX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART6_RX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#endif

/* Driver_USART7 Configuration (Maps to MSP hardware UART7 peripheral) */
#define DRIVER_CONFIG_HAS_USART7 (0)
#if (DRIVER_CONFIG_HAS_USART7==1) && defined(UART7_BASE)
#define DRIVER_USART7_TX_PINCM         (DRIVER_IO_MSP_NONE)
#define DRIVER_USART7_TX_PF            (0U)
#define DRIVER_USART7_RX_PINCM         (DRIVER_IO_MSP_NONE)
#define DRIVER_USART7_RX_PF            (0U)
#define DRIVER_USART7_RTS_PINCM        (DRIVER_IO_MSP_NONE)
#define DRIVER_USART7_RTS_PF           (0U)
#define DRIVER_USART7_CTS_PINCM        (DRIVER_IO_MSP_NONE)
#define DRIVER_USART7_CTS_PF           (0U)
#define DRIVER_USART7_CLOCK_SEL        (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_USART7_CLOCK_FREQ       (32000000U)
#define DRIVER_USART7_TX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART7_TX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART7_TX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#define DRIVER_USART7_RX_DMA_HW        (DRIVER_DMA_HW_NONE)
#define DRIVER_USART7_RX_DMA_CH        (DRIVER_DMA_CH_NONE)
#define DRIVER_USART7_RX_DMA_TRIG      (DRIVER_DMA_TRIG_NONE)
#endif

 #ifdef  __cplusplus
 }
 #endif
 
 #endif /* DRIVER_CONFIG_MSP_H_ */
 