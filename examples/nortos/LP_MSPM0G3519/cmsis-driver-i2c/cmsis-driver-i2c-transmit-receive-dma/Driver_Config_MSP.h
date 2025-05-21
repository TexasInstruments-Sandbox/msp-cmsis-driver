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
#include <Driver_I2C_MSP.h>

/* I2C Driver Configuration Options */

/* Driver_I2C0 Configuration (Maps to MSP hardware I2C0 peripheral) */
#define DRIVER_CONFIG_HAS_I2C0 (1)
#if (DRIVER_CONFIG_HAS_I2C0==1) && defined(I2C0_BASE)
#define DRIVER_I2C0_SDA_PINCM               (IOMUX_PINCM1)
#define DRIVER_I2C0_SDA_PF                  (IOMUX_PINCM1)
#define DRIVER_I2C0_SCL_PINCM               (IOMUX_PINCM2)
#define DRIVER_I2C0_SCL_PF                  (IOMUX_PINCM2_PF_I2C0_SCL)
#define DRIVER_I2C0_CLOCK_SEL               (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_I2C0_CLOCK_FREQ              (32000000U)
#define DRIVER_I2C0_TRANSMIT_DMA_HW         (DMA)
#define DRIVER_I2C0_TRANSMIT_DMA_CH         (0U)
#define DRIVER_I2C0_TRANSMIT_DMA_TRIG       (DMA_I2C0_TX_TRIG)
#define DRIVER_I2C0_RECEIVE_DMA_HW          (DMA)
#define DRIVER_I2C0_RECEIVE_DMA_CH          (1U)
#define DRIVER_I2C0_RECEIVE_DMA_TRIG        (DMA_I2C0_RX_TRIG)
#endif

/* Driver_I2C1 Configuration (Maps to MSP hardware I2C1 peripheral) */
#define DRIVER_CONFIG_HAS_I2C1 (1)
#if (DRIVER_CONFIG_HAS_I2C1==1) && defined(I2C1_BASE)
#define DRIVER_I2C1_SDA_PINCM               (IOMUX_PINCM16)
#define DRIVER_I2C1_SDA_PF                  (IOMUX_PINCM16_PF_I2C1_SDA)
#define DRIVER_I2C1_SCL_PINCM               (IOMUX_PINCM15)
#define DRIVER_I2C1_SCL_PF                  (IOMUX_PINCM15_PF_I2C1_SCL)
#define DRIVER_I2C1_CLOCK_SEL               (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_I2C1_CLOCK_FREQ              (32000000U)
#define DRIVER_I2C1_TRANSMIT_DMA_HW         (DMA)
#define DRIVER_I2C1_TRANSMIT_DMA_CH         (2U)
#define DRIVER_I2C1_TRANSMIT_DMA_TRIG       (DMA_I2C1_TX_TRIG)
#define DRIVER_I2C1_RECEIVE_DMA_HW          (DMA)
#define DRIVER_I2C1_RECEIVE_DMA_CH          (3U)
#define DRIVER_I2C1_RECEIVE_DMA_TRIG        (DMA_I2C1_RX_TRIG)
#endif

/* Driver_I2C2 Configuration (Maps to MSP hardware I2C2 peripheral) */
#define DRIVER_CONFIG_HAS_I2C2 (0)
#if (DRIVER_CONFIG_HAS_I2C2==1) && defined(I2C2_BASE)
#define DRIVER_I2C2_SDA_PINCM               (IOMUX_PINCM38)
#define DRIVER_I2C2_SDA_PF                  (IOMUX_PINCM38_PF_I2C2_SDA)
#define DRIVER_I2C2_SCL_PINCM               (IOMUX_PINCM37)
#define DRIVER_I2C2_SCL_PF                  (IOMUX_PINCM37_PF_I2C2_SCL)
#define DRIVER_I2C2_CLOCK_FREQ              (32000000U)
#define DRIVER_I2C2_TRANSMIT_DMA_HW         (DRIVER_DMA_HW_NONE)
#define DRIVER_I2C2_TRANSMIT_DMA_CH         (DRIVER_DMA_CH_NONE)
#define DRIVER_I2C2_TRANSMIT_DMA_TRIG       (DRIVER_DMA_TRIG_NONE)
#define DRIVER_I2C2_RECEIVE_DMA_HW          (DRIVER_DMA_HW_NONE)
#define DRIVER_I2C2_RECEIVE_DMA_CH          (DRIVER_DMA_CH_NONE)
#define DRIVER_I2C2_RECEIVE_DMA_TRIG        (DRIVER_DMA_TRIG_NONE)
#endif

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_CONFIG_MSP_H_ */
