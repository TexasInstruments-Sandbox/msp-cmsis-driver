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
#include <Driver_SPI_MSP.h>

/* SPI Driver Configuration Options */

/* Driver_SPI0 Configuration (Maps to MSP hardware SPI0 peripheral) */
#define DRIVER_CONFIG_HAS_SPI0 (1)
#if (DRIVER_CONFIG_HAS_SPI0==1) && defined(SPI0_BASE)
#define DRIVER_SPI0_MOSI_PINCM              (IOMUX_PINCM43)
#define DRIVER_SPI0_MOSI_PF                 (IOMUX_PINCM43_PF_SPI0_PICO)
#define DRIVER_SPI0_MISO_PINCM              (IOMUX_PINCM21)
#define DRIVER_SPI0_MISO_PF                 (IOMUX_PINCM21_PF_SPI0_POCI)
#define DRIVER_SPI0_SS                      (DL_SPI_CHIP_SELECT_0)
#define DRIVER_SPI0_SS_GPIO_PORT            (GPIOA)
#define DRIVER_SPI0_SS_PIN_NUMBER           (DL_GPIO_PIN_8)
#define DRIVER_SPI0_SS_PINCM                (IOMUX_PINCM19)
#define DRIVER_SPI0_SS_PF                   (IOMUX_PINCM19_PF_SPI0_CS0)
#define DRIVER_SPI0_SCLK_PINCM              (IOMUX_PINCM22)
#define DRIVER_SPI0_SCLK_PF                 (IOMUX_PINCM22_PF_SPI0_SCLK)
#define DRIVER_SPI0_CLOCK_SEL               (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_SPI0_CLOCK_FREQ              (32000000U)
#define DRIVER_SPI0_TRANSMIT_DMA_HW         (DMA)
#define DRIVER_SPI0_TRANSMIT_DMA_CH         (0U)
#define DRIVER_SPI0_TRANSMIT_DMA_TRIG       (DMA_SPI0_TX_TRIG)
#define DRIVER_SPI0_RECEIVE_DMA_HW          (DMA)
#define DRIVER_SPI0_RECEIVE_DMA_CH          (1U)
#define DRIVER_SPI0_RECEIVE_DMA_TRIG        (DMA_SPI0_RX_TRIG)
#endif

/* Driver_SPI1 Configuration (Maps to MSP hardware SPI1 peripheral) */
#define DRIVER_CONFIG_HAS_SPI1 (0)
#if (DRIVER_CONFIG_HAS_SPI1==1) && defined(SPI1_BASE)
#define DRIVER_SPI1_MOSI_PINCM              (IOMUX_PINCM25)
#define DRIVER_SPI1_MOSI_PF                 (IOMUX_PINCM25_PF_SPI1_PICO)
#define DRIVER_SPI1_MISO_PINCM              (IOMUX_PINCM24)
#define DRIVER_SPI1_MISO_PF                 (IOMUX_PINCM24_PF_SPI1_POCI)
#define DRIVER_SPI1_SS                      (DL_SPI_CHIP_SELECT_0)
#define DRIVER_SPI1_SS_GPIO_PORT            (GPIOA)
#define DRIVER_SPI1_SS_PIN_NUMBER           (DL_GPIO_PIN_2)
#define DRIVER_SPI1_SS_PINCM                (IOMUX_PINCM7)
#define DRIVER_SPI1_SS_PF                   (IOMUX_PINCM7_PF_SPI1_CS0)
#define DRIVER_SPI1_SCLK_PINCM              (IOMUX_PINCM26)
#define DRIVER_SPI1_SCLK_PF                 (IOMUX_PINCM26_PF_SPI1_SCLK)
#define DRIVER_SPI1_CLOCK_SEL               (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_SPI1_CLOCK_FREQ              (32000000U)
#define DRIVER_SPI1_TRANSMIT_DMA_HW         (DRIVER_DMA_HW_NONE)
#define DRIVER_SPI1_TRANSMIT_DMA_CH         (DRIVER_DMA_CH_NONE)
#define DRIVER_SPI1_TRANSMIT_DMA_TRIG       (DRIVER_DMA_TRIG_NONE)
#define DRIVER_SPI1_RECEIVE_DMA_HW          (DRIVER_DMA_HW_NONE)
#define DRIVER_SPI1_RECEIVE_DMA_CH          (DRIVER_DMA_CH_NONE)
#define DRIVER_SPI1_RECEIVE_DMA_TRIG        (DRIVER_DMA_TRIG_NONE)
#endif

/* Driver_SPI2 Configuration (Maps to MSP hardware SPI2 peripheral) */
#define DRIVER_CONFIG_HAS_SPI2 (0)
#if (DRIVER_CONFIG_HAS_SPI2==1) && defined(SPI2_BASE)
#define DRIVER_SPI2_MOSI_PINCM              (IOMUX_PINCM17)
#define DRIVER_SPI2_MOSI_PF                 (IOMUX_PINCM17_PF_SPI2_PICO)
#define DRIVER_SPI2_MISO_PINCM              (IOMUX_PINCM7)
#define DRIVER_SPI2_MISO_PF                 (IOMUX_PINCM7_PF_SPI2_POCI)
#define DRIVER_SPI2_SS                      (DL_SPI_CHIP_SELECT_0)
#define DRIVER_SPI2_SS_GPIO_PORT            (GPIOA)
#define DRIVER_SPI2_SS_PIN_NUMBER           (DL_GPIO_PIN_4)
#define DRIVER_SPI2_SS_PINCM                (IOMUX_PINCM9)
#define DRIVER_SPI2_SS_PF                   (IOMUX_PINCM9_PF_SPI2_CS0)
#define DRIVER_SPI2_SCLK_PINCM              (IOMUX_PINCM21)
#define DRIVER_SPI2_SCLK_PF                 (IOMUX_PINCM21_PF_SPI2_SCLK)
#define DRIVER_SPI2_CLOCK_SEL               (DRIVER_CLK_MSP_BUSCLK)
#define DRIVER_SPI2_CLOCK_FREQ              (32000000U)
#define DRIVER_SPI2_TRANSMIT_DMA_HW         (DRIVER_DMA_HW_NONE)
#define DRIVER_SPI2_TRANSMIT_DMA_CH         (DRIVER_DMA_CH_NONE)
#define DRIVER_SPI2_TRANSMIT_DMA_TRIG       (DRIVER_DMA_TRIG_NONE)
#define DRIVER_SPI2_RECEIVE_DMA_HW          (DRIVER_DMA_HW_NONE)
#define DRIVER_SPI2_RECEIVE_DMA_CH          (DRIVER_DMA_CH_NONE)
#define DRIVER_SPI2_RECEIVE_DMA_TRIG        (DRIVER_DMA_TRIG_NONE)
#endif

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_CONFIG_MSP_H_ */
