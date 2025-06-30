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
 *  @file       Driver_SPI_MSP_Priv.h
 *  @brief      MSP SPI CMSIS-Driver for Cortex-M devices (V0 HW version)
 * 
 *  The CMSIS-Drivers SPI driver allows for simple, non-blocking SPI
 *  send and receive communication via a high-level, Arm-standardized API.
 *  This driver is developed to be compatible with the Arm CMSIS-Driver
 *  specification.
 * 
 *  This driver implements synchronous communication mode via DMA or
 *  interrupt-driven input/output.
 *
 *  This module is the private (internal) driver layer for SPI.  Applications
 *  are expected to use the driver through the Driver_SPI_MSP.h layer.
 *
 * 
 *  Known limitations:
 *
 ******************************************************************************
 */

#ifndef DRIVER_SPI_MSP_PRIV_H_
#define DRIVER_SPI_MSP_PRIV_H_

#include <Driver_SPI.h>
#include <Driver_Common_MSP.h>

/*!
 * @brief The CMSIS-Driver SPI MSP driver version (major/minor)
 */
#define ARM_SPI_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(0, 30)

/**
 * @brief Stores a pointer to the address of the MSP SPI hardware registers.
 *        for the corresponding SPI hardware peripheral instance.
 */
typedef SPI_Regs DRIVER_SPI_MSP_HW;

/**
 * @brief Define value for driver operating in target mode.
 */
#define DIRECTION_SLAVE 0

/**
 * @brief Define value for driver operating in controller mode.
 */
#define DIRECTION_MASTER 1

/**
 * @brief Define value for module inactive.
 */
#define DRIVER_INACTIVE 0

/**
 * @brief Define value for module active.
 */
#define DRIVER_ACTIVE 1

/**
 * @brief Define value for module inactive.
 */
#define DRIVER_NOT_BUSY 0

/**
 * @brief Define value for module active.
 */
#define DRIVER_BUSY 1

/**
 * @brief Define value for data lost flag clear
 */
#define DATA_LOST_CLEAR 0

/**
 * @brief Define value for data lost flag set
 */
#define DATA_LOST_SET 1

/**
 * @brief Define value for mode fault clear. There is not corresponding set flag
 *        since mode fault is not supported.
 */
#define MODE_FAULT_CLEAR 0

/**
 * @brief Define value for maximum possible SCR register value. If the requested
 *        bus speed would require a SCR value that exceed this, it will not be
 *        supported.
 */
#define MAX_SCR_VALUE 0x3FF

/**
 * @brief Enum data structure to capture the state of the SPI bus. 
 */
typedef enum SPI_STATE {
	SPI_STATE_IDLE,
    SPI_STATE_SEND_WAITING,
    SPI_STATE_SEND_IN_PROGRESS,
    SPI_STATE_RECEIVE_WAITING,
    SPI_STATE_RECEIVE_IN_PROGRESS,
    SPI_STATE_TRANSFER_IN_PROGRESS
} SPI_STATE;

/**
 * @brief Driver instance state data structure to hold run-time modified
 *        state information for the driver.  One structure is used per module
 *        instance deployed in an application.  This structure must be stored
 *        in SRAM and be read/write accessible by the driver.
 */
typedef struct
{
    /*! Pointer to callback function for event handling. */
    ARM_SPI_SignalEvent_t callback;
    /* Driver status structure */
    ARM_SPI_STATUS status;
    /* State of the SPI Driver */
    SPI_STATE spiState;
    /* Current power state */
    ARM_POWER_STATE powerState;
    /* Pointer to transmit buffer (not modified) */
    const void *txBuf;
    /* Pointer to receive buffer */
    void *rxBuf;
    /* Current received item count. 
    * Resets to zero at start of new receive operation. */
    uint32_t rxCnt;
    /* Current received item target count.
    * Set by calling application.
    * Receive completion signal is asserted when rxCnt == rxTarCnt. */
    uint32_t rxTarCnt;
    /* Current transmitted item count. 
    * Resets to zero at start of new transmit operation. */
    uint32_t txCnt;
    /* Current transmitted item target count.
    * Set by calling application.
    * Transmit completion signal is asserted when txCnt == txTarCnt. */
    uint32_t txTarCnt;
    /* Number of bytes sent in one frame */
    uint8_t bytesPerFrame;
    /* Default transmission value, set by Control */
    uint32_t defaultTxValue;
    /* SPI active flag */
    uint8_t active  : 1;
    /* SPI peripheral direction */
    uint8_t direction : 1;
    /* Slave select pin controlled by SW */
    uint8_t ssPinSWControlled : 1;
    /* Transmitter DMA available */
    uint8_t transmitDMAAvail : 1;
    /* Receiver DMA available */
    uint8_t receiveDMAAvail : 1;
} DRIVER_SPI_MSP_STATE;

/**
 * @brief Driver instance configuration data structure to hold the configuration
 *        information for each driver instance, including hw module access,
 *        pin / clock / irq definitions, and a link to the state (r/w) data for
 *        the corresponding instance.  This is the primary data structure used 
 *        by the driver and a pointer to this structure is passed to most APIs
 *        in the driver.  This is a static data structure and may be stored in
 *        read-only memory for better memory layout optimization.
 */
typedef struct
{
    /*! Pointer to SPI module base address used by this driver instance. */
    DRIVER_SPI_MSP_HW *hw;
    /*! Pointer to MSP SPI driver state data structure. */
    DRIVER_SPI_MSP_STATE *state;
    /*! Slave Select (Active Low) pin definition */
    DRIVER_IO_MSP ssPin;
    /*! Port location of SS pin */
    GPIO_Regs *ssGPIOPort;
    /*! Pin number of SS pin */
    uint32_t ssPinNumber;
    /*! Master Out, Slave In pin definition */
    DRIVER_IO_MSP mosiPin;
    /*! Serial clock pin definition */
    DRIVER_IO_MSP sclkPin;
    /*! Master In, Slave Out pin definition */
    DRIVER_IO_MSP misoPin;
    /*! Clock source definition */
    DRIVER_CLK_MSP clock;
    /*! Clock source definition */
    uint32_t clockFreq;
    /*! Transmitter DMA (optional) */
    DRIVER_DMA_MSP transmitDMA;
    /*! Receiver DMA (optional) */
    DRIVER_DMA_MSP receiveDMA;
    /*! Peripheral Slave Select Number (CS0-CS3) */
    uint8_t ss;
    /*! SPI module NVIC interrupt port */
    uint8_t irq;
} DRIVER_SPI_MSP;

/**
 * @brief DriverVersion stores the ARM CMSIS-Driver API version
 *        and the CMSIS-Driver MSP driver version.
 */
static const ARM_DRIVER_VERSION DriverVersion = 
{ 
    ARM_SPI_API_VERSION,
    ARM_SPI_DRV_VERSION
};

/**
 *  @brief      Get the version of the SPI driver.
 * 
 *  @return     ARM_DRIVER_VERSION with API and DRV versions encoded.
 * 
 */
extern ARM_DRIVER_VERSION MSP_ARM_SPI_GetVersion(void);

/**
 *  @brief      Get the capabilities of the SPI driver.
 * 
 *  @return     ARM_SPI_CAPABILITIES with supported features set to '1'
 * 
 */
extern ARM_SPI_CAPABILITIES MSP_ARM_SPI_GetCapabilities(
    DRIVER_SPI_MSP *module);

/**
 *  @brief      Initialize the SPI driver instance.  This is the first
 *              function expected to be called in the driver use sequence.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 *  @param[in]  cb_event Pointer to the application callback function.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK
 *
 *  @pre        The peripheral instance (hardware) is expected to be disabled.
 * 
 *  @post       The MOSI, MISO, SCLK, and CS pins, if not DRIVER_IO_MSP_NONE,
 *              are configured with the defined pin functions.  The
 *              callback function, if provided, is linked to the driver
 *              instance.  The SPI bus state is reset to idle. 
 *              Counters rxCnt, rxTarCnt, txCnt, txTarCnt are reset to 0U. 
 *              The mode of the SPI peripheral (master or slave) 
 *              is reset to none.  
 * 
 */
extern int32_t MSP_ARM_SPI_Initialize(DRIVER_SPI_MSP *module, 
    ARM_SPI_SignalEvent_t cb_event);

/**
 *  @brief      Uninitialize the SPI driver instance.  This is the last
 *              function expected to be called in the driver use sequence.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK
 *
 *  @pre        The peripheral instance (hardware) is expected to be disabled.
 * 
 *  @post       The MOSI, MISO, SCLK, and CS pins, if not DRIVER_IO_MSP_NONE,
 *              are unmuxed from the defined pin functions and are left Hi-Z.  
 *              The callback function is unlinked and set to NULL in the
 *              driver instance data structure.
 * 
 */
extern int32_t MSP_ARM_SPI_Uninitialize(DRIVER_SPI_MSP *module);

/**
 *  @brief      Set the power state of the SPI driver instance to 
 *              either ARM_POWER_FULL or ARM_POWER_OFF.  ARM_POWER_LOW
 *              is not supported.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[in]  state The ARM power state to put the driver in.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if successful.
 *
 *  @pre        The MSP_ARM_SPI_Initialize() function has already been
 *              called.
 * 
 *  @post       The SPI driver instance is configured with standard clock
 *              configuration. The direction of the perihperal is left 
 *              uninitialized.
 * 
 */
extern int32_t MSP_ARM_SPI_PowerControl(DRIVER_SPI_MSP *module, 
    ARM_POWER_STATE state);

/**
 *  @brief      Control the operational properties of a SPI driver instance,
 *              including configuring bus speed, master or slave mode, and
 *              other configuration properties of the driver. Also includes 
 *              operational commands to cancel an ongoing trasnfer.
 *                                    
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[in]  control The control operation to execute:
 *              Supported standard operations include:
 *                  - Mode Controls
 *                      - ARM_SPI_MODE_INACTIVE, ARM_SPI_MODE_MASTER, or
 *                        ARM_SPI_MODE_SLAVE
 *                  - Clock Polarity (Frame Format)
 *                      - ARM_SPI_CPOL0_CPHA0 (default), ARM_SPI_CPOL0_CPHA1,
 *                        ARM_SPI_CPOL1_CPHA0, ARM_SPI_CPOL1_CPHA1,
 *                        ARM_SPI_TI_SSI, or ARM_SPI_MICROWIRE
 *                  - Data Bits:
 *                      - ARM_SPI_DATA_BITS(n), where n in range [4, 16]
 *                  - Bit Order
 *                      - ARM_SPI_MSB_LSB (default) or ARM_SPI_LSB_MSB
 *                  - Slave Select
 *                      - When Master:
 *                          - ARM_SPI_SS_MASTER_UNUSED (default), 
 *                            ARM_SPI_SS_MASTER_SW, ARM_SPI_SS_MASTER_HW_OUTPUT,
 *                            or ARM_SPI_SS_MASTER_HW_INPUT (unsupported)
 *                      - When Slave:
 *                          - ARM_SPI_SS_SLAVE_HW (default) or 
 *                            ARM_SPI_SS_SLAVE_SW
 *                  - Miscellaneous Controls (cannot be ORed)
 *                      - ARM_SPI_SET_BUS_SPEED (arg = bus speed in bps),
 *                        ARM_SPI_GET_BUS_SPEED, ARM_SPI_DEFAULT_TX_VALUE (arg
 *                        = default tx value), ARM_SPI_CONTROL_SS (arg is one
 *                        of: ARM_SPI_SS_IANCTIVE, ARM_SPI_SS_ACTIVE), or 
 *                        ARM_SPI_ABORT_TRANSFER
 * 
 *  @param[in]  arg The optional argument field to the control operation
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if successful.
 *  @retval     ARM_DRIVER_ERROR_PARAMETER if the parameter is not valid.
 *  @retval     ARM_DRIVER_ERROR if driver power state is not ARM_POWER_FULL.
 *  @retval     ARM_DRIVER_ERROR_BUSY if a send/receive operation is ongoing.
 *  @retval     ARM_DRIVER_ERROR_UNSUPPORTED if ARM_I2C_BUS_SPEED_HIGH is.
 *              passed in.
 * 
 *  @pre        The MSP_ARM_SPI_Initialize() function has already been
 *              called and the power mode is set to ARM_POWER_FULL and
 *              there is no active send or receive operation, unless the
 *              control is the abort command in which case
 *              it is expected that an operation is active.
 * 
 *  @post       The new commanded operational state is set, provided no
 *              errors occurred.
 */
extern int32_t MSP_ARM_SPI_Control(DRIVER_SPI_MSP *module, uint32_t control,
    uint32_t arg);

/**
 *  @brief      Send the contents of a data buffer using interrupt or DMA 
 *              driven data transfer. If the driver is configued as a slave,
 *              the operation is only registered. 
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[in]  data A pointer to the data buffer to send (8-bit or 16-bit data)
 * 
 *  @param[in]  num The number of data items to send
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if successful
 *  @retval     ARM_DRIVER_ERROR_PARAMETER if data is NULL or num is 0U
 *  @retval     ARM_DRIVER_ERROR_BUSY if a previous send is still in progress
 *
 *  @pre        The MSP_ARM_SPI_Initialize() function has already been
 *              called, MSP_ARM_SPI_PowerControl() function has been
 *              called to set power state to ARM_POWER_FULL,
 *              and no previous send operation is still in progress.
 * 
 *  @post       The SPI driver is processing the send via interrupt
 *              driven or DMA data transfers and returns without blocking.
 * 
 */
extern int32_t MSP_ARM_SPI_Send(DRIVER_SPI_MSP *module, const void *data,
    uint32_t num);

/**
 *  @brief      Receives data into specified target buffer. If the driver is 
 *              configued as a slave, the operation is only registered. If the
 *              driver is configued as a master, it will transmit the default
 *              value set by ARM_SPI_Control with ARM_SPI_SET_DEFAULT_TX_VALUE.
 *              Data type of buffer is uint8_t or uint16_t, depending on 
*               ARM_SPI_DATA_BITS(n) with ARM_SPI_Control.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[in]  data A pointer to the data buffer to receive into
 * 
 *  @param[in]  num The number of data items to receive
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if successful
 *  @retval     ARM_DRIVER_ERROR_PARAMETER if data is NULL or num is 0U
 *  @retval     ARM_DRIVER_ERROR_BUSY if a previous send is still in progress
 *
 *  @pre        The MSP_ARM_SPI_Initialize() function has already been
 *              called, MSP_ARM_SPI_PowerControl() function has been
 *              called to set power state to ARM_POWER_FULL,
 *              and no previous operation is still in progress.
 * 
 *  @post       The SPI driver is processing the receive via interrupt
 *              driven or DMA data transfers and returns without blocking.
 * 
 */
extern int32_t MSP_ARM_SPI_Receive(DRIVER_SPI_MSP *module, void *data,
    uint32_t num);

/**
 *  @brief      Simultaneously send and receive data into specified buffers. 
 *              If the driver is configued as a slave, the operation is only 
 *              registered. Data type of buffers are uint8_t or uint16_t, 
 *               depending on ARM_SPI_DATA_BITS(n) with ARM_SPI_Control.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
*   @param[in]  data_out A pointer to the data buffer to send
 *
 *  @param[in]  data_in A pointer to the data buffer to receive into
 * 
 *  @param[in]  num The number of data items to receive
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if successful
 *  @retval     ARM_DRIVER_ERROR_PARAMETER if data is NULL or num is 0U
 *  @retval     ARM_DRIVER_ERROR_BUSY if a previous send is still in progress
 *
 *  @pre        The MSP_ARM_SPI_Initialize() function has already been
 *              called, MSP_ARM_SPI_PowerControl() function has been
 *              called to set power state to ARM_POWER_FULL,
 *              and no previous operation is still in progress.
 * 
 *  @post       The SPI driver is processing the transfer via interrupt
 *              driven or DMA data transfers and returns without blocking.
 * 
 */
extern int32_t MSP_ARM_SPI_Transfer(DRIVER_SPI_MSP *module,
    const void *data_out, void *data_in, uint32_t num);

/**
 *  @brief      Get the number of data bytes transferred by the last operation.
 * 
 *  @return     The return value depends on the last operation:
 *                  - ARM_SPI_Send: number of data bytes sent
 *                  - ARM_SPI_Receive: number of data bytes received
 *                  - ARM_SPI_Transfer: number of data bytes transferred
 * 
 */
extern uint32_t MSP_ARM_SPI_GetDataCount(DRIVER_SPI_MSP *module);

/**
 *  @brief      Get the status of the SPI driver instance.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @return     ARM_SPI_STATUS reflecting current driver instance status.
 * 
 */
extern ARM_SPI_STATUS MSP_ARM_SPI_GetStatus(DRIVER_SPI_MSP *module);

/**
 *  @brief      This is the SPI driver interrupt handler to be called by the
 *              wrapping interrupt service routine for each HW instance.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @return     None.
 * 
 */
extern void MSP_ARM_SPI_IRQHandler(DRIVER_SPI_MSP *module);

#endif /* DRIVER_SPI_MSP_PRIV_H_ */