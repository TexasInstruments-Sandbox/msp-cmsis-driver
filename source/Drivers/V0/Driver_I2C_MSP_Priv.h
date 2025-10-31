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
 *  @file       Driver_I2C_MSP_Priv.h
 *  @brief      MSP I2C CMSIS-Driver for Cortex-M devices (V0 HW version)
 * 
 *  The CMSIS-Drivers I2C driver allows for simple, non-blocking I2C
 *  master/slave transmit/receive communication via a high-level, 
 *  Arm-standardized API. This driver is developed to be compatible with
 *  the Arm CMSIS-Driver specification.
 *
 *  This module is the private (internal) driver layer for I2C.  Applications
 *  are expected to use the driver through the Driver_I2C_MSP.h layer.
 *
 *  Known limitations:
 *
 ******************************************************************************
 */

#ifndef DRIVER_I2C_MSP_PRIV_H_
#define DRIVER_I2C_MSP_PRIV_H_

#include <Driver_I2C.h>
#include <Driver_Common_MSP.h>

/*!
 * @brief The CMSIS-Driver I2C MSP driver version (major/minor)
 */
#define ARM_I2C_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(0, 10)

/**
 * @brief Stores a pointer to the address of the MSP I2C hardware registers.
 *        for the corresponding I2C hardware peripheral instance.
 */
typedef I2C_Regs DRIVER_I2C_MSP_HW;

/**
 * @brief Define value for driver uninitialized.
 */
#define DRIVER_UNINITIALIZED 0

/**
 * @brief Define value for driver initialized.
 */
#define DRIVER_INITIALIZED 1


/**
 * @brief Enum data structure to capture the state of the I2C bus. 
 */
typedef enum I2C_STATE {
	I2C_STATE_IDLE,
	I2C_STATE_TX_STARTED,
	I2C_STATE_TX_INPROGRESS,
	I2C_STATE_TX_COMPLETE,
	I2C_STATE_RX_STARTED,
	I2C_STATE_RX_INPROGRESS,
	I2C_STATE_RX_COMPLETE
} I2C_STATE;

/**
 * @brief Driver instance state data structure to hold run-time modified
 *        state information for the driver.  One structure is used per module
 *        instance deployed in an application.  This structure must be stored
 *        in SRAM and be read/write accessible by the driver.
 */
typedef struct
{
    /*! Pointer to callback function for event handling. */
    ARM_I2C_SignalEvent_t callback;
    /* Driver status structure */
    ARM_I2C_STATUS status;
    /* Current power state */
    ARM_POWER_STATE powerState;
    /* Pointer to transmit buffer (not modified) */
    const uint8_t *txBuf;
    /* Pointer to receive buffer */
    uint8_t *rxBuf;
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
    /* State of the I2C Driver */
    I2C_STATE i2cState;
    /* I2C initialization done flag */
    uint8_t initDone : 1;
    /* I2C Controller mode enabled */
    uint8_t controllerEnabled : 1;
    /* I2C Target mode enabled */
    uint8_t targetEnabled : 1;
    /* Transmitter DMA available */
    uint8_t transmitDMAAvail : 1;
    /* Receiver DMA available */
    uint8_t receiveDMAAvail : 1;
} DRIVER_I2C_MSP_STATE;

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
    /*! Pointer to I2C module base address used by this driver instance. */
    DRIVER_I2C_MSP_HW *hw;
    /*! Pointer to MSP I2C driver state data structure. */
    DRIVER_I2C_MSP_STATE *state;
    /*! SDA pin definition */
    DRIVER_IO_MSP sdaPin;
    /*! SCL pin definition */
    DRIVER_IO_MSP sclPin;
    /*! Clock source definition */
    DRIVER_CLK_MSP clock;
    /*! Clock source definition */
    uint32_t clockFreq;
    /*! I2C module NVIC interrupt port */
    uint8_t irq;
    /*! Transmitter DMA (optional) */
    DRIVER_DMA_MSP transmitDMA;
    /*! Receiver DMA (optional) */
    DRIVER_DMA_MSP receiveDMA;
} DRIVER_I2C_MSP;

/**
 * @brief DriverVersion stores the ARM CMSIS-Driver API version
 *        and the CMSIS-Driver MSP driver version.
 */
static const ARM_DRIVER_VERSION DriverVersion = 
{ 
    ARM_I2C_API_VERSION,
    ARM_I2C_DRV_VERSION
};

/**
 *  @brief      Get the version of the I2C driver.
 * 
 *  @return     ARM_DRIVER_VERSION with API and DRV versions encoded.
 * 
 */
extern ARM_DRIVER_VERSION MSP_ARM_I2C_GetVersion(void);

/**
 *  @brief      Get the capabilities of the I2C driver.
 * 
 *  @return     ARM_I2C_CAPABILITIES with supported features set to '1'
 * 
 */
extern ARM_I2C_CAPABILITIES MSP_ARM_I2C_GetCapabilities(
    DRIVER_I2C_MSP *module);

/**
 *  @brief      Initialize the I2C driver instance.  This is the first
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
 *  @post       The SDA and SCL pins, if not DRIVER_IO_MSP_NONE,
 *              are configured with the defined pin functions.  The
 *              callback function, if provided, is linked to the driver
 *              instance.  The I2C bus state is reset to idle. 
 *              Counters rxCnt, rxTarCnt, txCnt, txTarCnt are reset to 0U. 
 *              The direction of the I2C peripheral (master or slave) 
 *              is reset to none.  
 * 
 */
extern int32_t MSP_ARM_I2C_Initialize(DRIVER_I2C_MSP *module, 
    ARM_I2C_SignalEvent_t cb_event);

/**
 *  @brief      Uninitialize the I2C driver instance.  This is the last
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
 *  @post       The SCL and SDA pins, if not DRIVER_IO_MSP_NONE,
 *              are unmuxed from the defined pin functions and are left Hi-Z.  
 *              The callback function is unlinked and set to NULL in the
 *              driver instance data structure.
 * 
 */
extern int32_t MSP_ARM_I2C_Uninitialize(DRIVER_I2C_MSP *module);

/**
 *  @brief      Set the power state of the I2C driver instance to 
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
 *  @pre        The MSP_ARM_I2C_Initialize() function has already been
 *              called.
 * 
 *  @post       The I2C driver instance is configured with standard clock
 *              configuration. The controller and target modes are left disabled 
 *              until enabled by the application.
 * 
 */
extern int32_t MSP_ARM_I2C_PowerControl(DRIVER_I2C_MSP *module, 
    ARM_POWER_STATE state);

/**
 *  @brief      Configures the hardware instance to be in controller mode.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @return     None
 *
 *  @pre        The MSP_ARM_I2C_Initialize() and MSP_ARM_I2C_PowerControl() 
 *              functions have already been called.
 * 
 *  @post       The I2C driver instance is configured to act in controller mode.
 * 
 */
extern void MSP_ARM_I2C_ConfigureController(DRIVER_I2C_MSP *module);

/**
 *  @brief      Configures the hardware instance to be in target mode
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 *  
 *  @return     None
 *
 *  @pre        The MSP_ARM_I2C_Initialize() and MSP_ARM_I2C_PowerControl() 
 *              functions have already been called.
 * 
 *  @post       The I2C driver instance is configured to act in target mode.
 * 
 */
extern void MSP_ARM_I2C_ConfigureTarget(DRIVER_I2C_MSP *module);

/**
 *  @brief      Send the contents of a data buffer from master to target with 
 *              specified address via using interrupt or DMA 
 *              driven data transfer.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[in]  addr Slave address (7-bit or 10-bit)
 * 
 *  @param[in]  data A pointer to the data buffer to send (8-bit data)
 * 
 *  @param[in]  num The number of 8-bit data items to send (length of data)
 * 
 *  @param[in]  xfer_pending Transfer operation is pending: 
 *                           stop condition will not be generated.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if successful
 *  @retval     ARM_DRIVER_ERROR_PARAMETER if data is NULL or num is 0U
 *  @retval     ARM_DRIVER_ERROR_BUSY if a previous send is still in progress
 *
 *  @pre        The MSP_ARM_I2C_Initialize() function has already been
 *              called, MSP_ARM_I2C_PowerControl() function has been
 *              called to set power state to ARM_POWER_FULL,
 *              and no previous send operation is still in progress.
 * 
 *  @post       The I2C driver is processing the send via interrupt
 *              driven or DMA data transfers and returns without blocking.
 * 
 */
extern int32_t MSP_ARM_I2C_MasterTransmit(DRIVER_I2C_MSP *module, 
    uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending);

/**
 *  @brief      Receives num bytes of data from the target with the 
 *              specified address into data buffer.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[in]  addr Slave address (7-bit or 10-bit)
 * 
 *  @param[in]  data A pointer to the data buffer to receive (8-bit data)
 * 
 *  @param[in]  num The number of 8-bit data items to receive (length of data)
 * 
 *  @param[in]  xfer_pending Transfer operation is pending: 
 *                           stop condition will not be generated.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if successful
 *  @retval     ARM_DRIVER_ERROR_PARAMETER if data is NULL or num is 0U
 *  @retval     ARM_DRIVER_ERROR_BUSY if a previous send is still in progress
 *
 *  @pre        The MSP_ARM_I2C_Initialize() function has already been
 *              called, MSP_ARM_I2C_PowerControl() function has been
 *              called to set power state to ARM_POWER_FULL,
 *              and no previous send operation is still in progress.
 * 
 *  @post       The I2C driver is processing the receive via interrupt
 *              driven or DMA data transfers and returns without blocking.
 * 
 */
extern int32_t MSP_ARM_I2C_MasterReceive(DRIVER_I2C_MSP *module, 
    uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending);

/**
 *  @brief      Send the contents of a data buffer from slave to master with 
 *              using interrupt or DMA driven data transfer.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[in]  data A pointer to the data buffer to send (8-bit data)
 * 
 *  @param[in]  num The number of 8-bit data items to send (length of data)
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if successful
 *  @retval     ARM_DRIVER_ERROR_PARAMETER if data is NULL or num is 0U
 *  @retval     ARM_DRIVER_ERROR_BUSY if a previous send is still in progress
 *
 *  @pre        The MSP_ARM_I2C_Initialize() function has already been
 *              called, MSP_ARM_I2C_PowerControl() function has been
 *              called to set power state to ARM_POWER_FULL,
 *              and no previous send operation is still in progress.
 * 
 *  @post       The I2C driver is processing the send via interrupt
 *              driven or DMA data transfers and returns without blocking.
 * 
 */
extern int32_t MSP_ARM_I2C_SlaveTransmit(DRIVER_I2C_MSP *module, 
    const uint8_t *data, uint32_t num);

/**
 *  @brief      Receives num bytes of data from the master into the data buffer.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[in]  data A pointer to the data buffer to receive (8-bit data)
 * 
 *  @param[in]  num The number of 8-bit data items to receive (length of data)
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if successful
 *  @retval     ARM_DRIVER_ERROR_PARAMETER if data is NULL or num is 0U
 *  @retval     ARM_DRIVER_ERROR_BUSY if a previous send is still in progress
 *
 *  @pre        The MSP_ARM_I2C_Initialize() function has already been
 *              called, MSP_ARM_I2C_PowerControl() function has been
 *              called to set power state to ARM_POWER_FULL,
 *              and no previous send operation is still in progress.
 * 
 *  @post       The I2C driver is processing the receive via interrupt
 *              driven or DMA data transfers and returns without blocking.
 * 
 */
extern int32_t MSP_ARM_I2C_SlaveReceive(DRIVER_I2C_MSP *module, uint8_t *data, 
    uint32_t num);

/**
 *  @brief      Get the number of data bytes transferred by the last operation.
 * 
 *  @return     The return value depends on the last operation:
 *                  - ARM_I2C_MasterTransmit: number of data bytes transmitted 
                                              and acknowledged
 *                  - ARM_I2C_MasterReceive: number of data bytes received
 *                  - ARM_I2C_SlaveTransmit: number of data bytes transmitted
 *                  - ARM_I2C_SlaveReceive: number of data bytes received 
                                            and acknowledged
 * 
 */
extern int32_t MSP_ARM_I2C_GetDataCount(DRIVER_I2C_MSP *module);

/**
 *  @brief      Control the operational properties of a I2C driver instance,
 *              including configuring bus speed and the target address 
 *              to respond to. Also includes operational commands to cancel 
 *              ongoing operations and clear the I2C bus.
 *                                    
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[in]  control The control operation to execute:
 *              Supported standard operations include:
 *                  - ARM_I2C_OWN_ADDRESS
 *                      - Set Own Slave Address: requires arg parameter set 
                                                 to slave address
 *                      - Slave Address may be OR'd with ARM_I2C_ADDRESS_GC 
 *                        to indicate the slave accepts a General Call. Slave 
                          address value may be set to only ARM_I2C_ADDRESS_GC.
 *                  - ARM_I2C_BUS_SPEED:
 *                      - ARM_I2C_BUS_SPEED_STANDARD
 *                      - ARM_I2C_BUS_SPEED_FAST
 *                      - ARM_I2C_BUS_SPEED_FAST_PLUS
 *                      - ARM_I2C_BUS_SPEED_HIGH (not supported)
 *                  - ARM_I2C_BUS_CLEAR
 *                  - ARM_I2C_ABORT_TRANSFER
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
 *  @pre        The MSP_ARM_I2C_Initialize() function has already been
 *              called and the power mode is set to ARM_POWER_FULL and
 *              there is no active send or receive operation, unless the
 *              control is the abort command in which case
 *              it is expected that an operation is active.
 * 
 *  @post       The new commanded operational state is set, provided no
 *              errors occurred.
 * 
 */
extern int32_t MSP_ARM_I2C_Control(DRIVER_I2C_MSP *module, uint32_t control, 
    uint32_t arg);

/**
 *  @brief      Get the status of the I2C driver instance.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @return     ARM_I2C_STATUS reflecting current driver instance status.
 * 
 */
extern ARM_I2C_STATUS MSP_ARM_I2C_GetStatus(DRIVER_I2C_MSP *module);

/**
 *  @brief      This is the I2C driver interrupt handler to be called by the
 *              wrapping interrupt service routine for each HW instance.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @return     None.
 * 
 */
extern void MSP_ARM_I2C_IRQHandler(DRIVER_I2C_MSP *module);

#endif /* DRIVER_I2C_MSP_PRIV_H_ */
