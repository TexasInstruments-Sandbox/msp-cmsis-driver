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
 *  @file       Driver_USART_MSP_Priv.h
 *  @brief      MSP USART CMSIS-Driver for Cortex-M devices (V0 HW version)
 * 
 *  The CMSIS-Drivers USART driver allows for simple, non-blocking UART
 *  send and receive communication via a high-level, Arm-standardized API.
 *  This driver is developed to be compatible with the Arm CMSIS-Driver
 *  specification.
 * 
 *  This driver implements asynchronous communication mode via DMA or
 *  interrupt-driven input/output.  It does not implement synchronous mode
 *  communication, as the underlying hardware peripheral is asynchronous only.
 *
 *  This module is the private (internal) driver layer for USART.  Applications
 *  are expected to use the driver through the Driver_USART_MSP.h layer.
 *
 *  V0 implementation covers MSP devices with the UART main or UART extend 
 *  peripheral instances.
 * 
 *  Known limitations:
 *      LIMITATION-1: IRDA is not currently supported.
 *      LIMITATION-2: Smart card mode is not currently supported.
 *      LIMITATION-3: Synchronous modes are not supported.
 *      LIMITATION-4: Single wire mode is not supported.
 *      LIMITATION-5: DTR, DCD, RI lines are not supported.
 *
 ******************************************************************************
 */

#ifndef DRIVER_USART_MSP_PRIV_H_
#define DRIVER_USART_MSP_PRIV_H_

#include <Driver_USART.h>
#include <Driver_Common_MSP.h>

/*!
 * @brief The CMSIS-Driver USART MSP driver version (major/minor)
 */
#define ARM_USART_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)

/*!
 * @brief The USART driver RX timeout value, defaulted to MAX value.
 *        Currently this is a hard-coded value and is not broken out 
 *        as a parameter on a per-instance basis.  It can be modified 
 *        by a user within the driver if desired by changing this value.  
 *        The RX timeout value here is used to generate the 
 *        ARM_USART_EVENT_RX_PARITY_ERROR event signal.
 */
#define DRIVER_USART_MSP_TIMEOUT_BIT_PERIODS (0x0FU)

/**
 * @brief Define value for driver uninitialized.
 */
#define DRIVER_UNINITIALIZED 0

/**
 * @brief Define value for driver initialized.
 */
#define DRIVER_INITIALIZED 1
 
/**
 * @brief Stores a pointer to the address of the MSP UART hardware registers.
 *        for the corresponding UART hardware peripheral instance.
 */
typedef UART_Regs DRIVER_USART_MSP_HW;
 
/**
 * @brief Driver instance state data structure to hold run-time modified
 *        state information for the driver.  One structure is used per module
 *        instance deployed in an application.  This structure must be stored
 *        in SRAM and be read/write accessible by the driver.
 */
typedef struct
{
    /*! Pointer to callback function for event handling. */
    ARM_USART_SignalEvent_t callback;
    /* Driver status structure */
    ARM_USART_STATUS status;
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
    /* USART initialization done flag */
    uint8_t initDone : 1;
    /* Transmitter enable */
    uint8_t transmitterEnabled : 1;
    /* Receiver enable */
    uint8_t receiverEnabled : 1;
    /* Transmitter DMA available */
    uint8_t transmitterDMAAvail : 1;
    /* Receiver DMA available */
    uint8_t receiverDMAAvail : 1;
} DRIVER_USART_MSP_STATE;
 
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
    /*! Pointer to UART module base address used by this driver instance. */
    DRIVER_USART_MSP_HW *hw;
    /*! Pointer to MSP USART driver state data structure. */
    DRIVER_USART_MSP_STATE *state;
    /*! TX pin definition */
    DRIVER_IO_MSP txPin;
    /*! RX pin definition */
    DRIVER_IO_MSP rxPin;
    /*! RTS pin definition */
    DRIVER_IO_MSP rtsPin;
    /*! CTS pin definition */
    DRIVER_IO_MSP ctsPin;
    /*! Clock source definition */
    DRIVER_CLK_MSP clock;
    /*! Clock source definition */
    uint32_t clockFreq;
    /*! UART module NVIC interrupt port */
    uint8_t irq;
    /*! Transmitter DMA (optional) */
    DRIVER_DMA_MSP txDMA;
    /*! Receiver DMA (optional) */
    DRIVER_DMA_MSP rxDMA;
} DRIVER_USART_MSP;

/**
 * @brief DriverVersion stores the ARM CMSIS-Driver API version
 *        and the CMSIS-Driver MSP driver version.
 */
static const ARM_DRIVER_VERSION DriverVersion = 
{ 
    ARM_USART_API_VERSION,
    ARM_USART_DRV_VERSION
};

/**
 *  @brief      Get the version of the USART driver.
 * 
 *  @return     ARM_DRIVER_VERSION with API and DRV versions encoded.
 * 
 */
extern ARM_DRIVER_VERSION MSP_ARM_USART_GetVersion(void);

/**
 *  @brief      Get the capabilities of the USART driver.
 * 
 *  @return     ARM_USART_CAPABILITIES with supported features set to '1'
 * 
 */
extern ARM_USART_CAPABILITIES MSP_ARM_USART_GetCapabilities(
    DRIVER_USART_MSP *module);

/**
 *  @brief      Initialize the USART driver instance.  This is the first
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
 *  @post       The TX, RX, RTS, and CTS pins, if not DRIVER_IO_MSP_NONE,
 *              are configured with the defined pin functions.  The
 *              callback function, if provided, is linked to the driver
 *              instance.  Status indicators for tx_busy, tx_underflow,
 *              rx_break, rx_busy, rx_framing_error, rx_overflow,
 *              rx_parity_error are reset to 0U.  Counters rxCnt, rxTarCnt,
 *              txCnt, txTarCnt are reset to 0U.  Transmit/receive enables
 *              are disabled (0U).
 * 
 */
extern int32_t MSP_ARM_USART_Initialize(DRIVER_USART_MSP *module, 
    ARM_USART_SignalEvent_t cb_event);

/**
 *  @brief      Uninitialize the USART driver instance.  This is the last
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
 *  @post       The TX, RX, RTS, and CTS pins, if not DRIVER_IO_MSP_NONE,
 *              are unmuxed from the defined pin functions and are left Hi-Z.  
 *              The callback function is unlinked and set to NULL in the
 *              driver instance data structure.
 * 
 */
extern int32_t MSP_ARM_USART_Uninitialize(DRIVER_USART_MSP *module);

/**
 *  @brief      Set the power state of the USART driver instance to 
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
 *  @retval     ARM_USART_ERROR_BAUDRATE if baud rate default of 9600 could
 *              not be set when moving to ARM_POWER_FULL due to mismatch
 *              with functional clock frequency for the module.
 *
 *  @pre        The MSP_ARM_USART_Initialize() function has already been
 *              called.
 * 
 *  @post       The USART driver instance is configured in asynchronous
 *              8N1 mode at 9600 baud with receiver and transmitter
 *              modes left disabled until enabled by the application.
 * 
 */
extern int32_t MSP_ARM_USART_PowerControl(DRIVER_USART_MSP *module, \
    ARM_POWER_STATE state);

/**
 *  @brief      Send the contents of a data buffer out on the line via
 *              the USART driver using interrupt or DMA driven data transfer.
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
 *  @retval     ARM_DRIVER_ERROR if transmitter was not enabled
 *  @retval     ARM_DRIVER_ERROR_BUSY if a previous send is still in progress
 *
 *  @pre        The MSP_ARM_USART_Initialize() function has already been
 *              called, MSP_ARM_USART_PowerControl() function has been
 *              called to set power state to ARM_POWER_FULL,
 *              the MSP_ARM_USART_Control() function has been called
 *              to enable the driver transmitter, and no previous
 *              send operation is still in progress.
 * 
 *  @post       The USART driver is processing the send via interrupt
 *              driven or DMA data transfers and returns without blocking.
 * 
 */
extern int32_t MSP_ARM_USART_Send(DRIVER_USART_MSP *module, 
    const void *data, uint32_t num);

/**
 *  @brief      Receive data from the line to the data buffer with
 *              the USART driver using interrupt-driven or DMA data transfer.
 *              Any stale data left in the UART hardware FIFO is flushed
 *              before reception is started and an overflow event
 *              is generated if stale data was present at the start of
 *              the receive operation to alert the application of this.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[out] data A pointer to the data buffer to receive to (8-bit data)
 * 
 *  @param[in]  num The number of 8-bit data items to receive (length of data)
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if successful
 *  @retval     ARM_DRIVER_ERROR_PARAMETER if data is NULL or num is 0U
 *  @retval     ARM_DRIVER_ERROR if receiver was not enabled
 *  @retval     ARM_DRIVER_ERROR_BUSY if a previous receive is in progress
 *
 *  @pre        The MSP_ARM_USART_Initialize() function has already been
 *              called, MSP_ARM_USART_PowerControl() function has been
 *              called to set power state to ARM_POWER_FULL,
 *              the MSP_ARM_USART_Control() function has been called
 *              to enable the driver receiver, and no previous
 *              receive operation is still in progress.
 * 
 *  @post       The USART driver is processing the receive via interrupt
 *              driven or DMA data transfers and returns without blocking.
 * 
 */
extern int32_t MSP_ARM_USART_Receive(DRIVER_USART_MSP *module, void *data,
    uint32_t num);

/**
 *  @brief      This function is included to return an error if called
 *              by the driver access struct.  Transfer mode is not
 *              supported by this driver and any call will return
 *              an unsupported error and not process the transaction.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[in]  data_out A pointer to the data buffer to send from
 *
 *  @param[out] data_in A pointer to the data buffer to receive to
 * 
 *  @param[in]  num The number of 8-bit data items to transact
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_ERROR_UNSUPPORTED
 * 
 */
extern int32_t MSP_ARM_USART_Transfer(DRIVER_USART_MSP *module, \
    const void *data_out, void *data_in, uint32_t num);

/**
 *  @brief      Get the number of words sent since the start
 *              of the last send operation.
 * 
 *  @return     The number of words sent
 * 
 */
extern uint32_t MSP_ARM_USART_GetTxCount(DRIVER_USART_MSP *module);

/**
 *  @brief      Get the number of words received since the start
 *              of the last receive operation.
 * 
 *  @return     The number of words received.
 * 
 */
extern uint32_t MSP_ARM_USART_GetRxCount(DRIVER_USART_MSP *module);

/**
 *  @brief      Control the operational properties of a USART driver instance,
 *              including configuring transmission properties such as word
 *              size, stop bits, parity bits, and baud rate; also send
 *              operational commands to control ongoing operations.
 *                                    
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[in]  control The control operation to execute:
 *              Supported standard operations include:
 *                  - Operation mode: (may be OR'ed)
 *                      - ARM_USART_MODE_ASYNCHRONOUS
 *                  - Data Bits: (may be OR'ed)
 *                      - ARM_USART_DATA_BITS_5
 *                      - ARM_USART_DATA_BITS_6
 *                      - ARM_USART_DATA_BITS_7
 *                      - ARM_USART_DATA_BITS_8
 *                  - Parity Bits: (may be OR'ed)
 *                      - ARM_USART_PARITY_EVEN
 *                      - ARM_USART_PARITY_ODD
 *                      - ARM_USART_PARITY_NONE
 *                  - Stop Bits: (may be OR'ed)
 *                      - ARM_USART_STOP_BITS_1
 *                      - ARM_USART_STOP_BITS_2
 *                  - Flow control: (may be OR'ed)
 *                      - ARM_USART_FLOW_CONTROL_NONE
 *                      - ARM_USART_FLOW_CONTROL_RTS
 *                      - ARM_USART_FLOW_CONTROL_CTS
 *                      - ARM_USART_FLOW_CONTROL_RTS_CTS
 *              Supported miscellaneous operations include:
 *                  - ARM_USART_ABORT_RECEIVE
 *                  - ARM_USART_ABORT_SEND
 *                  - ARM_USART_CONTROL_RX
 *                  - ARM_USART_CONTROL_TX
 * 
 *  @param[in]  arg The optional argument field to the control operation
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if successful
 *  @retval     ARM_DRIVER_ERROR_PARAMETER if the parameter is not valid
 *  @retval     ARM_DRIVER_ERROR if driver power state is not ARM_POWER_FULL
 *  @retval     ARM_DRIVER_ERROR_BUSY if a send/receive operation is ongoing
 *  @retval     ARM_USART_ERROR_MODE if the mode selection is not supported
 *  @retval     ARM_USART_ERROR_BAUDRATE if baud rate not achievable
 *  @retval     ARM_USART_ERROR_DATA_BITS if selecting 9 data bits
 *  @retval     ARM_USART_ERROR_STOP_BITS if attempting 1.5 or 0.5 stop bits
 * 
 *  @pre        The MSP_ARM_USART_Initialize() function has already been
 *              called and the power mode is set to ARM_POWER_FULL and
 *              there is no active send or receive operation, unless the
 *              control is a send or receive abort command in which case
 *              it is expected that an operation is active.
 * 
 *  @post       The new commanded operational state is set, provided no
 *              errors occured.
 * 
 */
extern int32_t MSP_ARM_USART_Control(DRIVER_USART_MSP *module, \
    uint32_t control, uint32_t arg);

/**
 *  @brief      Get the status of the USART driver instance.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @return     ARM_USART_STATUS reflecting current driver instance status.
 * 
 */
extern ARM_USART_STATUS MSP_ARM_USART_GetStatus(DRIVER_USART_MSP *module);

/**
 *  @brief      If flow control is not hardware managed, it can be
 *              managed here.  RTS may be set and cleared with this function.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @param[in]  control The modem control parameter to use to update the state.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 * 
 *  @retval     ARM_DRIVER_OK if no errors occured.
 *  @retval     ARM_DRIVER_ERROR_UNSUPPORTED if DTR clear/set is attempted.
 *  @retval     ARM_USART_ERROR_FLOW_CONTROL if RTS change is requested
 *              but driver is in hardware flow control mode.
 * 
 */
extern int32_t MSP_ARM_USART_SetModemControl(DRIVER_USART_MSP *module, \
    ARM_USART_MODEM_CONTROL control);

/**
 *  @brief      If flow control is not hardware managed, it can be
 *              managed here.  CTS may be read with this function.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @return     ARM_USART_MODEM_STATUS
 * 
 *  @retval     modemStatus.cts == 1 if CTS is set
 *  @retval     modemStatus.cts == 0 if CTS is cleared
 * 
 */
extern ARM_USART_MODEM_STATUS MSP_ARM_USART_GetModemStatus( \
    DRIVER_USART_MSP *module);

/**
 *  @brief      This is the USART driver interrupt handler to be called by the
 *              wrapping interrupt service routine for each HW instance.
 *
 *  @param[in]  module Pointer to the driver instance root data structure.
 * 
 *  @return     None.
 * 
 */
extern void MSP_ARM_USART_IRQHandler(DRIVER_USART_MSP *module);

#endif /* DRIVER_USART_MSP_PRIV_H_ */
