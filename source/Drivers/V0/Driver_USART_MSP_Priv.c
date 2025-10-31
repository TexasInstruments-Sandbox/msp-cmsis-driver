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
 *  @file       Driver_USART_MSP_Priv.c
 *  
 *              This implementation supports HW version V0.
 *
 ******************************************************************************
 */

#include <ti/driverlib/driverlib.h>
#include <Driver_USART_MSP_Priv.h>

/**
 * @brief DriverCapabilities stores the ARM CMSIS-Driver USART
 *        driver capabilities supported by this driver implementation.
 */
static const ARM_USART_CAPABILITIES DriverCapabilities = {
    1, /* supports UART (Asynchronous) mode */
    0, /* supports Synchronous Master mode */
    0, /* supports Synchronous Slave mode */
    0, /* supports UART Single-wire mode */
    0, /* supports UART IrDA mode */
    0, /* supports UART Smart Card mode */
    0, /* Smart Card Clock generator available */
    1, /* RTS Flow Control available */
    1, /* CTS Flow Control available */
    1, /* Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE */
    1, /* Signal receive character timeout event: \
       \ref ARM_USART_EVENT_RX_TIMEOUT */
    1, /* RTS Line: 0=not available, 1=available */
    1, /* CTS Line: 0=not available, 1=available */
    0, /* DTR Line: 0=not available, 1=available */
    0, /* DSR Line: 0=not available, 1=available */
    0, /* DCD Line: 0=not available, 1=available */
    0, /* RI Line: 0=not available, 1=available */
    0, /* Signal CTS change event: \ref ARM_USART_EVENT_CTS */
    0, /* Signal DSR change event: \ref ARM_USART_EVENT_DSR */
    0, /* Signal DCD change event: \ref ARM_USART_EVENT_DCD */
    0, /* Signal RI change event: \ref ARM_USART_EVENT_RI */
    0  /* Reserved (must be zero) */
};

/**
 *  @brief      Initialize the DMA configuration for use with MSP USART
 *              CMSIS driver subsequent functions.
 *
 *  @param[in]  cfg  Pointer to the DMA channel/trigger to init
 *  @param[in]  incDest When true, sets destination address to increment and
 *              sets source address to be fixed during transfers.  When not
 *              true, sets destination address to fixed and sets source
 *              address to increment during transfers.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if DMA channel initialized successfully.
 *  @retval     ARM_DRIVER_ERROR_PARAMETER if DMA channel structure does not
 *              have valid DMA channels/triggers configured.
 * 
 *  @post The DMA channel and trigger are initialized and operations may
 *        be configured and carried out by DMA-capable send/receive functions.
 * 
 */
static int32_t MSP_ARM_USART_initDMACh(DRIVER_DMA_MSP *cfg, bool incDest);

/**
 *  @brief      Set the USART driver baud rate, including IBRD, FBRD, OSAM.
 *              This function is used internally by the driver and is not
 *              exposed to the calling application middleware directly.
 *              Application middleware shall set baud rate via the Arm-defined
 *              DRIVER->Control() application interface only.
 *
 *  @param[in]  module  Pointer to the driver instance root data structure.
 *  @param[in]  baud  The desired baud rate.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if baud set successfully, else 
 *              ARM_USART_ERROR_BAUDRATE if targeted baud rate could not be
 *              achieved with the given clkFreq and peripheral options.
 *
 *  @pre  The UART hardware must be idle and the UART hardware must be in a
 *        disabled state before calling this function.  The UART hardware
 *        source functional clock frequency (clockFreq) must be set in the
 *        module's configuration data structure.
 * 
 *  @post The UART hardware must be enabled before being used by the calling
 *        function.  This function does not enable the UART after config
 *        of OVSAM, IBRD, FBRD registers.
 * 
 */
static int32_t MSP_ARM_USART_SetBaud(DRIVER_USART_MSP *module, uint32_t baud);

/**
 *  @brief      Set the USART driver mode based on a Arm CMSIS-Driver Control
 *              command received by DRIVER->Control() API.  This is an internal
 *              function only and is inteded for use by the driver, not by the
 *              calling middleware application.
 *
 *  @param[in]  module  Pointer to the driver instance root data structure.
 *  @param[in]  control The control command passed to the Control() API.
 *              See the Arm CMSIS-Driver specification for valid inputs.
 *  @param[in]  arg The control command argument, in this case the UART baud
 *              rate to configure the UART hardware to operate with.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if baud set successfully, else 
 *              ARM_USART_ERROR_BAUDRATE, ARM_USART_ERROR_BUSY,
 *              ARM_USART_ERROR_DATA_BITS, ARM_USART_ERROR_PARITY,
 *              ARM_USART_ERROR_DATA_BITS, ARM_USART_ERROR_STOP_BITS
 *              or ARM_USART_ERROR_FLOW_CONTROL based on where the API
 *              encountered an error in the command and/or arguments.
 *
 *  @pre  The driver & hardware must be idle before calling this function (no
 *        active send/receive operation may be pending completion).
 *        If the driver and/or hardware are not idle, this function will
 *        return a busy error. The UART hardware does not need be in a
 *        disabled state before calling this function.  The function will 
 *        disable the UART hardware if required and re-enable if prev. enabled.
 *        The source functional clock frequency (clockFreq) must be set
 *        in the module's configuration data structure.
 * 
 */
static int32_t MSP_ARM_USART_SetMode(DRIVER_USART_MSP *module, 
                                     uint32_t control, uint32_t arg);

/**
 *  @brief      Handle the USART driver control miscellanous command
 *              based on a Arm CMSIS-Driver Control command received 
 *              by DRIVER->Control() API.  This is an internal function
 *              only and is inteded for use by the driver, not by the
 *              calling middleware application.
 *
 *  @param[in]  module  Pointer to the driver instance root data structure.
 *  @param[in]  control The control command passed to the Control() API.
 *              See the Arm CMSIS-Driver specification for valid inputs.
 *              Currently supported miscellaneous commands include:
 *              ARM_USART_CONTROL_TX, ARM_USART_CONTROL_RX.  Notably,
 *              this function does not handle abort commands (send abort,
 *              receive abort).  These are special cases that are
 *              not "re-configuration" and they are handled by the
 *              MSP_ARM_USART_AbortSend, MSP_ARM_USART_AbortReceive
 *              functions, respectively.
 *  @param[in]  arg The control command argument (command specific).
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if baud set successfully, else 
 *              ARM_DRIVER_ERROR_PARAMETER or ARM_USART_ERROR_BUSY
 *
 *  @pre  The driver & hardware must be idle before calling this function (no
 *        active send/receive operation may be pending completion).
 *        If the driver and/or hardware are not idle, this function will
 *        return a busy error. The UART hardware does not need be in a
 *        disabled state before calling this function.  The function will 
 *        disable the UART hardware if required and re-enable if prev. enabled.
 * 
 */
static int32_t MSP_ARM_USART_SetMisc(DRIVER_USART_MSP *module, 
                                    uint32_t control, uint32_t arg);

/**
 *  @brief      Reset the runtime state variables to defaults.
 *              This is an internal function only and is inteded for use 
 *              by the driver, not by the calling middleware application.
 *              This is called during initialization (first setup of the
 *              driver) to put the data structures into a known default
 *              state, or whencalling PowerControl to switch the driver 
 *              to a power off state.
 *
 *  @param[in]  module  Pointer to the driver instance root data structure.
 *
 *  @return     None
 *
 */
static void MSP_ARM_USART_ResetState(DRIVER_USART_MSP *module);

/**
 *  @brief      Abort a previously started send operation.
 *
 *  @param[in]  module  Pointer to the driver instance root data structure.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if baud set successfully, else 
 *              ARM_USART_ERROR if there was no active send operation.
 *
 *  @pre  This function shall only be called after a send operation
 *        has been started via MSP_ARM_USART_Send() and the operation
 *        has not yet run to completion.
 * 
 *  @post This function disables interrupts for the active transmit
 *        operation, resets the txTarCnt to 0 (no tx), and clears
 *        the txBuf pointer to NULL.  It notably does not clear
 *        the tx_busy flag- this is later cleared after any bytes
 *        contained within the UART hardware buffers are transmitted
 *        out on the line, at which point it is cleared via ISR.
 * 
 */
static int32_t MSP_ARM_USART_AbortSend(DRIVER_USART_MSP *module);

/**
 *  @brief      Abort a previously started receive operation.
 *
 *  @param[in]  module  Pointer to the driver instance root data structure.
 * 
 *  @return     Arm CMSIS-Driver status code (see Arm Driver_Common.h)
 *
 *  @retval     ARM_DRIVER_OK if baud set successfully, else 
 *              ARM_USART_ERROR if there was no active receive operation.
 *
 *  @pre  This function shall only be called after a receive operation
 *        has been started via MSP_ARM_USART_Receive() and the operation
 *        has not yet run to completion.
 * 
 *  @post This function disables interrupts for the active receive
 *        operation, resets the rxTarCnt to 0 (no rx), clears
 *        the rxBuf pointer to NULL, and clears the rx_busy flag.
 * 
 */
static int32_t MSP_ARM_USART_AbortReceive(DRIVER_USART_MSP *module);

/*  ARM CMSIS-Driver API instance generic function implementations
 *  are given below to implement the CMSIS-Driver.
 *  Refer to the Arm CMSIS-Driver specifications for API definitions.
 */

ARM_DRIVER_VERSION MSP_ARM_USART_GetVersion(void)
{
  return DriverVersion;
}

ARM_USART_CAPABILITIES MSP_ARM_USART_GetCapabilities(DRIVER_USART_MSP *module)
{
  return DriverCapabilities;
}

int32_t MSP_ARM_USART_Initialize(DRIVER_USART_MSP *module, 
                                        ARM_USART_SignalEvent_t cb_event)
{
    MSP_ARM_USART_ResetState(module);
    module->state->callback = cb_event;

    if (module->txPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        DL_GPIO_initPeripheralOutputFunction(
            module->txPin.iomuxPinCtlMgmtRegIndex,
            module->txPin.iomuxPinFunction);
    }
    else
    {
        // If no IO is defined, we have nothing to configure.
    }

    if (module->rxPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        DL_GPIO_initPeripheralInputFunction(
            module->rxPin.iomuxPinCtlMgmtRegIndex, 
            module->rxPin.iomuxPinFunction);
    }
    else
    {
        // If no IO is defined, we have nothing to configure.
    }

    if (module->rtsPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        DL_GPIO_initPeripheralOutputFunction(
            module->rtsPin.iomuxPinCtlMgmtRegIndex, 
            module->rtsPin.iomuxPinFunction);
    }
    else
    {
        // If no IO is defined, we have nothing to configure.
    }

    if (module->ctsPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        DL_GPIO_initPeripheralInputFunction(
            module->ctsPin.iomuxPinCtlMgmtRegIndex, 
            module->ctsPin.iomuxPinFunction);
    }
    else
    {
        // If no IO is defined, we have nothing to configure.
    }

    module->state->initDone = DRIVER_INITIALIZED;
    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_USART_Uninitialize(DRIVER_USART_MSP *module)
{
    module->state->callback = NULL;

    if (module->txPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        IOMUX->SECCFG.PINCM[module->txPin.iomuxPinCtlMgmtRegIndex] =\
            IOMUX_PINCM_PC_UNCONNECTED;
    }
    else
    {
        // If no IO is defined, we have nothing to configure.
    }

    if (module->rxPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        IOMUX->SECCFG.PINCM[module->rxPin.iomuxPinCtlMgmtRegIndex] =\
            IOMUX_PINCM_PC_UNCONNECTED;
    }
    else
    {
        // If no IO is defined, we have nothing to configure.
    }

    if (module->rtsPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        IOMUX->SECCFG.PINCM[module->rtsPin.iomuxPinCtlMgmtRegIndex] =\
            IOMUX_PINCM_PC_UNCONNECTED;
    }
    else
    {
        // If no IO is defined, we have nothing to configure.
    }

    if (module->ctsPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        IOMUX->SECCFG.PINCM[module->ctsPin.iomuxPinCtlMgmtRegIndex] =\
            IOMUX_PINCM_PC_UNCONNECTED;
    }
    else
    {
        // If no IO is defined, we have nothing to configure.
    }

    module->state->initDone = DRIVER_UNINITIALIZED;
    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_USART_PowerControl(DRIVER_USART_MSP *module, \
                                   ARM_POWER_STATE state)
{
    if(state != ARM_POWER_OFF && module->state->initDone != DRIVER_INITIALIZED)
    {
        return ARM_DRIVER_ERROR;
    }

    DL_UART_ClockConfig clkCfg;
    int32_t err;
    int32_t baudErr;
    int32_t txDMAErr;
    int32_t rxDMAErr;

    err = ARM_DRIVER_ERROR;

    switch (state)
    {
        case ARM_POWER_OFF:
            if (module->state->status.tx_busy == 1U)
            {
                (void)MSP_ARM_USART_AbortSend(module);
            }
            else
            {
                // No send operation to cancel
            }
            if (module->state->status.rx_busy == 1U)
            {
                (void)MSP_ARM_USART_AbortReceive(module);
            }
            else
            {
                // No receive operation to cancel
            }
            NVIC_DisableIRQ(module->irq);
            DL_UART_reset(module->hw);
            DL_UART_disablePower(module->hw);
            NVIC_ClearPendingIRQ(module->irq);
            MSP_ARM_USART_ResetState(module);
            module->state->powerState = ARM_POWER_OFF;
            err = ARM_DRIVER_OK;
            break;
        case ARM_POWER_LOW:
            err = ARM_DRIVER_ERROR_UNSUPPORTED;
            break;
        case ARM_POWER_FULL:
            if (module->state->powerState == ARM_POWER_FULL)
            {
                /* Already at ARM_POWER_FULL, do nothing */
                err = ARM_DRIVER_OK;
            }
            else
            {
                /* Switch to ARM_POWER_FULL */
                DL_UART_enablePower(module->hw);
                __NOP(); /* Insert 4 NOPs for power enable cycle time */
                __NOP();
                __NOP();
                __NOP();
                if (module->clock==DRIVER_CLK_MSP_MFCLK)
                {
                    clkCfg.clockSel = DL_UART_CLOCK_MFCLK;
                }
                else if (module->clock==DRIVER_CLK_MSP_LFCLK)
                {
                    clkCfg.clockSel = DL_UART_CLOCK_LFCLK;
                }
                else
                {
                    clkCfg.clockSel = DL_UART_CLOCK_BUSCLK;
                }
                clkCfg.divideRatio = DL_UART_CLOCK_DIVIDE_RATIO_1;
                DL_UART_setClockConfig(module->hw, &clkCfg);
                baudErr = MSP_ARM_USART_SetMode(module, 
                                            ARM_USART_MODE_ASYNCHRONOUS, 
                                            9600U);
                
                /* Configure any DMA channels if required */
                if (module->txDMA.hw != DRIVER_DMA_HW_NONE)
                {
                    module->state->transmitterDMAAvail = true;
                    txDMAErr = MSP_ARM_USART_initDMACh(&(module->txDMA), \
                                                       false);
                    DL_UART_enableDMATransmitEvent(module->hw);
                }
                else
                {
                    module->state->transmitterDMAAvail = false;
                    txDMAErr = ARM_DRIVER_OK;
                }
                if (module->rxDMA.hw != DRIVER_DMA_HW_NONE)
                {
                    module->state->receiverDMAAvail = true;
                    rxDMAErr = MSP_ARM_USART_initDMACh(&(module->rxDMA), \
                                                       true);
                    DL_UART_enableDMAReceiveEvent(module->hw, \
                        DL_UART_DMA_INTERRUPT_RX);
                }
                else
                {
                    module->state->receiverDMAAvail = false;
                    rxDMAErr = ARM_DRIVER_OK;
                }

                /* Identify any error code to return */
                if (baudErr != ARM_DRIVER_OK)
                {
                    err = baudErr;
                }
                else if ((txDMAErr != ARM_DRIVER_OK) ||\
                         (rxDMAErr != ARM_DRIVER_OK))
                {
                    err = ARM_DRIVER_ERROR_PARAMETER;
                }
                else
                {
                    err = ARM_DRIVER_OK;
                }

                /* Close out based on error state */
                if (err == ARM_DRIVER_OK)
                {
                    /* No errors, proceed to ARM_POWER_FULL */
                    DL_UART_enable(module->hw);
                    NVIC_EnableIRQ(module->irq);
                    module->state->powerState = ARM_POWER_FULL;
                }
                else
                {
                    /* Errors, gracefully crash and proceed to ARM_POWER_OFF */
                    NVIC_DisableIRQ(module->irq);
                    DL_UART_reset(module->hw);
                    DL_UART_disablePower(module->hw);
                    NVIC_ClearPendingIRQ(module->irq);
                    MSP_ARM_USART_ResetState(module);
                    module->state->powerState = ARM_POWER_OFF;
                }
            }
            break;
        default:
            break;
    }
    return err;
}

int32_t MSP_ARM_USART_initDMACh(DRIVER_DMA_MSP *cfg, bool incDest)
{
    DL_DMA_Config dmaCfg = 
    {
        .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
        .extendedMode   = DL_DMA_NORMAL_MODE,
        .destWidth      = DL_DMA_WIDTH_BYTE,
        .srcWidth       = DL_DMA_WIDTH_BYTE,
        .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
    };
    int32_t err;

    if (incDest == true)
    {
        dmaCfg.destIncrement = DL_DMA_ADDR_INCREMENT;
        dmaCfg.srcIncrement = DL_DMA_ADDR_UNCHANGED;
    }
    else
    {
        dmaCfg.destIncrement = DL_DMA_ADDR_UNCHANGED;
        dmaCfg.srcIncrement = DL_DMA_ADDR_INCREMENT;
    }

    if ((cfg->ch != DRIVER_DMA_CH_NONE) &&\
        (cfg->trig != DRIVER_DMA_TRIG_NONE))
    {
        dmaCfg.trigger = cfg->trig;
        DL_DMA_initChannel(cfg->hw, cfg->ch, &dmaCfg);
        err = ARM_DRIVER_OK;
    }
    else
    {
        err = ARM_DRIVER_ERROR_PARAMETER;
    }

    return err;
}

int32_t MSP_ARM_USART_Send(DRIVER_USART_MSP *module, 
                           const void *data, uint32_t num)
{
    if ((data == NULL) || (num == 0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }
    else
    {
        // Valid parameters, no special action to take.
    }

    if (module->state->powerState != ARM_POWER_FULL || module->state->transmitterEnabled == 0U)
    {
        return ARM_DRIVER_ERROR;
    }
    else
    {
        // Transmitter enabled, no special action to take.
    }

    if (module->state->txTarCnt != 0U)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }
    else
    {
        // Driver not busy, no special action to take.
    }

    /* Set up transmission parameters */
    module->state->status.tx_busy = 1U;
    module->state->status.tx_underflow = 0U;
    module->state->txBuf = (uint8_t*)data;
    module->state->txTarCnt = num;
    module->state->txCnt = 0U;

    /* Start send based on interrupt-driven I/O or DMA-driven I/O */
    if (module->state->transmitterDMAAvail != true)
    {
        /* This is the interrupt-driven scenario */
        
        /* Clear end of transmission interrupt, this is enabled when
         * the last data word is loaded to FIFO and the interrupt
         * will be generated to indicate end of transmission signal
         * to the calling application.
         */
        DL_UART_clearInterruptStatus(module->hw, DL_UART_INTERRUPT_EOT_DONE);

        /* Start transmission via transmit interrupt.
         * Transmit interrupt is expected to be set already.
         * EOT interrupt used for lapse in TX shift register activity.
         */
        DL_UART_enableInterrupt(module->hw, DL_UART_INTERRUPT_TX |\
                                            DL_UART_INTERRUPT_EOT_DONE);
    }
    else
    {
        /* This is the DMA-driven scenario */

        /* Configure DMA channel for transmit operation */
        DL_DMA_setSrcAddr(module->txDMA.hw, module->txDMA.ch, \
                          (uint32_t)(module->state->txBuf));
        DL_DMA_setDestAddr(module->txDMA.hw, module->txDMA.ch, \
                           (uint32_t)(&module->hw->TXDATA));
        DL_DMA_setTransferSize(module->txDMA.hw, module->txDMA.ch, \
                               module->state->txTarCnt);

        /* Clear end of transmission interrupt and DMA done interrupt.
         * DMA done indicates send is complete and the buffer is free.
         * EOT indicates transmission on the line is complete.
         */
        DL_UART_clearInterruptStatus(module->hw, \
                                     DL_UART_INTERRUPT_DMA_DONE_TX |\
                                     DL_UART_INTERRUPT_EOT_DONE);

        /* Enable interrupt.
         * Driver will signal send is done based on DMA done interrupt.
         */
        DL_UART_enableInterrupt(module->hw, DL_UART_INTERRUPT_DMA_DONE_TX |\
                                            DL_UART_INTERRUPT_EOT_DONE);

        /* Start the transfer by enabling and kicking the DMA channel
         * to ensure we always get started.
         */
        DL_DMA_enableChannel(module->txDMA.hw, module->txDMA.ch);
        module->hw->DMA_TRIG_TX.ISET = DL_UART_INTERRUPT_TX;
    }

    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_USART_Receive(DRIVER_USART_MSP *module, void *data,
                              uint32_t num)
{
    if ((data == NULL) || (num == 0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }
    else
    {
        // Valid data pointer and length, no special action to take
    }

    if (module->state->powerState != ARM_POWER_FULL || module->state->receiverEnabled == 0U)
    {
        return ARM_DRIVER_ERROR;
    }
    else
    {
        // Receiver is enabled, no special action to take
    }

    if (module->state->rxTarCnt != 0U)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }
    else
    {
        // Receiver not busy, no special action to take
    }

    /* Set up start of reception status */
    module->state->status.rx_busy = 1U;
    module->state->status.rx_break = 0U;
    module->state->status.rx_framing_error = 0U;
    module->state->status.rx_parity_error = 0U;
    module->state->status.rx_overflow = 0U;

    /* Set up reception buffer & counter */
    module->state->rxBuf = (uint8_t*)data;
    module->state->rxTarCnt = num;
    module->state->rxCnt = 0U;

    /* Check for lost data and signal RX overflow if the UART
     * received data before receive function was called */
    if ((DL_UART_isRXFIFOEmpty(module->hw) == false) ||\
        (DL_UART_getRawInterruptStatus(module->hw, \
             DL_UART_INTERRUPT_OVERRUN_ERROR)))
    {
        if (module->state->callback != NULL)
        {
            module->state->callback(ARM_USART_EVENT_RX_OVERFLOW);
        }
        else
        {
            // Do not call NULL callback
        }
    }
    else
    {
        // If there is no stale data, proceed normally.
    }

    /* Flush out any stale data left in RX FIFO */
    while(DL_UART_isRXFIFOEmpty(module->hw) == false)
    {
        (void)DL_UART_receiveData(module->hw);
    }

    /* Start receive based on interrupt-driven I/O or DMA-driven I/O */
    if (module->state->receiverDMAAvail != true)
    {
        /* This is the interrupt-driven scenario */

        DL_UART_clearInterruptStatus(module->hw, \
            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |\
            DL_UART_INTERRUPT_FRAMING_ERROR |\
            DL_UART_INTERRUPT_PARITY_ERROR |\
            DL_UART_INTERRUPT_BREAK_ERROR |\
            DL_UART_INTERRUPT_OVERRUN_ERROR |\
            DL_UART_INTERRUPT_RX);
        DL_UART_enableInterrupt(module->hw, \
            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |\
            DL_UART_INTERRUPT_FRAMING_ERROR |\
            DL_UART_INTERRUPT_PARITY_ERROR |\
            DL_UART_INTERRUPT_BREAK_ERROR |\
            DL_UART_INTERRUPT_OVERRUN_ERROR |\
            DL_UART_INTERRUPT_RX);
    }
    else
    {
        /* This is the DMA-driven scenario */

        /* Configure DMA channel for transmit operation */
        module->hw->DMA_TRIG_RX.ICLR = DL_UART_INTERRUPT_RX;
        DL_DMA_setSrcAddr(module->rxDMA.hw, module->rxDMA.ch, \
                          (uint32_t)(&module->hw->RXDATA));
        DL_DMA_setDestAddr(module->rxDMA.hw, module->rxDMA.ch, \
                           (uint32_t)(module->state->rxBuf));
        DL_DMA_setTransferSize(module->rxDMA.hw, module->rxDMA.ch, \
                               module->state->rxTarCnt);
        DL_DMA_enableChannel(module->rxDMA.hw, module->rxDMA.ch);

        /* Enable status reporting interrupts and DMA done interrupt */
        DL_UART_clearInterruptStatus(module->hw, \
            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |\
            DL_UART_INTERRUPT_FRAMING_ERROR |\
            DL_UART_INTERRUPT_PARITY_ERROR |\
            DL_UART_INTERRUPT_BREAK_ERROR |\
            DL_UART_INTERRUPT_OVERRUN_ERROR |\
            DL_UART_INTERRUPT_DMA_DONE_RX);
        DL_UART_enableInterrupt(module->hw, \
            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |\
            DL_UART_INTERRUPT_FRAMING_ERROR |\
            DL_UART_INTERRUPT_PARITY_ERROR |\
            DL_UART_INTERRUPT_BREAK_ERROR |\
            DL_UART_INTERRUPT_OVERRUN_ERROR |\
            DL_UART_INTERRUPT_DMA_DONE_RX);
    }

    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_USART_Transfer(DRIVER_USART_MSP *module, \
                               const void *data_out, void *data_in, \
                               uint32_t num)
{
    if(module->state->powerState != ARM_POWER_FULL)
    {
        return ARM_DRIVER_ERROR;
    }
    else
    {
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
}

uint32_t MSP_ARM_USART_GetTxCount(DRIVER_USART_MSP *module)
{
    uint32_t cnt;
    uint32_t tarCnt;
    uint32_t dmaCnt;

    if (module->state->transmitterDMAAvail != true)
    {
        cnt = module->state->txCnt;
    }
    else
    {
        dmaCnt = DL_DMA_getTransferSize(module->txDMA.hw, module->txDMA.ch);
        tarCnt = module->state->txTarCnt;
        if (tarCnt != 0)
        {
            cnt = tarCnt - dmaCnt;
        }
        else
        {
            cnt = module->state->txCnt;
        }
    }
    return cnt;
}

uint32_t MSP_ARM_USART_GetRxCount(DRIVER_USART_MSP *module)
{
    uint32_t cnt;
    uint32_t tarCnt;
    uint32_t dmaCnt;

    if (module->state->receiverDMAAvail != true)
    {
        cnt = module->state->rxCnt;
    }
    else
    {
        dmaCnt = DL_DMA_getTransferSize(module->rxDMA.hw, module->rxDMA.ch);
        tarCnt = module->state->rxTarCnt;
        if (tarCnt != 0)
        {
            cnt = tarCnt - dmaCnt;
        }
        else
        {
            cnt = module->state->rxCnt;
        }
    }
    return cnt;
}

int32_t MSP_ARM_USART_Control(DRIVER_USART_MSP *module, \
                              uint32_t control, uint32_t arg)
{
    int32_t err;
    uint32_t cmd;

    /* Control changes are only allowed when power state is full. */
    if (module->state->powerState != ARM_POWER_FULL)
    {
        return ARM_DRIVER_ERROR;
    }
    else
    {
        // Power state is ARM_POWER_FULL, no special action to take.
    }

    /* Determine if this is a mode control or misc command.
     * Then call the appropriate command handler. */
    cmd = (control & ARM_USART_CONTROL_Msk) >> ARM_USART_CONTROL_Pos;
    if (cmd < 0x10) /* Command is in the mode control region */
    {
        err = MSP_ARM_USART_SetMode(module, control, arg);
    }
    else if (cmd == ARM_USART_ABORT_SEND) /* Command is send abort */
    {
        err = MSP_ARM_USART_AbortSend(module);
    }
    else if (cmd == ARM_USART_ABORT_RECEIVE) /* Command is receive abort */
    {
        err = MSP_ARM_USART_AbortReceive(module);
    }
    else /* Command is in the miscellaneous control region and not an abort */
    {
        err = MSP_ARM_USART_SetMisc(module, control, arg);
    }

    return err;
}

ARM_USART_STATUS MSP_ARM_USART_GetStatus(DRIVER_USART_MSP *module)
{
    return module->state->status;
}

int32_t MSP_ARM_USART_SetModemControl(DRIVER_USART_MSP *module,
                                      ARM_USART_MODEM_CONTROL control)
{
    int32_t err;
    DL_UART_FLOW_CONTROL flowCtl;

    flowCtl = DL_UART_getFlowControl(module->hw);
    if (flowCtl & DL_UART_FLOW_CONTROL_RTS)
    {
        err = ARM_USART_ERROR_FLOW_CONTROL;
    }
    else
    {
        if (control == ARM_USART_RTS_SET)
        {
            err = ARM_DRIVER_OK;
            DL_UART_setRTSOutput(module->hw,  DL_UART_RTS_ASSERT);
        }
        else if (control == ARM_USART_RTS_CLEAR)
        {
            err = ARM_DRIVER_OK;
            DL_UART_setRTSOutput(module->hw,  DL_UART_RTS_DEASSERT);
        }
        else
        {
            err = ARM_DRIVER_ERROR_UNSUPPORTED;
        }
    }
    return err;
}

ARM_USART_MODEM_STATUS MSP_ARM_USART_GetModemStatus(DRIVER_USART_MSP *module)
{
    ARM_USART_MODEM_STATUS modemStatus;
    bool cts;

    cts = DL_UART_isClearToSend(module->hw);
    if (cts == true)
    {
        modemStatus.cts = 1U;
    }
    else
    {
        modemStatus.cts = 0U;
    }
    modemStatus.dcd = 0U;
    modemStatus.dsr = 0U;
    modemStatus.ri = 0U;
    
    return modemStatus;
}

/* Internal driver function implementations */

static int32_t MSP_ARM_USART_SetBaud(DRIVER_USART_MSP *module, uint32_t baud)
{
    int32_t err;
    uint32_t inClk;
    uint32_t ovsam;
    uint32_t ibrd;
    uint32_t fbrd;
    DL_UART_OVERSAMPLING_RATE ovsamTag;

    err = ARM_DRIVER_OK;

    inClk = module->clockFreq;

    /* Identify the proper oversampling rate (16x, 8x, 3x).
     * Try 16x first, then 8x, then 3x, then error out. 
     * Idenfify integer baud rate divider at the same time. */
    ovsamTag = DL_UART_OVERSAMPLING_RATE_16X;
    ovsam = 16;
    ibrd = inClk / (ovsam * baud);
    if (ibrd == 0U)
    {
        ovsamTag = DL_UART_OVERSAMPLING_RATE_8X;
        ovsam = 8;
        ibrd = inClk / (ovsam * baud);
    }
    else
    {
        // If not zero already, do nothing here.
    }
    if (ibrd == 0U)
    {
        ovsamTag = DL_UART_OVERSAMPLING_RATE_3X;
        ovsam = 3;
        ibrd = inClk / (ovsam * baud);
    }
    else
    {
        // If not zero already, do nothing here.
    }
    if (ibrd == 0U)
    {
        err = ARM_USART_ERROR_BAUDRATE;
    }
    else
    {
        // If not zero here, no error and we can go ahead.
    }

	fbrd = (((inClk % (ovsam * baud)) * 64 ) + ((ovsam * baud) / 2)) / (ovsam * baud);

    /* Load oversampling, integer divider, and fractional divider to HW */
    DL_UART_setOversampling(module->hw, ovsamTag);
    DL_UART_setBaudRateDivisor(module->hw, ibrd, fbrd);

    return err;
}

static int32_t MSP_ARM_USART_SetMode(DRIVER_USART_MSP *module, 
                                     uint32_t control, uint32_t arg)
{
    DL_UART_Config hwCfg;
    int32_t err;
    uint32_t cmd;
    uint32_t param;
    bool hwWasEnabled;
    bool setBaud;

    /* Mode changes are not allowed when the driver is busy.  It is
     * the application's responsibility to bring to an idle state. */
    if ((module->state->status.tx_busy) || (module->state->status.rx_busy))
    {
        return ARM_DRIVER_ERROR_BUSY;
    }
    else
    {
        // Transmitter and receiver are not busy, no special action to take.
    }

    err = ARM_DRIVER_OK;

    /* Set up operating mode based on mode control command */
    setBaud = false;
    switch (control & ARM_USART_CONTROL_Msk)
    {
        case 0U:
            break;
        case ARM_USART_MODE_ASYNCHRONOUS:
            setBaud = true;
            break;
        default:
            err = ARM_USART_ERROR_MODE;
            break;
    }
    if (err != ARM_DRIVER_OK)
    {
        return err;
    }
    else
    {
        // Parameters valid, no special action to take.
    }

    /* Set up word length */
    switch (control & ARM_USART_DATA_BITS_Msk)
    {
        case ARM_USART_DATA_BITS_5: 
            hwCfg.wordLength = DL_UART_WORD_LENGTH_5_BITS;
            break;
        case ARM_USART_DATA_BITS_6: 
            hwCfg.wordLength = DL_UART_WORD_LENGTH_6_BITS;
            break;
        case ARM_USART_DATA_BITS_7: 
            hwCfg.wordLength = DL_UART_WORD_LENGTH_7_BITS;
            break;
        case ARM_USART_DATA_BITS_8: 
            hwCfg.wordLength = DL_UART_WORD_LENGTH_8_BITS;
            break;
        case ARM_USART_DATA_BITS_9:
            err = ARM_USART_ERROR_DATA_BITS;
            break;
        default:
            err = ARM_USART_ERROR_DATA_BITS;
            break;
    }
    if (err != ARM_DRIVER_OK)
    {
        return err;
    }
    else
    {
        // Word length correctly set, no special action to take.
    }

    /* Set up parity */
    switch (control & ARM_USART_PARITY_Msk)
    {
        case ARM_USART_PARITY_NONE:
            hwCfg.parity = DL_UART_PARITY_NONE;
            break;
        case ARM_USART_PARITY_EVEN:
            hwCfg.parity = DL_UART_PARITY_EVEN;
            break;
        case ARM_USART_PARITY_ODD:
            hwCfg.parity = DL_UART_PARITY_ODD;
            break;
        default:
            err = ARM_USART_ERROR_PARITY;
            break;
    }
    if (err != ARM_DRIVER_OK)
    {
        return err;
    }
    else
    {
        // Parity set correctly, no special action to take.
    }

    /* Set up stop bit(s) */
    switch (control & ARM_USART_STOP_BITS_Msk)
    {
        case ARM_USART_STOP_BITS_1:
            hwCfg.stopBits = DL_UART_STOP_BITS_ONE;
            break;
        case ARM_USART_STOP_BITS_2:
            hwCfg.stopBits = DL_UART_STOP_BITS_TWO;
            break;
        case ARM_USART_STOP_BITS_1_5:
            err = ARM_USART_ERROR_STOP_BITS;
            break;
        case ARM_USART_STOP_BITS_0_5:
            err = ARM_USART_ERROR_STOP_BITS;
            break;
    }
    if (err != ARM_DRIVER_OK)
    {
        return err;
    }
    else
    {
        // Stop bits set correctly, no special action to take.
    }

    /* Set up flow control */
    switch (control & ARM_USART_FLOW_CONTROL_Msk)
    {
        case ARM_USART_FLOW_CONTROL_NONE:
            hwCfg.flowControl = DL_UART_FLOW_CONTROL_NONE;
            break;
        case ARM_USART_FLOW_CONTROL_RTS:
            hwCfg.flowControl = DL_UART_FLOW_CONTROL_RTS;
            break;
        case ARM_USART_FLOW_CONTROL_CTS:
            hwCfg.flowControl = DL_UART_FLOW_CONTROL_CTS;
            break;
        case ARM_USART_FLOW_CONTROL_RTS_CTS:
            hwCfg.flowControl = DL_UART_FLOW_CONTROL_RTS_CTS;
            break;
    }

    /* Disable UART hardware, if enabled.
     * In most cases, UART will be enabled and it needs to be 
     * disabled before changing modes.  In the initial case where
     * this function is called by PowerControl during PowerControl to FULL,
     * UART is not enabled yet so we leave it disabled as it was. */
    hwWasEnabled = DL_UART_isEnabled(module->hw);
    if (hwWasEnabled == true)
    {
        DL_UART_disable(module->hw);
    }

    /* No unsupported errors or parameter errors seen if arriving
     * here.  Now it's safe to apply the new configuration to hardware. */
    hwCfg.mode = DL_UART_MODE_NORMAL;
    hwCfg.direction = DL_UART_DIRECTION_TX_RX;
    DL_UART_init(module->hw, &hwCfg);

    /* Configure baud rate*/
    if (setBaud == true)
    {
        err = MSP_ARM_USART_SetBaud(module, arg);
    }
    else
    {
        // Do not set baud rate
    }

    /* Set RX timeout value */
    DL_UART_setRXInterruptTimeout(module->hw,
                                  DRIVER_USART_MSP_TIMEOUT_BIT_PERIODS);

    /* Re-enable hardware UART if it was previously enabled. */
    if (hwWasEnabled == true)
    {
        DL_UART_enable(module->hw);
    }
    else
    {
        // Do nothing, no need to enable HW if it was not enabled previously.
    }

    return err;
}

static int32_t MSP_ARM_USART_SetMisc(DRIVER_USART_MSP *module, 
                                     uint32_t control, uint32_t arg)
{
    int32_t err;
    uint32_t cmd;
    bool hwWasEnabled;

    /* Mode changes are not allowed when the driver is busy.  It is
     * the application's responsibility to bring to an idle state. */
    if ((module->state->status.tx_busy) || (module->state->status.rx_busy))
    {
        return ARM_DRIVER_ERROR_BUSY;
    }
    else
    {
        // Transmitter and receiver are not busy, no special action to take.
    }

    err = ARM_DRIVER_OK;

    cmd = (control & ARM_USART_CONTROL_Msk) >> ARM_USART_CONTROL_Pos;
    switch (cmd)
    {
        case ARM_USART_CONTROL_TX:
            if (arg == 1U)
            {
                module->state->transmitterEnabled = 1U;
            }
            else if (arg == 0U)
            {
                module->state->transmitterEnabled = 0U;
            }
            else
            {
                err = ARM_DRIVER_ERROR_PARAMETER;
            }
            break;
        case ARM_USART_CONTROL_RX:
            if (arg == 1U)
            {
                module->state->receiverEnabled = 1U;
            }
            else if (arg == 0U)
            {
                module->state->receiverEnabled = 0U;
            }
            else
            {
                err = ARM_DRIVER_ERROR_PARAMETER;
            }
            break;
        default:
            err = ARM_DRIVER_ERROR_PARAMETER;
            break;
    }

    return err;
}

static void MSP_ARM_USART_ResetState(DRIVER_USART_MSP *module)
{
    /* Reset status indicators */
    module->state->status.tx_busy = 0U;
    module->state->status.tx_underflow = 0U;
    module->state->status.rx_break = 0U;
    module->state->status.rx_busy = 0U;
    module->state->status.rx_framing_error = 0U;
    module->state->status.rx_overflow = 0U;
    module->state->status.rx_parity_error = 0U;

    /* Reset counters */
    module->state->rxCnt = 0U;
    module->state->rxTarCnt = 0U;
    module->state->txCnt = 0U;
    module->state->txTarCnt = 0U;

    /* Reset enables */
    module->state->powerState = ARM_POWER_OFF;
    module->state->transmitterEnabled = 0U;
    module->state->receiverEnabled = 0U;
    module->state->receiverDMAAvail = false;
    module->state->transmitterDMAAvail = false;
}

static int32_t MSP_ARM_USART_AbortSend(DRIVER_USART_MSP *module)
{
    if (module->state->txTarCnt == 0U)
    {
        return ARM_DRIVER_ERROR;
    }
    else
    {
        // Send was found in progress, no special action to take.
    }

    if (module->state->transmitterDMAAvail != true)
    {
        DL_UART_disableInterrupt(module->hw, DL_UART_INTERRUPT_TX |\
                                             DL_UART_INTERRUPT_EOT_DONE);
    }
    else
    {
        DL_DMA_disableChannel(module->txDMA.hw, module->txDMA.ch);
        DL_UART_disableInterrupt(module->hw, DL_UART_INTERRUPT_DMA_DONE_TX |\
                                             DL_UART_INTERRUPT_EOT_DONE);
        module->state->txCnt = module->state->txTarCnt - \
            DL_DMA_getTransferSize(module->txDMA.hw, module->txDMA.ch);
    }
    module->state->txTarCnt = 0U;
    module->state->txBuf = NULL;
    module->state->status.tx_busy = 0U;

    return ARM_DRIVER_OK;
}

static int32_t MSP_ARM_USART_AbortReceive(DRIVER_USART_MSP *module)
{
    if (module->state->rxTarCnt == 0U)
    {
        return ARM_DRIVER_ERROR;
    }
    else
    {
        // Receive was found in progress, no special action to take.
    }

    if (module->state->receiverDMAAvail != true)
    {
        DL_UART_disableInterrupt(module->hw, \
            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |\
            DL_UART_INTERRUPT_FRAMING_ERROR |\
            DL_UART_INTERRUPT_PARITY_ERROR |\
            DL_UART_INTERRUPT_BREAK_ERROR |\
            DL_UART_INTERRUPT_OVERRUN_ERROR |\
            DL_UART_INTERRUPT_RX);
    }
    else
    {
        DL_DMA_disableChannel(module->rxDMA.hw, module->rxDMA.ch);
        DL_UART_disableInterrupt(module->hw, \
            DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |\
            DL_UART_INTERRUPT_FRAMING_ERROR |\
            DL_UART_INTERRUPT_PARITY_ERROR |\
            DL_UART_INTERRUPT_BREAK_ERROR |\
            DL_UART_INTERRUPT_OVERRUN_ERROR |\
            DL_UART_INTERRUPT_DMA_DONE_RX);
        module->state->rxCnt = module->state->rxTarCnt - \
            DL_DMA_getTransferSize(module->rxDMA.hw, module->rxDMA.ch);
    }
    module->state->rxTarCnt = 0U;
    module->state->rxBuf = NULL;
    module->state->status.rx_busy = 0U;

    return ARM_DRIVER_OK;
}

void MSP_ARM_USART_IRQHandler(DRIVER_USART_MSP *module)
{
    uint8_t data;

    switch (DL_UART_getPendingInterrupt(module->hw))
    {
        case DL_UART_IIDX_RX_TIMEOUT_ERROR:
            if (module->state->callback != NULL)
            {
                module->state->callback(ARM_USART_EVENT_RX_TIMEOUT);
            }
            else
            {
                // Do not call the callback if it is NULL
            }
            break;
        case DL_UART_IIDX_FRAMING_ERROR:
            module->state->status.rx_framing_error = 1U;
            if (module->state->callback != NULL)
            {
                module->state->callback(ARM_USART_EVENT_RX_FRAMING_ERROR);
            }
            else
            {
                // Do not call the callback if it is NULL
            }
            break;
        case DL_UART_IIDX_PARITY_ERROR:
            module->state->status.rx_parity_error = 1U;
            if (module->state->callback != NULL)
            {
                module->state->callback(ARM_USART_EVENT_RX_PARITY_ERROR);
            }
            else
            {
                // Do not call the callback if it is NULL
            }
            break;
        case DL_UART_IIDX_BREAK_ERROR:
            module->state->status.rx_break = 1U;
            if (module->state->callback != NULL)
            {
                module->state->callback(ARM_USART_EVENT_RX_BREAK);
            }
            else
            {
                // Do not call the callback if it is NULL
            }
            break;
        case DL_UART_IIDX_OVERRUN_ERROR:
            module->state->status.rx_overflow = 1U;
            if (module->state->callback != NULL)
            {
                module->state->callback(ARM_USART_EVENT_RX_OVERFLOW);
            }
            else
            {
                // Do not call the callback if it is NULL
            }
            break;
        case DL_UART_IIDX_RX:
            /* Grab data early from HW to free up HW buffer */
            data = DL_UART_receiveData(module->hw);
            /* Double chek that we are still in bounds of the application
             * specified receive buffer before writing to application
             * memory, then write to application memory receive buffer */
            if (module->state->rxCnt < module->state->rxTarCnt)
            {
                module->state->rxBuf[module->state->rxCnt++] = data;
            }
            else
            {
                // Do not overflow allowed buffer space.
            }
            /* If the received count has reached the target count,
             * close the receive operation and signal recevie complete */
            if (module->state->rxCnt == module->state->rxTarCnt)
            {
                DL_UART_disableInterrupt(module->hw, \
                    DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |\
                    DL_UART_INTERRUPT_FRAMING_ERROR |\
                    DL_UART_INTERRUPT_PARITY_ERROR |\
                    DL_UART_INTERRUPT_BREAK_ERROR |\
                    DL_UART_INTERRUPT_OVERRUN_ERROR |\
                    DL_UART_INTERRUPT_RX);
                module->state->rxTarCnt = 0U;
                module->state->rxBuf = NULL;
                module->state->status.rx_busy = 0U;
                if (module->state->callback != NULL)
                {
                    module->state->callback(ARM_USART_EVENT_RECEIVE_COMPLETE);
                }
                else
                {
                    // Do not call the callback if it is NULL
                }
            }
            else
            {
                // If we have not reached the target count, do nothing.
            }
            break;
        case DL_UART_IIDX_TX:
            /* Transmit the next byte in the application specified transmit
             * buffer */
            DL_UART_transmitData(module->hw, 
                                 module->state->txBuf[module->state->txCnt++]);
            /* If the transmitted count is now equal to the transmit target,
             * close the send operation and signal send complete */
            if (module->state->txCnt == module->state->txTarCnt)
            {
                DL_UART_disableInterrupt(module->hw, DL_UART_INTERRUPT_TX);
                module->state->txTarCnt = 0U;
                module->state->txBuf = NULL;
                if (module->state->callback != NULL)
                {
                    module->state->callback(ARM_USART_EVENT_SEND_COMPLETE);
                }
                else
                {
                    // Do not call the callback if it is NULL
                }
            }
            break;
        case DL_UART_IIDX_EOT_DONE:
            
            if (module->state->txTarCnt != 0U)
            {
                /* If the HW has an EOT but txTarCnt is still >0, this means 
                 * that there is still data to be transmitted. We will flag this
                 * scenario as a tx undeflow error and continue to wait for
                 * the real finished transmission.
                 */
                module->state->status.tx_underflow = 1U;
            }
            else
            {
                /* The HW has finished transmission, signal the application */
                DL_UART_disableInterrupt(module->hw, DL_UART_INTERRUPT_EOT_DONE);
                module->state->status.tx_busy = 0U;
                if (module->state->callback != NULL)
                {
                    module->state->callback(ARM_USART_EVENT_TX_COMPLETE);
                }
                else
                {
                    // Do not call the callback if it is NULL
                }
            }
            break;
        case DL_UART_IIDX_DMA_DONE_RX:
            DL_UART_disableInterrupt(module->hw, \
                DL_UART_INTERRUPT_RX_TIMEOUT_ERROR |\
                DL_UART_INTERRUPT_FRAMING_ERROR |\
                DL_UART_INTERRUPT_PARITY_ERROR |\
                DL_UART_INTERRUPT_BREAK_ERROR |\
                DL_UART_INTERRUPT_OVERRUN_ERROR |\
                DL_UART_INTERRUPT_DMA_DONE_RX);
            module->state->rxCnt = module->state->rxTarCnt;
            module->state->rxTarCnt = 0U;
            module->state->rxBuf = NULL;
            module->state->status.rx_busy = 0U;
            if (module->state->callback != NULL)
            {
                module->state->callback(ARM_USART_EVENT_RECEIVE_COMPLETE);
            }
            else
            {
                // Do not call the callback if it is NULL
            }
            break;
        case DL_UART_IIDX_DMA_DONE_TX:
            DL_UART_disableInterrupt(module->hw, DL_UART_INTERRUPT_DMA_DONE_TX);
            DL_UART_enableInterrupt(module->hw, DL_UART_INTERRUPT_EOT_DONE);
            module->state->txCnt = module->state->txTarCnt;
            module->state->txTarCnt = 0U;
            module->state->txBuf = NULL;
            if (module->state->callback != NULL)
            {
                module->state->callback(ARM_USART_EVENT_SEND_COMPLETE);
            }
            else
            {
                // Do not call the callback if it is NULL
            }
            break;
        default:
            break;
    }
}
