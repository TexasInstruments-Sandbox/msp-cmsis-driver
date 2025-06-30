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
 *  @file       Driver_SPI_MSP_Priv.c
 *
 *              This implementation supports HW version V0.
 *
 ******************************************************************************
 */

#include <ti/driverlib/driverlib.h>
#include <Driver_SPI_MSP_Priv.h>

/**
 * @brief DriverCapabilities stores the ARM CMSIS-Driver SPI
 *        driver capabilities supported by this driver implementation.
 */
static const ARM_SPI_CAPABILITIES DriverCapabilities = {
    0, /* Simplex Mode: deprecated and must be zero */
    1, /* Supports TI Synchronous Serial Interface */
    0, /* Supports Microwire Interface (unsupported) */
    0, /* Supports Signal Mode Fault Event (unsupported) */
};


/*  ARM CMSIS-Driver API instance generic function implementations
 *  are given below to implement the CMSIS-Driver.
 *  Refer to the Arm CMSIS-Driver specifications for API definitions.
 */

ARM_DRIVER_VERSION MSP_ARM_SPI_GetVersion(void)
{
  return DriverVersion;
}

ARM_SPI_CAPABILITIES MSP_ARM_SPI_GetCapabilities(DRIVER_SPI_MSP *module)
{
  return DriverCapabilities;
}

static void MSP_ARM_SPI_ResetState(DRIVER_SPI_MSP *module)
{
    module->state->txTarCnt = 0;
    module->state->txCnt = 0;
    module->state->rxTarCnt = 0;
    module->state->rxCnt = 0;
    module->state->status.busy = DRIVER_NOT_BUSY;
    module->state->spiState = SPI_STATE_IDLE;
    module->state->active = DRIVER_INACTIVE;
    module->state->status.data_lost = DATA_LOST_CLEAR;
    module->state->status.mode_fault = MODE_FAULT_CLEAR;
    module->state->direction = DIRECTION_SLAVE;
}

int32_t MSP_ARM_SPI_initDMACh(DRIVER_DMA_MSP *cfg, bool incDest)
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

int32_t MSP_ARM_SPI_Initialize(DRIVER_SPI_MSP *module,
                                        ARM_SPI_SignalEvent_t cb_event)
{
    MSP_ARM_SPI_ResetState(module);
    module->state->callback = cb_event;

    /* Since it is not known what mode (controller or target) the driver 
       will be operating in, the configuration of pin functions are left 
       to MSP_ARM_SPI_Control with the control parameter 
       ARM_SPI_MODE_MASTER or ARM_SPI_MODE_SLAVE.
    */
    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_SPI_Uninitialize(DRIVER_SPI_MSP *module)
{
    module->state->callback = NULL;

    if (module->ssPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        IOMUX->SECCFG.PINCM[module->ssPin.iomuxPinCtlMgmtRegIndex] =\
            IOMUX_PINCM_PC_UNCONNECTED;
    }
    else
    {
        // If no IO is defined, we have nothing to uninitialize.
    }

    if (module->mosiPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        IOMUX->SECCFG.PINCM[module->mosiPin.iomuxPinCtlMgmtRegIndex] =\
            IOMUX_PINCM_PC_UNCONNECTED;
    }
    else
    {
        // If no IO is defined, we have nothing to uninitialize.
    }

    if (module->sclkPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        IOMUX->SECCFG.PINCM[module->sclkPin.iomuxPinCtlMgmtRegIndex] =\
            IOMUX_PINCM_PC_UNCONNECTED;
    }
    else
    {
        // If no IO is defined, we have nothing to uninitialize.
    }

    if (module->misoPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        IOMUX->SECCFG.PINCM[module->misoPin.iomuxPinCtlMgmtRegIndex] =\
            IOMUX_PINCM_PC_UNCONNECTED;
    }
    else
    {
        // If no IO is defined, we have nothing to uninitialize.
    }

    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_SPI_PowerControl(DRIVER_SPI_MSP *module, ARM_POWER_STATE state)
{
    DL_SPI_ClockConfig clkCfg;
    int32_t err = ARM_DRIVER_OK;
    int32_t transmitDMAErr;
    int32_t receiveDMAErr;

    switch(state)
    {
        case ARM_POWER_OFF:
            NVIC_DisableIRQ(module->irq);
            DL_SPI_reset(module->hw);
            DL_SPI_disablePower(module->hw);
            NVIC_ClearPendingIRQ(module->irq);
            MSP_ARM_SPI_ResetState(module);
            module->state->powerState = ARM_POWER_OFF;
            break;
        case ARM_POWER_LOW:
            err = ARM_DRIVER_ERROR_UNSUPPORTED;
            break;
        case ARM_POWER_FULL:
            if(module->state->powerState == ARM_POWER_FULL)
            {
                //Do nothing, already ARM_POWER_FULL
            }
            else
            {
                DL_SPI_enablePower(module->hw);
                __NOP(); /* Insert 4 NOPs for power enable cycle time */
                __NOP();
                __NOP();
                __NOP();
                if (module->clock == DRIVER_CLK_MSP_MFCLK)
                {
                    clkCfg.clockSel = DL_SPI_CLOCK_MFCLK;
                }
                else if(module->clock == DRIVER_CLK_MSP_BUSCLK)
                {
                    clkCfg.clockSel = DL_SPI_CLOCK_BUSCLK;
                }
                else if(module->clock == DRIVER_CLK_MSP_LFCLK)
                {
                    clkCfg.clockSel = DL_SPI_CLOCK_LFCLK;
                }
                else
                {
                    err = ARM_DRIVER_ERROR;
                }

                if(err == ARM_DRIVER_OK)
                {
                    clkCfg.divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1;
                    DL_SPI_setClockConfig(module->hw, &clkCfg);
                    DL_SPI_setFIFOThreshold(module->hw,\
                                            DL_SPI_RX_FIFO_LEVEL_ONE_FRAME,\
                                            DL_SPI_TX_FIFO_LEVEL_ONE_FRAME);
                    if (module->transmitDMA.hw != DRIVER_DMA_HW_NONE)
                    {
                        module->state->transmitDMAAvail = true;
                        transmitDMAErr =\
                        MSP_ARM_SPI_initDMACh(&(module->transmitDMA), false);
                        DL_SPI_enableDMATransmitEvent(module->hw);
                    }
                    else
                    {
                        module->state->transmitDMAAvail = false;
                        transmitDMAErr = ARM_DRIVER_OK;
                    }
                    if(module->receiveDMA.hw != DRIVER_DMA_HW_NONE)
                    {
                        module->state->receiveDMAAvail = true;
                        receiveDMAErr =\
                        MSP_ARM_SPI_initDMACh(&(module->receiveDMA), true);
                        DL_SPI_enableDMAReceiveEvent(module->hw, DL_SPI_DMA_INTERRUPT_RX);
                    }
                    else
                    {
                        module->state->receiveDMAAvail = false;
                        receiveDMAErr = ARM_DRIVER_OK;
                    }

                    if ((transmitDMAErr != ARM_DRIVER_OK) ||\
                        (receiveDMAErr != ARM_DRIVER_OK))
                    {
                        err = ARM_DRIVER_ERROR_PARAMETER;
                    }
                    else
                    {

                    }
                }
                else
                {
                    /* Driver error will be caught below */
                }

                /* Verify enabling DMA did not cause error */
                if(err == ARM_DRIVER_OK)
                {
                    NVIC_EnableIRQ(module->irq);
                    DL_SPI_enableInterrupt(module->hw, DL_SPI_INTERRUPT_RX);
                    module->state->powerState = ARM_POWER_FULL;
                }
                else
                {
                    /* Errors, gracefully crash and proceed to ARM_POWER_OFF */
                    NVIC_DisableIRQ(module->irq);
                    DL_SPI_reset(module->hw);
                    DL_SPI_disablePower(module->hw);
                    NVIC_ClearPendingIRQ(module->irq);
                    MSP_ARM_SPI_ResetState(module);
                    module->state->powerState = ARM_POWER_OFF;
                }
            }
            break;
        default:
            break;
    }
    return err;
}

int32_t MSP_ARM_SPI_Control(DRIVER_SPI_MSP *module, uint32_t control,
                            uint32_t arg)
{
    uint32_t SCR = 0;
    uint32_t clockPhase = 0;
    uint32_t clockPolarity = 0;
    uint32_t frameFormat = 0;
    uint32_t chipSelect = 0;
    DL_SPI_backupConfig config;
    int32_t ret = ARM_DRIVER_OK;

    /* Control changes are only allowed when power state is full. */
    if (module->state->powerState != ARM_POWER_FULL)
    {
        return ARM_DRIVER_ERROR;
    }
    else
    {
        // Power state is ARM_POWER_FULL, no special action to take.
    }

    DL_SPI_disable(module->hw);

    //Control Mode parameters, cannot be OR'ed
    switch(control & ARM_SPI_CONTROL_Msk)
    {
        case ARM_SPI_MODE_INACTIVE:
            module->state->active = DRIVER_INACTIVE;
            break;
        case ARM_SPI_MODE_MASTER:
            module->state->active = DRIVER_ACTIVE;
            module->state->direction = DIRECTION_MASTER;
            DL_SPI_setMode(module->hw, DL_SPI_MODE_CONTROLLER);
            /* Configure MOSI pin as output and MISO as input */
            if (module->mosiPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
            {
                DL_GPIO_initPeripheralOutputFunction(
                    module->mosiPin.iomuxPinCtlMgmtRegIndex,
                    module->mosiPin.iomuxPinFunction);
            }
            else
            {
                // If no IO is defined, we have nothing to configure.
            }
            if (module->misoPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
            {
                DL_GPIO_initPeripheralInputFunction(
                    module->misoPin.iomuxPinCtlMgmtRegIndex,
                    module->misoPin.iomuxPinFunction);
            }
            else
            {
                // If no IO is defined, we have nothing to configure.
            }
            if (module->ssPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
            {
                DL_GPIO_initPeripheralOutputFunction(
                    module->ssPin.iomuxPinCtlMgmtRegIndex,
                    module->ssPin.iomuxPinFunction);
            }
            else
            {
                // If no IO is defined, we have nothing to configure.
            }
            if (module->sclkPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
            {
                DL_GPIO_initPeripheralOutputFunction(
                    module->sclkPin.iomuxPinCtlMgmtRegIndex,
                    module->sclkPin.iomuxPinFunction);
            }
            else
            {
                // If no IO is defined, we have nothing to configure.
            }
            break;
        case ARM_SPI_MODE_SLAVE:
            module->state->active = DRIVER_ACTIVE;
            module->state->direction = DIRECTION_SLAVE;
            DL_SPI_setMode(module->hw, DL_SPI_MODE_PERIPHERAL);
            /* Configure MOSI pin as input and MISO as output */
                    /* Configure MOSI pin as output and MISO as input */
            if (module->mosiPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
            {
                DL_GPIO_initPeripheralInputFunction(
                    module->mosiPin.iomuxPinCtlMgmtRegIndex,
                    module->mosiPin.iomuxPinFunction);
            }
            else
            {
                // If no IO is defined, we have nothing to configure.
            }
            if (module->misoPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
            {
                DL_GPIO_initPeripheralOutputFunction(
                    module->misoPin.iomuxPinCtlMgmtRegIndex,
                    module->misoPin.iomuxPinFunction);
            }
            else
            {
                // If no IO is defined, we have nothing to configure.
            }
            if (module->ssPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
            {
                DL_GPIO_initPeripheralInputFunction(
                    module->ssPin.iomuxPinCtlMgmtRegIndex,
                    module->ssPin.iomuxPinFunction);
            }
            else
            {
                // If no IO is defined, we have nothing to configure.
            }
            if (module->sclkPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
            {
                DL_GPIO_initPeripheralInputFunction(
                    module->sclkPin.iomuxPinCtlMgmtRegIndex,
                    module->sclkPin.iomuxPinFunction);
            }
            else
            {
                // If no IO is defined, we have nothing to configure.
            }
            break;
        case ARM_SPI_SET_BUS_SPEED:
            SCR = ((module->clockFreq / arg) - 2) / 2;
            if(SCR > MAX_SCR_VALUE)
            {
                ret = ARM_DRIVER_ERROR_PARAMETER;
            }
            else
            {
                DL_SPI_setBitRateSerialClockDivider(module->hw, SCR);
                DL_SPI_enable(module->hw);
            }
            return ret;
        case ARM_SPI_GET_BUS_SPEED:
            SCR = DL_SPI_getBitRateSerialClockDivider(module->hw);
            ret = module->clockFreq / ((SCR * 2) + 2);
            DL_SPI_enable(module->hw);
            return ret;
        case ARM_SPI_SET_DEFAULT_TX_VALUE:
            module->state->defaultTxValue = arg;
            DL_SPI_enable(module->hw);
            return ret;
        case ARM_SPI_CONTROL_SS:
            if(module->state->ssPinSWControlled == 0)
            {
                ret = ARM_DRIVER_ERROR_PARAMETER;
            }
            else
            {
                if(module->state->direction == DIRECTION_SLAVE)
                {
                    if(arg == ARM_SPI_SS_INACTIVE)
                    {
                        module->state->active = DRIVER_INACTIVE;
                    }
                    else if(arg == ARM_SPI_SS_ACTIVE)
                    {
                        module->state->active = DRIVER_ACTIVE;
                    }
                }
                else
                {
                    if(arg == ARM_SPI_SS_INACTIVE)
                    {
                        DL_GPIO_setPins(module->ssGPIOPort,\
                                        module->ssPinNumber);
                    }
                    else if(arg == ARM_SPI_SS_ACTIVE)
                    {
                        DL_GPIO_clearPins(module->ssGPIOPort,\
                                          module->ssPinNumber);
                    }
                }
            }
            DL_SPI_enable(module->hw);
            return ret;
        case ARM_SPI_ABORT_TRANSFER:
            DL_SPI_disable(module->hw);
            DL_SPI_saveConfiguration(module->hw, &config);
            DL_SPI_reset(module->hw);
            DL_SPI_restoreConfiguration(module->hw, &config);
            module->state->rxCnt = 0;
            module->state->rxTarCnt = 0;
            module->state->txCnt = 0;
            module->state->txTarCnt = 0;
            module->state->spiState = SPI_STATE_IDLE;
            DL_SPI_enable(module->hw);
            return ret;
        default:
            ret = ARM_DRIVER_ERROR_PARAMETER;
            DL_SPI_enable(module->hw);
            return ret;
    }

    /* Configure Phase and Polarity */
    if((control & ARM_SPI_FRAME_FORMAT_Msk) == ARM_SPI_CPOL0_CPHA1)
    {
        clockPhase = SPI_CTL0_SPH_SECOND;
        clockPolarity = SPI_CTL0_SPO_LOW;
    }
    else if((control & ARM_SPI_FRAME_FORMAT_Msk) == ARM_SPI_CPOL1_CPHA0)
    {
        clockPhase = SPI_CTL0_SPH_FIRST;
        clockPolarity = SPI_CTL0_SPO_HIGH;
    }
    else if((control & ARM_SPI_FRAME_FORMAT_Msk) == ARM_SPI_CPOL1_CPHA1)
    {
        clockPhase = SPI_CTL0_SPH_SECOND;
        clockPolarity = SPI_CTL0_SPO_HIGH;
    }
    else if((control & ARM_SPI_FRAME_FORMAT_Msk) == ARM_SPI_TI_SSI)
    {
        clockPhase = 0;
        clockPolarity = 0;
        frameFormat = SPI_CTL0_FRF_TI_SYNC;
    }
    else if((control & ARM_SPI_FRAME_FORMAT_Msk) == ARM_SPI_MICROWIRE)
    {
        ret = ARM_SPI_ERROR_FRAME_FORMAT;
    }
    else
    {
        clockPhase = SPI_CTL0_SPH_FIRST;
        clockPolarity = SPI_CTL0_SPO_LOW;
    }    

    /* Configure how the SS pin is being used */
    if(module->state->direction == DIRECTION_MASTER)
    {
        /* Master is using the SS line but SW controlled -> 3 wire mode */
        if(control & ARM_SPI_SS_MASTER_SW)
        {
            module->state->ssPinSWControlled = 1;
            frameFormat = SPI_CTL0_FRF_MOTOROLA_3WIRE;
            chipSelect = DL_SPI_CHIP_SELECT_NONE;
            DL_GPIO_enablePower(module->ssGPIOPort);
            DL_Common_delayCycles(4);
            DL_GPIO_initDigitalOutput(
                module->ssPin.iomuxPinCtlMgmtRegIndex);
            DL_GPIO_enableOutput(module->ssGPIOPort, module->ssPinNumber);
            DL_GPIO_setPins(module->ssGPIOPort, module->ssPinNumber);
        }
        /* Master is actively using SS -> 4 wire mode */
        else if(control & ARM_SPI_SS_MASTER_HW_OUTPUT)
        {
            /* Edge case: Using SS line but in TI_SYNC frame format from
             * which was configured from above.
            */
            if(frameFormat == SPI_CTL0_FRF_TI_SYNC)
            {
                //Do nothing
            }
            else
            {
                frameFormat = SPI_CTL0_FRF_MOTOROLA_4WIRE;
            }
            chipSelect = (uint32_t)(module->ss) << SPI_CTL0_CSSEL_OFS;
        }
        /* Master monitoring SS not supported (multi-master configuration) */
        else if(control & ARM_SPI_SS_MASTER_HW_INPUT)
        {
            ret = ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        //Control group default, SS line not used -> 3 wire mode
        else
        {
            frameFormat = SPI_CTL0_FRF_MOTOROLA_3WIRE;
            chipSelect = DL_SPI_CHIP_SELECT_NONE;
        }
    }
    else
    {
        /* Slave is using the SS line but SW controlled -> 3 wire mode */
        if(control & ARM_SPI_SS_SLAVE_SW)
        {
            module->state->active = DRIVER_INACTIVE;
            frameFormat = SPI_CTL0_FRF_MOTOROLA_3WIRE;
            chipSelect = DL_SPI_CHIP_SELECT_0;
        }
        /* Slave actively monitoring the SS line -> 4 wire mode */
        else
        {
            /* Edge case: Using SS line but in TI_SYNC frame format from
             * which was configured from above.
            */
            if(frameFormat == SPI_CTL0_FRF_TI_SYNC)
            {
                //Do nothing
            }
            else
            {
                frameFormat = SPI_CTL0_FRF_MOTOROLA_4WIRE;
            }
            chipSelect = (uint32_t)(module->ss) << SPI_CTL0_CSSEL_OFS;
        }
    }
    
    DL_SPI_setFrameFormat(module->hw, (DL_SPI_FRAME_FORMAT)\
                                      (clockPhase | clockPolarity | frameFormat));
    DL_SPI_setChipSelect(module->hw, chipSelect);

    if(control & ARM_SPI_DATA_BITS_Msk)
    {
        switch(control & ARM_SPI_DATA_BITS_Msk)
        {
            case ARM_SPI_DATA_BITS(4U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_4);
                break;
            case ARM_SPI_DATA_BITS(5U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_5);
                break;
            case ARM_SPI_DATA_BITS(6U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_6);
                break;
            case ARM_SPI_DATA_BITS(7U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_7);
                break;
            case ARM_SPI_DATA_BITS(8U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_8);
                break;
            case ARM_SPI_DATA_BITS(9U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_9);
                break;
            case ARM_SPI_DATA_BITS(10U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_10);
                break;
            case ARM_SPI_DATA_BITS(11U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_11);
                break;
            case ARM_SPI_DATA_BITS(12U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_12);
                break;
            case ARM_SPI_DATA_BITS(13U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_13);
                break;
            case ARM_SPI_DATA_BITS(14U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_14);
                break;
            case ARM_SPI_DATA_BITS(15U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_15);
                break;
            case ARM_SPI_DATA_BITS(16U):
                DL_SPI_setDataSize(module->hw, DL_SPI_DATA_SIZE_16);
                break;
            default:
                DL_SPI_enable(module->hw);
                return ARM_SPI_ERROR_DATA_BITS;
                break;
        }
    }
    else
    {

    }

    if(control & ARM_SPI_LSB_MSB)
    {
        DL_SPI_setBitOrder(module->hw, DL_SPI_BIT_ORDER_LSB_FIRST);
    }
    else
    {
        DL_SPI_setBitOrder(module->hw, DL_SPI_BIT_ORDER_MSB_FIRST);
    }

    DL_SPI_enable(module->hw);
    return ret;
}

int32_t MSP_ARM_SPI_Send(DRIVER_SPI_MSP *module, const void* data, uint32_t num)
{
    if((data == NULL) || (num == 0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }
    else if(module->state->status.busy == DRIVER_BUSY)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }
    else
    {
        DL_SPI_disable(module->hw);
    }

    if((uint8_t)DL_SPI_getDataSize(module->hw) >= 8)
    {
        module->state->bytesPerFrame = 2;
    }
    else
    {
        module->state->bytesPerFrame = 1;
    }

    module->state->status.data_lost = DATA_LOST_CLEAR;
    module->state->status.busy = DRIVER_BUSY;
    module->state->spiState = SPI_STATE_SEND_IN_PROGRESS;
    module->state->txBuf = data;
    module->state->txTarCnt = num;
    module->state->txCnt = 0;

    /* DMA-Driven Send */
    if(module->state->transmitDMAAvail == true)
    {
        if(module->state->bytesPerFrame == 2)
        {
            DL_DMA_setSrcWidth(module->transmitDMA.hw, module->transmitDMA.ch,\
                                  DL_DMA_WIDTH_HALF_WORD);
        }
        else
        {
            DL_DMA_setSrcWidth(module->transmitDMA.hw, module->transmitDMA.ch,\
                                  DL_DMA_WIDTH_BYTE);
        }

        DL_DMA_setSrcAddr(module->transmitDMA.hw, module->transmitDMA.ch,\
                          (uint32_t)module->state->txBuf);
        DL_DMA_setDestAddr(module->transmitDMA.hw, module->transmitDMA.ch,\
                          (uint32_t)(&module->hw->TXDATA));
        DL_DMA_setTransferSize(module->transmitDMA.hw, module->transmitDMA.ch,\
                               num);
        DL_DMA_enableChannel(module->transmitDMA.hw, module->transmitDMA.ch);
        DL_SPI_enableInterrupt(module->hw, DL_SPI_INTERRUPT_DMA_DONE_TX);
    }
    /* Interrupt Driven Transfer */
    else
    {
        if(module->state->bytesPerFrame == 2)
        {
            module->state->txCnt +=
            DL_SPI_fillTXFIFO16(module->hw,\
                                (uint16_t*)module->state->txBuf,\
                                module->state->txTarCnt);
        }
        else
        {
            module->state->txCnt +=
            DL_SPI_fillTXFIFO8(module->hw,\
                                (uint8_t*)module->state->txBuf,\
                                module->state->txTarCnt);
        }

        if(module->state->txCnt >= module->state->txTarCnt)
        {
            if(module->state->callback)
            {
                module->state->callback(ARM_SPI_EVENT_TRANSFER_COMPLETE);
            }
            else
            {

            }
        }
        else
        {
            DL_SPI_clearInterruptStatus(module->hw, DL_SPI_INTERRUPT_TX);
            DL_SPI_enableInterrupt(module->hw,  DL_SPI_INTERRUPT_TX);
        }
    }
    DL_SPI_enable(module->hw);
    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_SPI_Receive(DRIVER_SPI_MSP *module, void *data,
                            uint32_t num)
{
    uint32_t i;

    if((data == NULL) || (num == 0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }
    else if(module->state->status.busy == DRIVER_BUSY)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }
    /* Driver configured is configued as a slave with a software controlled
     * select, but the select signal has not be set active via
     * MSP_ARM_SPI_CONTROL with ARM_SPI_CONTROL_SS.
     */
    else if(module->state->ssPinSWControlled &&\
            module->state->direction == DIRECTION_SLAVE &&\
            module->state->active == DRIVER_INACTIVE)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        DL_SPI_disable(module->hw);
    }

    module->state->status.data_lost = DATA_LOST_CLEAR;    
    module->state->status.busy = DRIVER_BUSY;
    module->state->spiState = SPI_STATE_RECEIVE_IN_PROGRESS;
    module->state->rxBuf = data;
    module->state->rxTarCnt = num;
    module->state->rxCnt = 0;

    if((uint8_t)DL_SPI_getDataSize(module->hw) >= 8)
    {
        module->state->bytesPerFrame = 2;
    }
    else
    {
        module->state->bytesPerFrame = 1;
    }

    /* DMA-Driven Receive */
    if(module->state->receiveDMAAvail == true)
    {
        if(module->state->bytesPerFrame == 2)
        {
            DL_DMA_setSrcWidth(module->receiveDMA.hw, module->receiveDMA.ch,\
                                  DL_DMA_WIDTH_HALF_WORD);
        }
        else
        {
            DL_DMA_setSrcWidth(module->receiveDMA.hw, module->receiveDMA.ch,\
                                  DL_DMA_WIDTH_BYTE);
        }

        DL_DMA_setSrcAddr(module->receiveDMA.hw, module->receiveDMA.ch,\
                          (uint32_t)(&module->hw->RXDATA));
        DL_DMA_setDestAddr(module->receiveDMA.hw, module->receiveDMA.ch,\
                          (uint32_t)module->state->rxBuf);
        DL_DMA_setTransferSize(module->receiveDMA.hw, module->receiveDMA.ch,\
                               num);
        DL_DMA_enableChannel(module->receiveDMA.hw, module->receiveDMA.ch);
        DL_SPI_enableInterrupt(module->hw, DL_SPI_INTERRUPT_DMA_DONE_RX);
    }
    else
    {
        // If driver is the master, it can start the transaction with default TX
        if(module->state->direction == DIRECTION_MASTER)
        {
            if(module->state->bytesPerFrame == 2)
            {
                uint16_t buf[module->state->rxTarCnt];
                for(i = 0; i < module->state->rxTarCnt; i++)
                {
                    buf[i] = module->state->defaultTxValue;
                }
                DL_SPI_fillTXFIFO16(module->hw, &buf[0], module->state->rxTarCnt);
            }
            else
            {
                uint8_t buf[module->state->rxTarCnt];
                for(i = 0; i < module->state->rxTarCnt; i++)
                {
                    buf[i] = module->state->defaultTxValue;
                }
                DL_SPI_fillTXFIFO8(module->hw, &buf[0], module->state->rxTarCnt);
            }
            DL_SPI_clearInterruptStatus(module->hw, DL_SPI_INTERRUPT_TX);
            DL_SPI_enableInterrupt(module->hw, DL_SPI_INTERRUPT_TX);
        }
        else
        {
            /* Slave can only register the operation */
        }
    }
    DL_SPI_enable(module->hw);
    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_SPI_Transfer(DRIVER_SPI_MSP *module, const void *data_out,
    void *data_in, uint32_t num)
{
    if((data_out == NULL) || (data_in == NULL) || (num == 0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }
    else if(module->state->status.busy == DRIVER_BUSY)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }
    else
    {
        DL_SPI_disable(module->hw);
    }

    if((uint8_t)DL_SPI_getDataSize(module->hw) >= 8)
    {
        module->state->bytesPerFrame = 2;
    }
    else
    {
        module->state->bytesPerFrame = 1;
    }

    module->state->status.data_lost = DATA_LOST_CLEAR;
    module->state->status.busy = DRIVER_BUSY;
    module->state->spiState = SPI_STATE_TRANSFER_IN_PROGRESS;
    module->state->txBuf = data_out;
    module->state->txTarCnt = num;
    module->state->txCnt = 0;
    module->state->rxBuf = data_in;
    module->state->rxTarCnt = num;
    module->state->rxCnt = 0;

    /* DMA-Driven Transfer */
    if(module->state->transmitDMAAvail == true &&\
       module->state->receiveDMAAvail == true)
    {
        if(module->state->bytesPerFrame == 2)
        {
            DL_DMA_setSrcWidth(module->transmitDMA.hw, module->transmitDMA.ch,\
                                  DL_DMA_WIDTH_HALF_WORD);
            DL_DMA_setSrcWidth(module->receiveDMA.hw, module->receiveDMA.ch,\
                                  DL_DMA_WIDTH_HALF_WORD);
        }
        else
        {
            DL_DMA_setSrcWidth(module->transmitDMA.hw, module->transmitDMA.ch,\
                                  DL_DMA_WIDTH_BYTE);
            DL_DMA_setSrcWidth(module->receiveDMA.hw, module->receiveDMA.ch,\
                                  DL_DMA_WIDTH_BYTE);
        }
        DL_DMA_setSrcAddr(module->transmitDMA.hw, module->transmitDMA.ch,\
                          (uint32_t)module->state->txBuf);
        DL_DMA_setDestAddr(module->transmitDMA.hw, module->transmitDMA.ch,\
                          (uint32_t)(&module->hw->TXDATA));
        DL_DMA_setTransferSize(module->transmitDMA.hw, module->transmitDMA.ch,\
                               num);
        DL_DMA_enableChannel(module->transmitDMA.hw, module->transmitDMA.ch);
        DL_SPI_enableInterrupt(module->hw, DL_SPI_INTERRUPT_DMA_DONE_TX);

        DL_DMA_setSrcAddr(module->receiveDMA.hw, module->receiveDMA.ch,\
                          (uint32_t)(&module->hw->RXDATA));
        DL_DMA_setDestAddr(module->receiveDMA.hw, module->receiveDMA.ch,\
                          (uint32_t)module->state->rxBuf);
        DL_DMA_setTransferSize(module->receiveDMA.hw, module->receiveDMA.ch,\
                               num);
        DL_DMA_enableChannel(module->receiveDMA.hw, module->receiveDMA.ch);
        DL_SPI_enableInterrupt(module->hw, DL_SPI_INTERRUPT_DMA_DONE_TX|\
                                           DL_SPI_INTERRUPT_DMA_DONE_RX);
    }
    else
    {
        if(module->state->bytesPerFrame == 2)
        {
            module->state->txCnt +=
                DL_SPI_fillTXFIFO16(module->hw,
                    (uint16_t*)module->state->txBuf, module->state->txTarCnt);
        }
        else
        {
            module->state->txCnt +=
                DL_SPI_fillTXFIFO8(module->hw, (uint8_t*)module->state->txBuf,\
                                   module->state->txTarCnt);
        }
        DL_SPI_clearInterruptStatus(module->hw, DL_SPI_INTERRUPT_TX |
                                                DL_SPI_INTERRUPT_RX);
        DL_SPI_enableInterrupt(module->hw, DL_SPI_INTERRUPT_RX |\
                                           DL_SPI_INTERRUPT_TX);
    }
    DL_SPI_enable(module->hw);
    return ARM_DRIVER_OK;
}

uint32_t MSP_ARM_SPI_GetDataCount(DRIVER_SPI_MSP *module)
{
    uint32_t count;

    if(module->state->status.busy == DRIVER_NOT_BUSY ||\
       module->state->spiState == SPI_STATE_IDLE)
    {
        count = 0;
    }
    else
    {
        if(module->state->spiState == SPI_STATE_SEND_IN_PROGRESS)
        {
            if(module->state->transmitDMAAvail == 1)
            {
                count = module->state->txTarCnt - 
                        DL_DMA_getTransferSize(module->transmitDMA.hw,\
                                                module->transmitDMA.ch);
            }
            else
            {
                count = module->state->txCnt;
            }
        }
        else if(module->state->spiState == SPI_STATE_RECEIVE_IN_PROGRESS ||\
                module->state->spiState == SPI_STATE_TRANSFER_IN_PROGRESS)
        {
            if(module->state->receiveDMAAvail == 1)
            {
                count = module->state->rxTarCnt - 
                        DL_DMA_getTransferSize(module->receiveDMA.hw,\
                                                module->receiveDMA.ch);
            }
            else
            {
                count = module->state->rxCnt;
            }
        }
        else
        {
            count = 0;
        }
    }
    return count;
}

ARM_SPI_STATUS MSP_ARM_SPI_GetStatus(DRIVER_SPI_MSP *module)
{
    return module->state->status;
}

void MSP_ARM_SPI_IRQHandler(DRIVER_SPI_MSP *module)
{
    switch(DL_SPI_getPendingInterrupt(module->hw))
    {
        case DL_SPI_IIDX_DMA_DONE_TX:
            if(module->state->spiState == SPI_STATE_SEND_IN_PROGRESS)
            {
                if(module->state->callback)
                {
                    module->state->status.busy = DRIVER_NOT_BUSY;
                    module->state->spiState = SPI_STATE_IDLE;
                    module->state->callback(ARM_SPI_EVENT_TRANSFER_COMPLETE);
                }
                else
                {

                }
            }
            break;
        case DL_SPI_IIDX_DMA_DONE_RX:
            if(module->state->spiState == SPI_STATE_SEND_IN_PROGRESS ||
               module->state->spiState == SPI_STATE_TRANSFER_IN_PROGRESS)
            {
                if(module->state->callback)
                {
                    module->state->status.busy = DRIVER_NOT_BUSY;
                    module->state->spiState = SPI_STATE_IDLE;
                    module->state->callback(ARM_SPI_EVENT_TRANSFER_COMPLETE);
                }
                else
                {

                }
            }
            break;
        case DL_SPI_IIDX_RX:
            /* Driver actively receiving or transferring */
            if((module->state->spiState == SPI_STATE_RECEIVE_IN_PROGRESS ||\
                module->state->spiState == SPI_STATE_TRANSFER_IN_PROGRESS)\
                && module->state->rxCnt < module->state->rxTarCnt)
            {
                if(module->state->bytesPerFrame == 2)
                {
                    while(!DL_SPI_isRXFIFOEmpty(module->hw) &&\
                         (module->state->rxCnt < module->state->rxTarCnt))
                    {
                        ((uint16_t*)module->state->rxBuf)[module->state->rxCnt++] =\
                                                    DL_SPI_receiveData16(module->hw);
                    }
                }
                else
                {
                    while(!DL_SPI_isRXFIFOEmpty(module->hw) &&\
                         (module->state->rxCnt < module->state->rxTarCnt))
                     {
                        ((uint8_t*)module->state->rxBuf)[module->state->rxCnt++] =\
                                                DL_SPI_receiveData8(module->hw);
                     }
                }
                if(module->state->spiState == SPI_STATE_RECEIVE_IN_PROGRESS &&\
                   module->state->rxCnt == module->state->rxTarCnt)
                {
                    DL_SPI_disableInterrupt(module->hw, DL_SPI_INTERRUPT_RX);
                    if(module->state->callback)
                    {
                        module->state->status.busy = DRIVER_NOT_BUSY;
                        module->state->spiState = SPI_STATE_IDLE;
                        module->state->callback\
                            (ARM_SPI_EVENT_TRANSFER_COMPLETE);
                    }
                    else
                    {

                    }
                }
                else if(module->state->spiState == SPI_STATE_TRANSFER_IN_PROGRESS &&\
                        module->state->rxCnt == module->state->rxTarCnt &&\
                        module->state->txCnt == module->state->txTarCnt)
                {
                    DL_SPI_disableInterrupt(module->hw, DL_SPI_INTERRUPT_RX |\
                                                        DL_SPI_INTERRUPT_TX);
                    if(module->state->callback)
                    {
                        module->state->status.busy = DRIVER_NOT_BUSY;
                        module->state->spiState = SPI_STATE_IDLE;
                        module->state->callback\
                            (ARM_SPI_EVENT_TRANSFER_COMPLETE);
                    }
                    else
                    {

                    }
                }
                /* If the Driver is the master and receiving, it is responsible
                   for clocking out more data until rxTarCnt reached. */
                else if((module->state->direction == DIRECTION_MASTER) &&\
                (module->state->spiState == SPI_STATE_RECEIVE_IN_PROGRESS) &&\
                (module->state->rxCnt < module->state->rxTarCnt))
                {
                    module->hw->TXDATA =\
                                    (uint8_t)module->state->defaultTxValue;
                }
            }
            /* Slave RX data but no operation was started -> data lost event */
            else if(module->state->spiState == SPI_STATE_IDLE)
            {
                if(module->state->callback)
                {
                    module->state->status.data_lost = DATA_LOST_SET;
                    module->state->callback(ARM_SPI_EVENT_DATA_LOST);
                }
                else
                {

                }
            }
            else
            {

            }
            break;
        case DL_SPI_IIDX_TX:
            /* Driver actively sending or transferring */
            if((module->state->spiState == SPI_STATE_SEND_IN_PROGRESS ||
                module->state->spiState == SPI_STATE_TRANSFER_IN_PROGRESS)\
                && (module->state->txCnt < module->state->txTarCnt))
            {
                if(module->state->bytesPerFrame == 2)
                {
                    module->state->txCnt += DL_SPI_fillTXFIFO16(module->hw,\
                    &((uint16_t*)module->state->txBuf)[module->state->txCnt],\
                    module->state->txTarCnt - module->state->txCnt);
                }
                else
                {
                    module->state->txCnt += DL_SPI_fillTXFIFO8(module->hw,\
                    &((uint8_t*)module->state->txBuf)[module->state->txCnt],\
                    module->state->txTarCnt - module->state->txCnt);
                }

                if(module->state->spiState == SPI_STATE_SEND_IN_PROGRESS &&\
                   module->state->txCnt >= module->state->txTarCnt)
                {
                    DL_SPI_disableInterrupt(module->hw, DL_SPI_INTERRUPT_TX);
                    if(module->state->callback)
                    {
                        module->state->status.busy = DRIVER_NOT_BUSY;
                        module->state->spiState = SPI_STATE_IDLE;
                        DL_SPI_disableInterrupt(module->hw, DL_SPI_INTERRUPT_TX);
                        module->state->callback(ARM_SPI_EVENT_TRANSFER_COMPLETE);
                    }
                    else
                    {

                    }
                }
                else
                {

                }
            }
            else
            {

            }
            break;
        default:
            break;
    }
}
