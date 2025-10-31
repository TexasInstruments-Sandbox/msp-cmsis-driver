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
 *  @file       Driver_I2C_MSP_Priv.c
 *  
 *              This implementation supports HW version V0.
 *
 ******************************************************************************
 */

#include <ti/driverlib/driverlib.h>
#include <Driver_I2C_MSP_Priv.h>

/**
 * @brief DriverCapabilities stores the ARM CMSIS-Driver I2C
 *        driver capabilities supported by this driver implementation.
 */
static const ARM_I2C_CAPABILITIES DriverCapabilities = {
    1, /* supports 10-bit addressing */
    0  /* Reserved (must be zero) */
};


/*  ARM CMSIS-Driver API instance generic function implementations
 *  are given below to implement the CMSIS-Driver.
 *  Refer to the Arm CMSIS-Driver specifications for API definitions.
 */

ARM_DRIVER_VERSION MSP_ARM_I2C_GetVersion(void)
{
  return DriverVersion;
}

ARM_I2C_CAPABILITIES MSP_ARM_I2C_GetCapabilities(DRIVER_I2C_MSP *module)
{
  return DriverCapabilities;
}

static void MSP_ARM_I2C_ResetState(DRIVER_I2C_MSP *module)
{
    module->state->i2cState = I2C_STATE_IDLE;
    module->state->txTarCnt = 0;
    module->state->txCnt = 0;
    module->state->rxTarCnt = 0;
    module->state->rxCnt = 0;
    module->state->controllerEnabled = 0;
    module->state->targetEnabled = 0;
    module->state->status.busy = 0;
    module->state->status.mode = 0;
    module->state->status.direction = 0;
    module->state->status.general_call = 0;
    module->state->status.arbitration_lost = 0;
    module->state->status.bus_error = 0;
}

void MSP_ARM_I2C_ConfigureController(DRIVER_I2C_MSP *module)
{
    if(module->state->targetEnabled == 1)
    {
        DL_I2C_disableTarget(module->hw);
        DL_I2C_clearInterruptStatus(module->hw, \
                                    DL_I2C_INTERRUPT_TARGET_START |
                                    DL_I2C_INTERRUPT_TARGET_STOP |
                                    DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
                                    DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
                                    DL_I2C_INTERRUPT_TARGET_ARBITRATION_LOST);

        DL_I2C_disableInterrupt(module->hw, \
                                    DL_I2C_INTERRUPT_TARGET_START |
                                    DL_I2C_INTERRUPT_TARGET_STOP |
                                    DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
                                    DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER |
                                    DL_I2C_INTERRUPT_TARGET_ARBITRATION_LOST);
        module->state->targetEnabled = 0;
    }
    else 
    {

    }

    DL_I2C_clearInterruptStatus(module->hw, \
                                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE | 
                                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
                                DL_I2C_INTERRUPT_CONTROLLER_NACK | 
                                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
    DL_I2C_enableInterrupt(module->hw, \
                                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE | 
                                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
                                DL_I2C_INTERRUPT_CONTROLLER_NACK | 
                                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

    DL_I2C_setControllerTXFIFOThreshold(module->hw, \
                                        DL_I2C_TX_FIFO_LEVEL_BYTES_1);
    DL_I2C_setControllerRXFIFOThreshold(module->hw, \
                                        DL_I2C_RX_FIFO_LEVEL_BYTES_1);       
    DL_I2C_enableControllerClockStretching(module->hw);                                
    DL_I2C_enableController(module->hw);
    module->state->controllerEnabled = 1;
    module->state->status.mode = 1;
}

void MSP_ARM_I2C_ConfigureTarget(DRIVER_I2C_MSP *module) 
{
    if(module->state->controllerEnabled == 1)
    {
        DL_I2C_disableController(module->hw);
        DL_I2C_clearInterruptStatus(module->hw, \
                                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
                                DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
                                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                                DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER |
                                DL_I2C_INTERRUPT_CONTROLLER_NACK |
                                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);

        DL_I2C_disableInterrupt(module->hw, \
                                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
                                DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
                                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                                DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER |
                                DL_I2C_INTERRUPT_CONTROLLER_NACK |
                                DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST);
        module->state->controllerEnabled = 0;
    }
    else 
    {

    }

    DL_I2C_clearInterruptStatus(module->hw, \
                                DL_I2C_INTERRUPT_TARGET_START |
                                DL_I2C_INTERRUPT_TARGET_STOP |
                                DL_I2C_INTERRUPT_TARGET_ARBITRATION_LOST);
    DL_I2C_enableInterrupt(module->hw, \
                                DL_I2C_INTERRUPT_TARGET_START |
                                DL_I2C_INTERRUPT_TARGET_STOP |
                                DL_I2C_INTERRUPT_TARGET_ARBITRATION_LOST);

    DL_I2C_setTargetTXFIFOThreshold(module->hw, DL_I2C_TX_FIFO_LEVEL_BYTES_1);
    DL_I2C_setTargetRXFIFOThreshold(module->hw, DL_I2C_RX_FIFO_LEVEL_BYTES_1);                                       
    /* Workaround for I2C_ERR_04 */
    DL_I2C_disableTargetWakeup(module->hw);
    DL_I2C_enableTarget(module->hw);
    module->state->targetEnabled = 1;
    module->state->status.mode = 0;
}

int32_t MSP_ARM_I2C_initDMACh(DRIVER_DMA_MSP *cfg, bool incDest)
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

int32_t MSP_ARM_I2C_Initialize(DRIVER_I2C_MSP *module, 
                                        ARM_I2C_SignalEvent_t cb_event)
{
    MSP_ARM_I2C_ResetState(module);
    module->state->callback = cb_event;

    if (module->sdaPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        DL_GPIO_initPeripheralInputFunctionFeatures(
            module->sdaPin.iomuxPinCtlMgmtRegIndex, 
            module->sdaPin.iomuxPinFunction, DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, 
            DL_GPIO_WAKEUP_DISABLE);
        DL_GPIO_enableHiZ(module->sdaPin.iomuxPinCtlMgmtRegIndex);
    }
    else
    {
        // If no IO is defined, we have nothing to configure.
    }

    if (module->sclPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        DL_GPIO_initPeripheralInputFunctionFeatures(
            module->sclPin.iomuxPinCtlMgmtRegIndex, 
            module->sclPin.iomuxPinFunction, DL_GPIO_INVERSION_DISABLE,
            DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE, 
            DL_GPIO_WAKEUP_DISABLE);
        DL_GPIO_enableHiZ(module->sclPin.iomuxPinCtlMgmtRegIndex);
    }
    else
    {
        // If no IO is defined, we have nothing to configure.
    }

    module->state->initDone = DRIVER_INITIALIZED;
    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_I2C_Uninitialize(DRIVER_I2C_MSP *module)
{
    module->state->callback = NULL;

    if (module->sdaPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        IOMUX->SECCFG.PINCM[module->sdaPin.iomuxPinCtlMgmtRegIndex] =\
            IOMUX_PINCM_PC_UNCONNECTED;
    }
    else
    {
        // If no IO is defined, we have nothing to uninitialize.
    }

    if (module->sclPin.iomuxPinCtlMgmtRegIndex != DRIVER_IO_MSP_NONE)
    {
        IOMUX->SECCFG.PINCM[module->sclPin.iomuxPinCtlMgmtRegIndex] =\
            IOMUX_PINCM_PC_UNCONNECTED;
    }
    else
    {
        // If no IO is defined, we have nothing to uninitialize.
    }

    module->state->initDone = DRIVER_UNINITIALIZED;
    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_I2C_PowerControl(DRIVER_I2C_MSP *module, ARM_POWER_STATE state)
{
    if(state != ARM_POWER_OFF && module->state->initDone != DRIVER_INITIALIZED)
    {
        return ARM_DRIVER_ERROR;
    }

    DL_I2C_ClockConfig clkCfg;
    int32_t err = ARM_DRIVER_OK;
    int32_t transmitDMAErr;
    int32_t receiveDMAErr;

    switch(state)
    {
        case ARM_POWER_OFF:
            NVIC_DisableIRQ(module->irq);
            DL_I2C_reset(module->hw);
            DL_I2C_disablePower(module->hw);
            NVIC_ClearPendingIRQ(module->irq);
            MSP_ARM_I2C_ResetState(module);
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
                DL_I2C_enablePower(module->hw);
                __NOP(); /* Insert 4 NOPs for power enable cycle time */
                __NOP();
                __NOP();
                __NOP();
                if (module->clock == DRIVER_CLK_MSP_MFCLK)
                {
                    clkCfg.clockSel = DL_I2C_CLOCK_MFCLK;
                }
                else if(module->clock == DRIVER_CLK_MSP_BUSCLK)
                {
                    clkCfg.clockSel = DL_I2C_CLOCK_BUSCLK;
                }
                else
                {
                    err = ARM_DRIVER_ERROR;
                }

                if(err == ARM_DRIVER_OK)
                {
                    clkCfg.divideRatio = DL_I2C_CLOCK_DIVIDE_1;
                    DL_I2C_setClockConfig(module->hw, &clkCfg);
                    if (module->transmitDMA.hw != DRIVER_DMA_HW_NONE)
                    {
                        module->state->transmitDMAAvail = true;
                        transmitDMAErr =\
                        MSP_ARM_I2C_initDMACh(&(module->transmitDMA), false);
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
                        MSP_ARM_I2C_initDMACh(&(module->receiveDMA), true);
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
                    module->state->powerState = ARM_POWER_FULL;
                }
                else
                {
                    /* Errors, gracefully crash and proceed to ARM_POWER_OFF */
                    NVIC_DisableIRQ(module->irq);
                    DL_I2C_reset(module->hw);
                    DL_I2C_disablePower(module->hw);
                    NVIC_ClearPendingIRQ(module->irq);
                    MSP_ARM_I2C_ResetState(module);
                    module->state->powerState = ARM_POWER_OFF;
                }   
            }
            break;
        default:
            break;
    }
    return err;
}

int32_t MSP_ARM_I2C_Control(DRIVER_I2C_MSP *module, uint32_t control, 
                            uint32_t arg)
{
    /* Control changes are only allowed when power state is full. */
    if (module->state->powerState != ARM_POWER_FULL)
    {
        return ARM_DRIVER_ERROR;
    }
    else
    {
        // Power state is ARM_POWER_FULL, no special action to take.
    }

    if(control == ARM_I2C_OWN_ADDRESS)
    {

        // Configure as target if not yet already done
        if(module->state->targetEnabled == 0)
        {   
            MSP_ARM_I2C_ConfigureTarget(module);
        }
        else
        {

        }

        if(arg & ARM_I2C_ADDRESS_10BIT)
        {
            DL_I2C_setTargetAddressingMode(module->hw, 
                                    DL_I2C_TARGET_ADDRESSING_MODE_10_BIT);
        }
        else
        {
            DL_I2C_setTargetAddressingMode(module->hw, 
                                    DL_I2C_TARGET_ADDRESSING_MODE_7_BIT);
        }

        if(arg & ARM_I2C_ADDRESS_GC)
        {
            DL_I2C_enableGeneralCall(module->hw);
            DL_I2C_enableInterrupt(module->hw, 
                                    DL_I2C_INTERRUPT_TARGET_GENERAL_CALL);
        }
        else
        {
            DL_I2C_disableGeneralCall(module->hw);
        }
        DL_I2C_setTargetOwnAddress(module->hw, arg);
    }
    else
    {
        //Do nothing
    }

    if(control == ARM_I2C_BUS_SPEED)
    {
        const uint32_t I2C_CLK  = module->clockFreq;
        uint8_t TPR = 0;
        if(arg & ARM_I2C_BUS_SPEED_STANDARD)
        {   
            TPR = (I2C_CLK / (100000U * 10)) - 1;
        }
        else if(arg & ARM_I2C_BUS_SPEED_FAST)
        {
            TPR = (I2C_CLK / (400000U * 10)) - 1;
        }
        else if(arg & ARM_I2C_BUS_SPEED_FAST_PLUS)
        {
            TPR = (I2C_CLK / (1000000U * 10)) - 1;
        }
        else if(arg & ARM_I2C_BUS_SPEED_HIGH)
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        else
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
        DL_I2C_setTimerPeriod(module->hw, TPR);
    }
    else
    {
        //Do nothing
    }

    if((control == ARM_I2C_BUS_CLEAR) &&\
            module->state->controllerEnabled == 1)
    {
        DL_I2C_resetControllerTransfer(module->hw);
        uint8_t bus_clear[] = { 0x00 };
        DL_I2C_fillControllerTXFIFO(module->hw, &bus_clear[0], 1);
        DL_I2C_startControllerTransferAdvanced(module->hw, \
                                        0x00, DL_I2C_CONTROLLER_DIRECTION_TX, 1, 
                                        DL_I2C_CONTROLLER_START_ENABLE, 
                                        DL_I2C_CONTROLLER_STOP_ENABLE, 
                                        DL_I2C_CONTROLLER_ACK_DISABLE);
        while (DL_I2C_getControllerStatus(module->hw) & \
               DL_I2C_CONTROLLER_STATUS_BUSY_BUS);
    }
    else
    {

    }

    if((control == ARM_I2C_ABORT_TRANSFER) &&\
            module->state->i2cState != I2C_STATE_IDLE)
    {
        if(module->state->controllerEnabled)
        {
            DL_I2C_resetControllerTransfer(module->hw);
        }
        else if(module->state->targetEnabled)
        {
            
        }
        else
        {
            return ARM_DRIVER_ERROR;
        }
        module->state->i2cState = I2C_STATE_IDLE;
        module->state->status.busy = 0;
        module->state->txTarCnt = 0;
        module->state->txCnt = 0;
        module->state->rxTarCnt = 0;
        module->state->rxCnt = 0;
        module->state->status.direction = 0;
        module->state->status.general_call = 0;
        module->state->status.arbitration_lost = 0;
        module->state->status.bus_error = 0;
    }
    else
    {

    }
    
    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_I2C_MasterTransmit(DRIVER_I2C_MSP *module, uint32_t addr, 
                                   const uint8_t *data, uint32_t num, 
                                   bool xfer_pending) 
{
    DL_I2C_CONTROLLER_STOP stopEnable;

    if((data == NULL) || (num == 0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }
    else
    {

    }

    if(module->state->i2cState != I2C_STATE_IDLE)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }
    else
    {
        module->state->i2cState = I2C_STATE_TX_STARTED;
        module->state->status.busy = 1;
        module->state->status.direction = 0;
        module->state->status.arbitration_lost = 0;
    }

    // The I2C module has not yet been configured to act in controller mode 
    if(module->state->controllerEnabled != 1)
    {
        MSP_ARM_I2C_ConfigureController(module);
    }
    else
    {
        // Already in controller mode 
    }

    if(addr & ARM_I2C_ADDRESS_10BIT)
    {
        DL_I2C_setControllerAddressingMode(module->hw, \
                                    DL_I2C_CONTROLLER_ADDRESSING_MODE_10_BIT);
    }
    else
    {
        DL_I2C_setControllerAddressingMode(module->hw, \
                                    DL_I2C_CONTROLLER_ADDRESSING_MODE_7_BIT);
    }

    module->state->rxTarCnt = 0;
    module->state->rxCnt = 0;   
    module->state->txBuf = data;
    module->state->txTarCnt = num;
    module->state->txCnt = 0;
    DL_I2C_clearInterruptStatus(module->hw, \
                                DL_I2C_INTERRUPT_CONTROLLER_TX_DONE |
                                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    /* DMA-Driven Transfer */
    if(module->state->transmitDMAAvail == true)
    {
        DL_DMA_setSrcAddr(module->transmitDMA.hw, module->transmitDMA.ch, \
                                (uint32_t)(module->state->txBuf));
        DL_DMA_setDestAddr(module->transmitDMA.hw, module->transmitDMA.ch, \
                                (uint32_t)(&module->hw->MASTER.MTXDATA));
        DL_DMA_setTransferSize(module->transmitDMA.hw, module->transmitDMA.ch, \
                                module->state->txTarCnt);

        DL_I2C_clearInterruptStatus(module->hw, \
                                DL_I2C_INTERRUPT_CONTROLLER_EVENT1_DMA_DONE);
        DL_I2C_enableInterrupt(module->hw, \
                                DL_I2C_INTERRUPT_CONTROLLER_EVENT1_DMA_DONE);
        DL_I2C_enableDMAEvent(module->hw, \
                                DL_I2C_EVENT_ROUTE_1, 
                                DL_I2C_DMA_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        DL_DMA_enableChannel(module->transmitDMA.hw, module->transmitDMA.ch);
    }
    /* Interrupt Driven Transfer */
    else
    {
        DL_I2C_flushControllerTXFIFO(module->hw);
        module->state->txCnt = DL_I2C_fillControllerTXFIFO(module->hw, \
                                                            data, num);
        if(module->state->txCnt < module->state->txTarCnt) 
        {
            DL_I2C_clearInterruptStatus(module->hw, \
                                    DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            DL_I2C_enableInterrupt(module->hw, \
                                    DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        }
        else
        {
            DL_I2C_disableInterrupt(module->hw, \
                                    DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
        }
    }
    
    stopEnable = xfer_pending ? DL_I2C_CONTROLLER_STOP_DISABLE : 
                                DL_I2C_CONTROLLER_STOP_ENABLE;
    DL_I2C_startControllerTransferAdvanced(module->hw, addr, 
                                            DL_I2C_CONTROLLER_DIRECTION_TX, num, 
                                            DL_I2C_CONTROLLER_START_ENABLE, 
                                            stopEnable, 
                                            DL_I2C_CONTROLLER_ACK_DISABLE);
   
    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_I2C_MasterReceive(DRIVER_I2C_MSP *module, uint32_t addr, 
                                    uint8_t *data, uint32_t num, 
                                    bool xfer_pending)
{
    DL_I2C_CONTROLLER_STOP stopEnable;

    if((data == NULL) || (num == 0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }
    else
    {

    }

    if(module->state->i2cState != I2C_STATE_IDLE)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }
    else
    {
        module->state->i2cState = I2C_STATE_RX_STARTED;
        module->state->status.busy = 1;
        module->state->status.direction = 1;
        module->state->status.arbitration_lost = 0;
    }

    // The I2C module has not yet been configured to act in controller mode 
    if(module->state->controllerEnabled != 1)
    {
        MSP_ARM_I2C_ConfigureController(module);
    }
    else
    {
        // Already in controller mode 
    }

    module->state->txTarCnt = 0;
    module->state->txCnt = 0;   
    module->state->rxBuf = data;
    module->state->rxTarCnt = num;
    module->state->rxCnt = 0;
    DL_I2C_clearInterruptStatus(module->hw, \
                                DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    /* DMA-Driven Transfer */
    if(module->state->receiveDMAAvail == true)
    {
        DL_DMA_setSrcAddr(module->receiveDMA.hw, module->receiveDMA.ch, 
                            (uint32_t)(&module->hw->MASTER.MRXDATA));
        DL_DMA_setDestAddr(module->receiveDMA.hw, module->receiveDMA.ch, 
                            (uint32_t)(module->state->rxBuf));
        DL_DMA_setTransferSize(module->receiveDMA.hw, module->receiveDMA.ch, 
                            module->state->rxTarCnt);

        DL_I2C_enableInterrupt(module->hw, \
                                DL_I2C_INTERRUPT_CONTROLLER_EVENT2_DMA_DONE);
        DL_I2C_enableDMAEvent(module->hw, DL_I2C_EVENT_ROUTE_2, \
                                DL_I2C_DMA_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
        DL_DMA_enableChannel(module->receiveDMA.hw, module->receiveDMA.ch);
    }
    /* Interrupt Driven Transfer */
    else
    {
        DL_I2C_enableInterrupt(module->hw, \
                                    DL_I2C_INTERRUPT_CONTROLLER_RX_DONE | 
                                    DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
    }

    stopEnable = xfer_pending ? DL_I2C_CONTROLLER_STOP_DISABLE : 
                                DL_I2C_CONTROLLER_STOP_ENABLE;
    
    DL_I2C_startControllerTransferAdvanced(module->hw, addr, 
                                    DL_I2C_CONTROLLER_DIRECTION_RX, num, 
                                    DL_I2C_CONTROLLER_START_ENABLE, stopEnable, 
                                    DL_I2C_CONTROLLER_ACK_DISABLE); 

    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_I2C_SlaveTransmit(DRIVER_I2C_MSP *module, const uint8_t *data, 
                                    uint32_t num)
{
    if((data == NULL) || (num == 0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }
    else
    {

    }

    if(module->state->i2cState != I2C_STATE_IDLE)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }
    else
    {
        module->state->i2cState = I2C_STATE_TX_STARTED;
    }

    // The I2C module has not yet been configured to act in target mode 
    if(module->state->targetEnabled != 1)
    {
        MSP_ARM_I2C_ConfigureTarget(module);
    }
    else
    {
        // Already in target mode 
    }

    module->state->rxTarCnt = 0;
    module->state->rxCnt = 0;   
    module->state->txBuf = data;
    module->state->txTarCnt = num;
    module->state->txCnt = 0;
    DL_I2C_clearInterruptStatus(module->hw, DL_I2C_INTERRUPT_TARGET_START | 
                                            DL_I2C_INTERRUPT_TARGET_STOP);

    /* DMA-Driven Transfer */
    if(module->state->transmitDMAAvail == true)
    {
        DL_DMA_setSrcAddr(module->transmitDMA.hw, module->transmitDMA.ch, 
                            (uint32_t)(module->state->txBuf));
        DL_DMA_setDestAddr(module->transmitDMA.hw, module->transmitDMA.ch, 
                            (uint32_t)(&module->hw->SLAVE.STXDATA));
        DL_DMA_setTransferSize(module->transmitDMA.hw, module->transmitDMA.ch, 
                                module->state->txTarCnt);

        DL_I2C_enableInterrupt(module->hw, \
                                DL_I2C_INTERRUPT_TARGET_EVENT1_DMA_DONE);
        DL_I2C_enableDMAEvent(module->hw, DL_I2C_EVENT_ROUTE_1, 
                                DL_I2C_DMA_INTERRUPT_TARGET_TXFIFO_TRIGGER);
        /* Do not enable channel until start condition detected */
    }
    /* Interrupt Driven Transfer */
    else
    {

    }

    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_I2C_SlaveReceive(DRIVER_I2C_MSP *module, uint8_t *data, 
                                    uint32_t num)
{
    if((data == NULL) || (num == 0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }
    else
    {
        module->state->i2cState = I2C_STATE_RX_STARTED;
    }

    // The I2C module has not yet been configured to act in target mode 
    if(module->state->targetEnabled != 1)
    {
        MSP_ARM_I2C_ConfigureTarget(module);
    }
    else
    {
        // Already in target mode 
    }

    module->state->txTarCnt = 0;
    module->state->txCnt = 0;   
    module->state->rxBuf = data;
    module->state->rxTarCnt = num;
    module->state->rxCnt = 0;
    DL_I2C_clearInterruptStatus(module->hw, DL_I2C_INTERRUPT_TARGET_START | 
                                            DL_I2C_INTERRUPT_TARGET_STOP);

    /* DMA-Driven Transfer */
    if(module->state->receiveDMAAvail == true)
    {
        DL_DMA_setSrcAddr(module->receiveDMA.hw, module->receiveDMA.ch, 
                            (uint32_t)(&module->hw->SLAVE.SRXDATA));
        DL_DMA_setDestAddr(module->receiveDMA.hw, module->receiveDMA.ch, 
                            (uint32_t)(module->state->rxBuf));
        DL_DMA_setTransferSize(module->receiveDMA.hw, module->receiveDMA.ch, 
                                module->state->rxTarCnt);
        
        DL_I2C_enableInterrupt(module->hw, \
                                DL_I2C_INTERRUPT_TARGET_EVENT2_DMA_DONE);
        DL_I2C_enableDMAEvent(module->hw, DL_I2C_EVENT_ROUTE_2, 
                                DL_I2C_DMA_INTERRUPT_TARGET_RXFIFO_TRIGGER);
        DL_DMA_enableChannel(module->receiveDMA.hw, module->receiveDMA.ch);
    }
    /* Interrupt-Driver Transfer */
    else
    {
        DL_I2C_clearInterruptStatus(module->hw, \
                                        DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER);
        DL_I2C_enableInterrupt(module->hw, \
                                        DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER);
    }

    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_I2C_GetDataCount(DRIVER_I2C_MSP *module)
{
    int32_t count = 0;
    if(module->state->controllerEnabled)
    {
        if(module->state->txTarCnt != 0)
        {
            if(module->state->transmitDMAAvail == true)
            {
                count = module->state->txTarCnt - 
                        DL_DMA_getTransferSize(module->transmitDMA.hw, \
                                                module->transmitDMA.ch); 
            }
            else
            {
                count = module->state->txCnt;
            }
        }
        else if(module->state->rxTarCnt != 0)
        {
            if(module->state->receiveDMAAvail == true)
            {
                count = module->state->txTarCnt - 
                        DL_DMA_getTransferSize(module->receiveDMA.hw, \
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
    else if(module->state->targetEnabled)
    {
        if(module->state->txTarCnt != 0)
        {
            if(module->state->transmitDMAAvail == true)
            {
                count = module->state->txTarCnt - 
                        DL_DMA_getTransferSize(module->transmitDMA.hw, \
                                                module->transmitDMA.ch); 
            }
            else
            {
                count = module->state->txCnt;
            }
        }
        else if(module->state->rxTarCnt != 0)
        {
            if(module->state->receiveDMAAvail == true)
            {
                count = module->state->rxTarCnt - 
                        DL_DMA_getTransferSize(module->receiveDMA.hw, \
                                                module->receiveDMA.ch); 
            }
            else
            {
                count = module->state->rxCnt;
            }
        }
        else
        {
            // If the slave has not yet been addressed by the Master return -1
            count = -1;
        }
    }
    else
    {
        count = 0;
    }
    return count;
}

ARM_I2C_STATUS MSP_ARM_I2C_GetStatus(DRIVER_I2C_MSP *module)
{
    return module->state->status;
}

void MSP_ARM_I2C_IRQHandler(DRIVER_I2C_MSP *module)
{
    switch(DL_I2C_getPendingInterrupt(module->hw))
    {
        case DL_I2C_IIDX_CONTROLLER_TX_DONE:
            DL_I2C_disableInterrupt(module->hw, \
                                    DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            module->state->i2cState = I2C_STATE_IDLE;
            module->state->status.busy = 0;
            if(module->state->transmitDMAAvail == true)
            {
                if(DL_DMA_getTransferSize(module->transmitDMA.hw, \
                                            module->transmitDMA.ch) == 0)
                {
                    if(module->state->callback) 
                    {
                        module->state->callback(ARM_I2C_EVENT_TRANSFER_DONE);
                    }
                }
                else
                {
                    if(module->state->callback) 
                    {
                        module->state->callback(
                                            ARM_I2C_EVENT_TRANSFER_DONE |
                                            ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
                    }
                }
            }   
            else
            {
                if(module->state->txCnt < module->state->txTarCnt)
                {
                    if(module->state->callback) 
                    {
                        module->state->callback(
                                            ARM_I2C_EVENT_TRANSFER_DONE | 
                                            ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
                    }
                }
                else
                {                 
                    if(module->state->callback) 
                    {
                        module->state->callback(ARM_I2C_EVENT_TRANSFER_DONE);
                    }
                }
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
            if(module->state->txCnt < module->state->txTarCnt)
            {
                module->state->txCnt += DL_I2C_fillControllerTXFIFO(
                                module->hw, 
                                &module->state->txBuf[module->state->txCnt],
                                module->state->txTarCnt - module->state->txCnt);
            }
            else
            {

            }
            break;
        case DL_I2C_IIDX_CONTROLLER_EVENT1_DMA_DONE:
            DL_DMA_disableChannel(module->transmitDMA.hw, \
                                    module->transmitDMA.ch);
            DL_I2C_disableDMAEvent(module->hw, DL_I2C_EVENT_ROUTE_1, \
                                DL_I2C_DMA_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
            DL_I2C_disableInterrupt(module->hw, \
                                DL_I2C_INTERRUPT_CONTROLLER_EVENT1_DMA_DONE);
            module->state->i2cState = I2C_STATE_IDLE;
            break;
        case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
            while(DL_I2C_isControllerRXFIFOEmpty(module->hw) != true)
            {
                if(module->state->rxCnt < module->state->rxTarCnt)
                {
                    uint32_t data = DL_I2C_receiveControllerData(module->hw);
                    module->state->rxBuf[(module->state->rxCnt)++] = data;
                }
                else
                {
                    DL_I2C_receiveControllerData(module->hw);
                }
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_RX_DONE:
            /* DL_I2C_INTERRUPT_CONTROLLER_RX_DONE has higher than priority 
             * than DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER. Because of 
             * this, the RX_DONE case can execute first while there is 
             * still pending data in the RXFIFO.
             */
            if(DL_I2C_getEnabledInterruptStatus(module->hw, \
               DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER))
            {
                while(DL_I2C_isControllerRXFIFOEmpty(module->hw) != true)
                {
                    if(module->state->rxCnt < module->state->rxTarCnt)
                    {
                        module->state->rxBuf[(module->state->rxCnt)++] =\
                        DL_I2C_receiveControllerData(module->hw);
                    }
                    else
                    {
                        DL_I2C_receiveControllerData(module->hw);
                    }
                }
            }
            module->state->i2cState = I2C_STATE_IDLE;
            module->state->status.busy = 0;
            if(module->state->receiveDMAAvail == true)
            {
                if(DL_DMA_getTransferSize(module->receiveDMA.hw, \
                   module->receiveDMA.ch) == 0)
                {
                    if(module->state->callback) 
                    {
                        module->state->callback(ARM_I2C_EVENT_TRANSFER_DONE);
                    }
                }
                else
                {
                    if(module->state->callback) 
                    {
                        module->state->callback(
                                            ARM_I2C_EVENT_TRANSFER_DONE |
                                            ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
                    }
                }   
            }
            else
            {
                if(module->state->rxCnt < module->state->rxTarCnt)
                {
                    if(module->state->callback)
                    { 
                        module->state->callback(
                                            ARM_I2C_EVENT_TRANSFER_DONE | 
                                            ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
                    }
                }
                else
                {
                    if(module->state->callback) 
                    {
                        module->state->callback(ARM_I2C_EVENT_TRANSFER_DONE);
                    }
                }
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_EVENT2_DMA_DONE:
            DL_I2C_disableDMAEvent(module->hw, DL_I2C_EVENT_ROUTE_2, \
                                DL_I2C_DMA_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER);
            DL_DMA_disableChannel(module->receiveDMA.hw, module->receiveDMA.ch);
            module->state->i2cState = I2C_STATE_IDLE;
            break;
        case DL_I2C_IIDX_CONTROLLER_NACK:
            module->state->i2cState = I2C_STATE_IDLE;
            module->state->status.busy = 0;
            if(module->state->callback) 
            {
                module->state->callback(ARM_I2C_EVENT_ADDRESS_NACK);
            }
            break;
        case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
            module->state->status.arbitration_lost = 1;
            if(module->state->callback) 
            {
                module->state->callback(ARM_I2C_EVENT_ARBITRATION_LOST);
            }
            break;
        case DL_I2C_IIDX_TARGET_START:
            DL_I2C_flushTargetTXFIFO(module->hw);
            DL_I2C_flushTargetRXFIFO(module->hw);
            /* Slave has not been addressed but start condition detected */
            if(module->state->i2cState == I2C_STATE_IDLE)
            {
                if(DL_I2C_getEnabledInterruptStatus(module->hw, \
                   DL_I2C_INTERRUPT_TARGET_GENERAL_CALL))
                {
                    module->state->status.general_call = 1;
                    module->state->status.busy = 1;
                    module->state->status.direction = 0;
                    module->state->i2cState = I2C_STATE_RX_STARTED;
                }
                else
                {
                    uint32_t status = DL_I2C_getTargetStatus(module->hw);
                    if(status & DL_I2C_TARGET_STATUS_TX_MODE)
                    {
                        if(module->state->callback) 
                        {
                            module->state->callback(
                                                ARM_I2C_EVENT_SLAVE_TRANSMIT);
                        }
                    }
                    else
                    {
                        if(module->state->callback)
                        { 
                            module->state->callback(
                                                ARM_I2C_EVENT_SLAVE_RECEIVE);
                        }
                    }
                }
            }
            else if(module->state->i2cState == I2C_STATE_TX_STARTED)
            {
                module->state->status.busy = 1;
                module->state->status.direction = 0;
                if(module->state->transmitDMAAvail)
                {
                    DL_DMA_enableChannel(module->transmitDMA.hw, \
                                         module->transmitDMA.ch);
                }
                else
                {
                    module->state->txCnt += \
                            DL_I2C_fillTargetTXFIFO(module->hw, 
                                &module->state->txBuf[module->state->txCnt], 
                                module->state->txTarCnt - module->state->txCnt);
                    if(module->state->txCnt < module->state->txTarCnt)
                    {
                        DL_I2C_enableInterrupt(module->hw, \
                            DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER);
                    }
                    else
                    {
                        module->state->i2cState = I2C_STATE_IDLE;
                        if(module->state->callback) 
                        {
                            module->state->callback(
                                                ARM_I2C_EVENT_TRANSFER_DONE);
                        }
                    }
                }
            }
            else if(module->state->i2cState == I2C_STATE_RX_STARTED)
            {
                module->state->status.busy = 1;
                module->state->status.direction = 1;
                //For read requests, RXFIFO_TRIGGER will handle all logic 
            }
            break;
        case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
            if(module->state->txCnt < module->state->txTarCnt)
            {
                module->state->txCnt +=\
                    DL_I2C_fillTargetTXFIFO(module->hw, 
                                &module->state->txBuf[module->state->txCnt], 
                                module->state->txTarCnt - module->state->txCnt);
                if(module->state->txCnt >= module->state->txTarCnt)
                {
                    module->state->i2cState = I2C_STATE_IDLE;
                    if(module->state->callback) 
                    {
                        module->state->callback(ARM_I2C_EVENT_TRANSFER_DONE);
                    }
                }
            }
            else
            {
                while (DL_I2C_transmitTargetDataCheck(module->hw, 0x00));
            }
            break;
        case DL_I2C_IIDX_TARGET_EVENT1_DMA_DONE:
            DL_I2C_disableDMAEvent(module->hw, DL_I2C_EVENT_ROUTE_1, \
                                   DL_I2C_DMA_INTERRUPT_TARGET_TXFIFO_TRIGGER);
            DL_DMA_disableChannel(module->transmitDMA.hw, \
                                  module->transmitDMA.ch);
            module->state->i2cState = I2C_STATE_IDLE;
            module->state->status.busy = 0;
            if(module->state->callback) 
            {
                module->state->callback(ARM_I2C_EVENT_TRANSFER_DONE);
            }
            break;
        case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
            while(DL_I2C_isTargetRXFIFOEmpty(module->hw) != true)
            {
                if(module->state->rxCnt < module->state->rxTarCnt)
                {
                    module->state->rxBuf[(module->state->rxCnt)] =\
                        DL_I2C_receiveTargetData(module->hw);
                    module->state->rxCnt += 1;
                    if(module->state->rxCnt == module->state->rxTarCnt)
                    {
                        module->state->i2cState = I2C_STATE_IDLE;
                        if(module->state->callback) 
                        {
                            module->state->callback(
                                                ARM_I2C_EVENT_TRANSFER_DONE);
                        }
                    }
                }   
                else
                {
                    DL_I2C_receiveTargetData(module->hw);
                }
            }
            break;
        case DL_I2C_IIDX_TARGET_EVENT2_DMA_DONE:
            DL_I2C_disableDMAEvent(module->hw, DL_I2C_EVENT_ROUTE_2, \
                                   DL_I2C_DMA_INTERRUPT_TARGET_RXFIFO_TRIGGER);
            DL_DMA_disableChannel(module->receiveDMA.hw, module->receiveDMA.ch);
            module->state->i2cState = I2C_STATE_IDLE;
            module->state->status.busy = 0;
            if(module->state->callback) 
            {
                module->state->callback(ARM_I2C_EVENT_TRANSFER_DONE);
            }
            break;
        case DL_I2C_IIDX_TARGET_STOP:
            if(module->state->i2cState == I2C_STATE_RX_STARTED)
            {
                if(module->state->status.general_call == 1)
                {
                    module->state->status.general_call = 0;
                    if(module->state->callback) 
                    {
                        module->state->callback(
                                            ARM_I2C_EVENT_TRANSFER_DONE | 
                                            ARM_I2C_EVENT_GENERAL_CALL  | 
                                            ARM_I2C_EVENT_SLAVE_RECEIVE);
                    }
                }
                else
                {
                    if(module->state->receiveDMAAvail == true)
                    {
                        if(DL_DMA_getTransferSize(module->receiveDMA.hw, \
                                                    module->receiveDMA.ch) == 0)
                        {
                            if(module->state->callback) 
                            {
                                module->state->callback(
                                                ARM_I2C_EVENT_TRANSFER_DONE);
                            }
                        }
                        else
                        {
                            if(module->state->callback) 
                            {
                                module->state->callback(
                                            ARM_I2C_EVENT_TRANSFER_DONE | 
                                            ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
                            }
                        }   
                    }
                    else
                    {
                        if(module->state->rxCnt < module->state->rxTarCnt)
                        {
                            if(module->state->callback) 
                            {
                                module->state->callback(
                                            ARM_I2C_EVENT_TRANSFER_DONE | 
                                            ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
                            }
                        }
                        else
                        {
                            if(module->state->callback)
                            {
                                module->state->callback(
                                                ARM_I2C_EVENT_TRANSFER_DONE);
                            } 
                        }
                    }
                }
            }
            else if(module->state->i2cState == I2C_STATE_TX_STARTED)
            {
                DL_I2C_disableInterrupt(module->hw, \
                                        DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER);
                if(module->state->transmitDMAAvail == true)
                {
                    if(DL_DMA_getTransferSize(module->transmitDMA.hw, \
                                              module->transmitDMA.ch) == 0)
                    {
                        if(module->state->callback)
                        {
                            module->state->callback(
                                            ARM_I2C_EVENT_TRANSFER_DONE);
                        }
                    }
                    else
                    {
                        if(module->state->callback)
                        {
                            module->state->callback(
                                            ARM_I2C_EVENT_TRANSFER_DONE | 
                                            ARM_I2C_EVENT_TRANSFER_INCOMPLETE);
                        } 
                    }   
                }
                else
                {
                    if(module->state->txCnt < module->state->txTarCnt)
                    {
                        if(module->state->callback) 
                        {
                            module->state->callback(
                                            ARM_I2C_EVENT_TRANSFER_DONE | 
                                            ARM_I2C_EVENT_TRANSFER_INCOMPLETE);

                        } 
                    }
                    else
                    {
                        if(module->state->callback) 
                        {
                            module->state->callback(
                                            ARM_I2C_EVENT_TRANSFER_DONE);
                        }

                    }
                }          
            }
            else
            {

            }
            module->state->i2cState = I2C_STATE_IDLE;
            break;
        case DL_I2C_IIDX_TARGET_ARBITRATION_LOST:
            module->state->status.arbitration_lost = 1;
            if(module->state->callback)
            {
                module->state->callback(ARM_I2C_EVENT_ARBITRATION_LOST);
            } 
            break;
        default:
            break;
    }
}
