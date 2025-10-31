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
 *  @file       Driver_GPIO_MSP_Priv.c
 *  
 *              This implementation supports HW version V0.
 *
 ******************************************************************************
 */

#include <ti/driverlib/driverlib.h>
#include <Driver_GPIO_MSP_Priv.h>

int32_t MSP_ARM_GPIO_Setup(DRIVER_GPIO_MSP *module, ARM_GPIO_Pin_t pin,
    ARM_GPIO_SignalEvent_t cb_event)
{
    if(pin > MAX_PIN_NUM)
    {
        return ARM_GPIO_ERROR_PIN;
    }
    else
    {

    }

    if(module->state->enabled == 0)
    {
        DL_GPIO_enablePower(module->hw);
        NVIC_EnableIRQ(module->irq);
        module->state->enabled = 1;
    }
    else
    {

    }

    module->state->callbacks[pin] = cb_event;
    /* Clear any existing PINCM configuration */
    IOMUX->SECCFG.PINCM[module->pinCMs[pin]] = 0;
    DL_GPIO_initDigitalInput(module->pinCMs[pin]);

    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_GPIO_SetDirection(DRIVER_GPIO_MSP *module, ARM_GPIO_Pin_t pin,
    ARM_GPIO_DIRECTION direction)
{
    if(pin > MAX_PIN_NUM)
    {
        return ARM_GPIO_ERROR_PIN;
    }
    else
    {

    }

    if(direction == ARM_GPIO_OUTPUT)
    {
        DL_GPIO_initDigitalOutput(module->pinCMs[pin]);
        DL_GPIO_enableOutput(module->hw, (1 << pin));
    }
    else
    {
        DL_GPIO_disableOutput(module->hw, (1 << pin));
        DL_GPIO_initDigitalInput(module->pinCMs[pin]);
    }

    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_GPIO_SetOutputMode(DRIVER_GPIO_MSP *module, ARM_GPIO_Pin_t pin,
    ARM_GPIO_OUTPUT_MODE mode)
{
    if(pin > MAX_PIN_NUM)
    {
        return ARM_GPIO_ERROR_PIN;
    }
    else
    {

    }

    if(mode == ARM_GPIO_OPEN_DRAIN)
    {
        DL_GPIO_enableHiZ(module->pinCMs[pin]);
    }
    else
    {
        DL_GPIO_disableHiZ(module->pinCMs[pin]);
    }

    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_GPIO_SetPullResistor(DRIVER_GPIO_MSP *module, ARM_GPIO_Pin_t pin,
    ARM_GPIO_PULL_RESISTOR resistor)
{
    DL_GPIO_RESISTOR mspResistorVal;
    
    if(pin > MAX_PIN_NUM)
    {
        return ARM_GPIO_ERROR_PIN;
    }
    else
    {

    }

    if(resistor == ARM_GPIO_PULL_UP)
    {
        mspResistorVal = DL_GPIO_RESISTOR_PULL_UP;
    }
    else if(resistor == ARM_GPIO_PULL_DOWN)
    {
        mspResistorVal = DL_GPIO_RESISTOR_PULL_DOWN;
    }
    else
    {
        mspResistorVal = DL_GPIO_RESISTOR_NONE;
    }
	/* Clear current resistor configuration if present */
	IOMUX->SECCFG.PINCM[module->pinCMs[pin]] &= ~(3 << IOMUX_PINCM_PIPD_OFS);
    IOMUX->SECCFG.PINCM[module->pinCMs[pin]] |= mspResistorVal;

    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_GPIO_SetEventTrigger(DRIVER_GPIO_MSP *module, ARM_GPIO_Pin_t pin,
    ARM_GPIO_EVENT_TRIGGER trigger)
{
    uint8_t polarity;
    uint32_t regOffset;
    uint32_t *regAddr;

    if(pin > MAX_PIN_NUM)
    {
        return ARM_GPIO_ERROR_PIN;
    }
    else
    {

    }

    if(trigger ==  ARM_GPIO_TRIGGER_RISING_EDGE)
    {
        polarity = 1;
    }
    else if(trigger == ARM_GPIO_TRIGGER_FALLING_EDGE)
    {
        polarity = 2;
    }
    else if(trigger == ARM_GPIO_TRIGGER_EITHER_EDGE)
    {
        polarity = 3;
    }
    else
    {
        polarity = 0;
    }

	/* Edge polarity register is split between lower and upper halves */
    if(pin <= 15)
    {
        module->hw->POLARITY15_0 &= ~(3 << (pin * 2));
        module->hw->POLARITY15_0 |= polarity << (pin * 2);
    }
    else
    {
        module->hw->POLARITY31_16 &= ~(3 << ((pin % 16) * 2));
        module->hw->POLARITY31_16 |= polarity << ((pin % 16) * 2);
    }

    /* Enable interrupts for that particular pin */
    DL_GPIO_enableInterrupt(module->hw, (1 << pin));

    return ARM_DRIVER_OK;
}

int32_t MSP_ARM_GPIO_SetOutput(DRIVER_GPIO_MSP *module, ARM_GPIO_Pin_t pin,
    uint32_t val)
{
    if(pin > MAX_PIN_NUM)
    {
        return ARM_GPIO_ERROR_PIN;
    }
    else
    {

    }

    if(val == 0)
    {
        DL_GPIO_clearPins(module->hw, (1 << pin));
    }
    else
    {
        DL_GPIO_setPins(module->hw, (1 << pin));
    }

    return ARM_DRIVER_OK;
}

uint32_t MSP_ARM_GPIO_GetInput(DRIVER_GPIO_MSP *module, uint32_t pin)
{
    if(pin > MAX_PIN_NUM)
    {
        return ARM_GPIO_ERROR_PIN;
    }
    else
    {

    }

    return DL_GPIO_readPins(module->hw, (1 << pin)) > 0 ? 1 : 0;
}

void MSP_ARM_GPIO_IRQHandler(DRIVER_GPIO_MSP *module)
{
    uint32_t triggerSrc;
    uint32_t pin = DL_GPIO_getPendingInterrupt(module->hw) - 1;
    if(module->state->callbacks[pin])
    {
        if(pin > 15)
        {
            triggerSrc = module->hw->POLARITY31_16 & (3 << ((pin % 16) * 2));
			triggerSrc = triggerSrc >> ((pin % 16) * 2);
		}
        else
        {
            triggerSrc = module->hw->POLARITY15_0 & (3 << ((pin) * 2));
			triggerSrc = triggerSrc >> (pin * 2);
        }

        if(triggerSrc == 1)
        {
            module->state->callbacks[pin](pin, ARM_GPIO_EVENT_RISING_EDGE);
        }
        else if(triggerSrc == 2)
        {
            module->state->callbacks[pin](pin, ARM_GPIO_EVENT_FALLING_EDGE);
        }
        else
        {
            module->state->callbacks[pin](pin, ARM_GPIO_EVENT_EITHER_EDGE);
        }
    }
    else
    {

    }
}
