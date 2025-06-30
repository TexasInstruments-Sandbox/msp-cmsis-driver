/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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

#include "ti_msp_dl_config.h"
#include "Driver_GPIO_MSP.h"

/**
 * @brief Callback handler for GPIO button
 */
static void gpiob_cb(ARM_GPIO_Pin_t pin, uint32_t event);

volatile uint8_t count;

#define SWITCH_BUTTON_PIN (3)
#define RED_LED_PIN (26)
#define GREEN_LED_PIN (27)
#define BLUE_LED_PIN (22)

/* This results in approximately 1s of delay assuming 32MHz CPU_CLK */
#define DELAY (32000000)

int main(void)
{
    ARM_DRIVER_GPIO *gpioB = &Driver_GPIOB;

    /* Execute SysConfig generated initialization routine */
    SYSCFG_DL_init();

    /* Configure switch button on PB3 */
    gpioB->Setup(SWITCH_BUTTON_PIN, gpiob_cb);
    gpioB->SetPullResistor(SWITCH_BUTTON_PIN, ARM_GPIO_PULL_UP);
    gpioB->SetEventTrigger(SWITCH_BUTTON_PIN, ARM_GPIO_TRIGGER_FALLING_EDGE);
    
    /* Configure LEDs on PB22, PB26, PB27 */
    gpioB->Setup(RED_LED_PIN, gpiob_cb);
    gpioB->SetDirection(RED_LED_PIN, ARM_GPIO_OUTPUT);
    gpioB->SetOutputMode(RED_LED_PIN, ARM_GPIO_PUSH_PULL);

    gpioB->Setup(GREEN_LED_PIN, gpiob_cb);
    gpioB->SetDirection(GREEN_LED_PIN, ARM_GPIO_OUTPUT);
    gpioB->SetOutputMode(GREEN_LED_PIN, ARM_GPIO_PUSH_PULL);
    
    gpioB->Setup(BLUE_LED_PIN, gpiob_cb);
    gpioB->SetDirection(BLUE_LED_PIN, ARM_GPIO_OUTPUT);
    gpioB->SetOutputMode(BLUE_LED_PIN, ARM_GPIO_PUSH_PULL);

    count = 0;
    while (1) 
    {
        __WFI();
        if(count == 0)
        {
            gpioB->SetOutput(RED_LED_PIN, 1);
            gpioB->SetOutput(GREEN_LED_PIN, 0);
            gpioB->SetOutput(BLUE_LED_PIN, 0);
        }
        else if(count == 1)
        {
            gpioB->SetOutput(RED_LED_PIN, 0);
            gpioB->SetOutput(GREEN_LED_PIN, 1);
            gpioB->SetOutput(BLUE_LED_PIN, 0);
        }
        else
        {
            gpioB->SetOutput(RED_LED_PIN, 0);
            gpioB->SetOutput(GREEN_LED_PIN, 0);
            gpioB->SetOutput(BLUE_LED_PIN, 1);
        }
    }
}

static void gpiob_cb(ARM_GPIO_Pin_t pin, uint32_t event)
{
    if(pin == SWITCH_BUTTON_PIN && event == ARM_GPIO_EVENT_FALLING_EDGE)
    {
        count++;
        if(count == 3) count = 0;
    }   
}
