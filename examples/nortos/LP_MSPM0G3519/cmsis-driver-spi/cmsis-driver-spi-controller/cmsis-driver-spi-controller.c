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
#include "Driver_SPI_MSP.h"

/**
 * @brief Transaction length size in bytes
 */
#define TRANSACTION_LEN_SZ (16U)

/**
 * @brief Transaction complete flag
 */
static volatile bool mDone;

/**
 * @brief Transaction complete flag
 */
static volatile bool sendRequested;

/**
 * @brief Transaction complete flag
 */
static volatile bool transferRequested;

/**
 * @brief  Send buffer
 */
static uint8_t sBuf[TRANSACTION_LEN_SZ];

/**
 * @brief Receive buffer
 */
static uint8_t rBuf[TRANSACTION_LEN_SZ];

/**
 * @brief Callback handler for master I2C instance
 */
static void spi0_cb(uint32_t event);

/* This results in approximately 0.5s of delay assuming 32MHz CPU_CLK */
#define DELAY (16000000)

int main(void)
{
    ARM_DRIVER_SPI *spi = &Driver_SPI0;
    uint32_t i;

    /* Execute SysConfig generated initialization routine */
    SYSCFG_DL_init();
    NVIC_EnableIRQ(GPIO_SWITCHES1_INT_IRQN);
    NVIC_EnableIRQ(GPIO_SWITCHES2_INT_IRQN);
    DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN | GPIO_LEDS_USER_LED_2_PIN);

    /* Configure i2c0 */
    spi->Initialize(&spi0_cb);
    spi->PowerControl(ARM_POWER_FULL);
    spi->Control(ARM_SPI_MODE_MASTER | ARM_SPI_SS_MASTER_HW_OUTPUT | ARM_SPI_DATA_BITS(8U), 0);
    spi->Control(ARM_SPI_SET_BUS_SPEED, 1000000);
    
    /* Fill buffer with data */
    for(uint32_t i = 0U; i < TRANSACTION_LEN_SZ; i++)
    {
        sBuf[i] = i + 1;
    }

    /* Loop and either send or transfer depending on button press */
    while (1) 
    {
        
        if(sendRequested)
        {   
            mDone = false;
            spi->Send(&sBuf[0], TRANSACTION_LEN_SZ);
            while(!mDone);
            for(i=0U; i < TRANSACTION_LEN_SZ; i++)
            {
                DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
                delay_cycles(DELAY);
            }
            sendRequested = false;
        }
        else if(transferRequested)
        {
            mDone = false;
            spi->Transfer(&sBuf[0], &rBuf[0], TRANSACTION_LEN_SZ);
            while(!mDone);
            for(i=0U; i < TRANSACTION_LEN_SZ; i++)
            {
                if(rBuf[i] == i*2)
                {
                    DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_2_PIN);
                    delay_cycles(DELAY);
                }
            }
            transferRequested = false;
        }
        else
        {
            __WFI();
        }
    }
}

static void spi0_cb(uint32_t event)
{

    if (event & ARM_SPI_EVENT_TRANSFER_COMPLETE)
    {
        mDone = true;
    }
}

void GROUP1_IRQHandler(void)
{
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case DL_INTERRUPT_GROUP1_IIDX_GPIOB:
            switch (DL_GPIO_getPendingInterrupt(GPIO_SWITCHES1_PORT)) {
                case GPIO_SWITCHES1_USER_SWITCH_1_IIDX:
                    sendRequested = true;
                    break;
                default:
                    break;
            }
            break;
        case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
            switch (DL_GPIO_getPendingInterrupt(GPIO_SWITCHES2_PORT)) {
                case GPIO_SWITCHES2_USER_SWITCH_2_IIDX:
                    transferRequested = true;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }
}
