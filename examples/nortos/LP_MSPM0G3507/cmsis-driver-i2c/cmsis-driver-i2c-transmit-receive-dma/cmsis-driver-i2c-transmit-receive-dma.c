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
#include "Driver_I2C_MSP.h"

/**
 * @brief Transaction length size in bytes
 */
#define TRANSACTION_LEN_SZ (4U)

/**
 * @brief Master transmit complete flag
 */
static volatile bool mTransmit;

/**
 * @brief Master transaction complete flag
 */
static volatile bool mDone;

/**
 * @brief Slave transaction complete flag
 */
static volatile bool sDone;

/**
 * @brief Master receive buffer
 */
static uint8_t mBuf[TRANSACTION_LEN_SZ];

/**
 * @brief Master transmit buffer
 */
static uint8_t sBuf[TRANSACTION_LEN_SZ];

/**
 * @brief Callback handler for master I2C instance
 */
static void i2c0_cb(uint32_t event);

/**
 * @brief Callback handler for slave I2C instance
 */
static void i2c1_cb(uint32_t event);

/* This results in approximately 1s of delay assuming 32MHz CPU_CLK */
#define DELAY (32000000)

#define TARGET_ADDRESS (0x11)

int main(void)
{
    ARM_DRIVER_I2C *i2c0 = &Driver_I2C0;
    ARM_DRIVER_I2C *i2c1 = &Driver_I2C1;

    /* Execute SysConfig generated initialization routine */
    SYSCFG_DL_init();
    DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN | GPIO_LEDS_USER_LED_2_PIN);

    /* Configure i2c0 */
    i2c0->Initialize(&i2c0_cb);
    i2c0->PowerControl(ARM_POWER_FULL);
    i2c0->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    
    /* Configure i2c1 */
    i2c1->Initialize(&i2c1_cb);
    i2c1->PowerControl(ARM_POWER_FULL);
    i2c1->Control(ARM_I2C_OWN_ADDRESS, TARGET_ADDRESS);
    i2c1->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);

    /* Fill buffer with data */
    for(uint32_t i = 0U; i < TRANSACTION_LEN_SZ; i++)
    {
        mBuf[i] = i + 1;
    }

    /* Start the first transaction with a master transmit */
    mTransmit = true;

    /* Loop and send data in both directions continously */
    while (1) 
    {
        mDone = false;
        sDone = false;

        if(mTransmit)
        {
            i2c1->SlaveReceive(&sBuf[0], TRANSACTION_LEN_SZ);
            i2c0->MasterTransmit(TARGET_ADDRESS, &mBuf[0], TRANSACTION_LEN_SZ, false);
        }
        else
        {
            i2c1->SlaveTransmit(&sBuf[0], TRANSACTION_LEN_SZ);
            i2c0->MasterReceive(TARGET_ADDRESS, &mBuf[0], TRANSACTION_LEN_SZ, false);
        }
        
        while(mDone == false && sDone == false);

        if(mTransmit) 
        {
            DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
            DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_2_PIN);
        }
        else
        {
            DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_2_PIN);
            DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
        }
        /* Alternate which side is transmitting */
        mTransmit = !mTransmit;
        delay_cycles(DELAY);
    }
}

static void i2c0_cb(uint32_t event)
{

    if (event & ARM_I2C_EVENT_TRANSFER_DONE)
    {
        mDone = true;
    }
    else
    {

    }
    return;
}

static void i2c1_cb(uint32_t event)
{
    if (event & ARM_I2C_EVENT_TRANSFER_DONE)
    {
        sDone = true;
    }
    else
    {

    }
    return;
}