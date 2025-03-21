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
#include "Driver_USART_MSP.h"

/**
 * @brief RX buffer size in bytes
 */
#define RX_BUF_SZ (4U)

/**
 * @brief TX buffer size in bytes
 */
#define TX_BUF_SZ (4U)

/**
 * @brief Send complete flag
 */
static volatile bool sendDone;

/**
 * @brief Receive complete flag
 */
static volatile bool receiveDone;

/**
 * @brief Receive buffer
 */
static uint8_t rBuf[RX_BUF_SZ];

/**
 * @brief Send buffer
 */
static uint8_t tBuf[TX_BUF_SZ];

/**
 * @brief Welcome text string
 */
static const uint8_t welcomeStr[] = "\n\rWelcome to MSP-CMSIS-DRIVER USART!\n\r";

/**
 * @brief Callback handler for serial port
 */
static void usart0_cb(uint32_t event);

int main(void)
{
    ARM_DRIVER_USART *backchannelPort = &Driver_USART0;

    /* Execute SysConfig generated initialization routine */
    SYSCFG_DL_init();

    /* Configure uart0 */
    backchannelPort->Initialize(&usart0_cb);
    backchannelPort->PowerControl(ARM_POWER_FULL);
    backchannelPort->Control(ARM_USART_MODE_ASYNCHRONOUS, 115200U);
    backchannelPort->Control(ARM_USART_CONTROL_TX, 1U);
    backchannelPort->Control(ARM_USART_CONTROL_RX, 1U);

    /* Send welcome string */
    backchannelPort->Send(&welcomeStr[0], sizeof(welcomeStr));

    /* Loop with echo */
    while (1) 
    {
        /* Wait for character */
        receiveDone = false;
        backchannelPort->Receive(&rBuf[0], 1U);
        while (receiveDone != true);
        
        /* Echo character back */
        while(sendDone != true);
        sendDone = false;
        tBuf[0] = rBuf[0];
        backchannelPort->Send(&tBuf[0], 1U);
    }
}

static void usart0_cb(uint32_t event)
{
    if (event & ARM_USART_EVENT_RECEIVE_COMPLETE)
    {
        receiveDone = true;
    }
    if (event & ARM_USART_EVENT_SEND_COMPLETE)
    {
        sendDone = true;
    }
    return;
}
