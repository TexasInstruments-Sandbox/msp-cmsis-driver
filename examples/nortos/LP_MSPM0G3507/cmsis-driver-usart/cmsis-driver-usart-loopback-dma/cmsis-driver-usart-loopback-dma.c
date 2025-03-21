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

/*
 * cmsis-driver-usart-loopback-dma.c
 *
 * This code example demonstrates two CMSIS-Driver USART channels:
 *     Driver_USART0 (used as a back channel UART to XDS-110 probe)
 *         - Set up for 115200B 8N1
 *         - Sends diagnostic strings to host computer via XDS-110
 *     Driver_USART1 (used as demonstration UART for RX/TX)
 *         - Set up for 115200B 8N1
 *         - Pins PB7 and PB6 need to be shorted (TX->RX) to create loopback
 *         - This USART driver uses DMA channels 0 and 1 for TX and RX
 *
 * The example sends 64 bytes of data from Driver_USART1 TX to Driver_USART1 RX
 * using the CMSIS-Driver Send/Receive functions with DMA handling data
 * transfer.  Checks are done on the driver status and diagnostic information
 * about the transfer status is sent to the host computer via Driver_USART0
 * transmissions to the on-board XDS-110 debug probe back channel UART.
 *
 */

#include "ti_msp_dl_config.h"
#include "Driver_USART_MSP.h"

/**
 * @brief Driver_USART1 TX/RX buffer size in bytes
 */
#define BUF_SZ (64U)

/**
 * @brief Driver_USART1 receive buffer
 */
static uint8_t rBuf[BUF_SZ];

/**
 * @brief Driver_USART1 transmit buffer
 */
static uint8_t tBuf[BUF_SZ];

int main(void)
{
    ARM_DRIVER_USART *backchannelPort = &Driver_USART0;
    ARM_DRIVER_USART *loopbackPort = &Driver_USART1;
    int32_t retVal;
    uint32_t i;
    uint32_t badByte;

    /* Execute SysConfig generated initialization routine */
    SYSCFG_DL_init();

    /* Configure back channel UART (for communication to host via XDS-110) */
    backchannelPort->Initialize(NULL);
    backchannelPort->PowerControl(ARM_POWER_FULL);
    backchannelPort->Control(ARM_USART_MODE_ASYNCHRONOUS, 115200U);
    backchannelPort->Control(ARM_USART_CONTROL_TX, 1U);

    /* Configure loopback UART (needs to be externally connected)
     */
    loopbackPort->Initialize(NULL);
    loopbackPort->PowerControl(ARM_POWER_FULL);
    loopbackPort->Control(ARM_USART_MODE_ASYNCHRONOUS, 115200U);
    loopbackPort->Control(ARM_USART_CONTROL_TX, 1U);
    loopbackPort->Control(ARM_USART_CONTROL_RX, 1U);

    /* Send welcome message to back channel */
    backchannelPort->Send( \
        "\r\nStarting cmsis-driver-usart-loopback-dma!\r\n", \
        sizeof("\r\nStarting cmsis-driver-usart-loopback-dma!\r\n"));

    /* Fill buffers with example data for tx/rx */
    for (i=0; i<BUF_SZ; i++)
    {
        tBuf[i] = i + 1U; /* 1 to 64 integers to be sent */
        rBuf[i] = 0xFFU; /* 0xFF fill values to be overwritten during RX */
    }

    /* Configure loopback port reception operation */
    retVal = loopbackPort->Receive(&rBuf[0], BUF_SZ);
    while(backchannelPort->GetStatus().tx_busy != false);
    if (retVal == ARM_DRIVER_OK)
    {
        backchannelPort->Send("  -> USART1 receive pending...\r\n", \
            sizeof("  -> USART1 receive pending...\r\n"));
    }
    else 
    {
        backchannelPort->Send("  -> USART1 receive setup error...\r\n", \
            sizeof("  -> USART1 receive setup error...\r\n"));
    }

    /* Send data in tBuf via loopback port 
     * Report that send started or had an error.
     */
    retVal = loopbackPort->Send(&tBuf[0], BUF_SZ);
    while(backchannelPort->GetStatus().tx_busy != false);
    if (retVal == ARM_DRIVER_OK)
    {
        backchannelPort->Send("  -> USART1 send started...\r\n", \
            sizeof("  -> USART1 send started...\r\n"));
    }
    else 
    {
        backchannelPort->Send("  -> USART1 send setup error...\r\n", \
            sizeof("  -> USART1 send setup error...\r\n"));
    }

    /* Wait for completion, exit after BUF_SZ received or 32M loop iterations
     */
    i = 1000000U;
    while (i--)
    {
        if (loopbackPort->GetRxCount() == BUF_SZ)
        {
            break;
        }
    }

    /* Check # of sent bytes and report on back channel UART 
     * Report if TX sent 64 bytes or not.
     */
    while(backchannelPort->GetStatus().tx_busy != false);
    if (loopbackPort->GetTxCount() == BUF_SZ)
    {
        backchannelPort->Send("  -> USART1 sent 64 bytes.\r\n", \
            sizeof("  -> USART1 sent 64 bytes.\r\n"));
    }
    else 
    {
        backchannelPort->Send("  -> USART1 did not send 64 bytes.\r\n", \
            sizeof("  -> USART1 did not send 64 bytes.\r\n"));
    }

    /* Check # of sent received and report on back channel UART
     * Report if RX recived 64 bytes or not.
     */
    while(backchannelPort->GetStatus().tx_busy != false);
    if (loopbackPort->GetRxCount() == BUF_SZ)
    {
        backchannelPort->Send("  -> USART1 received 64 bytes.\r\n", \
            sizeof("  -> USART1 received 64 bytes.\r\n"));
    }
    else 
    {
        backchannelPort->Send("  -> USART1 did not receive 64 bytes.\r\n", \
            sizeof("  -> USART1 did not receive 64 bytes.\r\n"));
    }

    /* Check send completion and report on back channel UART
     * Report if tx status is busy or not.
     */
    while(backchannelPort->GetStatus().tx_busy != false);
    if (loopbackPort->GetStatus().tx_busy == false)
    {
        backchannelPort->Send("  -> USART1 send completed.\r\n", \
            sizeof("  -> USART1 send completed.\r\n"));
    }
    else 
    {
        backchannelPort->Send("  -> USART1 send did not complete.\r\n", \
            sizeof("  -> USART1 send did not complete.\r\n"));
    }

    /* Check # of receive completion and report on back channel UART 
     * Report if rx status is busy or not.
     */
    while(backchannelPort->GetStatus().tx_busy != false);
    if (loopbackPort->GetStatus().rx_busy == false)
    {
        backchannelPort->Send("  -> USART1 reception completed.\r\n", \
            sizeof("  -> USART1 reception completed.\r\n"));
    }
    else 
    {
        backchannelPort->Send("  -> USART1 reception did not complete.\r\n", \
            sizeof("  -> USART1 reception did not complete.\r\n"));
    }

    /* Check each recevied byte for proper reception.
     * Report if any bad bytes were found.
     */
    badByte = 0U;
    for (i=0; i<BUF_SZ; i++)
    {
        if (tBuf[i] != rBuf[i])
        {
            badByte++;
        }
    }
    while(backchannelPort->GetStatus().tx_busy != false);
    if (badByte == 0)
    {
        backchannelPort->Send("  -> All received bytes match sent bytes.\r\n", \
            sizeof("  -> All received bytes match sent bytes.\r\n"));
    }
    else 
    {
        backchannelPort->Send("  -> Received bytes did not match sent bytes. \r\n", \
            sizeof("  -> Received bytes did not match sent bytes. \r\n"));
    }

    /* Send "done" message on back channel UART */
    while(backchannelPort->GetStatus().tx_busy != false);
    backchannelPort->Send("End of example!\r\n", \
        sizeof("End of example!\r\n"));

    return 0;
}
 