## Example Summary

This is an example project for the CMSIS-Driver USART module, using two 
driver instances.  The example uses the UART0 device peripheral for sending 
diagnostic messages to the host computer via the XDS-110 debug probe back
channel, which presents as Driver_USART0 in the CMSIS-Driver USART module.
The example also uses the UART1 device peripheral in an external loopback
configuration to demonstrate CMSIS-Driver USART Send() and Receive()
functionality.  UART1 presents as Driver_USART1 in the CMSIS-Driver USART
module.  Driver_USART1 also is configured to use DMA channels 0 and 1
for send and receive, respectively.
The project uses SysConfig for initialization, but all functionality related
to the CMSIS-Driver USART module is handled by the CMSIS-Driver itself,
including all configuration of USART related IOs, peripherals, and DMA.

PA10 needs to be connected to the XDS-110 debug probe back channel UART.  
For this example, PB6 and PB7 are expected to be shorted together externally (at the 
LaunchPad header) to enable an external loopback for UART1.

## Peripherals, Pin Functions, MCU Pins, Launchpad Pins
| Peripheral | Function | Pin | Launchpad Pin |
| --- | --- | --- | --- |
| BOARD | Debug Clock | PA20 | J19_16 |
| BOARD | Debug Data In Out | PA19 | J19_14 |
| UART0 | UART0_TX | PA10 | J4_7 |
| UART1 | UART0_TX | PB6  | J2_3 |
| UART1 | UART0_RX | PB7  | J2_7 |
