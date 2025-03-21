## Example Summary

This is an example project for the CMSIS-Driver USART module.
The example uses the UART0 device peripheral for communication, which presents as Driver_USART0 in the CMSIS-Driver USART module.
The project uses SysConfig for initialization, but all functionality related to the CMSIS-Driver USART module is handled by the CMSIS-Driver itself.
The LP-MSPM0G3507 LaunchPad configuration must be set with J21 and J22 to connect PA10 and PA11 to the XDS-110 debug probe back channel UART.

## Peripherals, Pin Functions, MCU Pins, Launchpad Pins
| Peripheral | Function | Pin | Launchpad Pin |
| --- | --- | --- | --- |
| BOARD | Debug Clock | PA20 | J19_16 |
| BOARD | Debug Data In Out | PA19 | J19_14 |
| UART0 | UART0_TX | PA10 | J4_7 |
| UART0 | UART0_RX | PA11 | J4_8 |
