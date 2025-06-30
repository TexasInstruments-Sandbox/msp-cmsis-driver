## Example Summary

This is an example project for the CMSIS-Driver SPI module.
The example uses the SPI0 device peripherals for communication, which present as Driver_SPI0 in the CMSIS-Driver SPI module.
The project uses SysConfig for initialization, but all functionality related to the CMSIS-Driver SPI module is handled by the CMSIS-Driver itself.
This example is intended to be used in conjunction with cmsis-driver-spi-controller. Connect the MISO, MOSI, SS, and SCLK lines to each other.
All transfer are done via DMA.

This example uses on-board LEDs and buttons to demonstrate succesfully transmission between target and controller. Click S1 to request a send and
S2 to request a receieve. When a send is performed, LED1 will toggle equal to the number of bytes sent. When a receive is performed, LED2 will 
toggle equal to the number of data bytes correctly received from the controller.

## Peripherals, Pin Functions, MCU Pins, Launchpad Pins
| Peripheral | Function | Pin | Launchpad Pin |
| --- | --- | --- | --- |
| BOARD | Debug Clock | PA20 | J19_16 |
| BOARD | Debug Data In Out | PA19 | J19_14 |
| SPI0 | SPI0_MOSI | PB17 | 
| SPI0 | SPI0_MISO | PA10 |
| SPI0 | SPI0_SS | PA8 | 
| SPI0 | SPI0_SCLK | PA11 |
| GPIOB | USER_LED_1 | PB22 |
| GPIOB | USER_LED_2 | PB26 |
| GPIOB | USER_SWITCH_1 | PB3 |
| GPIOA | USER_SWITCH_2 | PA18 | 
