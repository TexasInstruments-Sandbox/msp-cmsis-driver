## Example Summary

This is an example project for the CMSIS-Driver GPIO module.
The example uses the GPIOB peripheral to demonstrate simple input and output control via the CMSIS APIs.
A switch button on PB3 is configured with an internal pull-up resistor and event trigger for a falling edge.
On each button press, the RBG LED will switch colors between red, green, and blue.

## Peripherals, Pin Functions, MCU Pins, Launchpad Pins
| Peripheral | Function | Pin | Launchpad Pin |
| --- | --- | --- | --- |
| BOARD | Debug Clock | PA20 | J19_16 |
| BOARD | Debug Data In Out | PA19 | J19_14 |
| GPIOB | USER_SWITCH | PB21 |
| GPIOB | RBG_RED_LED | PB26 | 
| GPIOB | RBG_GREEN_LED | PB27 |
| GPIOB | RBG_BLUE_LED | PB22 | 
