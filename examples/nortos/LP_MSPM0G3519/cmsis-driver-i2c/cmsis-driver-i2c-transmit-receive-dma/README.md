## Example Summary

This is an example project for the CMSIS-Driver I2C module.
The example uses the I2C0 and I2C1 device peripherals for communication, which present as Driver_I2C0 and Driver_I2C1 respectively in the CMSIS-Driver I2C module.
The project uses SysConfig for initialization, but all functionality related to the CMSIS-Driver I2C module is handled by the CMSIS-Driver itself. All transmissions
are done using the DMA. The LP-MSPM0G3519 LaunchPad configuration must be set with J21 and J22 to connect PA10 and PA11 to the boosterpack directly. Connect PA0 and PA10 with a jumper wire. Connected PA1 and P11 with a jumper wire. External resistors are needed and should be connected to PA0 and PA1, or PA10 and PA11. The on-board pull up resistors can be used by populating J16 and J17. 

This example uses on-board LEDs to demonstrate succesfully transmission between target and controller. When the controller transmits data to the target, the BLUE led 
is turned on. When the target transmit data to the controller, the RED led is turned on. 

## Peripherals, Pin Functions, MCU Pins, Launchpad Pins
| Peripheral | Function | Pin | Launchpad Pin |
| --- | --- | --- | --- |
| BOARD | Debug Clock | PA20 | J19_16 |
| BOARD | Debug Data In Out | PA19 | J19_14 |
| I2C0 | I2C0_SDA | PA0 | 
| I2C0 | I2C0_SCL | PA1 |
| I2C1 | I2C1_SCL | PA11 | 
| I2C1 | I2C1_SDA | PA10 |
| GPIOB | USER_LED_1 | PB22 |
| GPIOB | USER_LED_2 | PB26 | 
