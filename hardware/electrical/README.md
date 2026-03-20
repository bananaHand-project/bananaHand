# Electrical

The electrical system for bananaHand is built around the main board. The main board is responsible for communication with the controlling device, motor control, and FSR sensing. A simplified block diagram descibing the main board can be seen below. It requires a single +12V 24W input source and exposes UART, CAN bus, and UART over RS485 options for interfacing. Depending on the device you are trying to interface with, a buck converter may be needed to drop your source voltage down to the required 12V.

![block diagram of describing function of main board](../../assets/images/simplified-elec-block-diagram.png)

## Main Board

The main board has the following sub-circuits:

- **Power stage**: Basic fuse protection, and a MPM3610 buck converter for 12V -> 3V3 for logic level ICs
- **Motor driving**: There are eight DRV8876 motor driver circuits for the [PQ12P](https://www.actuonix.com/pq12-30-12-p) 
- 
