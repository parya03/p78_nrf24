# p78_nrf24
An single-file nrf24l01 library built on top of libopencm3 for STM32F4.

## How to use:
Assuming using the libopencm3-template:
1. Clone into my-common-code
2. Include header file in project code
3. Add ```p78_nrf/nrf24l01.c``` in my-project/Makefile under ```CFILES```
The steps should be similar for different project directory configurations.

TODO: Documentation, examples, port to other devices (?)

## Quick Start:
Pin and radio configuration is done by passing pointers to structs containing the configuration values into init_nrf().
The structs are defined in the header file.

Note: Make sure to set up SPI configuration for the SPI peripheral and pins prior to calling init_nrf24().

After the radio is set up, memory (registers, etc) can be read and written by read_mem() and write_mem() respectively. Payloads can be read with read_payload().

## Compatibility with RF24 library
It is possible to use this library in conjunction with devices running RF24 (ex. Arduino running RF24 transmitting and STM32 running p78_nrf recieving), but a few things need to be set up properly.
RF24 seems to use channel 0x4C to communicate, so make sure to use that channel (or the same channel as the other device is using, if another). Auto acknowledge from RF24's side (using this as TX, RF24 as RX) doesn't seem to work yet (I will have to look into that), so make sure to turn that off on both sides if TX'ing using this library. Auto ack does work if using p78_nrf to RX, though.
