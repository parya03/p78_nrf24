/**
 *
    MIT License

    Copyright (c) 2023 Pranit Arya

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

    ---------------------------------------------------------------------------------------
 
 * This file contains function prototypes specific to the platform that the library is running on.
 * These prototypes are implemented in .c files that #include this file and use #ifdefs to check which platform is being used. 
 * 
 * Porting the library to a new platform only involves writing these functions.
 * 
 * These functions mostly consist of low-level control of peripherals, GPIO, etc, so will be different for each platform.
 * 
*/

#ifndef NRF24_PLATFORM_SPECIFIC_H
#define NRF24_PLATFORM_SPECIFIC_H

#include <stdint.h>

// --------------------------------------------------------------------
// Device agnostic functions help with compatibility between devices
// --------------------------------------------------------------------

// Optional, but can come in use
// Busy wait for a given number of milliseconds
void device_agnostic_sleep(uint32_t ms);

// Required
// Set a GPIO pin high or low
// uint32_t port: GPIO port (may not be necessary for all platforms)
// uint32_t pin: GPIO pin
// uint8_t val: 0 for low, 1 for high
void device_agnostic_gpio_put(uint32_t port, uint32_t pin, uint8_t val);

// --------------------------------------------------------------------

// Mandatory
// Wait for SPI peripheral to finish transmitting - should be blocking
void wait_spi_not_busy(void);

// Mandatory
// Send over SPI - should be blocking
// uint32_t spi_peripheral: SPI peripheral to use, use casting if using other libraries with peripheral typedefs
// uint8_t *tx_data: pointer to data to send
// uint8_t *rx_data: pointer to data to receive - used if want to recieve data at the same time as sending, can be NULL
// uint32_t len: length of data to send
void nrf24_spi_write(uint32_t spi_peripheral, uint8_t *tx_data, uint8_t *rx_data, uint32_t len);

// Mandatory
// Receive over SPI - should be blocking
// uint32_t spi_peripheral: SPI peripheral to use, use casting if using other libraries with peripheral typedefs
// uint8_t *data: pointer to data to receive
// uint32_t len: length of data to receive
void nrf24_spi_read(uint32_t spi_peripheral, uint8_t *rx_data, uint32_t len);

#endif