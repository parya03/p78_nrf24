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
 
 * This file contains functions specific to the rpi-pico platform.
 * 
 * Porting the library to a new platform only involves writing these functions.
 * 
 * These functions mostly consist of low-level control of peripherals, GPIO, etc, so will be different for each platform.
 * 
*/

#ifdef STM32F429

#define STM32F4

#include <stdint.h>

#include "../nrf24l01.h"
#include "../platform_specific.h"

// Platform-specific includes
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

// Pointer to config structs for internal library use
extern const nrf24_pin_config_t *nrf24_pin_config;
extern const nrf24_config_t *nrf24_config;

// --------------------------------------------------------------------
// Device agnostic functions help with compatibility between devices
// --------------------------------------------------------------------

// Optional, but can come in use
// Busy wait for a given number of milliseconds (doesn't have to be accurate, mostly for testing, etc)
void device_agnostic_sleep(uint32_t ms) {
    for(int i = 0; i < (ms * 100000); i++) { // Not accurate, just gives ballpark figure
        __asm__("nop");
    }
}

// Required
// Set a GPIO pin high or low
// uint32_t port: GPIO port (may not be necessary for all platforms)
// uint32_t pin: GPIO pin
// uint8_t val: 0 for low, 1 for high
void device_agnostic_gpio_put(uint32_t port, uint32_t pin, uint8_t val) {
    if(val) {
        gpio_set(port, pin);
    } else {
        gpio_clear(port, pin);
    }
}

// --------------------------------------------------------------------

// Mandatory
// Wait for SPI peripheral to finish transmitting - should be blocking
void wait_spi_not_busy(void) {
    while (!(SPI_SR(nrf24_pin_config->spi_periph) & SPI_SR_TXE) || (SPI_SR(nrf24_pin_config->spi_periph) & SPI_SR_BSY)); // Wait for transfer finished
}

void nrf24_spi_write(uint32_t spi_peripheral, uint8_t *tx_data, uint8_t *rx_data, uint32_t len) {
    for(int i = 0; i < len; i++) {
        spi_send(nrf24_pin_config->spi_periph, *(tx_data + i));
    }

    wait_spi_not_busy();

    if(rx_data != NULL){
        *rx_data = spi_read(nrf24_pin_config->spi_periph);
    }
}

void nrf24_spi_read(uint32_t spi_peripheral, uint8_t *rx_data, uint32_t size) {
    // Read data
    for(int i = 0; i < size; i++) {
        spi_send(spi_peripheral, 0x00); // Keep clock clocking
        wait_spi_not_busy();
        *(rx_data + i) = spi_read(nrf24_pin_config->spi_periph);
    }
}

#endif