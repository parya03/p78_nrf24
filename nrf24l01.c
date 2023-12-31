/**
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

    Main code file for the p78_nrf24 library.
*/

#include "nrf24l01.h"
#include "platform_specific.h"

#define MAX_NRF24_READ_SIZE 32

// Global flag - contains current status
// Sending command to nrf24 makes it shift out current status which can be stored here
static uint8_t nrf24_status;

// Pointer to config structs for internal library use
const nrf24_pin_config_t *nrf24_pin_config;
const nrf24_config_t *nrf24_config;

// Set CS pin
static inline void set_cs(uint8_t status) {
    wait_spi_not_busy(); // Wait for transfer finished

    device_agnostic_gpio_put(nrf24_pin_config->csn_port, nrf24_pin_config->csn_pin, status);
}

// static inline void read_rx_fifo_until_empty(uint8_t *store_var, uint32_t max_size) {
//     uint32_t count = 0;
//     while ((SPI_SR(nrf24_pin_config->spi_periph) & SPI_SR_RXNE) && (count < max_size)) {
//         *(store_var + count) = spi_read(nrf24_pin_config->spi_periph);
//         count++;
//     }
// }

// Return static global status variable
uint8_t get_nrf_status(void) {
    return nrf24_status;
}

// Write nrf24 memory address - Usually for register writes
uint32_t write_mem(uint8_t address, uint8_t *data, uint32_t size) {
    
    set_cs(0);

    address &= 0x1F; // Mask address to 5 bits
    address |= 0x20; // Set write bit
    
    // Send instruction in format 001AAAAA where AAAAA is 5-bit address, store status
    nrf24_spi_write(nrf24_pin_config->spi_periph, &address, &nrf24_status, 1);
    wait_spi_not_busy();
    // Send data
    nrf24_spi_write(nrf24_pin_config->spi_periph, data, NULL, size);
    wait_spi_not_busy();
    
    set_cs(1);
    
    return 0;
}

// Write register with friendly write; read current contents then & and | with data you want to write
// to only write certain bits
uint32_t write_reg_friendly(uint8_t address, uint8_t *data) {
    uint8_t data_buf = 0;
    read_mem(address, &data_buf, 1);
    data_buf &= *data;
    data_buf |= *data;
    write_mem(address, &data_buf, 1);
}

uint32_t read_mem(uint8_t address, uint8_t *data, uint32_t size) {
    
    set_cs(0);

    address &= 0x1F; // Mask address to 5 bits
    
    // Send instruction in format 000AAAAA where AAAAA is 5-bit address
    nrf24_spi_write(nrf24_pin_config->spi_periph, &address, &nrf24_status, 1);
    wait_spi_not_busy();
    // Read data
    nrf24_spi_read(nrf24_pin_config->spi_periph, data, size);
    wait_spi_not_busy();
    
    set_cs(1);

    return 0;
}

uint32_t init_nrf24(const nrf24_pin_config_t *pin_config, const nrf24_config_t *config) {
    // TODO: Other registers should be autoinit'd for testing, but maybe change later??

    nrf24_pin_config = pin_config;
    nrf24_config = config;
    
    // Pin setup
    // SPI
	// Control CS# (PF6) in software, so not initialized as AF
	// gpio_mode_setup(pin_config->csn_port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, pin_config->csn_pin);
	// gpio_mode_setup(pin_config->ce_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin_config->ce_pin); // CE pin
	// gpio_set(pin_config->csn_port, pin_config->csn_pin); // Set CS high
	// gpio_set(pin_config->ce_port, pin_config->ce_pin); // Set CE high
    // gpio_set_output_options(pin_config->csn_port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, pin_config->csn_pin);
	// spi_reset(pin_config->spi_periph);
	// spi_init_master(pin_config->spi_periph,
	// 	SPI_CR1_BAUDRATE_FPCLK_DIV_32,
	// 	SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
	// 	SPI_CR1_CPHA_CLK_TRANSITION_1,
	// 	SPI_CR1_DFF_8BIT,
	// 	SPI_CR1_MSBFIRST);
	// // SPI_CR2(pin_config->spi_periph) |= SPI_CR2_SSOE;
	// // SPI_CR1(pin_config->spi_periph) = cr_tmp;
	// // spi_set_nss_high(pin_config->spi_periph);
	// spi_set_full_duplex_mode(pin_config->spi_periph);
	// // spi_enable_ss_output(pin_config->spi_periph);
	// // spi_enable_software_slave_management(pin_config->spi_periph);

	// spi_enable(pin_config->spi_periph);
    
    volatile uint8_t data = 0x0E + config->role;
    write_mem(NRF24_R_CONFIG, &data, 1);

    // Verify power on/SPI communication
    read_mem(NRF24_R_CONFIG, &data, 1);
    if(!(data & 0x02)) {
        return -1; // Not powered on or SPI error
    }

    data = 0x70;
    write_mem(NRF24_R_STATUS, &data, 1);
    data = 0x01;
    write_mem(NRF24_R_RX_PW_P0, &data, 1);

    // Set RF channel
    data = (config->rf_channel & 0x7F);
	write_mem(NRF24_R_RF_CH, &data, 1);

    // RF Setup
    data = (config->data_rate << 3) | (config->power_level << 1) | 0x01;
	write_mem(NRF24_R_RF_SETUP, &data, 1);

    if(config->role == NRF24_ROLE_RX) {
        data = config->enable_auto_ack * 0x3F;
        write_mem(NRF24_R_EN_AA, &data, 1);
        
        device_agnostic_gpio_put(pin_config->ce_port, pin_config->ce_pin, 1); // Set CE high
    }
    if(config->role == NRF24_ROLE_TX) {
        device_agnostic_gpio_put(pin_config->ce_port, pin_config->ce_pin, 0); // Set CE low
        
        // Turn off auto ACK if TX role
        data = config->enable_auto_ack * 0x3F;
        write_mem(NRF24_R_EN_AA, &data, 1);
    }

    return 0;
}

uint32_t read_payload(uint8_t *recieveptr, uint32_t size) {
    set_cs(0);
    
    uint8_t data = 0x61;
    nrf24_spi_write(nrf24_pin_config->spi_periph, &data, &nrf24_status, 1); // Instruction for R_RX_PAYLOAD
    wait_spi_not_busy();
    // nrf24_status = spi_read(nrf24_pin_config->spi_periph);

    // Read data
    // for(int i = 0; i < size; i++) {
    //     spi_send(nrf24_pin_config->spi_periph, 0x00); // Keep clock clocking
    //     wait_spi_not_busy();
    //     *(recieveptr + i) = spi_read(nrf24_pin_config->spi_periph);
    // }
    nrf24_spi_read(nrf24_pin_config->spi_periph, recieveptr, size);
    wait_spi_not_busy();
    
    set_cs(1);
}

uint32_t write_payload(uint8_t *writeptr, uint32_t size) {
    set_cs(0);
    
    uint8_t data = 0xA0;
    nrf24_spi_write(nrf24_pin_config->spi_periph, &data, &nrf24_status, 1); // Instruction for W_TX_PAYLOAD
    wait_spi_not_busy();
    // nrf24_status = spi_read(nrf24_pin_config->spi_periph);

    // Write data
    nrf24_spi_write(nrf24_pin_config->spi_periph, writeptr, NULL, size);
    wait_spi_not_busy();

    set_cs(1);

    wait_spi_not_busy();

    // Pulse CE to start transmission
    device_agnostic_gpio_put(nrf24_pin_config->ce_port, nrf24_pin_config->ce_pin, 1); // Set CE high
    for(volatile int i = 0; i < 180; i++);
    device_agnostic_gpio_put(nrf24_pin_config->ce_port, nrf24_pin_config->ce_pin, 0); // Set CE low
    
    set_cs(1);
}