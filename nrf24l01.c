#include "nrf24l01.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#define MAX_NRF24_READ_SIZE 32

// Global flag - contains current status
// Sending command to nrf24 makes it shift out current status which can be stored here
static uint8_t nrf24_status;

// Pointer to config structs for internal library use
static nrf24_pin_config_t *nrf24_pin_config;
static nrf24_config_t *nrf24_config;

// Set CS pin
static inline void set_cs(uint8_t status) {
    while (!(SPI_SR(nrf24_pin_config->spi_periph) & SPI_SR_TXE) || (SPI_SR(nrf24_pin_config->spi_periph) & SPI_SR_BSY)); // Wait for transfer finished
    if(status) {
        gpio_set(nrf24_pin_config->csn_pin, nrf24_pin_config->csn_pin);
    }
    else {
        gpio_clear(nrf24_pin_config->csn_pin, nrf24_pin_config->csn_pin);
    }
}

static inline void read_rx_fifo_until_empty(uint8_t *store_var, uint32_t max_size) {
    uint32_t count = 0;
    while ((SPI_SR(nrf24_pin_config->spi_periph) & SPI_SR_RXNE) && (count < max_size)) {
        *(store_var + count) = spi_read(nrf24_pin_config->spi_periph);
        count++;
    }
}

// Includes checking busy bit, not just FIFO empty bit
static inline void wait_spi_not_busy() {
    while (!(SPI_SR(nrf24_pin_config->spi_periph) & SPI_SR_TXE) || (SPI_SR(nrf24_pin_config->spi_periph) & SPI_SR_BSY)); // Wait for transfer finished
}

uint32_t write_mem(uint8_t address, uint8_t *data, uint32_t size) {
    
    set_cs(0);
    
    // Send instruction in format 001AAAAA where AAAAA is 5-bit address
    spi_send(nrf24_pin_config->spi_periph, ((address & 0x1F) | 0x20));
    wait_spi_not_busy();
    nrf24_status = spi_read(nrf24_pin_config->spi_periph);
    // Send data
    for(int i = 0; i < size; i++) {
        spi_send(nrf24_pin_config->spi_periph, *(data + i));
    }

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
    
    // Send instruction in format 000AAAAA where AAAAA is 5-bit address
    spi_send(nrf24_pin_config->spi_periph, (address & 0x1F));
    wait_spi_not_busy();
    nrf24_status = spi_read(nrf24_pin_config->spi_periph);
    // Read data
    for(int i = 0; i < size; i++) {
        spi_send(nrf24_pin_config->spi_periph, 0x00); // Keep clock clocking
        wait_spi_not_busy();
        *(data + i) = spi_read(nrf24_pin_config->spi_periph);
    }
    
    set_cs(1);

    return 0;
}

uint32_t init_nrf24(nrf24_pin_config_t *pin_config, nrf24_config_t *config) {
    // TODO: Other registers should be autoinit'd for testing, but maybe change later??

    nrf24_pin_config = pin_config;
    nrf24_config = config;
    
    // Pin setup
    // SPI
	// Control CS# (PF6) in software, so not initialized as AF
	gpio_mode_setup(pin_config->csn_port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, pin_config->csn_pin);
	gpio_mode_setup(pin_config->ce_port, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pin_config->ce_pin); // CE pin
	gpio_set(pin_config->csn_port, pin_config->csn_pin); // Set CS high
	gpio_set(pin_config->ce_port, pin_config->ce_pin); // Set CE high
    gpio_set_output_options(pin_config->csn_port, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, pin_config->csn_pin);
	spi_reset(pin_config->spi_periph);
	spi_init_master(pin_config->spi_periph,
		SPI_CR1_BAUDRATE_FPCLK_DIV_32,
		SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
		SPI_CR1_CPHA_CLK_TRANSITION_1,
		SPI_CR1_DFF_8BIT,
		SPI_CR1_MSBFIRST);
	// SPI_CR2(pin_config->spi_periph) |= SPI_CR2_SSOE;
	// SPI_CR1(pin_config->spi_periph) = cr_tmp;
	// spi_set_nss_high(pin_config->spi_periph);
	spi_set_full_duplex_mode(pin_config->spi_periph);
	// spi_enable_ss_output(pin_config->spi_periph);
	// spi_enable_software_slave_management(pin_config->spi_periph);

	spi_enable(pin_config->spi_periph);
    
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
    data = 0x03;
	write_mem(NRF24_R_RF_SETUP, &data, 1);

    if(config->role == NRF24_ROLE_RX) {
        gpio_set(pin_config->ce_port, pin_config->ce_pin); // Set CE high
    }
    if(config->role == NRF24_ROLE_TX) {
        gpio_clear(pin_config->ce_port, pin_config->ce_pin); // Set CE low
        
        // Turn off auto ACK if TX role
        data = 0;
        write_mem(NRF24_R_EN_AA, &data, 1);
    }

    return 0;
}

uint32_t read_payload(uint8_t *recieveptr, uint32_t size) {
    set_cs(0);

    spi_send(nrf24_pin_config->spi_periph, 0x61); // Instruction for R_RX_PAYLOAD
    wait_spi_not_busy();
    nrf24_status = spi_read(nrf24_pin_config->spi_periph);

    // Read data
    for(int i = 0; i < size; i++) {
        spi_send(nrf24_pin_config->spi_periph, 0x00); // Keep clock clocking
        wait_spi_not_busy();
        *(recieveptr + i) = spi_read(nrf24_pin_config->spi_periph);
    }

    set_cs(1);
}

uint32_t write_payload(uint8_t *writeptr, uint32_t size) {
    set_cs(0);

    spi_send(nrf24_pin_config->spi_periph, 0xA0); // Instruction for W_TX_PAYLOAD
    wait_spi_not_busy();
    nrf24_status = spi_read(nrf24_pin_config->spi_periph);

    // Write data
    for(int i = 0; i < size; i++) {
        spi_send(nrf24_pin_config->spi_periph, writeptr[i]);
        wait_spi_not_busy();
        // *(writeptr + i) = spi_read(nrf24_pin_config->spi_periph);
    }

    set_cs(1);

    wait_spi_not_busy();

    // Pulse CE to start transmission
    gpio_set(nrf24_pin_config->ce_port, nrf24_pin_config->ce_pin); // Set CE high
    for(volatile int i = 0; i < 180; i++);
    gpio_clear(nrf24_pin_config->ce_port, nrf24_pin_config->ce_pin); // Set CE low
}