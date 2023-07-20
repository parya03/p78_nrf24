#include "nrf24l01.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>

#define MAX_NRF_READ_SIZE 32

// Global flag - contains current status
// Sending command to nrf makes it shift out current status which can be stored here
static uint8_t nrf_status;

// Set CS pin
static inline void set_cs(uint8_t status) {
    while (!(SPI_SR(SPI5) & SPI_SR_TXE) || (SPI_SR(SPI5) & SPI_SR_BSY)); // Wait for transfer finished
    if(status) {
        gpio_set(GPIOF, GPIO6);
    }
    else {
        gpio_clear(GPIOF, GPIO6);
    }
}

static inline void read_rx_fifo_until_empty(uint8_t *store_var, uint32_t max_size) {
    uint32_t count = 0;
    while ((SPI_SR(SPI5) & SPI_SR_RXNE) && (count < max_size)) {
        *(store_var + count) = spi_read(SPI5);
        count++;
    }
}

static inline void wait_spi_not_busy() { // Includes checking busy bit, not just FIFO empty bit
    while (!(SPI_SR(SPI5) & SPI_SR_TXE) || (SPI_SR(SPI5) & SPI_SR_BSY)); // Wait for transfer finished
}

uint32_t write_mem(uint8_t address, uint8_t *data, uint32_t size) {
    
    set_cs(0);
    
    // Send instruction in format 001AAAAA where AAAAA is 5-bit address
    spi_send(SPI5, ((address & 0x1F) | 0x20));
    wait_spi_not_busy();
    nrf_status = spi_read(SPI5);
    // Send data
    for(int i = 0; i < size; i++) {
        spi_send(SPI5, *(data + i));
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
    spi_send(SPI5, (address & 0x1F));
    wait_spi_not_busy();
    nrf_status = spi_read(SPI5);
    // Read data
    for(int i = 0; i < size; i++) {
        spi_send(SPI5, 0x00); // Keep clock clocking
        wait_spi_not_busy();
        *(data + i) = spi_read(SPI5);
    }
    
    set_cs(1);

    return 0;
}

uint32_t init_nrf(uint8_t rf_channel, nrf_role_t role, nrf_power_level_t power_level) {
    // TODO: Other registers should be autoinit'd for testing, but maybe change later??
    volatile uint8_t data = 0x0E + role;
    write_mem(NRF_R_CONFIG, &data, 1);

    // Verify power on/SPI communication
    read_mem(NRF_R_CONFIG, &data, 1);
    if(!(data & 0x02)) {
        return -1; // Not powered on or SPI error
    }

    data = 0x70;
    write_mem(NRF_R_STATUS, &data, 1);
    data = 0x01;
    write_mem(NRF_R_RX_PW_P0, &data, 1);

    // Set RF channel
    data = (rf_channel & 0x7F);
	write_mem(NRF_R_RF_CH, &data, 1);

    // RF Setup
    data = 0x03;
	write_mem(NRF_R_RF_SETUP, &data, 1);

    if(role == NRF_ROLE_RX) {
        gpio_set(GPIOF, GPIO15); // Set CE high
    }
    if(role == NRF_ROLE_TX) {
        gpio_clear(GPIOF, GPIO15); // Set CE low
        
        // Turn off auto ACK if TX role
        data = 0;
        write_mem(NRF_R_EN_AA, &data, 1);
    }

    return 0;
}

uint32_t read_payload(uint8_t *recieveptr, uint32_t size) {
    set_cs(0);

    spi_send(SPI5, 0x61); // Instruction for R_RX_PAYLOAD
    wait_spi_not_busy();
    nrf_status = spi_read(SPI5);

    // Read data
    for(int i = 0; i < size; i++) {
        spi_send(SPI5, 0x00); // Keep clock clocking
        wait_spi_not_busy();
        *(recieveptr + i) = spi_read(SPI5);
    }

    set_cs(1);
}

uint32_t write_payload(uint8_t *writeptr, uint32_t size) {
    set_cs(0);

    spi_send(SPI5, 0xA0); // Instruction for W_TX_PAYLOAD
    wait_spi_not_busy();
    nrf_status = spi_read(SPI5);

    // Write data
    for(int i = 0; i < size; i++) {
        spi_send(SPI5, writeptr[i]);
        wait_spi_not_busy();
        // *(writeptr + i) = spi_read(SPI5);
    }

    set_cs(1);

    wait_spi_not_busy();

    // Pulse CE to start transmission
    gpio_set(GPIOF, GPIO15); // Set CE high
    for(volatile int i = 0; i < 180; i++);
    gpio_clear(GPIOF, GPIO15); // Set CE low
}