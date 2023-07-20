#ifndef NRF24_H
#define NRF24_H

#include <stdint.h>

typedef enum {
    NRF24_ROLE_TX = 0,
    NRF24_ROLE_RX = 1
} nrf24_role_t;

typedef enum {
    NRF24_PWR_MIN = 0,
    NRF24_PWR_MED_1 = 1,
    NRF24_PWR_MED_2 = 2,
    NRF24_PWR_MAX = 3
} nrf24_power_level_t;

typedef enum {
    NRF24_PIPE_0 = 0,
    NRF24_PIPE_1 = 1,
    NRF24_PIPE_2 = 2,
    NRF24_PIPE_3 = 3,
    NRF24_PIPE_4 = 4,
    NRF24_PIPE_5 = 5
} rx_pipe_t;

typedef struct {
    uint32_t spi_periph; // SPI1, SPI2, SPI3, etc
    uint32_t spi_port; // GPIO port for SPI
    uint32_t spi_pins; // GPIO pins for SPI bitwise OR (|)'d together
    uint32_t csn_port; // GPIO port for CSN
    uint32_t csn_pin; // GPIO pin for CSN
    uint32_t ce_port; // GPIO port for CE
    uint32_t ce_pin; // GPIO pin for CE
} nrf24_pin_config_t;

typedef struct {
    nrf24_role_t role; // NRF24_ROLE_TX or NRF24_ROLE_RX
    uint32_t rf_channel; // RF channel, 0x00 to 0x7F 
    nrf24_power_level_t power_level; // Power level for LNA
    uint8_t enable_auto_ack; // Enable auto ack TX/RX
    rx_pipe_t rx_pipe; // TODO
} nrf24_config_t;

uint32_t write_mem(uint8_t address, uint8_t *data, uint32_t size);
uint32_t read_mem(uint8_t address, uint8_t *data, uint32_t size);
uint32_t init_nrf24(nrf24_pin_config_t *pin_config, nrf24_config_t *config);
uint32_t read_payload(uint8_t *recieveptr, uint32_t size);
uint32_t write_payload(uint8_t *writeptr, uint32_t size);

// Register addresses:
#define NRF24_R_CONFIG 0x00
#define NRF24_R_EN_AA 0x01
#define NRF24_R_EN_RXADDR 0x02
#define NRF24_R_SETUP_AW 0x03
#define NRF24_R_SETUP_RETR 0x04
#define NRF24_R_RF_CH 0x05
#define NRF24_R_RF_SETUP 0x06
#define NRF24_R_STATUS 0x07
#define NRF24_R_OBSERVE_TX 0x08
#define NRF24_R_CD 0x09
#define NRF24_R_RX_ADDR_P0 0x0A
#define NRF24_R_RX_ADDR_P1 0x0B
#define NRF24_R_RX_ADDR_P2 0x0C
#define NRF24_R_RX_ADDR_P3 0x0D
#define NRF24_R_RX_ADDR_P4 0x0E
#define NRF24_R_RX_ADDR_P5 0x0F
#define NRF24_R_TX_ADDR 0x10
#define NRF24_R_RX_PW_P0 0x11
#define NRF24_R_RX_PW_P1 0x12
#define NRF24_R_RX_PW_P2 0x13
#define NRF24_R_RX_PW_P3 0x14
#define NRF24_R_RX_PW_P4 0x15
#define NRF24_R_RX_PW_P5 0x16
#define NRF24_R_FIFO_STATUS 0x17

#endif