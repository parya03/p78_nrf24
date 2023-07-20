#ifndef NRF_H
#define NRF_H

#include <stdint.h>

typedef enum {
    NRF_ROLE_TX = 0,
    NRF_ROLE_RX = 1
} nrf_role_t;

typedef enum {
    NRF_PWR_LOW = 0,
    NRF_PWR_MED_1 = 1,
    NRF_PWR_MED_2 = 2,
    NRF_PWR_MAX = 3
} nrf_power_level_t;

uint32_t write_mem(uint8_t address, uint8_t *data, uint32_t size);
uint32_t read_mem(uint8_t address, uint8_t *data, uint32_t size);
uint32_t init_nrf(uint8_t rf_channel, nrf_role_t role, nrf_power_level_t power_level);
uint32_t read_payload(uint8_t *recieveptr, uint32_t size);
uint32_t write_payload(uint8_t *writeptr, uint32_t size);

// Register addresses:
#define NRF_R_CONFIG 0x00
#define NRF_R_EN_AA 0x01
#define NRF_R_EN_RXADDR 0x02
#define NRF_R_SETUP_AW 0x03
#define NRF_R_SETUP_RETR 0x04
#define NRF_R_RF_CH 0x05
#define NRF_R_RF_SETUP 0x06
#define NRF_R_STATUS 0x07
#define NRF_R_OBSERVE_TX 0x08
#define NRF_R_CD 0x09
#define NRF_R_RX_ADDR_P0 0x0A
#define NRF_R_RX_ADDR_P1 0x0B
#define NRF_R_RX_ADDR_P2 0x0C
#define NRF_R_RX_ADDR_P3 0x0D
#define NRF_R_RX_ADDR_P4 0x0E
#define NRF_R_RX_ADDR_P5 0x0F
#define NRF_R_TX_ADDR 0x10
#define NRF_R_RX_PW_P0 0x11
#define NRF_R_RX_PW_P1 0x12
#define NRF_R_RX_PW_P2 0x13
#define NRF_R_RX_PW_P3 0x14
#define NRF_R_RX_PW_P4 0x15
#define NRF_R_RX_PW_P5 0x16
#define NRF_R_FIFO_STATUS 0x17

#endif