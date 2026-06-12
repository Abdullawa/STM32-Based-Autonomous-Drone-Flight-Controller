/*
 *  nrf24l01_plus.c
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 * 
 */


#include "nrf24l01p.h"


static void cs_high()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_SET);
}

static void cs_low()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_RESET);
}

static void ce_high()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_SET);
}

static void ce_low()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_RESET);
}

static uint8_t read_register(uint8_t reg)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
    uint8_t status;
    uint8_t read_val;

    cs_low();
    if (HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000) != HAL_OK)
    {
        cs_high();
        return 0xFF;
    }
    if (HAL_SPI_Receive(NRF24L01P_SPI, &read_val, 1, 2000) != HAL_OK)
    {
        cs_high();
        return 0xFF;
    }
    cs_high();

    return read_val;
}

static uint8_t write_register(uint8_t reg, uint8_t value)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
    uint8_t status;
    uint8_t write_val = value;

    cs_low();
    if (HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000) != HAL_OK)
    {
        cs_high();
        return 0xFF;
    }
    if (HAL_SPI_Transmit(NRF24L01P_SPI, &write_val, 1, 2000) != HAL_OK)
    {
        cs_high();
        return 0xFF;
    }
    cs_high();

    return write_val;
}


static void write_address(uint8_t reg, uint8_t *addr)
{
    uint8_t cmd = NRF24L01P_CMD_W_REGISTER | reg;

    cs_low();
    HAL_SPI_Transmit(NRF24L01P_SPI, &cmd, 1, 100);
    HAL_SPI_Transmit(NRF24L01P_SPI, addr, 5, 100);
    cs_high();
}

/* nRF24L01+ Main Functions */
void nrf24l01p_rx_init(channel MHz, air_data_rate bps)
{
    nrf24l01p_reset();

    write_register(NRF24L01P_REG_EN_AA, 0x00);  // disable auto-ack, match transmitter

    // Set pipe 0 address to match ESP32 RF24 library default
    uint8_t addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    cs_low();
    uint8_t cmd = NRF24L01P_CMD_W_REGISTER | NRF24L01P_REG_RX_ADDR_P0;
    HAL_SPI_Transmit(NRF24L01P_SPI, &cmd, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, addr, 5, 2000);
    cs_high();

    nrf24l01p_prx_mode();
    nrf24l01p_power_up();

    nrf24l01p_rx_set_payload_widths(NRF24L01P_PAYLOAD_LENGTH);

    nrf24l01p_set_rf_channel(MHz);
    nrf24l01p_set_rf_air_data_rate(bps);
    nrf24l01p_set_rf_tx_output_power(_0dBm);

    nrf24l01p_set_crc_length(2);
    nrf24l01p_set_address_widths(5);

    nrf24l01p_auto_retransmit_count(0);
    nrf24l01p_auto_retransmit_delay(0);

    ce_high();
}

void nrf24l01p_tx_init(channel MHz, air_data_rate bps)
{
    nrf24l01p_reset();

    uint8_t addr[5] =
    {
        0xE7,
        0xE7,
        0xE7,
        0xE7,
        0xE7
    };

    // TX address
    cs_low();
    uint8_t cmd = NRF24L01P_CMD_W_REGISTER | NRF24L01P_REG_TX_ADDR;
    HAL_SPI_Transmit(NRF24L01P_SPI, &cmd, 1, 100);
    HAL_SPI_Transmit(NRF24L01P_SPI, addr, 5, 100);
    cs_high();

    // Pipe0 address
    cs_low();
    cmd = NRF24L01P_CMD_W_REGISTER | NRF24L01P_REG_RX_ADDR_P0;
    HAL_SPI_Transmit(NRF24L01P_SPI, &cmd, 1, 100);
    HAL_SPI_Transmit(NRF24L01P_SPI, addr, 5, 100);
    cs_high();

    write_register(NRF24L01P_REG_EN_AA, 0x00);

    nrf24l01p_ptx_mode();
    nrf24l01p_power_up();

    nrf24l01p_rx_set_payload_widths(NRF24L01P_PAYLOAD_LENGTH);

    nrf24l01p_set_rf_channel(100);
    nrf24l01p_set_rf_air_data_rate(_1Mbps);
    nrf24l01p_set_rf_tx_output_power(_0dBm);

    nrf24l01p_set_crc_length(2);
    nrf24l01p_set_address_widths(5);

    nrf24l01p_auto_retransmit_count(0);
    nrf24l01p_auto_retransmit_delay(0);

    HAL_Delay(5);
}



void nrf24l01p_rx_receive(uint8_t* rx_payload)
{
    nrf24l01p_read_rx_fifo(rx_payload);
    nrf24l01p_clear_rx_dr();
}

void nrf24l01p_tx_transmit(uint8_t* tx_payload)
{
    ce_low();

    nrf24l01p_flush_tx_fifo();
    nrf24l01p_write_tx_fifo(tx_payload);

    ce_high();
    HAL_Delay(1);
    ce_low();
}

void nrf24l01p_tx_irq()
{
    uint8_t tx_ds = nrf24l01p_get_status();
    tx_ds &= 0x20;

    if(tx_ds)
    {   
        // TX_DS
        nrf24l01p_clear_tx_ds();
    }

    else
    {
        // MAX_RT
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
        nrf24l01p_clear_max_rt();
    }
}

/* nRF24L01+ Sub Functions */
void nrf24l01p_reset()
{
    // Reset pins
    cs_high();
    ce_low();

    // Reset registers
    write_register(NRF24L01P_REG_CONFIG, 0x08);
    write_register(NRF24L01P_REG_EN_AA, 0x3F);
    write_register(NRF24L01P_REG_EN_RXADDR, 0x03);
    write_register(NRF24L01P_REG_SETUP_AW, 0x03);
    write_register(NRF24L01P_REG_SETUP_RETR, 0x03);
    write_register(NRF24L01P_REG_RF_CH, 0x02);
    write_register(NRF24L01P_REG_RF_SETUP, 0x07);
    write_register(NRF24L01P_REG_STATUS, 0x7E);
    write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P1, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P2, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P3, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P4, 0x00);
    write_register(NRF24L01P_REG_RX_PW_P5, 0x00);
    write_register(NRF24L01P_REG_FIFO_STATUS, 0x11);
    write_register(NRF24L01P_REG_DYNPD, 0x00);
    write_register(NRF24L01P_REG_FEATURE, 0x00);

    // Reset FIFO
    nrf24l01p_flush_rx_fifo();
    nrf24l01p_flush_tx_fifo();
}

void nrf24l01p_prx_mode()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 0;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_ptx_mode()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFE;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

uint8_t nrf24l01p_read_rx_fifo(uint8_t* rx_payload)
{
    uint8_t command = NRF24L01P_CMD_R_RX_PAYLOAD;
    uint8_t status;
    cs_low();
    if (HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000) != HAL_OK)
    {
        cs_high();
        return 0xFF;
    }
    if (HAL_SPI_Receive(NRF24L01P_SPI, rx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000) != HAL_OK)
    {
        cs_high();
        return 0xFF;
    }
    cs_high();

    return status;
}

uint8_t nrf24l01p_write_tx_fifo(uint8_t* tx_payload)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
    uint8_t status;
    cs_low();
    if (HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000) != HAL_OK)
    {
        cs_high();
        return 0xFF;
    }
    if (HAL_SPI_Transmit(NRF24L01P_SPI, tx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000) != HAL_OK)
    {
        cs_high();
        return 0xFF;
    }
    cs_high(); 

    return status;
}

void nrf24l01p_flush_rx_fifo()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_RX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

void nrf24l01p_flush_tx_fifo()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_TX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

uint8_t nrf24l01p_get_status()
{
    uint8_t command = NRF24L01P_CMD_NOP;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high(); 

    return status;
}

uint8_t nrf24l01p_get_fifo_status()
{
    return read_register(NRF24L01P_REG_FIFO_STATUS);
}

void nrf24l01p_rx_set_payload_widths(widths bytes)
{
    write_register(NRF24L01P_REG_RX_PW_P0, bytes);
}

void nrf24l01p_clear_rx_dr()
{
    // Write only the RX_DR flag to clear it
    write_register(NRF24L01P_REG_STATUS, 0x40);
}

void nrf24l01p_clear_tx_ds()
{
    write_register(NRF24L01P_REG_STATUS, 0x20);
}

void nrf24l01p_clear_max_rt()
{
    write_register(NRF24L01P_REG_STATUS, 0x10);
}

void nrf24l01p_power_up()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 1;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_power_down()
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFD;

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_crc_length(length bytes)
{
    uint8_t new_config = read_register(NRF24L01P_REG_CONFIG);
    
    switch(bytes)
    {
        // CRCO bit in CONFIG resiger set 0
        case 1:
            new_config &= 0xFB;
            break;
        // CRCO bit in CONFIG resiger set 1
        case 2:
            new_config |= 1 << 2;
            break;
    }

    write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_set_address_widths(widths bytes)
{
    write_register(NRF24L01P_REG_SETUP_AW, bytes - 2);
}

void nrf24l01p_auto_retransmit_count(count cnt)
{
    uint8_t reg = read_register(NRF24L01P_REG_SETUP_RETR);
    // ARC is bits 3:0 — clear then set
    reg = (reg & 0xF0) | (cnt & 0x0F);
    write_register(NRF24L01P_REG_SETUP_RETR, reg);
}

void nrf24l01p_auto_retransmit_delay(delay us)
{
    uint8_t reg = read_register(NRF24L01P_REG_SETUP_RETR);
    uint8_t ard = 0;
    if (us >= 250) {
        ard = (uint8_t)((us / 250) - 1) & 0x0F;
    }
    // ARD occupies bits 7:4 — clear then set
    reg = (reg & 0x0F) | (ard << 4);
    write_register(NRF24L01P_REG_SETUP_RETR, reg);
}

void nrf24l01p_set_rf_channel(channel MHz)
{
    // Accept channel index 0..125
    uint8_t ch = (uint8_t)(MHz & 0xFF);
    if (ch > 125) ch = 125;
    write_register(NRF24L01P_REG_RF_CH, ch);
}

void nrf24l01p_set_rf_tx_output_power(output_power dBm)
{
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (dBm << 1);

    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

void nrf24l01p_set_rf_air_data_rate(air_data_rate bps)
{
    // Set value to 0
    uint8_t new_rf_setup = read_register(NRF24L01P_REG_RF_SETUP) & 0xD7;
    
    switch(bps)
    {
        case _1Mbps: 
            break;
        case _2Mbps: 
            new_rf_setup |= 1 << 3;
            break;
        case _250kbps:
            new_rf_setup |= 1 << 5;
            break;
    }
    write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

void nrf24l01p_switch_to_rx(void) {
    ce_low();
    nrf24l01p_prx_mode();  // Set to RX mode
    nrf24l01p_rx_set_payload_widths(NRF24L01P_PAYLOAD_LENGTH);
    ce_high();
}

void nrf24l01p_switch_to_tx(void) {
    ce_low();
    nrf24l01p_ptx_mode();  // Set to TX mode
    ce_high();
}