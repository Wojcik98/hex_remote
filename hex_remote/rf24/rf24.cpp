/*
 Copyright (C)
    2011 J. Coliz <maniacbug@ymail.com>
    2024            Wojcik98

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#include "rf24.hpp"
#include "gpio.hpp"
#include "nrf24l01.hpp"
#include "rf24_config.hpp"
#include "spi.hpp"
#include "stm32f4xx_hal.h"
#include <algorithm>
#include <cstdint>

/****************************************************************************/

SimpleRf24::SimpleRf24(Rf24Config &config, Stm32SpiDma *spi)
    : config(config), spi(spi) {
    logic_address[0] = 1;
    logic_address[1] = 1;
    logic_address[2] = 1;
    logic_address[3] = 1;
    logic_address[4] = 1;
}

/****************************************************************************/

void SimpleRf24::begin() {
    spi->begin();
    init_pins();
    init_radio();
}

/****************************************************************************/

void SimpleRf24::init_pins() {
    config.ce.init();
    config.csn.init();

    ce(OutputPin::LOW);
    csn(OutputPin::HIGH);
}

/****************************************************************************/

void SimpleRf24::init_radio() {
    HAL_Delay(STARTUP_DELAY);
    power_down();

    write_register(RF_SETUP, 0x06);        // 1Mbps, 0dBm
    write_register(SETUP_AW, 0x03);        // 5 bytes address width
    write_register(SETUP_RETR, 0x3F);      // 1500us, 15 retrans
    write_register(EN_AA, 0x3F);           // Enable auto-ack on all pipes
    write_register(EN_RXADDR, 0x01);       // Enable pipe 0
    write_register(RF_CH, config.channel); // Channel
    write_register(RX_ADDR_P0, logic_address, ADDRESS_WIDTH);
    write_register(TX_ADDR, logic_address, ADDRESS_WIDTH);
    write_register(RX_PW_P0, config.payload_size); // payload size

    bit_mask_rx_dr = true;
    bit_mask_tx_ds = true;
    bit_mask_max_rt = true;
    bit_en_crc = true;
    bit_crc0 = false;
    bit_power_up = true;
    bit_rx = false;
    write_register(NRF_CONFIG, create_config());
    HAL_Delay(STARTUP_DELAY);
}

/****************************************************************************/

void SimpleRf24::enable_irq(bool enable_rx_dr, bool enable_tx_ds,
                            bool enable_max_rt) {
    // Write 0 to enable an interrupt
    bit_mask_rx_dr = !enable_rx_dr;
    bit_mask_tx_ds = !enable_tx_ds;
    bit_mask_max_rt = !enable_max_rt;
    write_register(NRF_CONFIG, create_config());
}

/****************************************************************************/

void SimpleRf24::clear_irq(bool clear_rx_dr, bool clear_tx_ds,
                           bool clear_max_rt) {
    uint8_t reg = 0;
    if (clear_rx_dr) {
        reg |= _BV(RX_DR);
    }
    if (clear_tx_ds) {
        reg |= _BV(TX_DS);
    }
    if (clear_max_rt) {
        reg |= _BV(MAX_RT);
    }
    write_register(NRF_STATUS, reg);
}

/****************************************************************************/

uint8_t SimpleRf24::flush_rx() {
    read_register(FLUSH_RX, nullptr, 0);
    return status;
}

/****************************************************************************/

uint8_t SimpleRf24::flush_tx() {
    read_register(FLUSH_TX, nullptr, 0);
    return status;
}

/****************************************************************************/

uint8_t SimpleRf24::get_status() {
    read_register(RF24_NOP, nullptr, 0);
    return status;
}

/****************************************************************************/

void SimpleRf24::start_listening() {
    bit_rx = true;
    write_register(NRF_CONFIG, create_config());
    ce(OutputPin::HIGH);
}

/****************************************************************************/

void SimpleRf24::stop_listening() {
    ce(OutputPin::LOW);

    flush_tx();

    bit_rx = false;
    write_register(NRF_CONFIG, create_config());
}

/****************************************************************************/

void SimpleRf24::power_down() {
    ce(OutputPin::LOW); // Guarantee CE is low on powerDown
    bit_power_up = false;
    write_register(NRF_CONFIG, create_config());
}

/****************************************************************************/

// Power up now. Radio will not power down unless instructed by MCU for config
// changes etc.
void SimpleRf24::power_up() {
    bit_power_up = true;
    write_register(NRF_CONFIG, create_config());
    HAL_Delay(STARTUP_DELAY);
}

/****************************************************************************/

uint8_t SimpleRf24::create_config() {
    return (bit_mask_rx_dr ? _BV(MASK_RX_DR) : 0) |
           (bit_mask_tx_ds ? _BV(MASK_TX_DS) : 0) |
           (bit_mask_max_rt ? _BV(MASK_MAX_RT) : 0) |
           (bit_en_crc ? _BV(EN_CRC) : 0) | (bit_crc0 ? _BV(CRCO) : 0) |
           (bit_power_up ? _BV(PWR_UP) : 0) | (bit_rx ? _BV(PRIM_RX) : 0);
}

bool SimpleRf24::start_write(const uint8_t *buf, uint8_t len,
                             const bool multicast) {

    // Send the payload
    write_payload(buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD);
    ce(OutputPin::HIGH);
    HAL_Delay(1);
    ce(OutputPin::LOW);
    return !(status & _BV(TX_FULL));
}

/****************************************************************************/

bool SimpleRf24::available() {
    uint8_t pipe = RF24_NO_FETCH_PIPE;
    volatile uint8_t fifo_status = read_register(FIFO_STATUS);
    if (fifo_status & 1) { // if RX FIFO is empty
        return 0;
    } else {
        return 1;
    }
}

/****************************************************************************/

void SimpleRf24::read(uint8_t *buf, uint8_t len) {
    // Fetch the payload
    read_payload(buf, len);

    // Clear the only applicable interrupt flags
    write_register(NRF_STATUS, _BV(RX_DR));
}

/****************************************************************************/

void SimpleRf24::write_register(uint8_t reg, const uint8_t *buf, uint8_t len) {
    begin_transaction();
    status = spi->transfer(W_REGISTER | reg);
    while (len--) {
        spi->transfer(*buf++);
    }

    end_transaction();
}

/****************************************************************************/

void SimpleRf24::write_register(uint8_t reg, uint8_t value) {
    write_register(reg, &value, 1);
}

/****************************************************************************/

void SimpleRf24::read_register(uint8_t reg, uint8_t *buf, uint8_t len) {
    begin_transaction();
    status = spi->transfer(reg);
    while (len--) {
        *buf++ = spi->transfer(0xFF);
    }

    end_transaction();
}

/****************************************************************************/

uint8_t SimpleRf24::read_register(uint8_t reg) {
    uint8_t result;
    read_register(reg, &result, 1);

    return result;
}

/****************************************************************************/

void SimpleRf24::write_payload(const uint8_t *buf, uint8_t data_len,
                               const uint8_t writeType) {
    const uint8_t *current = buf;

    uint8_t blank_len = !data_len ? 1 : 0;
    if (!config.dynamic_payloads_enabled) {
        data_len = std::min(data_len, config.payload_size);
        blank_len = config.payload_size - data_len;
    } else {
        data_len = std::min(data_len, MAX_PAYLOAD_SIZE);
    }

    begin_transaction();
    status = spi->transfer(writeType);
    while (data_len--) {
        spi->transfer(*current++);
    }

    while (blank_len--) {
        spi->transfer(0);
    }
    end_transaction();
}

/****************************************************************************/

void SimpleRf24::read_payload(uint8_t *buf, uint8_t data_len) {
    uint8_t *current = buf;

    uint8_t blank_len = 0;
    if (!config.dynamic_payloads_enabled) {
        data_len = std::min(data_len, config.payload_size);
        blank_len = config.payload_size - data_len;
    } else {
        data_len = std::min(data_len, MAX_PAYLOAD_SIZE);
    }

#if defined(RF24_LINUX) || defined(RF24_RP2)
    begin_transaction();
    uint8_t *prx = spi_rxbuff;
    uint8_t *ptx = spi_txbuff;
    uint8_t size;
    size = static_cast<uint8_t>(data_len + blank_len +
                                1); // Add register value to transmit buffer

    *ptx++ = R_RX_PAYLOAD;
    while (--size) {
        *ptx++ = RF24_NOP;
    }

    size = static_cast<uint8_t>(
        data_len + blank_len + 1); // Size has been lost during while, re affect

#if defined(RF24_RP2)
    _spi->transfernb((const uint8_t *)spi_txbuff, spi_rxbuff, size);
#else  // !defined(RF24_RP2)
    _SPI.transfernb(reinterpret_cast<char *>(spi_txbuff),
                    reinterpret_cast<char *>(spi_rxbuff), size);
#endif // !defined(RF24_RP2)

    status = *prx++; // 1st byte is status

    if (data_len > 0) {
        // Decrement before to skip 1st status byte
        while (--data_len) {
            *current++ = *prx++;
        }

        *current = *prx;
    }
    end_transaction();
#else // !defined(RF24_LINUX) && !defined(RF24_RP2)

    begin_transaction();
#if defined(RF24_SPI_PTR)
    status = spi->transfer(R_RX_PAYLOAD);
    while (data_len--) {
        *current++ = spi->transfer(0xFF);
    }

    while (blank_len--) {
        spi->transfer(0xFF);
    }

#else // !defined(RF24_SPI_PTR)
    status = _SPI.transfer(R_RX_PAYLOAD);
    while (data_len--) {
        *current++ = _SPI.transfer(0xFF);
    }

    while (blank_len--) {
        _SPI.transfer(0xff);
    }

#endif // !defined(RF24_SPI_PTR)
    end_transaction();

#endif // !defined(RF24_LINUX) && !defined(RF24_RP2)
}

/****************************************************************************/

void SimpleRf24::begin_transaction() { csn(OutputPin::LOW); }

/****************************************************************************/

void SimpleRf24::end_transaction() { csn(OutputPin::HIGH); }

/****************************************************************************/

void SimpleRf24::csn(bool level) {
    config.csn.write(level);
    HAL_Delay(config.cs_delay);
}

/****************************************************************************/

void SimpleRf24::ce(bool level) { config.ce.write(level); }
