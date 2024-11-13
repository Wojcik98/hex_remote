/*
 Copyright (C)
    2011 J. Coliz <maniacbug@ymail.com>
    2024            Wojcik98

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

#ifndef RF24_H_
#define RF24_H_

#include <cstdint>

#include "gpio.hpp"
#include "spi.hpp"

constexpr uint8_t _BV(uint8_t bit) { return 1 << bit; }

struct Rf24Config {
    OutputPin &ce;
    OutputPin &csn;
    uint8_t channel;
    uint8_t payload_size;
    bool dynamic_payloads_enabled;
    uint8_t cs_delay;
};

/**
 * @}
 * @brief Simple driver class for nRF24L01(+) 2.4GHz Wireless Transceiver
 *
 * This class provides a simple interface for sending and receiving data,
 * using the nRF24L01(+) transceiver chip. It assumes only two devices are
 * communicating with each other.
 */
class SimpleRf24 {
public:
    constexpr static uint8_t MAX_PAYLOAD_SIZE = 32; // Maximum size of payload
    constexpr static uint8_t MAX_CHANNEL = 125;     // Maximum channel number
    constexpr static uint8_t NUM_PIPES = 1;         // Number of pipes

    /**
     * RF24 Constructor
     */
    SimpleRf24(Rf24Config &config, Stm32SpiDma *spi);

    virtual ~SimpleRf24(){};

    /**
     * Begin operation of the chip.
     */
    void begin();

    void enable_irq(bool enable_rx_dr, bool enable_tx_ds, bool enable_max_rt);

    void clear_irq(bool clear_rx_dr, bool clear_tx_ds, bool clear_max_rt);

    /**
     * Start listening on the pipes opened for reading.
     */
    void start_listening();

    /**
     * Stop listening for incoming messages, and switch to transmit mode.
     */
    void stop_listening();

    /**
     * Check whether there are bytes available to be read
     * @code
     * if(radio.available()){
     *   radio.read(&data,sizeof(data));
     * }
     * @endcode
     */
    bool available();

    /**
     * Read payload data from the RX FIFO buffer.
     */
    void read(uint8_t *buf, uint8_t len);

    /**
     * Non-blocking write to the open writing pipe
     *
     * Just like write(), but it returns immediately. To find out what happened
     * to the send, catch the IRQ and then call whatHappened().
     */
    bool start_write(const uint8_t *buf, uint8_t len, const bool multicast);

    /**
     * Empty all 3 of the TX (transmit) FIFO buffers. This is automatically
     * called by stopListening() if ACK payloads are enabled. However,
     * startListening() does not call this function.
     *
     * @return Current value of status register
     */
    uint8_t flush_tx();

    /**
     * Empty all 3 of the RX (receive) FIFO buffers.
     *
     * @return Current value of status register
     */
    uint8_t flush_rx();

    /**
     * Enter low-power mode
     *
     * To return to normal power mode, call powerUp().
     */
    void power_down();

    /**
     * Leave low-power mode - required for normal radio operation after calling
     * powerDown()
     */
    void power_up();

    /**
     * Retrieve the current status of the chip
     *
     * @return Current value of status register
     */
    uint8_t get_status();

private:
    static constexpr uint8_t STARTUP_DELAY = 50; // 5ms delay after power on
    static constexpr uint8_t ADDRESS_WIDTH = 5;  // 1 byte address width

    Stm32SpiDma *spi;
    Rf24Config &config;

    uint8_t logic_address[ADDRESS_WIDTH]; // The address of this node

    uint8_t status; // The status byte returned from every SPI transaction

    bool bit_mask_rx_dr;  // Mask RX_DR
    bool bit_mask_tx_ds;  // Mask TX_DS
    bool bit_mask_max_rt; // Mask MAX_RT
    bool bit_en_crc;      // Enable CRC
    bool bit_crc0;        // CRC encoding scheme
    bool bit_power_up;    // Power up
    bool bit_rx;          // RX/TX control

    /**
     * initialize radio by performing a soft reset.
     * @warning This function assumes the SPI bus object's begin() method has
     * been previously called.
     */
    void init_radio();

    /**
     * initialize the GPIO pins
     */
    void init_pins();

    uint8_t create_config();

    /**
     * Write a chunk of data to a register
     *
     * @param reg Which register. Use constants from nRF24L01.h
     * @param buf Where to get the data
     * @param len How many bytes of data to transfer
     * @return Nothing. Older versions of this function returned the status
     * byte, but that it now saved to a private member on all SPI transactions.
     */
    void write_register(uint8_t reg, const uint8_t *buf, uint8_t len);

    /**
     * Write a single byte to a register
     *
     * @param reg Which register. Use constants from nRF24L01.h
     * @param value The new value to write
     * @return Nothing. Older versions of this function returned the status
     * byte, but that it now saved to a private member on all SPI transactions.
     */
    void write_register(uint8_t reg, uint8_t value);

    /**
     * Read a chunk of data in from a register
     *
     * @param reg Which register. Use constants from nRF24L01.h
     * @param[out] buf Where to put the data
     * @param len How many bytes of data to transfer
     * @note This returns nothing. Older versions of this function returned the
     * status byte, but that it now saved to a private member on all SPI
     * transactions.
     */
    void read_register(uint8_t reg, uint8_t *buf, uint8_t len);

    /**
     * Read single byte from a register
     *
     * @param reg Which register. Use constants from nRF24L01.h
     * @return Current value of register @p reg
     */
    uint8_t read_register(uint8_t reg);

    /**
     * Write the transmit payload
     *
     * The size of data written is the fixed payload size, see getPayloadSize()
     *
     * @param buf Where to get the data
     * @param len Number of bytes to be sent
     * @param writeType Specify if individual payload should be acknowledged
     * @return Nothing. Older versions of this function returned the status
     * byte, but that it now saved to a private member on all SPI transactions.
     */
    void write_payload(const uint8_t *buf, uint8_t len,
                       const uint8_t writeType);

    /**
     * Read the receive payload
     *
     * The size of data read is the fixed payload size, see getPayloadSize()
     *
     * @param buf Where to put the data
     * @param len Maximum number of bytes to read
     * @return Nothing. Older versions of this function returned the status
     * byte, but that it now saved to a private member on all SPI transactions.
     */
    void read_payload(uint8_t *buf, uint8_t len);

    /**
     * SPI transactions
     *
     * Common code for SPI transactions including CSN toggle
     *
     */
    void begin_transaction();

    void end_transaction();

    /**
     * Set chip select pin
     *
     * @param mode HIGH to take this unit off the SPI bus, LOW to put it on
     */
    void csn(bool mode);

    /**
     * Set chip enable
     *
     * @param level HIGH to actively begin transmission or LOW to put in
     * standby.
     */
    void ce(bool level);
};

#endif // RF24_H_
