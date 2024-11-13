#ifndef RF24_UTILITY_STM32_SPI_H_
#define RF24_UTILITY_STM32_SPI_H_

#include <atomic>
#include <cstdint>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_dma.h"
#include "stm32f4xx_hal_spi.h"

#include "gpio.hpp"

class Lock {
public:
    Lock() : lock_(false) {}

    void acquire() {
        while (std::atomic_exchange_explicit(&lock_, true,
                                             std::memory_order_acquire)) {
        }
    }

    void release() {
        std::atomic_store_explicit(&lock_, false, std::memory_order_release);
    }

private:
    std::atomic_bool lock_;
};

// class Rf24Spi {
// public:
//     virtual ~Rf24Spi(){};
//     virtual void begin() = 0;
//     virtual uint8_t transfer(uint8_t data_to_send) = 0;
//     virtual uint8_t transfer_nb(uint8_t data_to_send) = 0;
// };

// class Stm32SpiDma : public Rf24Spi {
class Stm32SpiDma {
public:
    Stm32SpiDma(SPI_HandleTypeDef *hspi, DMA_HandleTypeDef *hdma_rx,
                DMA_HandleTypeDef *hdma_tx, HalPin &csn)
        : hspi(hspi), hdma_rx(hdma_rx), hdma_tx(hdma_tx), csn(csn) {}

    void begin();
    uint8_t transfer(uint8_t data_to_send);
    uint8_t transfer_nb(uint8_t data_to_send);

private:
    constexpr static uint8_t NUM_NOP = 6;

    SPI_HandleTypeDef *hspi;
    DMA_HandleTypeDef *hdma_rx;
    DMA_HandleTypeDef *hdma_tx;

    // Callback callback;
    HalPin &csn;
    Lock lock;

    void begin_cmd() {
        lock.acquire();
        csn.write(OutputPin::LOW);
    }

    void end_cmd() {
        csn.write(OutputPin::HIGH);
        // CSN must be high for at least 50ns
        for (uint8_t i = 0; i < NUM_NOP; i++) {
            __NOP();
        }

        lock.release();
    }
};

#define RF24_SPI_PTR

#endif // RF24_UTILITY_STM32_SPI_H_
