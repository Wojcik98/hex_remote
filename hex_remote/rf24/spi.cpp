#include "spi.hpp"

#include "stm32f4xx_hal_spi.h"
#include <cstdint>

void Stm32SpiDma::begin() {}

uint8_t Stm32SpiDma::transfer(uint8_t data_to_send) {
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(hspi, &data_to_send, &rx_data, 1, HAL_MAX_DELAY);
    return rx_data;
}

uint8_t Stm32SpiDma::transfer_nb(uint8_t data_to_send) {
    // TODO: Implement non-blocking transfer
    return transfer(data_to_send);
}
