/**
 * @file gpio.h
 * Class declaration for GPIO helper files
 */

#ifndef RF24_UTILITY_STM32_GPIO_H_
#define RF24_UTILITY_STM32_GPIO_H_

#include <cstdint>
#include <stdint.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

class OutputPin {
public:
    constexpr static bool HIGH = true;
    constexpr static bool LOW = false;

    virtual void init() = 0;
    virtual void write(bool value) = 0;
    virtual ~OutputPin(){};
};

class HalPin : public OutputPin {
public:
    HalPin(GPIO_TypeDef *port, uint16_t pin) : port(port), pin(pin){};
    void init() override;
    void write(bool value) override;
    ~HalPin(){};

private:
    GPIO_TypeDef *const port;
    const uint16_t pin;
};

#endif // RF24_UTILITY_STM32_GPIO_H_
