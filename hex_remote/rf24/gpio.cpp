#include "gpio.hpp"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

void HalPin::write(bool value) {
    HAL_GPIO_WritePin(port, pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void HalPin::init() {
    GPIO_InitTypeDef config;
    config.Pin = pin;
    config.Mode = GPIO_MODE_OUTPUT_PP;
    config.Pull = GPIO_PULLUP;
    config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(port, &config);

#if defined(GPIOA)
    if (port == GPIOA) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
#endif
#if defined(GPIOB)
    if (port == GPIOB) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
#endif
#if defined(GPIOC)
    if (port == GPIOC) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
#endif
#if defined(GPIOD)
    if (port == GPIOD) {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
#endif
#if defined(GPIOE)
    if (port == GPIOE) {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
#endif
#if defined(GPIOF)
    if (port == GPIOF) {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
#endif
#if defined(GPIOG)
    if (port == GPIOG) {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
#endif
#if defined(GPIOH)
    if (port == GPIOH) {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
#endif
}
