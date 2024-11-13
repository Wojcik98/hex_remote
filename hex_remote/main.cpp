#include "stm32f429i_discovery_ts.h"
#include <cstdint>
extern "C" {
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"
#include "stm32f4xx.h" // IWYU pragma: keep
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
}

#define RF24_SPI_SPEED (16000000 / 16)
#include "rf24/gpio.hpp"
#include "rf24/nrf24l01.hpp"
#include "rf24/rf24.hpp"
#include "rf24/spi.hpp"

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef dma_spi3_rx;

// PC1      ------> RF24_CS
// PC0      ------> RF24_CE
HalPin ce{GPIOG, GPIO_PIN_11};
HalPin csn{GPIOG, GPIO_PIN_12};

Rf24Config rf24_config = {
    ce, csn, 4, 32, false, 1,
};

Stm32SpiDma spi(&hspi3, &dma_spi3_rx, &dma_spi3_rx, csn);

SimpleRf24 radio(rf24_config, &spi);

uint8_t data[32];
TS_StateTypeDef ts_state;

static void SystemClock_Config();
void SPI_Init();

extern "C" { // Interrupts handlers need to be "visible" in C
void SysTick_Handler(void) { HAL_IncTick(); }
void SPI3_IRQHandler(void) { HAL_SPI_IRQHandler(&hspi3); }

void DMA1_Stream2_IRQHandler(void) { HAL_DMA_IRQHandler(hspi3.hdmarx); }

void DMA1_Stream5_IRQHandler(void) { HAL_DMA_IRQHandler(hspi3.hdmatx); }
}

void HAL_IncTick(void) { uwTick += 4; }

void led_on() { HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET); }

void led_off() { HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET); }

bool button_pressed() {
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET;
}

union FloatBytes {
    float f[2];
    uint8_t b[8];
};
FloatBytes fb;
uint8_t status;

int main() {
    HAL_Init();
    SystemInit();

    SystemClock_Config();
    SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / 250);
    BSP_LCD_Init();
    BSP_LCD_SelectLayer(0);
    BSP_LCD_LayerDefaultInit(0, LCD_FRAME_BUFFER);
    BSP_LCD_DisplayOn();

    uint16_t size_x = BSP_LCD_GetXSize();
    uint16_t size_y = BSP_LCD_GetYSize();

    BSP_TS_Init(size_x, size_y);

    __HAL_RCC_GPIOG_CLK_ENABLE();
    GPIO_InitTypeDef config;
    config.Pin = GPIO_PIN_13;
    config.Mode = GPIO_MODE_OUTPUT_PP;
    config.Pull = GPIO_NOPULL;
    config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOG, &config);

    // button on pa0
    __HAL_RCC_GPIOA_CLK_ENABLE();
    config.Pin = GPIO_PIN_0;
    config.Mode = GPIO_MODE_INPUT;
    config.Pull = GPIO_NOPULL;
    config.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &config);

    SPI_Init();

    radio.begin();
    HAL_Delay(10);
    radio.flush_rx();
    HAL_Delay(10);

    fb.f[0] = 0.0f;
    fb.f[1] = 0.0f;

    uint8_t tx_success[] = "TX SUCCESS";
    uint8_t tx_fail[] = "TX FAIL";

    while (true) {
        radio.start_write(fb.b, 8, false);
        status = radio.get_status();
        while (!(status & _BV(TX_DS)) && !(status & _BV(MAX_RT))) {
            status = radio.get_status();
        }

        BSP_TS_GetState(&ts_state);

        BSP_LCD_Clear(LCD_COLOR_WHITE);
        BSP_LCD_SetBackColor(LCD_COLOR_WHITE);

        if (ts_state.TouchDetected) {
            BSP_LCD_SetTextColor(LCD_COLOR_RED);
            // fb.f[0] = 0.3f;
            // fb.f[1] = -0.15f;
            float omz = 2.0f * ((float)ts_state.X / size_x) - 1.0f;
            float vx = 2.0f * ((float)ts_state.Y / size_y) - 1.0f;

            vx *= 0.3f;
            omz *= 0.6f;

            fb.f[0] = vx;
            fb.f[1] = omz;
        } else {
            BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
            fb.f[0] = 0.0f;
            fb.f[1] = 0.0f;
        }

        if (status & _BV(TX_DS)) {
            BSP_LCD_DisplayStringAt(10, 10, tx_success, CENTER_MODE);
            led_on();
        }
        if (status & _BV(MAX_RT)) {
            BSP_LCD_DisplayStringAt(10, 10, tx_fail, CENTER_MODE);
            led_off();
            radio.flush_tx();
        }

        radio.clear_irq(false, true, true);
        HAL_Delay(50);
    }
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 168000000
 *            HCLK(Hz)                       = 168000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 8000000
 *            PLL_M                          = 8
 *            PLL_N                          = 336
 *            PLL_P                          = 2
 *            PLL_Q                          = 7
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 5
 * @param  None
 * @retval None
 */
static void SystemClock_Config(void) {
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    /* Enable Power Control clock */
    __HAL_RCC_PWR_CLK_ENABLE();

    /* The voltage scaling allows optimizing the power consumption when the
       device is clocked below the maximum system frequency, to update the
       voltage scaling value regarding system frequency refer to product
       datasheet.  */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8U;
    RCC_OscInitStruct.PLL.PLLN = 336U;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7U;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
       clocks dividers */
    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    }

    /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported */
    if (HAL_GetREVID() == 0x1001) {
        /* Enable the Flash prefetch */
        __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
    }
}

void SPI_Init() {
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.BaudRatePrescaler =
        SPI_BAUDRATEPRESCALER_16; // 16MHz / 16 = 1MHz
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi3);

    HAL_NVIC_EnableIRQ(SPI3_IRQn);
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
    GPIO_InitTypeDef GPIO_InitStruct;

    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /**SPI3 GPIO Configuration
    PD6      ------> SPI3_MOSI
    PC10     ------> SPI3_SCK
    PC11     ------> SPI3_MISO
    TBD      ------> RF24_IRQ
    */
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI3;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // TODO interrupt
}
