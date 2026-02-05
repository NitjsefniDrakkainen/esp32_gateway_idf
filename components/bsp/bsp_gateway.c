/**
 * @file    bsp_gateway.c
 * @author  Your Name
 * @date    2026-01-27
 * @brief   Board Support Package implementation for ESP32-S3 Gateway.
 *
 * @details
 * This module initializes hardware peripherals including SPI bus,
 * GPIO pins, and radio transceivers (NRF24L01+).
 */

#include "hal/hal_spi.h"
#include "bsp/bsp.h"
#include "nrf24/nrf24.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* -------------------------------------------------------------------------- */
/*                              Private macros                                */
/* -------------------------------------------------------------------------- */

#define TAG "bsp_gateway"

/* NRF24 SPI clock speed (8 MHz max for NRF24L01+) */
#define BSP_NRF24_SPI_CLOCK_HZ  8000000

/* -------------------------------------------------------------------------- */
/*                              Private types                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Context structure for NRF24 HAL callbacks.
 */
typedef struct {
    hal_spi_device_handle_t spi_handle;
    gpio_num_t csn_pin;
    gpio_num_t ce_pin;
    gpio_num_t irq_pin;
} bsp_nrf24_ctx_t;

/* -------------------------------------------------------------------------- */
/*                             Private variables                              */
/* -------------------------------------------------------------------------- */

static nrf24_t s_nrf24_devices[BSP_NRF24_COUNT];
static bsp_nrf24_ctx_t s_nrf24_ctx[BSP_NRF24_COUNT];
static nrf24_hal_t s_nrf24_hal[BSP_NRF24_COUNT];

/* -------------------------------------------------------------------------- */
/*                             Private prototypes                             */
/* -------------------------------------------------------------------------- */

static esp_err_t _bsp_init_spi(void);
static esp_err_t _bsp_init_nrf24_gpio(const bsp_nrf24_ctx_t *ctx);
static esp_err_t _bsp_init_nrf24(nrf24_t *dev, nrf24_hal_t *hal, bsp_nrf24_ctx_t *ctx,
                                  gpio_num_t csn, gpio_num_t ce, gpio_num_t irq);
static esp_err_t _bsp_init_nrf24_interrupts(void);

/* NRF24 HAL callbacks */
static int _bsp_nrf24_spi_transfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len);
static void _bsp_nrf24_csn_low(void *ctx);
static void _bsp_nrf24_csn_high(void *ctx);
static void _bsp_nrf24_ce_low(void *ctx);
static void _bsp_nrf24_ce_high(void *ctx);
static void _bsp_nrf24_delay_us(uint32_t us);
static void _bsp_nrf24_delay_ms(uint32_t ms);
static int _bsp_nrf24_irq_read(void *ctx);

/**
 * @brief GPIO ISR handler for NRF24 radio 1 IRQ pin.
 *
 * Called from interrupt context when NRF24 radio 1 IRQ pin falls (active-low signal).
 * Directly signals radio 1 semaphore via nrf24_irq_handler() - no lookup overhead.
 *
 * @param arg Unused
 */
static void IRAM_ATTR _bsp_nrf24_1_gpio_isr_handler(void *arg)
{
    (void)arg;
    nrf24_irq_handler(&s_nrf24_devices[BSP_NRF24_1]);
}

/**
 * @brief GPIO ISR handler for NRF24 radio 2 IRQ pin.
 *
 * Called from interrupt context when NRF24 radio 2 IRQ pin falls (active-low signal).
 * Directly signals radio 2 semaphore via nrf24_irq_handler() - no lookup overhead.
 *
 * @param arg Unused
 */
static void IRAM_ATTR _bsp_nrf24_2_gpio_isr_handler(void *arg)
{
    (void)arg;
    nrf24_irq_handler(&s_nrf24_devices[BSP_NRF24_2]);
}

/* -------------------------------------------------------------------------- */
/*                              Public functions                              */
/* -------------------------------------------------------------------------- */

esp_err_t bsp_init(void)
{
    esp_err_t ret;

    /* Initialize SPI bus */
    ret = _bsp_init_spi();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus");
        return ret;
    }

    /* Initialize NRF24 radio 1 */
    ret = _bsp_init_nrf24(&s_nrf24_devices[BSP_NRF24_1], &s_nrf24_hal[BSP_NRF24_1],
                          &s_nrf24_ctx[BSP_NRF24_1],
                          BSP_NRF1_PIN_CSN, BSP_NRF1_PIN_CE, BSP_NRF1_PIN_IRQ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NRF24 radio 1");
        return ret;
    }
vTaskDelay(1 / portTICK_PERIOD_MS);
    /* Initialize NRF24 radio 2 */
    ret = _bsp_init_nrf24(&s_nrf24_devices[BSP_NRF24_2], &s_nrf24_hal[BSP_NRF24_2],
                          &s_nrf24_ctx[BSP_NRF24_2],
                          BSP_NRF2_PIN_CSN, BSP_NRF2_PIN_CE, BSP_NRF2_PIN_IRQ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NRF24 radio 2");
        return ret;
    }

    /* Initialize NRF24 interrupts */
    ret = _bsp_init_nrf24_interrupts();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NRF24 interrupts");
        return ret;
    }

    ESP_LOGI(TAG, "bsp_gateway initialized");
    return ESP_OK;
}

nrf24_t* bsp_get_nrf24_handle(bsp_nrf24_id_t id)
{
    if (id >= BSP_NRF24_COUNT) {
        return NULL;
    }
    return &s_nrf24_devices[id];
}

/* -------------------------------------------------------------------------- */
/*                              Private functions                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize GPIO interrupts for NRF24 IRQ pins.
 *
 * Configures edge-triggered interrupts on both NRF24 IRQ pins.
 * Must be called after nrf24_init() so device handles and semaphores exist.
 *
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
static esp_err_t _bsp_init_nrf24_interrupts(void)
{
    esp_err_t ret;

    /* Install GPIO ISR service if not already installed */
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        /* ESP_ERR_INVALID_STATE means already installed - that's OK */
        ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure interrupt for NRF24 radio 1 */
    ret = gpio_set_intr_type(BSP_NRF1_PIN_IRQ, GPIO_INTR_NEGEDGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set interrupt type for NRF24_1 IRQ pin");
        return ret;
    }

    ret = gpio_isr_handler_add(BSP_NRF1_PIN_IRQ, _bsp_nrf24_1_gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler for NRF24_1 IRQ pin");
        return ret;
    }

    ESP_LOGI(TAG, "NRF24_1 interrupt configured (IRQ pin: %d, edge-triggered)", BSP_NRF1_PIN_IRQ);

    /* Configure interrupt for NRF24 radio 2 */
    ret = gpio_set_intr_type(BSP_NRF2_PIN_IRQ, GPIO_INTR_NEGEDGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set interrupt type for NRF24_2 IRQ pin");
        /* Continue with partial setup - radio 1 will still work */
    }

    ret = gpio_isr_handler_add(BSP_NRF2_PIN_IRQ, _bsp_nrf24_2_gpio_isr_handler, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ISR handler for NRF24_2 IRQ pin");
        /* Continue - radio 1 will still work */
    } else {
        ESP_LOGI(TAG, "NRF24_2 interrupt configured (IRQ pin: %d, edge-triggered)", BSP_NRF2_PIN_IRQ);
    }

    return ESP_OK;
}

static esp_err_t _bsp_init_spi(void)
{
    hal_spi_bus_config_t bus_cfg = {
        .mosi_io_num = BSP_SPI_PIN_MOSI,
        .miso_io_num = BSP_SPI_PIN_MISO,
        .sclk_io_num = BSP_SPI_PIN_SCLK,
        .max_transfer_sz = 64,
    };

    return hal_spi_bus_init(HAL_SPI_HOST_DEFAULT, &bus_cfg);
}

static esp_err_t _bsp_init_nrf24_gpio(const bsp_nrf24_ctx_t *ctx)
{
    gpio_config_t io_conf = {0};

    /* Configure CSN and CE as outputs */
    io_conf.pin_bit_mask = (1ULL << ctx->csn_pin) | (1ULL << ctx->ce_pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Set CSN high (deselected) and CE low (standby) */
    gpio_set_level(ctx->csn_pin, 1);
    gpio_set_level(ctx->ce_pin, 0);

    /* Configure IRQ as input with pull-up */
    io_conf.pin_bit_mask = (1ULL << ctx->irq_pin);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;  /* Will be enabled later in _bsp_init_nrf24_interrupts */

    return gpio_config(&io_conf);
}

static esp_err_t _bsp_init_nrf24(nrf24_t *dev, nrf24_hal_t *hal, bsp_nrf24_ctx_t *ctx,
                                  gpio_num_t csn, gpio_num_t ce, gpio_num_t irq)
{
    esp_err_t ret;

    /* Set up context */
    ctx->csn_pin = csn;
    ctx->ce_pin = ce;
    ctx->irq_pin = irq;

    /* Initialize GPIO pins */
    ret = _bsp_init_nrf24_gpio(ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO for NRF24 (CSN=%d)", csn);
        return ret;
    }

    /* Add SPI device */
    hal_spi_device_config_t spi_cfg = {
        .clock_speed_hz = BSP_NRF24_SPI_CLOCK_HZ,
        .mode = 0,          /* NRF24 uses SPI mode 0 */
        .queue_size = 3,
    };

    ret = hal_spi_device_add(HAL_SPI_HOST_DEFAULT, &spi_cfg, &ctx->spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device for NRF24 (CSN=%d)", csn);
        return ret;
    }

    /* Set up HAL callbacks */
    hal->spi_transfer = _bsp_nrf24_spi_transfer;
    hal->csn_low = _bsp_nrf24_csn_low;
    hal->csn_high = _bsp_nrf24_csn_high;
    hal->ce_low = _bsp_nrf24_ce_low;
    hal->ce_high = _bsp_nrf24_ce_high;
    hal->delay_us = _bsp_nrf24_delay_us;
    hal->delay_ms = _bsp_nrf24_delay_ms;
    hal->irq_read = _bsp_nrf24_irq_read;
    hal->user_ctx = ctx;

    /* Default NRF24 configuration */
    nrf24_config_t nrf_cfg = {
        .channel = 76,
        .data_rate = NRF24_DR_1MBPS,
        .tx_power = NRF24_PWR_MAX,
        .crc = NRF24_CRC_16,
        .addr_width = NRF24_AW_5,
    };

    /* Initialize NRF24 driver */
    nrf24_err_t nrf_ret = nrf24_init(dev, hal, &nrf_cfg);
    if (nrf_ret != NRF24_OK) {
        ESP_LOGE(TAG, "Failed to initialize NRF24 driver (CSN=%d): %d", csn, nrf_ret);
        hal_spi_device_remove(ctx->spi_handle);
        return ESP_FAIL;
    }

    /* Verify communication by reading status register */
    uint8_t status;
    nrf_ret = nrf24_get_status(dev, &status);
    if (nrf_ret != NRF24_OK) {
        ESP_LOGE(TAG, "Failed to read NRF24 status (CSN=%d)", csn);
        nrf24_deinit(dev);
        hal_spi_device_remove(ctx->spi_handle);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "NRF24 initialized (CSN=%d, CE=%d, IRQ=%d, status=0x%02X)",
             csn, ce, irq, status);
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                          NRF24 HAL Callbacks                               */
/* -------------------------------------------------------------------------- */

static int _bsp_nrf24_spi_transfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    bsp_nrf24_ctx_t *nrf_ctx = (bsp_nrf24_ctx_t *)ctx;
    esp_err_t ret = hal_spi_transfer(nrf_ctx->spi_handle, tx, rx, len);
    return (ret == ESP_OK) ? 0 : -1;
}

static void _bsp_nrf24_csn_low(void *ctx)
{
    bsp_nrf24_ctx_t *nrf_ctx = (bsp_nrf24_ctx_t *)ctx;
    gpio_set_level(nrf_ctx->csn_pin, 0);
}

static void _bsp_nrf24_csn_high(void *ctx)
{
    bsp_nrf24_ctx_t *nrf_ctx = (bsp_nrf24_ctx_t *)ctx;
    gpio_set_level(nrf_ctx->csn_pin, 1);
}

static void _bsp_nrf24_ce_low(void *ctx)
{
    bsp_nrf24_ctx_t *nrf_ctx = (bsp_nrf24_ctx_t *)ctx;
    gpio_set_level(nrf_ctx->ce_pin, 0);
}

static void _bsp_nrf24_ce_high(void *ctx)
{
    bsp_nrf24_ctx_t *nrf_ctx = (bsp_nrf24_ctx_t *)ctx;
    gpio_set_level(nrf_ctx->ce_pin, 1);
}

static void _bsp_nrf24_delay_us(uint32_t us)
{
    ets_delay_us(us);
}

static void _bsp_nrf24_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static int _bsp_nrf24_irq_read(void *ctx)
{
    bsp_nrf24_ctx_t *nrf_ctx = (bsp_nrf24_ctx_t *)ctx;
    return gpio_get_level(nrf_ctx->irq_pin);
}
