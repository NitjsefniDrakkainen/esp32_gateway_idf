/**
 * @file    hal_spi.c
 * @author  Your Name
 * @date    2026-01-24
 * @brief   SPI Hardware Abstraction Layer implementation for ESP-IDF.
 *
 * @details
 * This module wraps ESP-IDF SPI master driver functions to provide
 * a hardware abstraction layer for SPI communication.
 */

#include "hal/hal_spi.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/*                              Private macros                                */
/* -------------------------------------------------------------------------- */

#define TAG "hal_spi"

/* Maximum number of SPI hosts on ESP32-S3 */
#define HAL_SPI_MAX_HOSTS   3

/* -------------------------------------------------------------------------- */
/*                              Private types                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Internal SPI device structure.
 */
struct hal_spi_device {
    spi_device_handle_t spi_handle;     /**< ESP-IDF SPI device handle */
    spi_host_device_t bus_host;         /**< Parent bus host */
};

/* -------------------------------------------------------------------------- */
/*                              Private variables                             */
/* -------------------------------------------------------------------------- */

/* Bus descriptor tracking array */
static struct hal_spi_bus_descriptor s_bus_descriptors[HAL_SPI_MAX_HOSTS] = {0};

/* -------------------------------------------------------------------------- */
/*                              Public functions                              */
/* -------------------------------------------------------------------------- */

esp_err_t hal_spi_init(void)
{
    ESP_LOGI(TAG, "hal_spi initialized (legacy)");
    return ESP_OK;
}

esp_err_t hal_spi_bus_init(spi_host_device_t host, const hal_spi_bus_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = config->mosi_io_num,
        .miso_io_num = config->miso_io_num,
        .sclk_io_num = config->sclk_io_num,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = config->max_transfer_sz > 0 ? config->max_transfer_sz : 4096,
    };

    esp_err_t ret = spi_bus_initialize(host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create bus mutex for thread-safe access */
    s_bus_descriptors[host].bus_mutex = xSemaphoreCreateMutex();
    if (s_bus_descriptors[host].bus_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create bus mutex for host %d", host);
        spi_bus_free(host);
        return ESP_ERR_NO_MEM;
    }

    s_bus_descriptors[host].host = host;
    s_bus_descriptors[host].initialized = true;

    ESP_LOGI(TAG, "SPI bus %d initialized (MOSI=%d, MISO=%d, SCLK=%d)",
             host, config->mosi_io_num, config->miso_io_num, config->sclk_io_num);
    return ESP_OK;
}

esp_err_t hal_spi_bus_deinit(spi_host_device_t host)
{
    /* Delete bus mutex before freeing the bus */
    if (s_bus_descriptors[host].initialized && s_bus_descriptors[host].bus_mutex != NULL) {
        vSemaphoreDelete(s_bus_descriptors[host].bus_mutex);
        s_bus_descriptors[host].bus_mutex = NULL;
        s_bus_descriptors[host].initialized = false;
    }

    esp_err_t ret = spi_bus_free(host);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to deinitialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPI bus %d deinitialized", host);
    return ESP_OK;
}

esp_err_t hal_spi_device_add(spi_host_device_t host, const hal_spi_device_config_t *config,
                             hal_spi_device_handle_t *handle)
{
    if (config == NULL || handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    struct hal_spi_device *dev = malloc(sizeof(struct hal_spi_device));
    if (dev == NULL) {
        ESP_LOGE(TAG, "Failed to allocate device handle");
        return ESP_ERR_NO_MEM;
    }

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = config->clock_speed_hz,
        .mode = config->mode,
        .spics_io_num = -1,             /* CS managed externally */
        .queue_size = config->queue_size > 0 ? config->queue_size : 1,
        .flags = 0,
    };

    esp_err_t ret = spi_bus_add_device(host, &dev_cfg, &dev->spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        free(dev);
        return ret;
    }

    /* Store parent bus host for mutex lookup */
    dev->bus_host = host;

    *handle = dev;
    ESP_LOGI(TAG, "SPI device added (clock=%d Hz, mode=%d)", config->clock_speed_hz, config->mode);
    return ESP_OK;
}

esp_err_t hal_spi_device_remove(hal_spi_device_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = spi_bus_remove_device(handle->spi_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove SPI device: %s", esp_err_to_name(ret));
        return ret;
    }

    free(handle);
    ESP_LOGI(TAG, "SPI device removed");
    return ESP_OK;
}

esp_err_t hal_spi_transfer(hal_spi_device_handle_t handle, const uint8_t *tx_data,
                           uint8_t *rx_data, size_t len)
{
    if (handle == NULL || tx_data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Get bus descriptor for mutex */
    spi_host_device_t bus_host = handle->bus_host;
    if (bus_host >= HAL_SPI_MAX_HOSTS || !s_bus_descriptors[bus_host].initialized) {
        ESP_LOGE(TAG, "Invalid bus host or bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    SemaphoreHandle_t bus_mutex = s_bus_descriptors[bus_host].bus_mutex;
    if (bus_mutex == NULL) {
        ESP_LOGE(TAG, "Bus mutex is NULL");
        return ESP_ERR_INVALID_STATE;
    }

    /* Lock bus mutex for exclusive access */
    if (xSemaphoreTake(bus_mutex, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire bus mutex");
        return ESP_ERR_TIMEOUT;
    }

    spi_transaction_t trans = {
        .length = len * 8,              /* Length in bits */
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };

    /* Perform SPI transfer */
    esp_err_t ret = spi_device_polling_transmit(handle->spi_handle, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI transfer failed: %s", esp_err_to_name(ret));
    }

    /* Always unlock bus mutex */
    xSemaphoreGive(bus_mutex);

    return ret;
}
