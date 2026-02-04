/**
 * @file    hal_spi.h
 * @author  Your Name
 * @date    2026-01-27
 * @brief   SPI Hardware Abstraction Layer for ESP-IDF.
 *
 * @details
 * This header provides an abstraction layer for SPI bus and device management.
 * It wraps ESP-IDF SPI driver functions to provide a cleaner interface for
 * the upper layers.
 */

#ifndef __HAL_SPI_H__
#define __HAL_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* -------------------------------------------------------------------------- */
/*                                Public macros                               */
/* -------------------------------------------------------------------------- */

#define HAL_SPI_HOST_DEFAULT    SPI2_HOST

/* -------------------------------------------------------------------------- */
/*                                Public types                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief SPI bus configuration structure.
 */
typedef struct {
    int mosi_io_num;        /**< GPIO pin for MOSI, or -1 if not used */
    int miso_io_num;        /**< GPIO pin for MISO, or -1 if not used */
    int sclk_io_num;        /**< GPIO pin for SCLK */
    int max_transfer_sz;    /**< Maximum transfer size in bytes (0 = default 4096) */
} hal_spi_bus_config_t;

/**
 * @brief SPI device configuration structure.
 */
typedef struct {
    int clock_speed_hz;     /**< SPI clock speed in Hz */
    uint8_t mode;           /**< SPI mode (0-3) */
    int queue_size;         /**< Transaction queue size */
} hal_spi_device_config_t;

/**
 * @brief Opaque SPI device handle.
 */
typedef struct hal_spi_device* hal_spi_device_handle_t;

/**
 * @brief Internal SPI bus descriptor with mutex protection.
 * @note This structure is opaque to users. It provides bus-level
 *       synchronization to prevent concurrent access to the same
 *       SPI bus from multiple devices.
 */
struct hal_spi_bus_descriptor {
    spi_host_device_t host;         /**< SPI host identifier */
    SemaphoreHandle_t bus_mutex;    /**< Mutex for bus access protection */
    bool initialized;               /**< Bus initialization status */
};

/* -------------------------------------------------------------------------- */
/*                            Public API functions                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the SPI module (deprecated, use hal_spi_bus_init instead).
 *
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t hal_spi_init(void);

/**
 * @brief Initialize an SPI bus.
 *
 * @param host SPI host (SPI2_HOST or SPI3_HOST)
 * @param config Pointer to bus configuration
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t hal_spi_bus_init(spi_host_device_t host, const hal_spi_bus_config_t *config);

/**
 * @brief Deinitialize an SPI bus.
 *
 * @param host SPI host to deinitialize
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t hal_spi_bus_deinit(spi_host_device_t host);

/**
 * @brief Add a device to an SPI bus.
 *
 * @param host SPI host
 * @param config Pointer to device configuration
 * @param handle Pointer to store the device handle
 * @return esp_err_t ESP_OK on success, error code otherwise.
 *
 * @note CS pin is not managed by the driver - must be controlled externally.
 */
esp_err_t hal_spi_device_add(spi_host_device_t host, const hal_spi_device_config_t *config,
                             hal_spi_device_handle_t *handle);

/**
 * @brief Remove a device from an SPI bus.
 *
 * @param handle Device handle to remove
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t hal_spi_device_remove(hal_spi_device_handle_t handle);

/**
 * @brief Perform a full-duplex SPI transfer.
 *
 * @param handle Device handle
 * @param tx_data Pointer to transmit data buffer
 * @param rx_data Pointer to receive data buffer (can be NULL if receive not needed)
 * @param len Number of bytes to transfer
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t hal_spi_transfer(hal_spi_device_handle_t handle, const uint8_t *tx_data,
                           uint8_t *rx_data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* __HAL_SPI_H__ */
