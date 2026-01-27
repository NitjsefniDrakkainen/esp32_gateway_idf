/**
 * @file    hal_spi.c
 * @author  Your Name
 * @date    2026-01-24
 * @brief   Short description of what this module does.
 *
 * @details
 * Detailed explanation of the module purpose, internal structure,
 * and relationships with other components (if needed).
 */

#include "hal/hal_spi.h"
#include "esp_err.h"
#include "esp_log.h"

/* -------------------------------------------------------------------------- */
/*                              Private macros                                */
/* -------------------------------------------------------------------------- */

#define TAG "hal_spi"

/* -------------------------------------------------------------------------- */
/*                             Private prototypes                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Example of a private function.
 */
static void _hal_spi_private_function(void);

/* -------------------------------------------------------------------------- */
/*                              Public functions                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the module.
 *
 * This function sets up any required peripherals or data structures.
 *
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t hal_spi_init(void)
{
    _hal_spi_private_function();
    ESP_LOGI(TAG, "hal_spi initialized");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                              Private functions                              */
/* -------------------------------------------------------------------------- */

static void _hal_spi_private_function(void)
{
    ESP_LOGD(TAG, "Private function executed");
}