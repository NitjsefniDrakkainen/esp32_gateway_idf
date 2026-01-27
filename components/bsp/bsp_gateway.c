/**
 * @file    bsp_gateway.c
 * @author  Your Name
 * @date    2026-01-27
 * @brief   Short description of what this module does.
 *
 * @details
 * Detailed explanation of the module purpose, internal structure,
 * and relationships with other components (if needed).
 */

#include "hal/hal_spi.h"
#include "bsp/bsp.h"
#include "esp_log.h"

/* -------------------------------------------------------------------------- */
/*                              Private macros                                */
/* -------------------------------------------------------------------------- */

#define TAG "bsp_gateway"

/* -------------------------------------------------------------------------- */
/*                             Private prototypes                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Example of a private function.
 */
static void _bsp_gateway_private_function(void);

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
esp_err_t bsp_init(void)
{
    _bsp_gateway_private_function();
    ESP_LOGI(TAG, "bsp_gateway initialized");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                              Private functions                              */
/* -------------------------------------------------------------------------- */

static void _bsp_gateway_private_function(void)
{
    hal_spi_init();
    ESP_LOGD(TAG, "Private function executed");
}