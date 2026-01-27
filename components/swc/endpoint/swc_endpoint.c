/**
 * @file    swc_endpoint.c
 * @author  Your Name
 * @date    2026-01-27
 * @brief   Short description of what this module does.
 *
 * @details
 * Detailed explanation of the module purpose, internal structure,
 * and relationships with other components (if needed).
 */

 #include "device_service/device_service.h"
#include "swc_endpoint/swc_endpoint.h"
#include "esp_log.h"

/* -------------------------------------------------------------------------- */
/*                              Private macros                                */
/* -------------------------------------------------------------------------- */

#define TAG "swc_endpoint"

/* -------------------------------------------------------------------------- */
/*                             Private prototypes                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Example of a private function.
 */
static void _swc_endpoint_private_function(void);

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
esp_err_t swc_endpoint_init(void)
{
    _swc_endpoint_private_function();
    ESP_LOGI(TAG, "swc_endpoint initialized");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                              Private functions                              */
/* -------------------------------------------------------------------------- */

static void _swc_endpoint_private_function(void)
{
    device_service_init();
    ESP_LOGD(TAG, "Private function executed");
}