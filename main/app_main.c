/**
 * @file    app_main.c
 * @author  Your Name
 * @date    2026-01-22
 * @brief   Short description of what this module does.
 *
 * @details
 * Detailed explanation of the module purpose, internal structure,
 * and relationships with other components (if needed).
 */

// #include "app_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

/* -------------------------------------------------------------------------- */
/*                              Private macros                                */
/* -------------------------------------------------------------------------- */

#define TAG "app_main"

/* -------------------------------------------------------------------------- */
/*                             Private prototypes                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Example of a private function.
 */
static void _app_main_private_function(void);

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
esp_err_t app_main_init(void)
{
    ESP_LOGI(TAG, "app_main initialized");
    _app_main_private_function();
    return ESP_OK;
}

void app_main(void)
{
    (void)app_main_init();
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* -------------------------------------------------------------------------- */
/*                              Private functions                              */
/* -------------------------------------------------------------------------- */

static void _app_main_private_function(void)
{
    ESP_LOGD(TAG, "Private function executed");
}