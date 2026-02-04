/**
 * @file    swc_protocol.c
 * @author  Your Name
 * @date    2026-01-27
 * @brief   Short description of what this module does.
 *
 * @details
 * Detailed explanation of the module purpose, internal structure,
 * and relationships with other components (if needed).
 */

#include "swc_protocol/swc_protocol.h"
#include "esp_log.h"
#include "bsp/bsp.h"
#include "nrf24/nrf24.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* -------------------------------------------------------------------------- */
/*                              Private macros                                */
/* -------------------------------------------------------------------------- */

#define TAG "swc_protocol"

/* Task configuration */
#define NRF24_RX_TASK_1_NAME        "nrf24_rx_1"
#define NRF24_RX_TASK_2_NAME        "nrf24_rx_2"
#define NRF24_RX_TASK_STACK_SIZE    4096
#define NRF24_RX_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

/* -------------------------------------------------------------------------- */
/*                             Private variables                              */
/* -------------------------------------------------------------------------- */

static TaskHandle_t s_nrf24_rx_task_handles[BSP_NRF24_COUNT] = {NULL};

/* -------------------------------------------------------------------------- */
/*                             Private prototypes                             */
/* -------------------------------------------------------------------------- */

static void _nrf24_rx_task(void *pvParameters);
static void _handle_nrf24_irq(nrf24_t *dev, bsp_nrf24_id_t radio_id);

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
esp_err_t swc_protocol_init(void)
{
    BaseType_t ret;

    /* Create NRF24 RX task for radio 1 */
    ret = xTaskCreate(
        _nrf24_rx_task,
        NRF24_RX_TASK_1_NAME,
        NRF24_RX_TASK_STACK_SIZE,
        (void*)BSP_NRF24_1,
        NRF24_RX_TASK_PRIORITY,
        &s_nrf24_rx_task_handles[BSP_NRF24_1]
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create NRF24 RX task for radio 1");
        return ESP_FAIL;
    }

    /* Create NRF24 RX task for radio 2 */
    ret = xTaskCreate(
        _nrf24_rx_task,
        NRF24_RX_TASK_2_NAME,
        NRF24_RX_TASK_STACK_SIZE,
        (void*)BSP_NRF24_2,
        NRF24_RX_TASK_PRIORITY,
        &s_nrf24_rx_task_handles[BSP_NRF24_2]
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create NRF24 RX task for radio 2");
        /* Clean up radio 1 task */
        vTaskDelete(s_nrf24_rx_task_handles[BSP_NRF24_1]);
        s_nrf24_rx_task_handles[BSP_NRF24_1] = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "swc_protocol initialized (2 RX tasks created)");
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/*                              Private functions                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Handle NRF24 IRQ event for a specific radio.
 *
 * Reads status register to identify interrupt source, logs it,
 * and clears all IRQ flags.
 *
 * @param dev Pointer to NRF24 device handle
 * @param radio_id Radio identifier (BSP_NRF24_1 or BSP_NRF24_2) for logging
 */
static void _handle_nrf24_irq(nrf24_t *dev, bsp_nrf24_id_t radio_id)
{
    uint8_t status = 0;
    nrf24_err_t err;

    /* Read status register to determine interrupt source */
    err = nrf24_get_status(dev, &status);
    if (err != NRF24_OK) {
        ESP_LOGW(TAG, "Radio %d: Failed to read status register (err=%d)", radio_id, err);
        return;
    }

    /* Log interrupt source(s) */
    if (status & NRF24_STATUS_RX_DR) {
        ESP_LOGI(TAG, "Radio %d: RX_DR interrupt (data ready) - status=0x%02X", radio_id, status);
    }
    if (status & NRF24_STATUS_TX_DS) {
        ESP_LOGI(TAG, "Radio %d: TX_DS interrupt (data sent) - status=0x%02X", radio_id, status);
    }
    if (status & NRF24_STATUS_MAX_RT) {
        ESP_LOGW(TAG, "Radio %d: MAX_RT interrupt (max retransmits) - status=0x%02X", radio_id, status);
    }

    /* Check for spurious interrupt (no flags set) */
    if ((status & (NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) == 0) {
        ESP_LOGW(TAG, "Radio %d: Spurious interrupt (no IRQ flags set) - status=0x%02X", radio_id, status);
    }

    /* Clear all interrupt flags */
    err = nrf24_clear_irq(dev, NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);
    if (err != NRF24_OK) {
        ESP_LOGE(TAG, "Radio %d: Failed to clear IRQ flags (err=%d)", radio_id, err);
    }
}

/**
 * @brief NRF24 RX task - waits for IRQ events and processes them.
 *
 * Dedicated task per radio for true parallel processing. Task blocks
 * waiting for GPIO ISR to signal semaphore, then reads status and clears flags.
 *
 * @param pvParameters Radio ID (bsp_nrf24_id_t) cast to void*
 */
static void _nrf24_rx_task(void *pvParameters)
{
    bsp_nrf24_id_t radio_id = (bsp_nrf24_id_t)(int)pvParameters;
    nrf24_t *radio = bsp_get_nrf24_handle(radio_id);

    if (radio == NULL) {
        ESP_LOGE(TAG, "Radio %d: Failed to get device handle", radio_id);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Radio %d: RX task started, waiting for interrupts...", radio_id);

    while (1) {
        /* Wait indefinitely for IRQ - dedicated task, no timeout needed */
        nrf24_err_t err = nrf24_wait_irq(radio, portMAX_DELAY);

        if (err == NRF24_OK) {
            /* Semaphore signaled - IRQ received */
            _handle_nrf24_irq(radio, radio_id);
        } else {
            /* Should never happen with portMAX_DELAY, but handle errors */
            ESP_LOGE(TAG, "Radio %d: wait_irq error %d", radio_id, err);
            vTaskDelay(pdMS_TO_TICKS(100));  /* Brief delay before retry */
        }
    }
}