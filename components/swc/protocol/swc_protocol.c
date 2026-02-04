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
#define NRF24_RX_TASK_NAME          "nrf24_rx"
#define NRF24_RX_TASK_STACK_SIZE    4096
#define NRF24_RX_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)
#define NRF24_IRQ_WAIT_TIMEOUT_MS   10

/* -------------------------------------------------------------------------- */
/*                             Private variables                              */
/* -------------------------------------------------------------------------- */

static TaskHandle_t s_nrf24_rx_task_handle = NULL;

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

    /* Create NRF24 RX task */
    ret = xTaskCreate(
        _nrf24_rx_task,
        NRF24_RX_TASK_NAME,
        NRF24_RX_TASK_STACK_SIZE,
        NULL,
        NRF24_RX_TASK_PRIORITY,
        &s_nrf24_rx_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create NRF24 RX task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "swc_protocol initialized (RX task created)");
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
 * Demonstrates interrupt-driven operation using semaphores. Task blocks
 * waiting for GPIO ISR to signal semaphore, then reads status and clears flags.
 *
 * @param pvParameters Unused task parameters
 */
static void _nrf24_rx_task(void *pvParameters)
{
    (void)pvParameters;

    nrf24_t *radio1 = bsp_get_nrf24_handle(BSP_NRF24_1);
    nrf24_t *radio2 = bsp_get_nrf24_handle(BSP_NRF24_2);

    if (radio1 == NULL || radio2 == NULL) {
        ESP_LOGE(TAG, "Failed to get NRF24 device handles");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "NRF24 RX task started, waiting for interrupts...");

    uint32_t timeout_ticks = pdMS_TO_TICKS(NRF24_IRQ_WAIT_TIMEOUT_MS);
    uint32_t radio1_idle_count = 0;
    uint32_t radio2_idle_count = 0;

    while (1) {
        /* Check radio 1 for IRQ */
        nrf24_err_t err = nrf24_wait_irq(radio1, timeout_ticks);
        if (err == NRF24_OK) {
            /* Semaphore signaled - IRQ received */
            _handle_nrf24_irq(radio1, BSP_NRF24_1);
            radio1_idle_count = 0;
        } else if (err == NRF24_ERR_TIMEOUT) {
            /* Timeout - no IRQ received */
            radio1_idle_count++;
            if (radio1_idle_count % 500 == 0) {  /* Log every 5 seconds */
                ESP_LOGD(TAG, "Radio 1: No activity for %lu ms",
                         radio1_idle_count * NRF24_IRQ_WAIT_TIMEOUT_MS);
            }
        } else {
            ESP_LOGE(TAG, "Radio 1: wait_irq error %d", err);
        }

        /* Check radio 2 for IRQ */
        err = nrf24_wait_irq(radio2, timeout_ticks);
        if (err == NRF24_OK) {
            /* Semaphore signaled - IRQ received */
            _handle_nrf24_irq(radio2, BSP_NRF24_2);
            radio2_idle_count = 0;
        } else if (err == NRF24_ERR_TIMEOUT) {
            /* Timeout - no IRQ received */
            radio2_idle_count++;
            if (radio2_idle_count % 500 == 0) {  /* Log every 5 seconds */
                ESP_LOGD(TAG, "Radio 2: No activity for %lu ms",
                         radio2_idle_count * NRF24_IRQ_WAIT_TIMEOUT_MS);
            }
        } else {
            ESP_LOGE(TAG, "Radio 2: wait_irq error %d", err);
        }

        /* Task yields when waiting on semaphore, no busy-wait */
    }
}