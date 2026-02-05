/**
 * @file    bsp.h
 * @author  Your Name
 * @date    2026-01-27
 * @brief   Board Support Package for ESP32-S3 Gateway.
 *
 * @details
 * This header defines hardware pin mappings and provides access to
 * initialized peripheral handles for the gateway board.
 */

#ifndef __BSP_H__
#define __BSP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "nrf24/nrf24.h"

/* -------------------------------------------------------------------------- */
/*                                Public macros                               */
/* -------------------------------------------------------------------------- */

/* SPI bus pins (shared by all SPI devices) */
#define BSP_SPI_PIN_MOSI    9
#define BSP_SPI_PIN_MISO    8
#define BSP_SPI_PIN_SCLK    7

/* NRF24 radio 1 pins */
#define BSP_NRF1_PIN_CSN    2
#define BSP_NRF1_PIN_CE     1
#define BSP_NRF1_PIN_IRQ    3

/* NRF24 radio 2 pins */
#define BSP_NRF2_PIN_CSN    5
#define BSP_NRF2_PIN_CE     4
#define BSP_NRF2_PIN_IRQ    6

/* CC1101 sub-GHz radio pins */
#define BSP_CC1101_PIN_CSN  43
#define BSP_CC1101_PIN_IRQ  44

/* Legacy pin definitions (for backward compatibility) */
#define NRF1_CS     BSP_NRF1_PIN_CSN
#define NRF1_CE     BSP_NRF1_PIN_CE
#define NRF1_IRQ    BSP_NRF1_PIN_IRQ
#define NRF2_CS     BSP_NRF2_PIN_CSN
#define NRF2_CE     BSP_NRF2_PIN_CE
#define NRF2_IRQ    BSP_NRF2_PIN_IRQ
#define CC1101_CS   BSP_CC1101_PIN_CSN
#define CC1101_IRQ  BSP_CC1101_PIN_IRQ

/* -------------------------------------------------------------------------- */
/*                                Public types                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief NRF24 radio identifier enumeration.
 */
typedef enum {
    BSP_NRF24_1 = 0,    /**< NRF24 radio 1 */
    BSP_NRF24_2,        /**< NRF24 radio 2 */
    BSP_NRF24_COUNT     /**< Total number of NRF24 radios */
} bsp_nrf24_id_t;

/* -------------------------------------------------------------------------- */
/*                            Public API functions                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the board support package.
 *
 * Initializes SPI bus, GPIO pins, and radio transceivers.
 *
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t bsp_init(void);

/**
 * @brief Get handle to an NRF24 radio by ID.
 *
 * @param id NRF24 radio identifier (BSP_NRF24_1 or BSP_NRF24_2)
 * @return Pointer to NRF24 driver handle, or NULL if invalid ID or not initialized.
 */
nrf24_t* bsp_get_nrf24_handle(bsp_nrf24_id_t id);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H__ */
