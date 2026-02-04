/**
 * @file    nrf24.h
 * @author  Your Name
 * @date    2026-02-03
 * @brief   NRF24L01/NRF24L01+ driver - MCU independent implementation.
 *
 * @details
 * This driver provides a hardware-independent interface for the NRF24L01
 * 2.4GHz transceiver. All hardware operations (SPI, GPIO, delays) are
 * abstracted through callbacks provided at initialization.
 *
 * Features:
 * - FreeRTOS support with mutex and semaphore (optional via NRF24_USE_FREERTOS)
 * - Bare metal compatible when FreeRTOS support disabled
 * - Enhanced ShockBurst support (auto-ack, retransmit, dynamic payload)
 * - MultiCeiver support (6 data pipes)
 */

#ifndef NRF24_H
#define NRF24_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* -------------------------------------------------------------------------- */
/*                          FreeRTOS / Bare Metal Abstraction                 */
/* -------------------------------------------------------------------------- */

#if defined(NRF24_USE_FREERTOS) && NRF24_USE_FREERTOS
    #include "freertos/FreeRTOS.h"
    #include "freertos/semphr.h"
    typedef SemaphoreHandle_t nrf24_mutex_t;
    typedef SemaphoreHandle_t nrf24_sem_t;
    #define NRF24_MUTEX_CREATE()        xSemaphoreCreateMutex()
    #define NRF24_MUTEX_TAKE(m, t)      (xSemaphoreTake((m), (t)) == pdTRUE)
    #define NRF24_MUTEX_GIVE(m)         xSemaphoreGive(m)
    #define NRF24_MUTEX_DELETE(m)       vSemaphoreDelete(m)
    #define NRF24_SEM_CREATE()          xSemaphoreCreateBinary()
    #define NRF24_SEM_GIVE(s)           xSemaphoreGive(s)
    #define NRF24_SEM_GIVE_ISR(s, w)    xSemaphoreGiveFromISR((s), (w))
    #define NRF24_SEM_TAKE(s, t)        (xSemaphoreTake((s), (t)) == pdTRUE)
    #define NRF24_SEM_DELETE(s)         vSemaphoreDelete(s)
    #define NRF24_WAIT_FOREVER          portMAX_DELAY
#else
    /* Bare metal - no-op stubs */
    typedef void* nrf24_mutex_t;
    typedef void* nrf24_sem_t;
    #define NRF24_MUTEX_CREATE()        ((void*)1)
    #define NRF24_MUTEX_TAKE(m, t)      (1)
    #define NRF24_MUTEX_GIVE(m)         ((void)0)
    #define NRF24_MUTEX_DELETE(m)       ((void)0)
    #define NRF24_SEM_CREATE()          ((void*)1)
    #define NRF24_SEM_GIVE(s)           ((void)0)
    #define NRF24_SEM_GIVE_ISR(s, w)    ((void)0)
    #define NRF24_SEM_TAKE(s, t)        (1)
    #define NRF24_SEM_DELETE(s)         ((void)0)
    #define NRF24_WAIT_FOREVER          0xFFFFFFFFUL
#endif

/* -------------------------------------------------------------------------- */
/*                              Register Map                                  */
/* -------------------------------------------------------------------------- */

/* Configuration registers */
#define NRF24_REG_CONFIG        0x00    /**< Configuration register */
#define NRF24_REG_EN_AA         0x01    /**< Enable auto acknowledgment */
#define NRF24_REG_EN_RXADDR     0x02    /**< Enabled RX addresses */
#define NRF24_REG_SETUP_AW      0x03    /**< Setup address widths */
#define NRF24_REG_SETUP_RETR    0x04    /**< Setup automatic retransmission */
#define NRF24_REG_RF_CH         0x05    /**< RF channel */
#define NRF24_REG_RF_SETUP      0x06    /**< RF setup register */
#define NRF24_REG_STATUS        0x07    /**< Status register */
#define NRF24_REG_OBSERVE_TX    0x08    /**< Transmit observe register */
#define NRF24_REG_RPD           0x09    /**< Received power detector */

/* Address registers */
#define NRF24_REG_RX_ADDR_P0    0x0A    /**< Receive address data pipe 0 */
#define NRF24_REG_RX_ADDR_P1    0x0B    /**< Receive address data pipe 1 */
#define NRF24_REG_RX_ADDR_P2    0x0C    /**< Receive address data pipe 2 */
#define NRF24_REG_RX_ADDR_P3    0x0D    /**< Receive address data pipe 3 */
#define NRF24_REG_RX_ADDR_P4    0x0E    /**< Receive address data pipe 4 */
#define NRF24_REG_RX_ADDR_P5    0x0F    /**< Receive address data pipe 5 */
#define NRF24_REG_TX_ADDR       0x10    /**< Transmit address */

/* Payload width registers */
#define NRF24_REG_RX_PW_P0      0x11    /**< RX payload width pipe 0 */
#define NRF24_REG_RX_PW_P1      0x12    /**< RX payload width pipe 1 */
#define NRF24_REG_RX_PW_P2      0x13    /**< RX payload width pipe 2 */
#define NRF24_REG_RX_PW_P3      0x14    /**< RX payload width pipe 3 */
#define NRF24_REG_RX_PW_P4      0x15    /**< RX payload width pipe 4 */
#define NRF24_REG_RX_PW_P5      0x16    /**< RX payload width pipe 5 */

/* FIFO and feature registers */
#define NRF24_REG_FIFO_STATUS   0x17    /**< FIFO status register */
#define NRF24_REG_DYNPD         0x1C    /**< Enable dynamic payload length */
#define NRF24_REG_FEATURE       0x1D    /**< Feature register */

/* -------------------------------------------------------------------------- */
/*                              Register Bits                                 */
/* -------------------------------------------------------------------------- */

/* CONFIG register bits */
#define NRF24_CONFIG_MASK_RX_DR     (1 << 6)    /**< Mask RX_DR interrupt */
#define NRF24_CONFIG_MASK_TX_DS     (1 << 5)    /**< Mask TX_DS interrupt */
#define NRF24_CONFIG_MASK_MAX_RT    (1 << 4)    /**< Mask MAX_RT interrupt */
#define NRF24_CONFIG_EN_CRC         (1 << 3)    /**< Enable CRC */
#define NRF24_CONFIG_CRCO           (1 << 2)    /**< CRC encoding scheme (0=1byte, 1=2bytes) */
#define NRF24_CONFIG_PWR_UP         (1 << 1)    /**< Power up */
#define NRF24_CONFIG_PRIM_RX        (1 << 0)    /**< RX/TX control (1=PRX, 0=PTX) */

/* STATUS register bits */
#define NRF24_STATUS_RX_DR          (1 << 6)    /**< Data ready RX FIFO interrupt */
#define NRF24_STATUS_TX_DS          (1 << 5)    /**< Data sent TX FIFO interrupt */
#define NRF24_STATUS_MAX_RT         (1 << 4)    /**< Max retransmissions interrupt */
#define NRF24_STATUS_RX_P_NO_MASK   (0x07 << 1) /**< Data pipe number for RX FIFO */
#define NRF24_STATUS_RX_P_NO_SHIFT  1
#define NRF24_STATUS_TX_FULL        (1 << 0)    /**< TX FIFO full flag */

/* RF_SETUP register bits */
#define NRF24_RF_SETUP_CONT_WAVE    (1 << 7)    /**< Continuous carrier transmit */
#define NRF24_RF_SETUP_RF_DR_LOW    (1 << 5)    /**< Set RF data rate to 250kbps */
#define NRF24_RF_SETUP_PLL_LOCK     (1 << 4)    /**< Force PLL lock signal */
#define NRF24_RF_SETUP_RF_DR_HIGH   (1 << 3)    /**< Select between 1Mbps or 2Mbps */
#define NRF24_RF_SETUP_RF_PWR_MASK  (0x03 << 1) /**< RF output power in TX mode */
#define NRF24_RF_SETUP_RF_PWR_SHIFT 1

/* FIFO_STATUS register bits */
#define NRF24_FIFO_STATUS_TX_REUSE  (1 << 6)    /**< Reuse last transmitted payload */
#define NRF24_FIFO_STATUS_TX_FULL   (1 << 5)    /**< TX FIFO full flag */
#define NRF24_FIFO_STATUS_TX_EMPTY  (1 << 4)    /**< TX FIFO empty flag */
#define NRF24_FIFO_STATUS_RX_FULL   (1 << 1)    /**< RX FIFO full flag */
#define NRF24_FIFO_STATUS_RX_EMPTY  (1 << 0)    /**< RX FIFO empty flag */

/* FEATURE register bits */
#define NRF24_FEATURE_EN_DPL        (1 << 2)    /**< Enable dynamic payload length */
#define NRF24_FEATURE_EN_ACK_PAY    (1 << 1)    /**< Enable payload with ACK */
#define NRF24_FEATURE_EN_DYN_ACK    (1 << 0)    /**< Enable W_TX_PAYLOAD_NOACK */

/* -------------------------------------------------------------------------- */
/*                              SPI Commands                                  */
/* -------------------------------------------------------------------------- */

#define NRF24_CMD_R_REGISTER        0x00    /**< Read register (OR with register addr) */
#define NRF24_CMD_W_REGISTER        0x20    /**< Write register (OR with register addr) */
#define NRF24_CMD_R_RX_PAYLOAD      0x61    /**< Read RX payload */
#define NRF24_CMD_W_TX_PAYLOAD      0xA0    /**< Write TX payload */
#define NRF24_CMD_FLUSH_TX          0xE1    /**< Flush TX FIFO */
#define NRF24_CMD_FLUSH_RX          0xE2    /**< Flush RX FIFO */
#define NRF24_CMD_REUSE_TX_PL       0xE3    /**< Reuse last transmitted payload */
#define NRF24_CMD_R_RX_PL_WID       0x60    /**< Read RX payload width */
#define NRF24_CMD_W_ACK_PAYLOAD     0xA8    /**< Write payload for ACK (OR with pipe) */
#define NRF24_CMD_W_TX_PAYLOAD_NOACK 0xB0   /**< Write TX payload, disable auto-ack */
#define NRF24_CMD_NOP               0xFF    /**< No operation (read STATUS) */

/* -------------------------------------------------------------------------- */
/*                              Constants                                     */
/* -------------------------------------------------------------------------- */

#define NRF24_MAX_PAYLOAD_SIZE      32      /**< Maximum payload size in bytes */
#define NRF24_MAX_CHANNEL           125     /**< Maximum RF channel number */
#define NRF24_PIPE_COUNT            6       /**< Number of data pipes */

/* IRQ flags for nrf24_clear_irq() */
#define NRF24_IRQ_RX_DR             NRF24_STATUS_RX_DR
#define NRF24_IRQ_TX_DS             NRF24_STATUS_TX_DS
#define NRF24_IRQ_MAX_RT            NRF24_STATUS_MAX_RT
#define NRF24_IRQ_ALL               (NRF24_IRQ_RX_DR | NRF24_IRQ_TX_DS | NRF24_IRQ_MAX_RT)

/* -------------------------------------------------------------------------- */
/*                              Type Definitions                              */
/* -------------------------------------------------------------------------- */

/**
 * @brief Driver error codes.
 */
typedef enum {
    NRF24_OK = 0,           /**< Success */
    NRF24_ERR_INVALID_ARG,  /**< Invalid argument */
    NRF24_ERR_NOT_INIT,     /**< Driver not initialized */
    NRF24_ERR_TIMEOUT,      /**< Operation timeout */
    NRF24_ERR_SPI,          /**< SPI transfer failed */
    NRF24_ERR_BUSY,         /**< Device busy */
    NRF24_ERR_NO_DATA,      /**< No data available */
    NRF24_ERR_MAX_RT,       /**< Max retransmissions reached */
    NRF24_ERR_MUTEX         /**< Mutex acquire failed */
} nrf24_err_t;

/**
 * @brief Data rate options.
 */
typedef enum {
    NRF24_DR_250KBPS = 0,   /**< 250 kbps */
    NRF24_DR_1MBPS,         /**< 1 Mbps */
    NRF24_DR_2MBPS          /**< 2 Mbps */
} nrf24_data_rate_t;

/**
 * @brief TX power options.
 */
typedef enum {
    NRF24_PWR_MIN = 0,      /**< -18 dBm */
    NRF24_PWR_LOW,          /**< -12 dBm */
    NRF24_PWR_HIGH,         /**< -6 dBm */
    NRF24_PWR_MAX           /**< 0 dBm */
} nrf24_tx_power_t;

/**
 * @brief Address width options.
 */
typedef enum {
    NRF24_AW_3 = 1,         /**< 3 bytes address */
    NRF24_AW_4 = 2,         /**< 4 bytes address */
    NRF24_AW_5 = 3          /**< 5 bytes address */
} nrf24_addr_width_t;

/**
 * @brief CRC configuration options.
 */
typedef enum {
    NRF24_CRC_OFF = 0,      /**< CRC disabled */
    NRF24_CRC_8,            /**< 1 byte CRC */
    NRF24_CRC_16            /**< 2 byte CRC */
} nrf24_crc_t;

/**
 * @brief Operating modes.
 */
typedef enum {
    NRF24_MODE_POWER_DOWN = 0,  /**< Power down mode */
    NRF24_MODE_STANDBY,         /**< Standby-I mode */
    NRF24_MODE_RX,              /**< RX mode */
    NRF24_MODE_TX               /**< TX mode */
} nrf24_mode_t;

/**
 * @brief Data pipe identifiers.
 */
typedef enum {
    NRF24_PIPE_0 = 0,
    NRF24_PIPE_1,
    NRF24_PIPE_2,
    NRF24_PIPE_3,
    NRF24_PIPE_4,
    NRF24_PIPE_5
} nrf24_pipe_t;

/**
 * @brief HAL callback structure for hardware abstraction.
 */
typedef struct {
    /**
     * @brief SPI transfer function.
     * @param ctx User context pointer
     * @param tx Pointer to transmit buffer
     * @param rx Pointer to receive buffer (can be NULL if not needed)
     * @param len Number of bytes to transfer
     * @return 0 on success, non-zero on failure
     */
    int (*spi_transfer)(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len);

    /**
     * @brief Set CSN pin low.
     * @param ctx User context pointer
     */
    void (*csn_low)(void *ctx);

    /**
     * @brief Set CSN pin high.
     * @param ctx User context pointer
     */
    void (*csn_high)(void *ctx);

    /**
     * @brief Set CE pin low.
     * @param ctx User context pointer
     */
    void (*ce_low)(void *ctx);

    /**
     * @brief Set CE pin high.
     * @param ctx User context pointer
     */
    void (*ce_high)(void *ctx);

    /**
     * @brief Microsecond delay.
     * @param us Microseconds to delay
     */
    void (*delay_us)(uint32_t us);

    /**
     * @brief Millisecond delay.
     * @param ms Milliseconds to delay
     */
    void (*delay_ms)(uint32_t ms);

    /**
     * @brief Read IRQ pin state (optional, can be NULL).
     * @param ctx User context pointer
     * @return IRQ pin state (0 = active/low, 1 = inactive/high)
     */
    int (*irq_read)(void *ctx);

    /**
     * @brief User context pointer passed to all callbacks.
     */
    void *user_ctx;
} nrf24_hal_t;

/**
 * @brief Driver configuration structure.
 */
typedef struct {
    uint8_t channel;            /**< RF channel (0-125) */
    nrf24_data_rate_t data_rate;/**< Data rate */
    nrf24_tx_power_t tx_power;  /**< TX power */
    nrf24_crc_t crc;            /**< CRC configuration */
    nrf24_addr_width_t addr_width; /**< Address width */
} nrf24_config_t;

/**
 * @brief Driver handle structure.
 */
typedef struct {
    const nrf24_hal_t *hal;     /**< HAL callbacks pointer */
    nrf24_config_t config;      /**< Current configuration */
    nrf24_mode_t current_mode;  /**< Current operating mode */
    bool initialized;           /**< Initialization flag */
    nrf24_mutex_t mutex;        /**< SPI access mutex */
    nrf24_sem_t irq_sem;        /**< IRQ signaling semaphore */
} nrf24_t;

/* -------------------------------------------------------------------------- */
/*                              Public Functions                              */
/* -------------------------------------------------------------------------- */

/* Initialization */

/**
 * @brief Initialize the NRF24 driver.
 *
 * @param dev Pointer to driver handle
 * @param hal Pointer to HAL callbacks (must remain valid during driver lifetime)
 * @param config Pointer to configuration (copied internally)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_init(nrf24_t *dev, const nrf24_hal_t *hal, const nrf24_config_t *config);

/**
 * @brief Deinitialize the NRF24 driver.
 *
 * @param dev Pointer to driver handle
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_deinit(nrf24_t *dev);

/* Mode Control */

/**
 * @brief Power up the device (enter Standby-I mode).
 *
 * @param dev Pointer to driver handle
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_power_up(nrf24_t *dev);

/**
 * @brief Power down the device.
 *
 * @param dev Pointer to driver handle
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_power_down(nrf24_t *dev);

/**
 * @brief Enter RX mode.
 *
 * @param dev Pointer to driver handle
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_set_mode_rx(nrf24_t *dev);

/**
 * @brief Enter TX mode.
 *
 * @param dev Pointer to driver handle
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_set_mode_tx(nrf24_t *dev);

/**
 * @brief Enter Standby-I mode.
 *
 * @param dev Pointer to driver handle
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_standby(nrf24_t *dev);

/* RF Configuration */

/**
 * @brief Set RF channel.
 *
 * @param dev Pointer to driver handle
 * @param channel RF channel (0-125)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_set_channel(nrf24_t *dev, uint8_t channel);

/**
 * @brief Get current RF channel.
 *
 * @param dev Pointer to driver handle
 * @param channel Pointer to store channel value
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_get_channel(nrf24_t *dev, uint8_t *channel);

/**
 * @brief Set data rate.
 *
 * @param dev Pointer to driver handle
 * @param data_rate Data rate setting
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_set_data_rate(nrf24_t *dev, nrf24_data_rate_t data_rate);

/**
 * @brief Set TX power.
 *
 * @param dev Pointer to driver handle
 * @param power TX power setting
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_set_tx_power(nrf24_t *dev, nrf24_tx_power_t power);

/**
 * @brief Set CRC configuration.
 *
 * @param dev Pointer to driver handle
 * @param crc CRC setting
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_set_crc(nrf24_t *dev, nrf24_crc_t crc);

/* Address Management */

/**
 * @brief Set TX address.
 *
 * @param dev Pointer to driver handle
 * @param addr Pointer to address bytes (length determined by addr_width config)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_set_tx_address(nrf24_t *dev, const uint8_t *addr);

/**
 * @brief Get TX address.
 *
 * @param dev Pointer to driver handle
 * @param addr Pointer to buffer to store address
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_get_tx_address(nrf24_t *dev, uint8_t *addr);

/**
 * @brief Set RX address for a pipe.
 *
 * @param dev Pointer to driver handle
 * @param pipe Data pipe (0-5)
 * @param addr Pointer to address bytes (pipes 2-5 use only 1 byte)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_set_rx_address(nrf24_t *dev, nrf24_pipe_t pipe, const uint8_t *addr);

/**
 * @brief Get RX address for a pipe.
 *
 * @param dev Pointer to driver handle
 * @param pipe Data pipe (0-5)
 * @param addr Pointer to buffer to store address
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_get_rx_address(nrf24_t *dev, nrf24_pipe_t pipe, uint8_t *addr);

/**
 * @brief Enable a data pipe.
 *
 * @param dev Pointer to driver handle
 * @param pipe Data pipe (0-5)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_enable_pipe(nrf24_t *dev, nrf24_pipe_t pipe);

/**
 * @brief Disable a data pipe.
 *
 * @param dev Pointer to driver handle
 * @param pipe Data pipe (0-5)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_disable_pipe(nrf24_t *dev, nrf24_pipe_t pipe);

/**
 * @brief Set payload width for a pipe (static payload mode).
 *
 * @param dev Pointer to driver handle
 * @param pipe Data pipe (0-5)
 * @param width Payload width (1-32 bytes)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_set_payload_width(nrf24_t *dev, nrf24_pipe_t pipe, uint8_t width);

/* Enhanced ShockBurst */

/**
 * @brief Enable or disable auto acknowledgment for a pipe.
 *
 * @param dev Pointer to driver handle
 * @param pipe Data pipe (0-5)
 * @param enable true to enable, false to disable
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_set_auto_ack(nrf24_t *dev, nrf24_pipe_t pipe, bool enable);

/**
 * @brief Enable or disable auto acknowledgment for all pipes.
 *
 * @param dev Pointer to driver handle
 * @param enable true to enable, false to disable
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_set_auto_ack_all(nrf24_t *dev, bool enable);

/**
 * @brief Configure automatic retransmission.
 *
 * @param dev Pointer to driver handle
 * @param delay Retransmission delay (0-15, delay = (delay+1)*250us)
 * @param count Retransmission count (0-15, 0 = disabled)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_set_retransmit(nrf24_t *dev, uint8_t delay, uint8_t count);

/**
 * @brief Enable or disable dynamic payload length for a pipe.
 *
 * @param dev Pointer to driver handle
 * @param pipe Data pipe (0-5)
 * @param enable true to enable, false to disable
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_enable_dynamic_payload(nrf24_t *dev, nrf24_pipe_t pipe, bool enable);

/**
 * @brief Enable or disable dynamic payload length for all pipes.
 *
 * @param dev Pointer to driver handle
 * @param enable true to enable, false to disable
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_enable_dynamic_payload_all(nrf24_t *dev, bool enable);

/**
 * @brief Enable or disable payload with ACK feature.
 *
 * @param dev Pointer to driver handle
 * @param enable true to enable, false to disable
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_enable_ack_payload(nrf24_t *dev, bool enable);

/**
 * @brief Enable or disable dynamic ACK feature (NO_ACK packets).
 *
 * @param dev Pointer to driver handle
 * @param enable true to enable, false to disable
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_enable_dynamic_ack(nrf24_t *dev, bool enable);

/* TX Operations */

/**
 * @brief Write payload to TX FIFO.
 *
 * @param dev Pointer to driver handle
 * @param data Pointer to payload data
 * @param len Payload length (1-32 bytes)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_write_payload(nrf24_t *dev, const uint8_t *data, size_t len);

/**
 * @brief Write payload to TX FIFO with NO_ACK flag.
 *
 * @param dev Pointer to driver handle
 * @param data Pointer to payload data
 * @param len Payload length (1-32 bytes)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_write_payload_noack(nrf24_t *dev, const uint8_t *data, size_t len);

/**
 * @brief Write payload for ACK packet (used in PRX mode).
 *
 * @param dev Pointer to driver handle
 * @param pipe Data pipe (0-5)
 * @param data Pointer to payload data
 * @param len Payload length (1-32 bytes)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_write_ack_payload(nrf24_t *dev, nrf24_pipe_t pipe, const uint8_t *data, size_t len);

/**
 * @brief Reuse last transmitted payload.
 *
 * @param dev Pointer to driver handle
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_reuse_tx_payload(nrf24_t *dev);

/**
 * @brief Pulse CE pin to trigger transmission.
 *
 * @param dev Pointer to driver handle
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_transmit(nrf24_t *dev);

/* RX Operations */

/**
 * @brief Read payload from RX FIFO.
 *
 * @param dev Pointer to driver handle
 * @param data Pointer to buffer to store payload
 * @param len Pointer to variable: input=buffer size, output=bytes read
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_read_payload(nrf24_t *dev, uint8_t *data, size_t *len);

/**
 * @brief Get width of payload in RX FIFO (dynamic payload mode).
 *
 * @param dev Pointer to driver handle
 * @param width Pointer to store payload width
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_get_rx_payload_width(nrf24_t *dev, uint8_t *width);

/**
 * @brief Check if data is available in RX FIFO.
 *
 * @param dev Pointer to driver handle
 * @param pipe Pointer to store pipe number (can be NULL)
 * @return true if data available, false otherwise
 */
bool nrf24_data_available(nrf24_t *dev, nrf24_pipe_t *pipe);

/* FIFO Management */

/**
 * @brief Flush TX FIFO.
 *
 * @param dev Pointer to driver handle
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_flush_tx(nrf24_t *dev);

/**
 * @brief Flush RX FIFO.
 *
 * @param dev Pointer to driver handle
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_flush_rx(nrf24_t *dev);

/**
 * @brief Get FIFO status register.
 *
 * @param dev Pointer to driver handle
 * @param status Pointer to store FIFO status
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_get_fifo_status(nrf24_t *dev, uint8_t *status);

/* Status and Diagnostics */

/**
 * @brief Get status register.
 *
 * @param dev Pointer to driver handle
 * @param status Pointer to store status value
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_get_status(nrf24_t *dev, uint8_t *status);

/**
 * @brief Clear interrupt flags.
 *
 * @param dev Pointer to driver handle
 * @param flags IRQ flags to clear (NRF24_IRQ_RX_DR, NRF24_IRQ_TX_DS, NRF24_IRQ_MAX_RT)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_clear_irq(nrf24_t *dev, uint8_t flags);

/**
 * @brief Get observe TX register (lost packets and retransmit count).
 *
 * @param dev Pointer to driver handle
 * @param lost_packets Pointer to store lost packets count (can be NULL)
 * @param retransmit_count Pointer to store retransmit count (can be NULL)
 * @return NRF24_OK on success, error code otherwise
 */
nrf24_err_t nrf24_get_observe_tx(nrf24_t *dev, uint8_t *lost_packets, uint8_t *retransmit_count);

/**
 * @brief Check if carrier is detected (RPD register).
 *
 * @param dev Pointer to driver handle
 * @return true if carrier detected, false otherwise
 */
bool nrf24_is_carrier_detected(nrf24_t *dev);

/* IRQ Handling */

/**
 * @brief IRQ handler - call from ISR to signal semaphore.
 *
 * @param dev Pointer to driver handle
 *
 * @note In FreeRTOS mode, this signals the IRQ semaphore.
 *       In bare metal mode, this is a no-op.
 */
void nrf24_irq_handler(nrf24_t *dev);

/**
 * @brief Wait for IRQ event (blocking).
 *
 * @param dev Pointer to driver handle
 * @param timeout_ticks Timeout in ticks (use NRF24_WAIT_FOREVER for indefinite)
 * @return NRF24_OK on success, NRF24_ERR_TIMEOUT on timeout
 *
 * @note In bare metal mode, this returns immediately with NRF24_OK.
 */
nrf24_err_t nrf24_wait_irq(nrf24_t *dev, uint32_t timeout_ticks);

#ifdef __cplusplus
}
#endif

#endif /* NRF24_H */
