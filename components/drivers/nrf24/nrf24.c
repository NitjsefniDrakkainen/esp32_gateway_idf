/**
 * @file    nrf24.c
 * @author  Your Name
 * @date    2026-02-03
 * @brief   NRF24L01/NRF24L01+ driver implementation.
 *
 * @details
 * MCU-independent driver implementation for the NRF24L01 2.4GHz transceiver.
 * All hardware operations are performed through HAL callbacks.
 */

#include "nrf24/nrf24.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/*                              Private macros                                */
/* -------------------------------------------------------------------------- */

#define TAG "nrf24"

/* Timing constants (in microseconds unless noted) */
#define NRF24_TIMING_POWER_ON_RESET_MS  11      /**< Power-on reset delay (10.3ms + margin) */
#define NRF24_TIMING_STANDBY_DELAY_MS   2       /**< Power down to standby delay (1.5ms + margin) */
#define NRF24_TIMING_MODE_SWITCH_US     150     /**< Standby to TX/RX delay (130us + margin) */
#define NRF24_TIMING_CE_PULSE_US        15      /**< Minimum CE high time (10us + margin) */

/* Default mutex timeout */
#define NRF24_MUTEX_TIMEOUT             1000

/* Register mask for R/W commands */
#define NRF24_REGISTER_MASK             0x1F

/* -------------------------------------------------------------------------- */
/*                             Private prototypes                             */
/* -------------------------------------------------------------------------- */

static nrf24_err_t _nrf24_read_register(nrf24_t *dev, uint8_t reg, uint8_t *data, size_t len);
static nrf24_err_t _nrf24_write_register(nrf24_t *dev, uint8_t reg, const uint8_t *data, size_t len);
static nrf24_err_t _nrf24_read_register_byte(nrf24_t *dev, uint8_t reg, uint8_t *value);
static nrf24_err_t _nrf24_write_register_byte(nrf24_t *dev, uint8_t reg, uint8_t value);
static nrf24_err_t _nrf24_send_command(nrf24_t *dev, uint8_t cmd, uint8_t *status);
static void _nrf24_ce_pulse(nrf24_t *dev);
static nrf24_err_t _nrf24_lock(nrf24_t *dev);
static void _nrf24_unlock(nrf24_t *dev);
static uint8_t _nrf24_get_addr_width_bytes(nrf24_addr_width_t aw);

/* -------------------------------------------------------------------------- */
/*                              Public functions                              */
/* -------------------------------------------------------------------------- */

nrf24_err_t nrf24_init(nrf24_t *dev, const nrf24_hal_t *hal, const nrf24_config_t *config)
{
    nrf24_err_t err;
    uint8_t reg_val;

    if (dev == NULL || hal == NULL || config == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }

    /* Validate required HAL functions */
    if (hal->spi_transfer == NULL || hal->csn_low == NULL || hal->csn_high == NULL ||
        hal->ce_low == NULL || hal->ce_high == NULL ||
        hal->delay_us == NULL || hal->delay_ms == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }

    /* Validate configuration */
    if (config->channel > NRF24_MAX_CHANNEL) {
        return NRF24_ERR_INVALID_ARG;
    }

    /* Initialize handle */
    memset(dev, 0, sizeof(nrf24_t));
    dev->hal = hal;
    dev->config = *config;
    dev->current_mode = NRF24_MODE_POWER_DOWN;
    dev->initialized = false;

    /* Create synchronization primitives */
    dev->mutex = NRF24_MUTEX_CREATE();
    dev->irq_sem = NRF24_SEM_CREATE();

    /* Ensure CE is low */
    hal->ce_low(hal->user_ctx);

    /* Wait for power-on reset */
    hal->delay_ms(NRF24_TIMING_POWER_ON_RESET_MS);

    /* Verify communication by reading CONFIG register */
    err = _nrf24_read_register_byte(dev, NRF24_REG_CONFIG, &reg_val);
    if (err != NRF24_OK) {
        return err;
    }

    /* Configure CRC */
    reg_val = 0;
    switch (config->crc) {
        case NRF24_CRC_OFF:
            /* CRC disabled */
            break;
        case NRF24_CRC_8:
            reg_val |= NRF24_CONFIG_EN_CRC;
            break;
        case NRF24_CRC_16:
            reg_val |= NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO;
            break;
    }
    err = _nrf24_write_register_byte(dev, NRF24_REG_CONFIG, reg_val);
    if (err != NRF24_OK) {
        return err;
    }

    /* Configure address width */
    err = _nrf24_write_register_byte(dev, NRF24_REG_SETUP_AW, (uint8_t)config->addr_width);
    if (err != NRF24_OK) {
        return err;
    }

    /* Configure RF channel */
    err = _nrf24_write_register_byte(dev, NRF24_REG_RF_CH, config->channel);
    if (err != NRF24_OK) {
        return err;
    }

    /* Configure data rate and TX power */
    reg_val = 0;
    switch (config->data_rate) {
        case NRF24_DR_250KBPS:
            reg_val |= NRF24_RF_SETUP_RF_DR_LOW;
            break;
        case NRF24_DR_1MBPS:
            /* Both bits cleared */
            break;
        case NRF24_DR_2MBPS:
            reg_val |= NRF24_RF_SETUP_RF_DR_HIGH;
            break;
    }
    reg_val |= ((uint8_t)config->tx_power << NRF24_RF_SETUP_RF_PWR_SHIFT);
    err = _nrf24_write_register_byte(dev, NRF24_REG_RF_SETUP, reg_val);
    if (err != NRF24_OK) {
        return err;
    }

    /* Flush FIFOs */
    err = nrf24_flush_tx(dev);
    if (err != NRF24_OK) {
        return err;
    }
    err = nrf24_flush_rx(dev);
    if (err != NRF24_OK) {
        return err;
    }

    /* Clear all IRQ flags */
    err = nrf24_clear_irq(dev, NRF24_IRQ_ALL);
    if (err != NRF24_OK) {
        return err;
    }

    dev->initialized = true;
    return NRF24_OK;
}

nrf24_err_t nrf24_deinit(nrf24_t *dev)
{
    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    /* Power down the device */
    nrf24_power_down(dev);

    /* Delete synchronization primitives */
    NRF24_MUTEX_DELETE(dev->mutex);
    NRF24_SEM_DELETE(dev->irq_sem);

    dev->initialized = false;
    return NRF24_OK;
}

/* Mode Control */

nrf24_err_t nrf24_power_up(nrf24_t *dev)
{
    nrf24_err_t err;
    uint8_t config;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_CONFIG, &config);
    if (err == NRF24_OK) {
        config |= NRF24_CONFIG_PWR_UP;
        err = _nrf24_write_register_byte(dev, NRF24_REG_CONFIG, config);
        if (err == NRF24_OK) {
            dev->hal->delay_ms(NRF24_TIMING_STANDBY_DELAY_MS);
            dev->current_mode = NRF24_MODE_STANDBY;
        }
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_power_down(nrf24_t *dev)
{
    nrf24_err_t err;
    uint8_t config;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    /* Ensure CE is low */
    dev->hal->ce_low(dev->hal->user_ctx);

    err = _nrf24_read_register_byte(dev, NRF24_REG_CONFIG, &config);
    if (err == NRF24_OK) {
        config &= ~NRF24_CONFIG_PWR_UP;
        err = _nrf24_write_register_byte(dev, NRF24_REG_CONFIG, config);
        if (err == NRF24_OK) {
            dev->current_mode = NRF24_MODE_POWER_DOWN;
        }
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_set_mode_rx(nrf24_t *dev)
{
    nrf24_err_t err;
    uint8_t config;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_CONFIG, &config);
    if (err == NRF24_OK) {
        config |= NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX;
        err = _nrf24_write_register_byte(dev, NRF24_REG_CONFIG, config);
        if (err == NRF24_OK) {
            dev->hal->ce_high(dev->hal->user_ctx);
            dev->hal->delay_us(NRF24_TIMING_MODE_SWITCH_US);
            dev->current_mode = NRF24_MODE_RX;
        }
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_set_mode_tx(nrf24_t *dev)
{
    nrf24_err_t err;
    uint8_t config;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_CONFIG, &config);
    if (err == NRF24_OK) {
        config |= NRF24_CONFIG_PWR_UP;
        config &= ~NRF24_CONFIG_PRIM_RX;
        err = _nrf24_write_register_byte(dev, NRF24_REG_CONFIG, config);
        if (err == NRF24_OK) {
            dev->hal->ce_low(dev->hal->user_ctx);
            dev->hal->delay_us(NRF24_TIMING_MODE_SWITCH_US);
            dev->current_mode = NRF24_MODE_TX;
        }
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_standby(nrf24_t *dev)
{
    nrf24_err_t err;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    dev->hal->ce_low(dev->hal->user_ctx);
    dev->current_mode = NRF24_MODE_STANDBY;

    _nrf24_unlock(dev);
    return NRF24_OK;
}

/* RF Configuration */

nrf24_err_t nrf24_set_channel(nrf24_t *dev, uint8_t channel)
{
    nrf24_err_t err;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }
    if (channel > NRF24_MAX_CHANNEL) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_write_register_byte(dev, NRF24_REG_RF_CH, channel);
    if (err == NRF24_OK) {
        dev->config.channel = channel;
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_get_channel(nrf24_t *dev, uint8_t *channel)
{
    nrf24_err_t err;

    if (dev == NULL || channel == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_RF_CH, channel);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_set_data_rate(nrf24_t *dev, nrf24_data_rate_t data_rate)
{
    nrf24_err_t err;
    uint8_t reg_val;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_RF_SETUP, &reg_val);
    if (err == NRF24_OK) {
        /* Clear data rate bits */
        reg_val &= ~(NRF24_RF_SETUP_RF_DR_LOW | NRF24_RF_SETUP_RF_DR_HIGH);

        switch (data_rate) {
            case NRF24_DR_250KBPS:
                reg_val |= NRF24_RF_SETUP_RF_DR_LOW;
                break;
            case NRF24_DR_1MBPS:
                /* Both bits cleared */
                break;
            case NRF24_DR_2MBPS:
                reg_val |= NRF24_RF_SETUP_RF_DR_HIGH;
                break;
        }

        err = _nrf24_write_register_byte(dev, NRF24_REG_RF_SETUP, reg_val);
        if (err == NRF24_OK) {
            dev->config.data_rate = data_rate;
        }
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_set_tx_power(nrf24_t *dev, nrf24_tx_power_t power)
{
    nrf24_err_t err;
    uint8_t reg_val;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_RF_SETUP, &reg_val);
    if (err == NRF24_OK) {
        reg_val &= ~NRF24_RF_SETUP_RF_PWR_MASK;
        reg_val |= ((uint8_t)power << NRF24_RF_SETUP_RF_PWR_SHIFT);
        err = _nrf24_write_register_byte(dev, NRF24_REG_RF_SETUP, reg_val);
        if (err == NRF24_OK) {
            dev->config.tx_power = power;
        }
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_set_crc(nrf24_t *dev, nrf24_crc_t crc)
{
    nrf24_err_t err;
    uint8_t reg_val;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_CONFIG, &reg_val);
    if (err == NRF24_OK) {
        reg_val &= ~(NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO);

        switch (crc) {
            case NRF24_CRC_OFF:
                /* CRC disabled */
                break;
            case NRF24_CRC_8:
                reg_val |= NRF24_CONFIG_EN_CRC;
                break;
            case NRF24_CRC_16:
                reg_val |= NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRCO;
                break;
        }

        err = _nrf24_write_register_byte(dev, NRF24_REG_CONFIG, reg_val);
        if (err == NRF24_OK) {
            dev->config.crc = crc;
        }
    }

    _nrf24_unlock(dev);
    return err;
}

/* Address Management */

nrf24_err_t nrf24_set_tx_address(nrf24_t *dev, const uint8_t *addr)
{
    nrf24_err_t err;
    uint8_t addr_len;

    if (dev == NULL || addr == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    addr_len = _nrf24_get_addr_width_bytes(dev->config.addr_width);

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_write_register(dev, NRF24_REG_TX_ADDR, addr, addr_len);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_get_tx_address(nrf24_t *dev, uint8_t *addr)
{
    nrf24_err_t err;
    uint8_t addr_len;

    if (dev == NULL || addr == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    addr_len = _nrf24_get_addr_width_bytes(dev->config.addr_width);

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register(dev, NRF24_REG_TX_ADDR, addr, addr_len);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_set_rx_address(nrf24_t *dev, nrf24_pipe_t pipe, const uint8_t *addr)
{
    nrf24_err_t err;
    uint8_t addr_len;
    uint8_t reg;

    if (dev == NULL || addr == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }
    if (pipe > NRF24_PIPE_5) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    reg = NRF24_REG_RX_ADDR_P0 + (uint8_t)pipe;

    /* Pipes 0 and 1 have full address, pipes 2-5 have only LSB */
    if (pipe <= NRF24_PIPE_1) {
        addr_len = _nrf24_get_addr_width_bytes(dev->config.addr_width);
    } else {
        addr_len = 1;
    }

    err = _nrf24_write_register(dev, reg, addr, addr_len);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_get_rx_address(nrf24_t *dev, nrf24_pipe_t pipe, uint8_t *addr)
{
    nrf24_err_t err;
    uint8_t addr_len;
    uint8_t reg;

    if (dev == NULL || addr == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }
    if (pipe > NRF24_PIPE_5) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    reg = NRF24_REG_RX_ADDR_P0 + (uint8_t)pipe;

    if (pipe <= NRF24_PIPE_1) {
        addr_len = _nrf24_get_addr_width_bytes(dev->config.addr_width);
    } else {
        addr_len = 1;
    }

    err = _nrf24_read_register(dev, reg, addr, addr_len);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_enable_pipe(nrf24_t *dev, nrf24_pipe_t pipe)
{
    nrf24_err_t err;
    uint8_t reg_val;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }
    if (pipe > NRF24_PIPE_5) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_EN_RXADDR, &reg_val);
    if (err == NRF24_OK) {
        reg_val |= (1 << (uint8_t)pipe);
        err = _nrf24_write_register_byte(dev, NRF24_REG_EN_RXADDR, reg_val);
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_disable_pipe(nrf24_t *dev, nrf24_pipe_t pipe)
{
    nrf24_err_t err;
    uint8_t reg_val;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }
    if (pipe > NRF24_PIPE_5) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_EN_RXADDR, &reg_val);
    if (err == NRF24_OK) {
        reg_val &= ~(1 << (uint8_t)pipe);
        err = _nrf24_write_register_byte(dev, NRF24_REG_EN_RXADDR, reg_val);
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_set_payload_width(nrf24_t *dev, nrf24_pipe_t pipe, uint8_t width)
{
    nrf24_err_t err;
    uint8_t reg;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }
    if (pipe > NRF24_PIPE_5 || width > NRF24_MAX_PAYLOAD_SIZE) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    reg = NRF24_REG_RX_PW_P0 + (uint8_t)pipe;
    err = _nrf24_write_register_byte(dev, reg, width);

    _nrf24_unlock(dev);
    return err;
}

/* Enhanced ShockBurst */

nrf24_err_t nrf24_set_auto_ack(nrf24_t *dev, nrf24_pipe_t pipe, bool enable)
{
    nrf24_err_t err;
    uint8_t reg_val;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }
    if (pipe > NRF24_PIPE_5) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_EN_AA, &reg_val);
    if (err == NRF24_OK) {
        if (enable) {
            reg_val |= (1 << (uint8_t)pipe);
        } else {
            reg_val &= ~(1 << (uint8_t)pipe);
        }
        err = _nrf24_write_register_byte(dev, NRF24_REG_EN_AA, reg_val);
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_set_auto_ack_all(nrf24_t *dev, bool enable)
{
    nrf24_err_t err;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_write_register_byte(dev, NRF24_REG_EN_AA, enable ? 0x3F : 0x00);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_set_retransmit(nrf24_t *dev, uint8_t delay, uint8_t count)
{
    nrf24_err_t err;
    uint8_t reg_val;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }
    if (delay > 15 || count > 15) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    reg_val = (delay << 4) | count;
    err = _nrf24_write_register_byte(dev, NRF24_REG_SETUP_RETR, reg_val);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_enable_dynamic_payload(nrf24_t *dev, nrf24_pipe_t pipe, bool enable)
{
    nrf24_err_t err;
    uint8_t feature_val;
    uint8_t dynpd_val;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }
    if (pipe > NRF24_PIPE_5) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    /* Enable DPL feature if enabling for any pipe */
    if (enable) {
        err = _nrf24_read_register_byte(dev, NRF24_REG_FEATURE, &feature_val);
        if (err == NRF24_OK) {
            feature_val |= NRF24_FEATURE_EN_DPL;
            err = _nrf24_write_register_byte(dev, NRF24_REG_FEATURE, feature_val);
        }
    }

    if (err == NRF24_OK) {
        err = _nrf24_read_register_byte(dev, NRF24_REG_DYNPD, &dynpd_val);
        if (err == NRF24_OK) {
            if (enable) {
                dynpd_val |= (1 << (uint8_t)pipe);
            } else {
                dynpd_val &= ~(1 << (uint8_t)pipe);
            }
            err = _nrf24_write_register_byte(dev, NRF24_REG_DYNPD, dynpd_val);
        }
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_enable_dynamic_payload_all(nrf24_t *dev, bool enable)
{
    nrf24_err_t err;
    uint8_t feature_val;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_FEATURE, &feature_val);
    if (err == NRF24_OK) {
        if (enable) {
            feature_val |= NRF24_FEATURE_EN_DPL;
        } else {
            feature_val &= ~NRF24_FEATURE_EN_DPL;
        }
        err = _nrf24_write_register_byte(dev, NRF24_REG_FEATURE, feature_val);
    }

    if (err == NRF24_OK) {
        err = _nrf24_write_register_byte(dev, NRF24_REG_DYNPD, enable ? 0x3F : 0x00);
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_enable_ack_payload(nrf24_t *dev, bool enable)
{
    nrf24_err_t err;
    uint8_t feature_val;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_FEATURE, &feature_val);
    if (err == NRF24_OK) {
        if (enable) {
            feature_val |= NRF24_FEATURE_EN_ACK_PAY | NRF24_FEATURE_EN_DPL;
        } else {
            feature_val &= ~NRF24_FEATURE_EN_ACK_PAY;
        }
        err = _nrf24_write_register_byte(dev, NRF24_REG_FEATURE, feature_val);
    }

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_enable_dynamic_ack(nrf24_t *dev, bool enable)
{
    nrf24_err_t err;
    uint8_t feature_val;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_FEATURE, &feature_val);
    if (err == NRF24_OK) {
        if (enable) {
            feature_val |= NRF24_FEATURE_EN_DYN_ACK;
        } else {
            feature_val &= ~NRF24_FEATURE_EN_DYN_ACK;
        }
        err = _nrf24_write_register_byte(dev, NRF24_REG_FEATURE, feature_val);
    }

    _nrf24_unlock(dev);
    return err;
}

/* TX Operations */

nrf24_err_t nrf24_write_payload(nrf24_t *dev, const uint8_t *data, size_t len)
{
    nrf24_err_t err;
    uint8_t tx_buf[NRF24_MAX_PAYLOAD_SIZE + 1];

    if (dev == NULL || data == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }
    if (len == 0 || len > NRF24_MAX_PAYLOAD_SIZE) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    tx_buf[0] = NRF24_CMD_W_TX_PAYLOAD;
    memcpy(&tx_buf[1], data, len);

    dev->hal->csn_low(dev->hal->user_ctx);
    if (dev->hal->spi_transfer(dev->hal->user_ctx, tx_buf, NULL, len + 1) != 0) {
        err = NRF24_ERR_SPI;
    }
    dev->hal->csn_high(dev->hal->user_ctx);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_write_payload_noack(nrf24_t *dev, const uint8_t *data, size_t len)
{
    nrf24_err_t err;
    uint8_t tx_buf[NRF24_MAX_PAYLOAD_SIZE + 1];

    if (dev == NULL || data == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }
    if (len == 0 || len > NRF24_MAX_PAYLOAD_SIZE) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    tx_buf[0] = NRF24_CMD_W_TX_PAYLOAD_NOACK;
    memcpy(&tx_buf[1], data, len);

    dev->hal->csn_low(dev->hal->user_ctx);
    if (dev->hal->spi_transfer(dev->hal->user_ctx, tx_buf, NULL, len + 1) != 0) {
        err = NRF24_ERR_SPI;
    }
    dev->hal->csn_high(dev->hal->user_ctx);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_write_ack_payload(nrf24_t *dev, nrf24_pipe_t pipe, const uint8_t *data, size_t len)
{
    nrf24_err_t err;
    uint8_t tx_buf[NRF24_MAX_PAYLOAD_SIZE + 1];

    if (dev == NULL || data == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }
    if (pipe > NRF24_PIPE_5 || len == 0 || len > NRF24_MAX_PAYLOAD_SIZE) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    tx_buf[0] = NRF24_CMD_W_ACK_PAYLOAD | (uint8_t)pipe;
    memcpy(&tx_buf[1], data, len);

    dev->hal->csn_low(dev->hal->user_ctx);
    if (dev->hal->spi_transfer(dev->hal->user_ctx, tx_buf, NULL, len + 1) != 0) {
        err = NRF24_ERR_SPI;
    }
    dev->hal->csn_high(dev->hal->user_ctx);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_reuse_tx_payload(nrf24_t *dev)
{
    nrf24_err_t err;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_send_command(dev, NRF24_CMD_REUSE_TX_PL, NULL);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_transmit(nrf24_t *dev)
{
    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    _nrf24_ce_pulse(dev);
    return NRF24_OK;
}

/* RX Operations */

nrf24_err_t nrf24_read_payload(nrf24_t *dev, uint8_t *data, size_t *len)
{
    nrf24_err_t err;
    uint8_t tx_buf[NRF24_MAX_PAYLOAD_SIZE + 1];
    uint8_t rx_buf[NRF24_MAX_PAYLOAD_SIZE + 1];
    uint8_t payload_len;
    size_t read_len;

    if (dev == NULL || data == NULL || len == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    /* Get payload width */
    err = nrf24_get_rx_payload_width(dev, &payload_len);
    if (err != NRF24_OK || payload_len > NRF24_MAX_PAYLOAD_SIZE) {
        /* Invalid payload, flush RX FIFO */
        _nrf24_send_command(dev, NRF24_CMD_FLUSH_RX, NULL);
        _nrf24_unlock(dev);
        return NRF24_ERR_NO_DATA;
    }

    read_len = (payload_len < *len) ? payload_len : *len;

    memset(tx_buf, 0xFF, sizeof(tx_buf));
    tx_buf[0] = NRF24_CMD_R_RX_PAYLOAD;

    dev->hal->csn_low(dev->hal->user_ctx);
    if (dev->hal->spi_transfer(dev->hal->user_ctx, tx_buf, rx_buf, read_len + 1) != 0) {
        err = NRF24_ERR_SPI;
    } else {
        memcpy(data, &rx_buf[1], read_len);
        *len = read_len;
    }
    dev->hal->csn_high(dev->hal->user_ctx);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_get_rx_payload_width(nrf24_t *dev, uint8_t *width)
{
    nrf24_err_t err = NRF24_OK;
    uint8_t tx_buf[2] = {NRF24_CMD_R_RX_PL_WID, 0xFF};
    uint8_t rx_buf[2];

    if (dev == NULL || width == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    /* Note: No lock here as this is often called from within locked context */
    dev->hal->csn_low(dev->hal->user_ctx);
    if (dev->hal->spi_transfer(dev->hal->user_ctx, tx_buf, rx_buf, 2) != 0) {
        err = NRF24_ERR_SPI;
    } else {
        *width = rx_buf[1];
    }
    dev->hal->csn_high(dev->hal->user_ctx);

    return err;
}

bool nrf24_data_available(nrf24_t *dev, nrf24_pipe_t *pipe)
{
    uint8_t status;

    if (dev == NULL || !dev->initialized) {
        return false;
    }

    if (nrf24_get_status(dev, &status) != NRF24_OK) {
        return false;
    }

    /* Check RX_DR flag or RX_P_NO != 0x07 (FIFO not empty) */
    uint8_t rx_p_no = (status & NRF24_STATUS_RX_P_NO_MASK) >> NRF24_STATUS_RX_P_NO_SHIFT;

    if (rx_p_no > 5) {
        return false;
    }

    if (pipe != NULL) {
        *pipe = (nrf24_pipe_t)rx_p_no;
    }

    return true;
}

/* FIFO Management */

nrf24_err_t nrf24_flush_tx(nrf24_t *dev)
{
    nrf24_err_t err;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_send_command(dev, NRF24_CMD_FLUSH_TX, NULL);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_flush_rx(nrf24_t *dev)
{
    nrf24_err_t err;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_send_command(dev, NRF24_CMD_FLUSH_RX, NULL);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_get_fifo_status(nrf24_t *dev, uint8_t *status)
{
    nrf24_err_t err;

    if (dev == NULL || status == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_FIFO_STATUS, status);

    _nrf24_unlock(dev);
    return err;
}

/* Status and Diagnostics */

nrf24_err_t nrf24_get_status(nrf24_t *dev, uint8_t *status)
{
    nrf24_err_t err;

    if (dev == NULL || status == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_send_command(dev, NRF24_CMD_NOP, status);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_clear_irq(nrf24_t *dev, uint8_t flags)
{
    nrf24_err_t err;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    /* Write 1 to IRQ bits to clear them */
    err = _nrf24_write_register_byte(dev, NRF24_REG_STATUS, flags & NRF24_IRQ_ALL);

    _nrf24_unlock(dev);
    return err;
}

nrf24_err_t nrf24_get_observe_tx(nrf24_t *dev, uint8_t *lost_packets, uint8_t *retransmit_count)
{
    nrf24_err_t err;
    uint8_t reg_val;

    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

    err = _nrf24_lock(dev);
    if (err != NRF24_OK) {
        return err;
    }

    err = _nrf24_read_register_byte(dev, NRF24_REG_OBSERVE_TX, &reg_val);
    if (err == NRF24_OK) {
        if (lost_packets != NULL) {
            *lost_packets = (reg_val >> 4) & 0x0F;
        }
        if (retransmit_count != NULL) {
            *retransmit_count = reg_val & 0x0F;
        }
    }

    _nrf24_unlock(dev);
    return err;
}

bool nrf24_is_carrier_detected(nrf24_t *dev)
{
    uint8_t rpd;

    if (dev == NULL || !dev->initialized) {
        return false;
    }

    if (_nrf24_read_register_byte(dev, NRF24_REG_RPD, &rpd) != NRF24_OK) {
        return false;
    }

    return (rpd & 0x01) != 0;
}

/* IRQ Handling */

void nrf24_irq_handler(nrf24_t *dev)
{
    if (dev == NULL) {
        return;
    }

#if defined(NRF24_USE_FREERTOS) && NRF24_USE_FREERTOS
    BaseType_t higher_priority_task_woken = pdFALSE;
    NRF24_SEM_GIVE_ISR(dev->irq_sem, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
#endif
}

nrf24_err_t nrf24_wait_irq(nrf24_t *dev, uint32_t timeout_ticks)
{
    if (dev == NULL) {
        return NRF24_ERR_INVALID_ARG;
    }
    if (!dev->initialized) {
        return NRF24_ERR_NOT_INIT;
    }

#if defined(NRF24_USE_FREERTOS) && NRF24_USE_FREERTOS
    if (!NRF24_SEM_TAKE(dev->irq_sem, timeout_ticks)) {
        return NRF24_ERR_TIMEOUT;
    }
#else
    (void)timeout_ticks;
#endif

    return NRF24_OK;
}

/* -------------------------------------------------------------------------- */
/*                              Private functions                             */
/* -------------------------------------------------------------------------- */

static nrf24_err_t _nrf24_read_register(nrf24_t *dev, uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t tx_buf[NRF24_MAX_PAYLOAD_SIZE + 1];
    uint8_t rx_buf[NRF24_MAX_PAYLOAD_SIZE + 1];
    nrf24_err_t err = NRF24_OK;

    if (len > NRF24_MAX_PAYLOAD_SIZE) {
        return NRF24_ERR_INVALID_ARG;
    }

    memset(tx_buf, 0xFF, len + 1);
    tx_buf[0] = NRF24_CMD_R_REGISTER | (reg & NRF24_REGISTER_MASK);

    dev->hal->csn_low(dev->hal->user_ctx);
    if (dev->hal->spi_transfer(dev->hal->user_ctx, tx_buf, rx_buf, len + 1) != 0) {
        err = NRF24_ERR_SPI;
    } else {
        memcpy(data, &rx_buf[1], len);
    }
    dev->hal->csn_high(dev->hal->user_ctx);

    return err;
}

static nrf24_err_t _nrf24_write_register(nrf24_t *dev, uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t tx_buf[NRF24_MAX_PAYLOAD_SIZE + 1];
    nrf24_err_t err = NRF24_OK;

    if (len > NRF24_MAX_PAYLOAD_SIZE) {
        return NRF24_ERR_INVALID_ARG;
    }

    tx_buf[0] = NRF24_CMD_W_REGISTER | (reg & NRF24_REGISTER_MASK);
    memcpy(&tx_buf[1], data, len);

    dev->hal->csn_low(dev->hal->user_ctx);
    if (dev->hal->spi_transfer(dev->hal->user_ctx, tx_buf, NULL, len + 1) != 0) {
        err = NRF24_ERR_SPI;
    }
    dev->hal->csn_high(dev->hal->user_ctx);

    return err;
}

static nrf24_err_t _nrf24_read_register_byte(nrf24_t *dev, uint8_t reg, uint8_t *value)
{
    return _nrf24_read_register(dev, reg, value, 1);
}

static nrf24_err_t _nrf24_write_register_byte(nrf24_t *dev, uint8_t reg, uint8_t value)
{
    return _nrf24_write_register(dev, reg, &value, 1);
}

static nrf24_err_t _nrf24_send_command(nrf24_t *dev, uint8_t cmd, uint8_t *status)
{
    uint8_t tx_buf = cmd;
    uint8_t rx_buf = 0;
    nrf24_err_t err = NRF24_OK;

    dev->hal->csn_low(dev->hal->user_ctx);
    if (dev->hal->spi_transfer(dev->hal->user_ctx, &tx_buf, &rx_buf, 1) != 0) {
        err = NRF24_ERR_SPI;
    } else if (status != NULL) {
        *status = rx_buf;
    }
    dev->hal->csn_high(dev->hal->user_ctx);

    return err;
}

static void _nrf24_ce_pulse(nrf24_t *dev)
{
    dev->hal->ce_high(dev->hal->user_ctx);
    dev->hal->delay_us(NRF24_TIMING_CE_PULSE_US);
    dev->hal->ce_low(dev->hal->user_ctx);
}

static nrf24_err_t _nrf24_lock(nrf24_t *dev)
{
    if (!NRF24_MUTEX_TAKE(dev->mutex, NRF24_MUTEX_TIMEOUT)) {
        return NRF24_ERR_MUTEX;
    }
    return NRF24_OK;
}

static void _nrf24_unlock(nrf24_t *dev)
{
    NRF24_MUTEX_GIVE(dev->mutex);
}

static uint8_t _nrf24_get_addr_width_bytes(nrf24_addr_width_t aw)
{
    switch (aw) {
        case NRF24_AW_3:
            return 3;
        case NRF24_AW_4:
            return 4;
        case NRF24_AW_5:
        default:
            return 5;
    }
}
