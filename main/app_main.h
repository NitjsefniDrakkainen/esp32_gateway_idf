/**
 * @file    app_main.h
 * @author  Your Name
 * @date    2026-01-27
 * @brief   Header file for app_main.c
 *
 * @details
 * This header exposes the public interface for the module.
 * It should contain type definitions, function declarations,
 * and constants accessible to other components.
 */

#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "swc_mqtt/swc_mqtt.h"
#include "swc_config/swc_config.h"
#include "swc_endpoint/swc_endpoint.h"
#include "swc_protocol/swc_protocol.h"
#include "swc_webserver/swc_webserver.h"
#include "bsp/bsp.h"

/* -------------------------------------------------------------------------- */
/*                                Public macros                               */
/* -------------------------------------------------------------------------- */
/* none */

/* -------------------------------------------------------------------------- */
/*                                Public types                                */
/* -------------------------------------------------------------------------- */
/* none */

/* -------------------------------------------------------------------------- */
/*                            Public API functions                            */
/* -------------------------------------------------------------------------- */

/**
 * @brief Initialize the module.
 *
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t app_main_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __${TM_FILENAME_BASE/(.*)/Your Name_H__}/ */