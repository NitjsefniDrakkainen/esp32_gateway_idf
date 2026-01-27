/**
 * @file    swc_protocol.h
 * @author  Your Name
 * @date    2026-01-27
 * @brief   Header file for swc_protocol.c
 *
 * @details
 * This header exposes the public interface for the module.
 * It should contain type definitions, function declarations,
 * and constants accessible to other components.
 */

#ifndef __SWC_PROTOCOL_H__
#define __SWC_PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"

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
esp_err_t swc_protocol_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __${TM_FILENAME_BASE/(.*)/Your Name_H__}/ */