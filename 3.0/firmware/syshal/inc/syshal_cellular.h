/* syshal_cellular.h - HAL for cellular device
 *
 * Copyright (C) 2018 cellular
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef _SYSHAL_CELLULAR_H_
#define _SYSHAL_CELLULAR_H_

#include <stdint.h>
#include "fs.h"

/* Macros */

#define SYSHAL_CELLULAR_NO_ERROR                    (  0)
#define SYSHAL_CELLULAR_ERROR_UNEXPECTED_RESPONSE   ( -1)
#define SYSHAL_CELLULAR_ERROR_INVALID_INSTANCE      ( -2)
#define SYSHAL_CELLULAR_ERROR_BUFFER_OVERFLOW       ( -3)
#define SYSHAL_CELLULAR_ERROR_TIMEOUT               ( -4)
#define SYSHAL_CELLULAR_ERROR_DEVICE                ( -5)
#define SYSHAL_CELLULAR_ERROR_FORMAT_NOT_SUPPORTED  ( -7)
#define SYSHAL_CELLULAR_ERROR_SIM_CARD_NOT_FOUND    ( -8)
#define SYSHAL_CELLULAR_ERROR_HTTP                  ( -9)
#define SYSHAL_CELLULAR_ERROR_FAILED_TO_BOOT        (-10)

#define SYSHAL_CELLULAR_GPIO_POWER_ON  (GPIO_EXT1_GPIO1)

#ifndef UART_CELLULAR_BAUDRATE
#define UART_CELLULAR_BAUDRATE (115200)
#endif

#ifndef SYSHAL_CELLULAR_TIMEOUT_MS
#define SYSHAL_CELLULAR_TIMEOUT_MS (200)
#endif

#ifndef SYSHAL_CELLULAR_FILE_TIMEOUT_MS
#define SYSHAL_CELLULAR_FILE_TIMEOUT_MS (2000)
#endif

/* Constants */

typedef enum
{
    SCAN_MODE_2G,
    SCAN_MODE_AUTO,
    SCAN_MODE_3G,
} scan_mode_t;

/* Types */

typedef struct
{
    uint8_t name[128];
    uint8_t username[32];
    uint8_t password[32];
} syshal_cellular_apn_t;

/* Functions */
int syshal_cellular_init(void);
int syshal_cellular_power_on(void);
int syshal_cellular_sync_comms(void);
int syshal_cellular_power_off(void);
int syshal_cellular_check_sim(uint8_t *imsi);
int syshal_cellular_create_secure_profile(void);
int syshal_cellular_set_rat(uint32_t timeout_ms, scan_mode_t mode);
int syshal_cellular_scan(uint32_t timeout_ms);
int syshal_cellular_attach(uint32_t timeout_ms);
int syshal_cellular_dettach(uint32_t timeout_ms);
int syshal_cellular_activate_pdp(syshal_cellular_apn_t *apn, uint32_t timeout_ms);
int syshal_cellular_https_get(uint32_t timeout_ms, const char *domain, uint32_t port, const char *path);
int syshal_cellular_https_post(uint32_t timeout_ms, const char *domain, uint32_t port, const char *path);
int syshal_cellular_read_from_file_to_fs(fs_handle_t handle, uint32_t *http_code, uint32_t *file_size);
int syshal_cellular_read_from_file_to_buffer(uint8_t *buffer, uint32_t buffer_size, uint32_t *bytes_written, uint32_t *http_code);
int syshal_cellular_write_from_buffer_to_file(const uint8_t * buffer, uint32_t buffer_size);
int syshal_cellular_write_from_fs_to_file(fs_handle_t handle, uint32_t length);
int syshal_cellular_send_raw(uint8_t * data, uint32_t size);
int syshal_cellular_receive_raw(uint8_t * data, uint32_t size);
uint32_t syshal_cellular_available_raw(void);
bool syshal_cellular_is_present(void);
#endif
