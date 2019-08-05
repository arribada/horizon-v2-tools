/* sm_iot.h - IOT state machine
 *
 * Copyright (C) 2019 Arribada
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

#ifndef _SM_IOT_H_
#define _SM_IOT_H_

#include "iot.h"

#define SM_IOT_NO_ERROR           ( 0)
#define SM_IOT_INVALID_PARAM      (-1)
#define SM_IOT_CONNECTION_FAILED  (-2)

typedef enum
{
    SM_IOT_CELLULAR_POWER_ON,
    SM_IOT_CELLULAR_POWER_OFF,
    SM_IOT_CELLULAR_CONNECT,
    SM_IOT_CELLULAR_FETCH_DEVICE_SHADOW,
    SM_IOT_CELLULAR_SEND_LOGGING,
    SM_IOT_CELLULAR_SEND_DEVICE_STATUS,
    SM_IOT_CELLULAR_DOWNLOAD_FIRMWARE_FILE,
    SM_IOT_CELLULAR_DOWNLOAD_CONFIG_FILE,
    SM_IOT_CELLULAR_MAX_BACKOFF_REACHED,
    SM_IOT_APPLY_FIRMWARE_UPDATE,
    SM_IOT_APPLY_CONFIG_UPDATE
} sm_iot_event_id_t;

typedef struct
{
    sm_iot_event_id_t id;
    int code;
    union
    {
        struct
        {
            uint32_t version;
            uint32_t length;
        } firmware_update;
        struct
        {
            uint32_t version;
            uint32_t length;
        } config_update;
    };
} sm_iot_event_t;

typedef struct
{
    iot_init_t iot_init;
    sys_config_gps_last_known_position_t * gps_last_known_position; // The last known GPS position. NULL valid
} sm_iot_init_t;

int sm_iot_init(sm_iot_init_t init);
int sm_iot_term(void);
int sm_iot_trigger(iot_radio_type_t radio_type);
int sm_iot_trigger_force(iot_radio_type_t radio_type);

void sm_iot_callback(sm_iot_event_t * event);

#endif /* _SM_IOT_H_ */
