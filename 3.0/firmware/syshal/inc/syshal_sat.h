/* syshal_sat.h - Abstraction layer for satallite comms
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

#ifndef _SYSHAL_SAT_H_
#define _SYSHAL_SAT_H_

#include <stdint.h>
#include "prepas.h"
#include "iot.h"

#define SYSHAL_SAT_NO_ERROR  (0)

int syshal_sat_init(iot_prepass_sats_t * sat_config, uint32_t num_sats);
int syshal_sat_power_on(void);
int syshal_sat_power_off(void);
int syshal_sat_program_firmware(uint32_t local_file_id);
int syshal_sat_send_message(uint8_t * buffer, size_t buffer_size);
int syshal_sat_calc_prepass(iot_last_gps_location_t * gps, iot_prepass_result_t * result);

#endif /* _SYSHAL_SAT_H_ */