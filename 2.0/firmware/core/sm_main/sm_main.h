/* sm_main.h - State machine handling code
 *
 * Copyright (C) 2019 Icoteq Ltd.
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

#ifndef _SM_MAIN_H_
#define _SM_MAIN_H_

#include "sm.h"
#include "fs.h"
#include "cexception.h"

#define STM32_FIRMWARE_VERSION      (11) // The current version of the firmware

#define FS_FILE_ID_CONF             (0) // The File ID of the configuration data
#define FS_FILE_ID_STM32_IMAGE      (1) // STM32 image
#define FS_FILE_ID_BLE_IMAGE        (2) // BLE image
#define FS_FILE_ID_LOG              (4) // Sensor log file

extern sm_state_func_t sm_main_states[]; // State function lookup table is populate in main_state.c

typedef enum
{
    SM_MAIN_BOOT,
    SM_MAIN_BATTERY_CHARGING,
    SM_MAIN_BATTERY_LEVEL_LOW,
    SM_MAIN_LOG_FILE_FULL,
    SM_MAIN_PROVISIONING_NEEDED,
    SM_MAIN_PROVISIONING,
    SM_MAIN_OPERATIONAL,
} sm_main_states_t;

void sm_main_exception_handler(CEXCEPTION_T e);

#ifdef GTEST
extern fs_handle_t file_handle;
#endif

#endif /* _SM_MAIN_H_ */
