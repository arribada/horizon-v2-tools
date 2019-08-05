/* syshal_device.c - HAL for getting details of the device
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

#include <string.h>
#include "syshal_device.h"
#include "nrf.h"
#include "nrf_soc.h"

#define BOOTLOADER_DFU_START (0xB1)

int syshal_device_id(device_id_t * device_id)
{
    if (sizeof(device_id_t) < sizeof(NRF_FICR->DEVICEID))
        return SYSHAL_DEVICE_ERROR_DEVICE;
    memcpy(device_id, (const uint8_t*)NRF_FICR->DEVICEID, sizeof(NRF_FICR->DEVICEID));
    return SYSHAL_DEVICE_NO_ERROR;
}

int syshal_device_set_dfu_entry_flag(bool set)
{
    if (set)
        sd_power_gpregret_set(0, BOOTLOADER_DFU_START);
    else
        sd_power_gpregret_set(0, 0x00);

    return SYSHAL_DEVICE_NO_ERROR;
}
