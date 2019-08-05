/**
  ******************************************************************************
  * @file     syshal_firmware.c
  * @brief    System hardware abstraction layer for writing firmware images to
  *           the FLASH.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2019 Arribada</center></h2>
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
  *
  ******************************************************************************
  */

#include "syshal_firmware.h"
#include "syshal_pmu.h"
#include "sm_main.h"
#include "nrf_nvmc.h"
#include "nrf_sdh.h"
#include "fs.h"

#define APPLICATION_BASE_ADDR (0x26000)
#define APPLICATION_LENGTH    (0xDA000)

static union
{
    uint32_t word;
    uint8_t bytes[4];
} buffer;

static uint32_t writing_address; // The FLASH address we're currently writing to
static uint32_t bytes_remaining; // Number of buffer.bytes yet to be written to FLASH

static int fs_error_mapping(int error_code)
{
    switch (error_code)
    {
        case FS_ERROR_FILE_NOT_FOUND:
            return SYSHAL_FIRMWARE_ERROR_FILE_NOT_FOUND;
        default:
            return SYSHAL_FIRMWARE_ERROR_FS;
    }
}

static __RAMFUNC int syshal_firmware_prepare(void)
{
    nrf_sdh_disable_request(); // Disable the softdevice

    __disable_irq(); // Don't allow any interrupts to interrupt us

    uint32_t page_size = NRF_FICR->CODEPAGESIZE;
    uint32_t pages_to_erase = APPLICATION_LENGTH / page_size;

    // Erase all application firmware pages
    for (uint32_t i = 0; i < pages_to_erase; ++i)
    {
        nrf_nvmc_page_erase(APPLICATION_BASE_ADDR + page_size * i);
    }

    writing_address = APPLICATION_BASE_ADDR;
    bytes_remaining = 0;

    return SYSHAL_FIRMWARE_NO_ERROR;
}

static __RAMFUNC int syshal_firmware_write(uint8_t * data, uint32_t size)
{
    // Iterate through every discrete byte
    for (uint32_t i = 0; i < size; ++i)
    {
        // Fill up blocks of 32 bits
        buffer.bytes[bytes_remaining] = data[i];
        bytes_remaining++;

        // If we have a full 32 bits, write it to FLASH
        if (bytes_remaining == 4)
        {
            nrf_nvmc_write_words(writing_address, &buffer.word, 1);
            writing_address += 4;
            bytes_remaining = 0;
        }
    }

    return SYSHAL_FIRMWARE_NO_ERROR;
}

static __RAMFUNC int syshal_firmware_flush(void)
{
    // Flush any remaining data to the FLASH
    if (bytes_remaining)
    {
        //DEBUG_PR_TRACE("writing word 0x%08lX to: 0x%08lX", buffer.word, writing_address);
        nrf_nvmc_write_words(writing_address, &buffer.word, 1);
        writing_address += 4;
        bytes_remaining = 0;
    }

    return SYSHAL_FIRMWARE_NO_ERROR;
}

__RAMFUNC int syshal_firmware_update(uint32_t local_file_id)
{
    uint8_t read_buffer[1024];
    uint32_t bytes_actually_read;
    fs_handle_t file_handle;
    int ret;

    ret = fs_open(file_system, &file_handle, local_file_id, FS_MODE_READONLY, NULL);
    if (ret)
        return fs_error_mapping(ret);

    syshal_firmware_prepare(); // Erase our FLASH
    // Everything from here on out MUST reside in RAM as our FLASH has been erased
    do
    {
        ret = fs_read(file_handle, &read_buffer, sizeof(read_buffer), &bytes_actually_read);
        syshal_firmware_write(read_buffer, bytes_actually_read);
        syshal_pmu_kick_watchdog();
    }
    while (FS_ERROR_END_OF_FILE != ret);

    syshal_firmware_flush();

    for (;;)
    {
        syshal_pmu_reset();
    }
}