/**
  ******************************************************************************
  * @file     syshal_spi.c
  * @brief    System hardware abstraction layer for SPI.
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

#include "syshal_spi.h"
#include "syshal_pmu.h"
#include "nrfx_spim.h"
#include "bsp.h"
#include "syshal_firmware.h"

//#define SPI_USE_IRQ

#ifdef SPI_USE_IRQ
static volatile bool spim_xfer_done = false;

static void spim_event_handler(nrfx_spim_evt_t const * p_event, void * p_context)
{
    spim_xfer_done = true;
}
#endif

int syshal_spi_init(uint32_t instance)
{
    if (instance >= SPI_TOTAL_NUMBER)
        return SYSHAL_SPI_ERROR_INVALID_INSTANCE;

#ifdef SPI_USE_IRQ
    nrfx_spim_init(&SPI_Inits[instance].spim, &SPI_Inits[instance].spim_config, spim_event_handler, NULL);
#else
    nrfx_spim_init(&SPI_Inits[instance].spim, &SPI_Inits[instance].spim_config, NULL, NULL);
#endif

    return SYSHAL_SPI_NO_ERROR;
}

int syshal_spi_term(uint32_t instance)
{
    if (instance >= SPI_TOTAL_NUMBER)
        return SYSHAL_SPI_ERROR_INVALID_INSTANCE;

    nrfx_spim_uninit(&SPI_Inits[instance].spim);

    return SYSHAL_SPI_NO_ERROR;
}

__RAMFUNC int syshal_spi_transfer(uint32_t instance, uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
    if (instance >= SPI_TOTAL_NUMBER)
        return SYSHAL_SPI_ERROR_INVALID_INSTANCE;

    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(tx_data, size, rx_data, size);

#ifdef SPI_USE_IRQ
    spim_xfer_done = false;
#endif

    //APP_ERROR_CHECK(
    nrfx_spim_xfer(&SPI_Inits[instance].spim, &xfer_desc, 0);
    //nrfx_spim_xfer(&spim, &xfer_desc, 0);
#ifdef SPI_USE_IRQ
    while (!spim_xfer_done)
        syshal_pmu_sleep(SLEEP_LIGHT);
#endif

    return SYSHAL_SPI_NO_ERROR;
}