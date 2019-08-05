/**
  ******************************************************************************
  * @file     syshal_pmu.c
  * @brief    System hardware abstraction layer for sleep states.
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

#include "syshal_pmu.h"
#include "nrfx_wdt.h"
#include "nrfx_power.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "syshal_firmware.h"
#include "syshal_time.h"
#include "retained_ram.h"
#include "syshal_rtc.h"
#include "bsp.h"
#include "debug.h"

#define WATCHDOG_PERIOD_MS ((uint32_t) 24 * 60 * 60 * 1000)

static nrfx_wdt_channel_id wdt_channel_id;
static uint32_t reset_reason;

void HardFault_Handler(void)
{
    NVIC_SystemReset();
}

static void stash_rtc_timestamp(void)
{
    // Store our current RTC time into a retained RAM section
    // so that it is maintained through a reset
    uint32_t timestamp;
    if (SYSHAL_RTC_NO_ERROR == syshal_rtc_get_timestamp(&timestamp))
        retained_ram_rtc_timestamp = timestamp;
}

static void wdt_event_handler(void)
{
    // We have roughly 50-60us in this interrupt before the device is reset
    stash_rtc_timestamp();
    while (1)
    {}
}

static void nrfx_power_usb_event_handler(nrfx_power_usb_evt_t event)
{

}

void syshal_pmu_assert_callback(uint16_t line_num, const uint8_t *file_name)
{
    stash_rtc_timestamp();
#ifdef DONT_RESTART_ASSERT
    DEBUG_PR_ERROR("Assertion %s:%u", file_name, line_num);
    while (1)
    {}
#else
    NVIC_SystemReset();
#endif
}

void syshal_pmu_init(void)
{
    nrf_pwr_mgmt_init();

    sd_power_reset_reason_get(&reset_reason);

    // The reset reasons are non-volatile so they must be explicitly cleared
    sd_power_reset_reason_clr(0xFFFFFFFF);

    const nrfx_wdt_config_t config =
    {
        .behaviour          = NRF_WDT_BEHAVIOUR_RUN_SLEEP_HALT,
        .reload_value       = WATCHDOG_PERIOD_MS,
        .interrupt_priority = INTERRUPT_PRIORITY_WATCHDOG
    };

    nrfx_wdt_init(&config, wdt_event_handler);
    nrfx_wdt_channel_alloc(&wdt_channel_id);
    nrfx_wdt_enable();

    // Enable interrupts for USB detected/power_ready/removed events
    // This is mainly just to wake the device when these occur
#ifdef SOFTDEVICE_PRESENT
    if (nrf_sdh_is_enabled())
    {
        sd_power_usbdetected_enable(true);
        sd_power_usbpwrrdy_enable(true);
        sd_power_usbremoved_enable(true);
    }
    else
#endif
    {
        nrfx_power_usbevt_config_t nrfx_power_usbevt_config;
        nrfx_power_usbevt_config.handler = nrfx_power_usb_event_handler;

        nrfx_power_usbevt_init(&nrfx_power_usbevt_config);
        nrfx_power_usbevt_enable();
    }
}

/**
 * @brief      Sleep the microcontroller
 */
void syshal_pmu_sleep(syshal_pmu_sleep_mode_t mode)
{
    bool soft_wdt_running;
    int ret;

    for (uint32_t i = 0; i < 47; ++i)
        if (NVIC_GetPendingIRQ(i))
            DEBUG_PR_TRACE("Can't sleep as IRQ %lu pending", i);

    switch (mode)
    {
        case SLEEP_DEEP:
            // We don't want our soft watchdog to run in deep sleep so disable
            ret = syshal_rtc_soft_watchdog_running(&soft_wdt_running);
            if (ret)
                soft_wdt_running = false;

            if (soft_wdt_running)
            {
                nrfx_rtc_disable(&RTC_Inits[RTC_SOFT_WATCHDOG].rtc);
                // Wait for the maximum amount of time it takes to disable the RTC
                // unfortunately there is no register we can poll to determine this
                syshal_time_delay_us(46);
            }

            nrf_pwr_mgmt_run();

            if (soft_wdt_running)
                nrfx_rtc_enable(&RTC_Inits[RTC_SOFT_WATCHDOG].rtc);
            break;

        case SLEEP_LIGHT:
        default:
            nrf_pwr_mgmt_run();
            break;
    }
}

/**
 * @brief      Causes a software reset of the MCU
 */
__RAMFUNC void syshal_pmu_reset(void)
{
    stash_rtc_timestamp();
    NVIC_SystemReset();
}

uint32_t syshal_pmu_get_startup_status(void)
{
    return reset_reason;
}

__RAMFUNC void syshal_pmu_kick_watchdog(void)
{
    nrfx_wdt_channel_feed(wdt_channel_id);
}
