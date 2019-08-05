/**
  ******************************************************************************
  * @file     syshal_time.c
  * @brief    System hardware abstraction layer for system time.
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

#include "syshal_time.h"
#include "nrf52840.h"
#include "nrf_delay.h"
#include "debug.h"

static uint32_t m_systick_cnt;

void SysTick_Handler(void)
{
    m_systick_cnt++;
}

int syshal_time_init(void)
{
    SysTick_Config(SystemCoreClock / 1000); // Set to trigger every 1ms

    return SYSHAL_TIME_NO_ERROR;
}

uint32_t syshal_time_get_ticks_ms(void)
{
    return m_systick_cnt;
}

uint32_t syshal_time_get_ticks_us(void)
{
    DEBUG_PR_TRACE("%s NOT IMPLEMENTED", __FUNCTION__);

    return m_systick_cnt * 1000;
}

void syshal_time_delay_us(uint32_t us)
{
    nrf_delay_us(us);
}

void syshal_time_delay_ms(uint32_t ms)
{
    nrf_delay_ms(ms);
}
