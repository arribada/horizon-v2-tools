/* syshal_timer.c - HAL for MCU timers
 *
 * Copyright (C) 2018 Arribada
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

#include <stdbool.h>
#include "syshal_timer.h"
#include "syshal_rtc.h"
#include "debug.h"

#define MILLISECONDS_IN_A_DAY (24 * 60 * 60 * 1000)

typedef struct
{
    bool init;                // Is this timer init/used
    bool running;             // Is this timer running?
    syshal_timer_mode_t mode; // Is this timer a one-shot or continuous
    uint32_t start;           // A timestamp of when this timer was started
    uint32_t duration;        // How long this timer should run for before triggering (in ms)
    void (*callback)(void);   // The function that is called when the timer is triggered
} syshal_timer_t;

static volatile syshal_timer_t timers_priv[SYSHAL_TIMER_NUMBER_OF_TIMERS];

uint32_t syshal_timer_time_in_milliseconds_priv(void)
{
    // Get the current time of day in milliseconds
    syshal_rtc_data_and_time_t current_date_time;
    syshal_rtc_get_date_and_time(&current_date_time);

    uint32_t milliseconds = 0;

    milliseconds += current_date_time.milliseconds;
    milliseconds += current_date_time.seconds * 1000;
    milliseconds += current_date_time.minutes * 60 * 1000;
    milliseconds += current_date_time.hours * 60 * 60 * 1000;

    return milliseconds;
}

int syshal_timer_init(timer_handle_t *handle, void (*callback)(void))
{
    // Look for the first free timer
    for (uint32_t i = 0; i < SYSHAL_TIMER_NUMBER_OF_TIMERS; ++i)
    {
        if (!timers_priv[i].init)
        {
            timers_priv[i].init = true;
            timers_priv[i].callback = callback;
            timers_priv[i].running = false;
            *handle = i; // Return a handle to this timer
            return SYSHAL_TIMER_NO_ERROR;
        }
    }

    return SYSHAL_TIMER_ERROR_NO_FREE_TIMER;
}

int syshal_timer_term(timer_handle_t handle)
{
    if (handle >= SYSHAL_TIMER_NUMBER_OF_TIMERS)
        return SYSHAL_TIMER_ERROR_INVALID_TIMER_HANDLE;

    timers_priv[handle].init = false;
    timers_priv[handle].callback = NULL;
    timers_priv[handle].running = false;
    return SYSHAL_TIMER_NO_ERROR;
}

int syshal_timer_set(timer_handle_t handle, syshal_timer_mode_t mode, uint32_t seconds)
{
    return syshal_timer_set_ms(handle, mode, seconds * 1000);
}

int syshal_timer_set_ms(timer_handle_t handle, syshal_timer_mode_t mode, uint32_t milliseconds)
{
    if (handle >= SYSHAL_TIMER_NUMBER_OF_TIMERS)
        return SYSHAL_TIMER_ERROR_INVALID_TIMER_HANDLE;

    if (milliseconds >= MILLISECONDS_IN_A_DAY)
        return SYSHAL_TIMER_ERROR_INVALID_TIME;

    if (milliseconds == 0)
        milliseconds = 1; // Our minimum duration is 1 millisecond

    timers_priv[handle].running = true;
    timers_priv[handle].mode = mode;
    timers_priv[handle].start = syshal_timer_time_in_milliseconds_priv();
    timers_priv[handle].duration = milliseconds;

    DEBUG_PR_SYS("%s(%lu, %d, %lu)", __FUNCTION__, handle, mode, milliseconds);

    return SYSHAL_TIMER_NO_ERROR;
}

int syshal_timer_reset(timer_handle_t handle)
{
    if (handle >= SYSHAL_TIMER_NUMBER_OF_TIMERS)
        return SYSHAL_TIMER_ERROR_INVALID_TIMER_HANDLE;

    timers_priv[handle].start = syshal_timer_time_in_milliseconds_priv();

    return SYSHAL_TIMER_NO_ERROR;
}

int syshal_timer_running(timer_handle_t handle)
{
    if (handle >= SYSHAL_TIMER_NUMBER_OF_TIMERS)
        return SYSHAL_TIMER_ERROR_INVALID_TIMER_HANDLE;

    return timers_priv[handle].running;
}

int syshal_timer_cancel(timer_handle_t handle)
{
    if (handle >= SYSHAL_TIMER_NUMBER_OF_TIMERS)
        return SYSHAL_TIMER_ERROR_INVALID_TIMER_HANDLE;

#ifndef DEBUG_DISABLED
    if (timers_priv[handle].running)
    {
        DEBUG_PR_SYS("Cancelling timer: %lu", handle);
    }
#endif

    timers_priv[handle].running = false;

    return SYSHAL_TIMER_NO_ERROR;
}

int syshal_timer_cancel_all(void)
{
    for (uint32_t i = 0; i < SYSHAL_TIMER_NUMBER_OF_TIMERS; ++i)
        syshal_timer_cancel(i);

    return SYSHAL_TIMER_NO_ERROR;
}

void syshal_timer_tick(void)
{
    for (uint32_t i = 0; i < SYSHAL_TIMER_NUMBER_OF_TIMERS; ++i)
    {
        uint32_t current_time_milliseconds = syshal_timer_time_in_milliseconds_priv();

        if (timers_priv[i].running)
        {
            uint32_t elapsed;
            if (current_time_milliseconds < timers_priv[i].start)
                elapsed = MILLISECONDS_IN_A_DAY + (current_time_milliseconds - timers_priv[i].start);
            else
                elapsed = current_time_milliseconds - timers_priv[i].start;

            if (elapsed >= timers_priv[i].duration)
            {
                // If this is a periodic timer, then auto-reload it
                if (periodic == timers_priv[i].mode)
                    syshal_timer_set_ms(i, timers_priv[i].mode, timers_priv[i].duration);
                else
                    syshal_timer_cancel(i); // Else cancel it

                if (timers_priv[i].callback)
                    timers_priv[i].callback(); // Call the callback function
            }
        }
    }
}