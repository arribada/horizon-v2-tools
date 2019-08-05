/* sm_iot.c - IOT state machine
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

#include <stdbool.h>
#include <string.h>
#include "debug.h"
#include "cexception.h"
#include "syshal_batt.h"
#include "syshal_pmu.h"
#include "syshal_timer.h"
#include "syshal_rtc.h"
#include "syshal_firmware.h"
#include "sm_main.h"
#include "sm_iot.h"

#define CELLULAR_CONNECT_TIMEOUT_MS (3 * 60 * 1000)
#define CELLULAR_DEFAULT_TIMEOUT_MS (30 * 1000)
#define CELLULAR_START_BACKOFF_TIME (30)

static sm_iot_init_t config;
static bool cellular_enabled;
static bool cellular_pending;
static bool cellular_max_backoff_reached;
static bool satellite_enabled;
static uint32_t last_successful_cellular_connection = 0;
static uint32_t cellular_backoff_time;

// Timer handles
static timer_handle_t timer_cellular_max_interval;
static timer_handle_t timer_cellular_min_interval;
static timer_handle_t timer_cellular_retry;

static int iot_error_mapping(int error_code)
{
    return -100;
}

static void generate_event(sm_iot_event_id_t id, int code)
{
    sm_iot_event_t event;
    event.id = id;
    event.code = code;
    sm_iot_callback(&event);
}

/**
 * @brief      Populate the device status using the latest values.
 *
 * @note       This does not modify status->last_log_file_read_pos
 *
 * @param      status  The pointer to the device status struct
 */
static void populate_device_status(iot_device_status_t * status)
{
    int ret;

    //status->last_sat_tx_timestamp = 0;  // TODO: TO BE IMPLEMENTED!
    //status->next_sat_tx_timestamp = 0;  // TODO: TO BE IMPLEMENTED!

    status->presence_flags = 0;

    if (config.iot_init.iot_cellular_config->contents.status_filter & IOT_LAST_GPS_LOCATION_BITMASK &&
        config.gps_last_known_position)
    {
        if (config.gps_last_known_position->hdr.set)
        {
            // Convert from date time to timestamp format
            syshal_rtc_data_and_time_t data_and_time;
            data_and_time.year =    config.gps_last_known_position->contents.year;
            data_and_time.month =   config.gps_last_known_position->contents.month;
            data_and_time.day =     config.gps_last_known_position->contents.day;
            data_and_time.hours =   config.gps_last_known_position->contents.hours;
            data_and_time.minutes = config.gps_last_known_position->contents.minutes;
            data_and_time.seconds = config.gps_last_known_position->contents.seconds;
            data_and_time.milliseconds = 0;

            ret = syshal_rtc_date_time_to_timestamp(data_and_time, &status->last_gps_location.timestamp);
            if (!ret)
            {
                status->last_gps_location.longitude = config.gps_last_known_position->contents.lon;
                status->last_gps_location.latitude =  config.gps_last_known_position->contents.lat;
                status->presence_flags |= IOT_LAST_GPS_LOCATION_BITMASK;
            }
        }
    }

    if (config.iot_init.iot_cellular_config->contents.status_filter & IOT_LAST_LOG_FILE_READ_POS_BITMASK)
        status->presence_flags |= IOT_LAST_LOG_FILE_READ_POS_BITMASK;

    if (config.iot_init.iot_cellular_config->contents.status_filter & IOT_BATTERY_LEVEL_BITMASK)
    {
        ret = syshal_batt_level(&status->battery_level);
        while (ret == SYSHAL_BATT_ERROR_BUSY)
            ret = syshal_batt_level(&status->battery_level);
        if (!ret)
            status->presence_flags |= IOT_BATTERY_LEVEL_BITMASK;
    }

    if (config.iot_init.iot_cellular_config->contents.status_filter & IOT_BATTERY_VOLTAGE_BITMASK)
    {
        ret = syshal_batt_voltage(&status->battery_voltage);
        if (!ret)
            status->presence_flags |= IOT_BATTERY_VOLTAGE_BITMASK;
    }

    if (config.iot_init.iot_cellular_config->contents.status_filter & IOT_LAST_CELLULAR_CONNECTED_TIMESTAMP_BITMASK &&
        last_successful_cellular_connection)
    {
        status->last_cellular_connected_timestamp = last_successful_cellular_connection;
        status->presence_flags |= IOT_LAST_CELLULAR_CONNECTED_TIMESTAMP_BITMASK;
    }

    // Fetch our current config version
    sys_config_version_t * conf_version;
    ret = sys_config_get(SYS_CONFIG_TAG_VERSION, (void *) &conf_version);
    if (!ret)
    {
        status->configuration_version = conf_version->contents.version;
        if (config.iot_init.iot_cellular_config->contents.status_filter & IOT_CONFIGURATION_VERSION_BITMASK)
            status->presence_flags |= IOT_CONFIGURATION_VERSION_BITMASK;
    }
    else
    {
        status->configuration_version = 0;
    }

    // Fetch our current firmware version
    status->firmware_version = APP_FIRMWARE_VERSION;
    if (config.iot_init.iot_cellular_config->contents.status_filter & IOT_FIRMWARE_VERSION_BITMASK)
        status->presence_flags |= IOT_FIRMWARE_VERSION_BITMASK;
}

static int run_cellular(void)
{
    iot_device_shadow_t shadow;
    bool firmware_update = false;
    bool config_update = false;
    CEXCEPTION_T e = CEXCEPTION_NONE;
    int ret;

    Try
    {
        ret = iot_power_on(IOT_RADIO_CELLULAR);
        generate_event(SM_IOT_CELLULAR_POWER_ON, ret);
        if (ret)
            Throw(ret);

        ret = iot_connect(CELLULAR_CONNECT_TIMEOUT_MS);
        generate_event(SM_IOT_CELLULAR_CONNECT, ret);
        if (ret)
            Throw(ret);

        ret = iot_fetch_device_shadow(CELLULAR_DEFAULT_TIMEOUT_MS, &shadow);
        generate_event(SM_IOT_CELLULAR_FETCH_DEVICE_SHADOW, ret);
        if (ret)
            Throw(ret);

        // If send logging backlog is enabled then send our log file
        if (config.iot_init.iot_cellular_config->contents.log_filter) // TODO: This is the incorrect use of log_filter and needs to be remedied
        {
            ret = iot_send_logging(CELLULAR_DEFAULT_TIMEOUT_MS, file_system, FILE_ID_LOG, shadow.device_status.last_log_file_read_pos);
            generate_event(SM_IOT_CELLULAR_SEND_LOGGING, ret);
            if (ret < 0)
                Throw(ret);
            shadow.device_status.last_log_file_read_pos += ret;
        }

        // Update the remote device status
        populate_device_status(&shadow.device_status);
        ret = iot_send_device_status(CELLULAR_DEFAULT_TIMEOUT_MS, &shadow.device_status);
        generate_event(SM_IOT_CELLULAR_SEND_DEVICE_STATUS, ret);
        if (ret)
            Throw(ret);
    }
    Catch(e)
    {
        DEBUG_PR_ERROR("%s() failed with %d", __FUNCTION__, e);
        ret = iot_power_off();
        generate_event(SM_IOT_CELLULAR_POWER_OFF, ret);

        syshal_timer_cancel(timer_cellular_max_interval);
        syshal_timer_set(timer_cellular_retry, one_shot, cellular_backoff_time);
        if (!cellular_max_backoff_reached)
        {
            cellular_backoff_time *= 2;
            if (cellular_backoff_time >= config.iot_init.iot_cellular_config->contents.max_backoff_interval)
            {
                cellular_backoff_time = config.iot_init.iot_cellular_config->contents.max_backoff_interval;
                cellular_max_backoff_reached = true;
                generate_event(SM_IOT_CELLULAR_MAX_BACKOFF_REACHED, cellular_backoff_time);
            }
        }
        return SM_IOT_CONNECTION_FAILED;
    }

    // If we are supposed to be checking for firmware updates
    if (config.iot_init.iot_cellular_config->contents.check_firmware_updates)
    {
        // And there is a firmware update
        if (shadow.device_update.presence_flags & IOT_FIRMWARE_UPDATE_BITMASK)
        {
            // And it is of a greater version than we have
            if (shadow.device_update.firmware_update.version > shadow.device_status.firmware_version)
            {
                sm_iot_event_t event;

                // Then download it
                DEBUG_PR_TRACE("Downloading firmware update from: %s:%u/%s", shadow.device_update.firmware_update.url.domain, shadow.device_update.firmware_update.url.port, shadow.device_update.firmware_update.url.path);

                ret = iot_download_file(CELLULAR_DEFAULT_TIMEOUT_MS, &shadow.device_update.firmware_update.url, file_system, FILE_ID_APP_FIRM_IMAGE, &event.firmware_update.length);
                if (!ret)
                {
                    firmware_update = true;
                }
                else
                {
                    DEBUG_PR_ERROR("Download failed with %d", ret);
                }

                event.id = SM_IOT_CELLULAR_DOWNLOAD_FIRMWARE_FILE;
                event.code = ret;
                event.firmware_update.version = shadow.device_update.firmware_update.version;
                sm_iot_callback(&event);
            }
        }
    }

    // If we are supposed to be checking for configuration updates
    if (config.iot_init.iot_cellular_config->contents.check_configuration_updates)
    {
        // And there is a config update
        if (shadow.device_update.presence_flags & IOT_CONFIGURATION_UPDATE_BITMASK)
        {
            // And it is of a greater version than we have
            if (shadow.device_update.configuration_update.version > shadow.device_status.configuration_version)
            {
                sm_iot_event_t event;

                // Then download it
                DEBUG_PR_TRACE("Downloading configuration update from: %s:%u/%s", shadow.device_update.configuration_update.url.domain, shadow.device_update.configuration_update.url.port, shadow.device_update.configuration_update.url.path);

                ret = iot_download_file(CELLULAR_DEFAULT_TIMEOUT_MS, &shadow.device_update.configuration_update.url, file_system, FILE_ID_CONF_COMMANDS, &event.config_update.length);
                if (!ret)
                {
                    config_update = true;
                }
                else
                {
                    DEBUG_PR_ERROR("Download failed with %d", ret);
                }

                event.id = SM_IOT_CELLULAR_DOWNLOAD_CONFIG_FILE;
                event.code = ret;
                event.config_update.version = shadow.device_update.configuration_update.version;
                sm_iot_callback(&event);
            }
        }
    }

    ret = iot_power_off();
    generate_event(SM_IOT_CELLULAR_POWER_OFF, ret);

    if (config.iot_init.iot_cellular_config->contents.max_interval)
        syshal_timer_set(timer_cellular_max_interval, one_shot, config.iot_init.iot_cellular_config->contents.max_interval);

    syshal_timer_cancel(timer_cellular_retry);
    cellular_backoff_time = CELLULAR_START_BACKOFF_TIME;
    cellular_max_backoff_reached = false;
    cellular_pending = false;

    syshal_rtc_get_timestamp(&last_successful_cellular_connection);

    // Apply any firmware update
    if (firmware_update)
    {
        generate_event(SM_IOT_APPLY_FIRMWARE_UPDATE, 0);
        syshal_firmware_update(FILE_ID_APP_FIRM_IMAGE);
    }

    // Reset as the config update is handled on startup
    if (config_update)
    {
        generate_event(SM_IOT_APPLY_CONFIG_UPDATE, 0);
        syshal_pmu_reset();
    }

    return SM_IOT_NO_ERROR;
}

static void cellular_timer_callback(void)
{
    run_cellular();
}

int sm_iot_init(sm_iot_init_t init)
{
    int ret;

    // Create a local copy of the init params
    config = init;

    cellular_enabled = false;
    satellite_enabled = false;

    // Back out if the IOT layer is not enabled
    if (!config.iot_init.iot_config->contents.enable || !config.iot_init.iot_config->hdr.set)
        return SM_IOT_NO_ERROR;

    if (config.iot_init.iot_cellular_config->contents.enable && config.iot_init.iot_cellular_config->hdr.set &&
        config.iot_init.iot_cellular_aws_config->hdr.set)
        cellular_enabled = true;

    if (config.iot_init.iot_sat_config->contents.enable && config.iot_init.iot_sat_config->hdr.set)
        satellite_enabled = true;

    // Back out if no IOT backend is enabled
    if (!cellular_enabled && !satellite_enabled)
        return SM_IOT_NO_ERROR;

    // Initialise the IOT subsystem
    ret = iot_init(config.iot_init);

    if (ret)
        return iot_error_mapping(ret);

    cellular_backoff_time = CELLULAR_START_BACKOFF_TIME;
    cellular_max_backoff_reached = false;
    cellular_pending = false;

    // Init timers
    syshal_timer_init(&timer_cellular_min_interval, cellular_timer_callback);
    syshal_timer_init(&timer_cellular_max_interval, cellular_timer_callback);
    syshal_timer_init(&timer_cellular_retry, cellular_timer_callback);

    // Set up cellular timer
    if (cellular_enabled && config.iot_init.iot_cellular_config->contents.max_interval)
        syshal_timer_set(timer_cellular_max_interval, one_shot, config.iot_init.iot_cellular_config->contents.max_interval);

    return SM_IOT_NO_ERROR;
}

int sm_iot_term(void)
{
    syshal_timer_term(timer_cellular_min_interval);
    syshal_timer_term(timer_cellular_max_interval);
    syshal_timer_term(timer_cellular_retry);
    return SM_IOT_NO_ERROR;
}

int sm_iot_trigger(iot_radio_type_t radio_type)
{
    switch (radio_type)
    {
        case IOT_RADIO_CELLULAR:
            if (cellular_enabled && !cellular_pending)
            {
                uint32_t current_time;
                syshal_rtc_get_timestamp(&current_time);
                cellular_pending = true;
                // Check if less than our minimum interval time has passed
                if (current_time - last_successful_cellular_connection < config.iot_init.iot_cellular_config->contents.min_interval)
                {
                    // If it hasn't set up a timer to fire when it has
                    syshal_timer_set(timer_cellular_min_interval, one_shot, current_time - last_successful_cellular_connection);
                }
                else
                {
                    // Else we are free to connect now
                    run_cellular();
                }
            }
            break;

        case IOT_RADIO_SATELLITE:
            return SM_IOT_INVALID_PARAM; // TODO: TO BE IMPLEMENTED
            break;

        default:
            return SM_IOT_INVALID_PARAM;
            break;
    }
    return SM_IOT_NO_ERROR;
}

int sm_iot_trigger_force(iot_radio_type_t radio_type)
{
    switch (radio_type)
    {
        case IOT_RADIO_CELLULAR:
            cellular_pending = true;
            return run_cellular();
            break;

        case IOT_RADIO_SATELLITE:
            return SM_IOT_INVALID_PARAM; // TODO: TO BE IMPLEMENTED
            break;

        default:
            return SM_IOT_INVALID_PARAM;
            break;
    }
    return SM_IOT_NO_ERROR;
}

__attribute__((weak)) void sm_iot_callback(sm_iot_event_t * event)
{
    ((void)(event)); // Remove unused variable compiler warning

    DEBUG_PR_WARN("%s Not implemented", __FUNCTION__);
#ifdef GTEST
    printf("IOT CALLBACK EVENT CODE: %d\n", event->code);
#endif
}