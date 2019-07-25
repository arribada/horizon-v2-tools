/* sys_config.c - Configuration data and tag accessors/mutators
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

#include "sys_config.h"
#include "syshal_rtc.h"
#include <string.h>
#include "debug.h"

sys_config_t sys_config; // Configuration data stored in RAM

// Lookup table for checking tag ID is valid and also iterating
static const uint16_t sys_config_lookup_priv[SYS_CONFIG_TAG_TOTAL_NUMBER] =
{
    SYS_CONFIG_TAG_GPS_LOG_POSITION_ENABLE,
    SYS_CONFIG_TAG_GPS_LOG_TTFF_ENABLE,
    SYS_CONFIG_TAG_GPS_TRIGGER_MODE,
    SYS_CONFIG_TAG_GPS_SCHEDULED_ACQUISITION_INTERVAL,
    SYS_CONFIG_TAG_GPS_MAXIMUM_ACQUISITION_TIME,
    SYS_CONFIG_TAG_GPS_SCHEDULED_ACQUISITION_NO_FIX_TIMEOUT,
    SYS_CONFIG_TAG_GPS_LAST_KNOWN_POSITION,
    SYS_CONFIG_TAG_GPS_VERY_FIRST_FIX_HOLD_TIME,
    SYS_CONFIG_TAG_GPS_DEBUG_LOGGING_ENABLE,
    SYS_CONFIG_SALTWATER_SWITCH_LOG_ENABLE,
    SYS_CONFIG_SALTWATER_SWITCH_HYSTERESIS_PERIOD,
    SYS_CONFIG_TAG_RTC_SYNC_TO_GPS_ENABLE,
    SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME,
    SYS_CONFIG_TAG_LOGGING_ENABLE,
    SYS_CONFIG_TAG_LOGGING_FILE_SIZE,
    SYS_CONFIG_TAG_LOGGING_FILE_TYPE,
    SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE,
    SYS_CONFIG_TAG_LOGGING_START_END_SYNC_ENABLE,
    SYS_CONFIG_TAG_LOGGING_DATE_TIME_STAMP_ENABLE,
    SYS_CONFIG_TAG_LOGGING_HIGH_RESOLUTION_TIMER_ENABLE,
    SYS_CONFIG_TAG_AXL_LOG_ENABLE,
    SYS_CONFIG_TAG_AXL_CONFIG,
    SYS_CONFIG_TAG_AXL_G_FORCE_HIGH_THRESHOLD,
    SYS_CONFIG_TAG_AXL_SAMPLE_RATE,
    SYS_CONFIG_TAG_AXL_MODE,
    SYS_CONFIG_TAG_AXL_SCHEDULED_ACQUISITION_INTERVAL,
    SYS_CONFIG_TAG_AXL_MAXIMUM_ACQUISITION_TIME,
    SYS_CONFIG_TAG_PRESSURE_SENSOR_LOG_ENABLE,
    SYS_CONFIG_TAG_PRESSURE_SAMPLE_RATE,
    SYS_CONFIG_TAG_PRESSURE_LOW_THRESHOLD,
    SYS_CONFIG_TAG_PRESSURE_HIGH_THRESHOLD,
    SYS_CONFIG_TAG_PRESSURE_MODE,
    SYS_CONFIG_TAG_PRESSURE_SCHEDULED_ACQUISITION_INTERVAL,
    SYS_CONFIG_TAG_PRESSURE_MAXIMUM_ACQUISITION_TIME,
    SYS_CONFIG_TAG_TEMP_SENSOR_LOG_ENABLE,
    SYS_CONFIG_TAG_TEMP_SENSOR_SAMPLE_RATE,
    SYS_CONFIG_TAG_TEMP_SENSOR_LOW_THRESHOLD,
    SYS_CONFIG_TAG_TEMP_SENSOR_HIGH_THRESHOLD,
    SYS_CONFIG_TAG_TEMP_SENSOR_MODE,
    SYS_CONFIG_TAG_SYSTEM_DEVICE_IDENTIFIER,
    SYS_CONFIG_TAG_BLUETOOTH_DEVICE_ADDRESS,
    SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL,
    SYS_CONFIG_TAG_BLUETOOTH_SCHEDULED_INTERVAL,
    SYS_CONFIG_TAG_BLUETOOTH_SCHEDULED_DURATION,
    SYS_CONFIG_TAG_BLUETOOTH_ADVERTISING_INTERVAL,
    SYS_CONFIG_TAG_BLUETOOTH_CONNECTION_INTERVAL,
    SYS_CONFIG_TAG_BLUETOOTH_CONNECTION_INACTIVITY_TIMEOUT,
    SYS_CONFIG_TAG_BLUETOOTH_PHY_MODE,
    SYS_CONFIG_TAG_BLUETOOTH_LOG_ENABLE,
    SYS_CONFIG_TAG_BATTERY_LOG_ENABLE,
    SYS_CONFIG_TAG_BATTERY_LOW_THRESHOLD,
};

/**
 * @brief      Gets the pointer to the data of the given tag
 *
 * @param[in]  tag   The configuration tag
 * @param[out] data  The pointer to the tags data
 *
 * @return     The length of the given configuration tag
 * @return     SYS_CONFIG_ERROR_INVALID_TAG if the given tag is invalid
 */
int sys_config_get_data_ptr_priv(uint16_t tag, void ** data)
{
    uint32_t len = 0;

    switch (tag)
    {
        case SYS_CONFIG_TAG_GPS_LOG_POSITION_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_log_position_enable_t);
            *data = &sys_config.sys_config_gps_log_position_enable;
            break;

        case SYS_CONFIG_TAG_GPS_LOG_TTFF_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_log_ttff_enable_t);
            *data = &sys_config.sys_config_gps_log_ttff_enable;
            break;

        case SYS_CONFIG_TAG_GPS_TRIGGER_MODE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_trigger_mode_t);
            *data = &sys_config.sys_config_gps_trigger_mode;
            break;

        case SYS_CONFIG_TAG_GPS_SCHEDULED_ACQUISITION_INTERVAL:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_scheduled_acquisition_interval_t);
            *data = &sys_config.sys_config_gps_scheduled_acquisition_interval;
            break;

        case SYS_CONFIG_TAG_GPS_MAXIMUM_ACQUISITION_TIME:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_maximum_acquisition_time_t);
            *data = &sys_config.sys_config_gps_maximum_acquisition_time;
            break;

        case SYS_CONFIG_TAG_GPS_SCHEDULED_ACQUISITION_NO_FIX_TIMEOUT:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_scheduled_acquisition_no_fix_timeout_t);
            *data = &sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout;
            break;

        case SYS_CONFIG_TAG_GPS_LAST_KNOWN_POSITION:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_last_known_position_t);
            *data = &sys_config.sys_config_gps_last_known_position;
            break;

        case SYS_CONFIG_TAG_GPS_VERY_FIRST_FIX_HOLD_TIME:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_very_first_fix_hold_time_t);
            *data = &sys_config.sys_config_gps_very_first_fix_hold_time;
            break;

        case SYS_CONFIG_TAG_GPS_DEBUG_LOGGING_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_gps_debug_logging_enable_t);
            *data = &sys_config.sys_config_gps_debug_logging_enable;
            break;

        case SYS_CONFIG_SALTWATER_SWITCH_LOG_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_saltwater_switch_log_enable_t);
            *data = &sys_config.sys_config_saltwater_switch_log_enable;
            break;

        case SYS_CONFIG_SALTWATER_SWITCH_HYSTERESIS_PERIOD:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_saltwater_switch_hysteresis_period_t);
            *data = &sys_config.sys_config_saltwater_switch_hysteresis_period;
            break;

        case SYS_CONFIG_TAG_RTC_SYNC_TO_GPS_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_rtc_sync_to_gps_enable_t);
            *data = &sys_config.sys_config_rtc_sync_to_gps_enable;
            break;

        case SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_rtc_current_date_and_time_t);
            *data = &sys_config.sys_config_rtc_current_date_and_time;
            break;

        case SYS_CONFIG_TAG_LOGGING_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_enable_t);
            *data = &sys_config.sys_config_logging_enable;
            break;

        case SYS_CONFIG_TAG_LOGGING_FILE_SIZE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_file_size_t);
            *data = &sys_config.sys_config_logging_file_size;
            break;

        case SYS_CONFIG_TAG_LOGGING_FILE_TYPE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_file_type_t);
            *data = &sys_config.sys_config_logging_file_type;
            break;

        case SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_group_sensor_readings_enable_t);
            *data = &sys_config.sys_config_logging_group_sensor_readings_enable;
            break;

        case SYS_CONFIG_TAG_LOGGING_START_END_SYNC_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_start_end_sync_enable_t);
            *data = &sys_config.sys_config_logging_start_end_sync_enable;
            break;

        case SYS_CONFIG_TAG_LOGGING_DATE_TIME_STAMP_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_date_time_stamp_enable_t);
            *data = &sys_config.sys_config_logging_date_time_stamp_enable;
            break;

        case SYS_CONFIG_TAG_LOGGING_HIGH_RESOLUTION_TIMER_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_high_resolution_timer_enable_t);
            *data = &sys_config.sys_config_logging_high_resolution_timer_enable;
            break;

        case SYS_CONFIG_TAG_AXL_LOG_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_log_enable_t);
            *data = &sys_config.sys_config_axl_log_enable;
            break;

        case SYS_CONFIG_TAG_AXL_CONFIG:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_config_t);
            *data = &sys_config.sys_config_axl_config;
            break;

        case SYS_CONFIG_TAG_AXL_G_FORCE_HIGH_THRESHOLD:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_g_force_high_threshold_t);
            *data = &sys_config.sys_config_axl_g_force_high_threshold;
            break;

        case SYS_CONFIG_TAG_AXL_SAMPLE_RATE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_sample_rate_t);
            *data = &sys_config.sys_config_axl_sample_rate;
            break;

        case SYS_CONFIG_TAG_AXL_MODE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_mode_t);
            *data = &sys_config.sys_config_axl_mode;
            break;

        case SYS_CONFIG_TAG_AXL_SCHEDULED_ACQUISITION_INTERVAL:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_scheduled_acquisition_interval_t);
            *data = &sys_config.sys_config_axl_scheduled_acquisition_interval;
            break;

        case SYS_CONFIG_TAG_AXL_MAXIMUM_ACQUISITION_TIME:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_axl_maximum_acquisition_time_t);
            *data = &sys_config.sys_config_axl_maximum_acquisition_time;
            break;

        case SYS_CONFIG_TAG_PRESSURE_SENSOR_LOG_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_sensor_log_enable_t);
            *data = &sys_config.sys_config_pressure_sensor_log_enable;
            break;

        case SYS_CONFIG_TAG_PRESSURE_SAMPLE_RATE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_sample_rate_t);
            *data = &sys_config.sys_config_pressure_sample_rate;
            break;

        case SYS_CONFIG_TAG_PRESSURE_LOW_THRESHOLD:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_low_threshold_t);
            *data = &sys_config.sys_config_pressure_low_threshold;
            break;

        case SYS_CONFIG_TAG_PRESSURE_HIGH_THRESHOLD:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_high_threshold_t);
            *data = &sys_config.sys_config_pressure_high_threshold;
            break;

        case SYS_CONFIG_TAG_PRESSURE_MODE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_mode_t);
            *data = &sys_config.sys_config_pressure_mode;
            break;

        case SYS_CONFIG_TAG_PRESSURE_SCHEDULED_ACQUISITION_INTERVAL:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_scheduled_acquisition_interval_t);
            *data = &sys_config.sys_config_pressure_scheduled_acquisition_interval;
            break;

        case SYS_CONFIG_TAG_PRESSURE_MAXIMUM_ACQUISITION_TIME:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_pressure_maximum_acquisition_time_t);
            *data = &sys_config.sys_config_pressure_maximum_acquisition_time;
            break;

        case SYS_CONFIG_TAG_TEMP_SENSOR_LOG_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_log_enable_t);
            *data = &sys_config.sys_config_temp_sensor_log_enable;
            break;

        case SYS_CONFIG_TAG_TEMP_SENSOR_SAMPLE_RATE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_sample_rate_t);
            *data = &sys_config.sys_config_temp_sensor_sample_rate;
            break;

        case SYS_CONFIG_TAG_TEMP_SENSOR_LOW_THRESHOLD:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_low_threshold_t);
            *data = &sys_config.sys_config_temp_sensor_low_threshold;
            break;

        case SYS_CONFIG_TAG_TEMP_SENSOR_HIGH_THRESHOLD:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_high_threshold_t);
            *data = &sys_config.sys_config_temp_sensor_high_threshold;
            break;

        case SYS_CONFIG_TAG_TEMP_SENSOR_MODE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_temp_sensor_mode_t);
            *data = &sys_config.sys_config_temp_sensor_mode;
            break;

        case SYS_CONFIG_TAG_SYSTEM_DEVICE_IDENTIFIER:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_system_device_identifier_t);
            *data = &sys_config.sys_config_system_device_identifier;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_DEVICE_ADDRESS:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_tag_bluetooth_device_address_t);
            *data = &sys_config.sys_config_tag_bluetooth_device_address;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_tag_bluetooth_trigger_control_t);
            *data = &sys_config.sys_config_tag_bluetooth_trigger_control;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_SCHEDULED_INTERVAL:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_tag_bluetooth_scheduled_interval_t);
            *data = &sys_config.sys_config_tag_bluetooth_scheduled_interval;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_SCHEDULED_DURATION:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_tag_bluetooth_scheduled_duration_t);
            *data = &sys_config.sys_config_tag_bluetooth_scheduled_duration;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_ADVERTISING_INTERVAL:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_tag_bluetooth_advertising_interval_t);
            *data = &sys_config.sys_config_tag_bluetooth_advertising_interval;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_CONNECTION_INTERVAL:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_tag_bluetooth_connection_interval_t);
            *data = &sys_config.sys_config_tag_bluetooth_connection_interval;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_CONNECTION_INACTIVITY_TIMEOUT:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_tag_bluetooth_connection_inactivity_timeout_t);
            *data = &sys_config.sys_config_tag_bluetooth_connection_inactivity_timeout;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_PHY_MODE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_tag_bluetooth_phy_mode_t);
            *data = &sys_config.sys_config_tag_bluetooth_phy_mode;
            break;

        case SYS_CONFIG_TAG_BLUETOOTH_LOG_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_tag_bluetooth_log_enable_t);
            *data = &sys_config.sys_config_tag_bluetooth_log_enable;
            break;

        case SYS_CONFIG_TAG_BATTERY_LOG_ENABLE:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_battery_log_enable_t);
            *data = &sys_config.sys_config_battery_log_enable;
            break;

        case SYS_CONFIG_TAG_BATTERY_LOW_THRESHOLD:
            len = SYS_CONFIG_TAG_DATA_SIZE(sys_config_battery_low_threshold_t);
            *data = &sys_config.sys_config_battery_low_threshold;
            break;

        default:
            return SYS_CONFIG_ERROR_INVALID_TAG;
    }

    return (int) len;
}

/**
 * @brief      Get data size of the given configuration tag
 *
 * @param[in]  tag   The configuration tag
 *
 * @return     Length of data in bytes on success
 * @return     SYS_CONFIG_ERROR_INVALID_TAG if the given tag is invalid
 */
int sys_config_size(uint16_t tag)
{
    void * data = NULL;
    int return_code = sys_config_get_data_ptr_priv(tag, &data);

    return return_code;
}

/**
 * @brief      Set the configuration tag to the given value
 *
 * @param[in]  tag     The configuration tag
 * @param[in]  value   The value to set the tag to
 * @param[in]  length  The length of the data to be written
 *
 * @return     SYS_CONFIG_NO_ERROR on success
 * @return     SYS_CONFIG_ERROR_INVALID_TAG if the given tag is invalid
 * @return     SYS_CONFIG_ERROR_WRONG_SIZE if the given length does not meet the
 *             configuration tag length
 */
int sys_config_set(uint16_t tag, void * value, uint32_t length)
{
    // Get the buffer of the configuration tag in RAM
    void * data = NULL;
    int return_code = sys_config_get_data_ptr_priv(tag, &data);

    if (return_code < 0)
        return return_code;

    if ( ((uint32_t) return_code) != length)
        return SYS_CONFIG_ERROR_WRONG_SIZE;

    // Copy data into the configuration tag
#pragma GCC diagnostic push // Suppress data = NULL warning as this cannot happen as the function returns if that is the case
#pragma GCC diagnostic ignored "-Wnonnull"
    memcpy(data + sizeof(sys_config_hdr_t), value, length);
#pragma GCC diagnostic pop

    // Set the configuration set tag
    // This is done so we can always tell if a configuration tag has been setup or has been erased or never initialised
    ((sys_config_hdr_t *)data)->set = true;

    // Populate the rtc values with the provided date and time
    if (tag == SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME)
    {
        syshal_rtc_data_and_time_t date_time;

        date_time.day = sys_config.sys_config_rtc_current_date_and_time.contents.day;
        date_time.month = sys_config.sys_config_rtc_current_date_and_time.contents.month;
        date_time.year = sys_config.sys_config_rtc_current_date_and_time.contents.year;
        date_time.hours = sys_config.sys_config_rtc_current_date_and_time.contents.hours;
        date_time.minutes = sys_config.sys_config_rtc_current_date_and_time.contents.minutes;
        date_time.seconds = sys_config.sys_config_rtc_current_date_and_time.contents.seconds;
        date_time.milliseconds = 0;

        syshal_rtc_set_date_and_time(date_time);
    }

    return SYS_CONFIG_NO_ERROR;
}

/**
 * @brief      Clear the given configuration tag
 *
 * @param[in]  tag   The configuration tag
 *
 * @return     SYS_CONFIG_NO_ERROR on success
 * @return     SYS_CONFIG_ERROR_INVALID_TAG if the given tag is invalid
 */
int sys_config_unset(uint16_t tag)
{
    // Get the address of the configuration data in RAM
    void * data = NULL;
    int return_code = sys_config_get_data_ptr_priv(tag, &data);

    if (return_code < 0)
        return return_code;

    ((sys_config_hdr_t *)data)->set = false; // Unclear the configuration set tag

    return SYS_CONFIG_NO_ERROR;
}

/**
 * @brief      Gets the value in the given configuration tag
 *
 * @param[in]  tag     The configuration tag
 * @param[out] value   The pointer to the configuration data
 *
 * @return     The length of the data read on success
 * @return     SYS_CONFIG_ERROR_INVALID_TAG if the given tag is invalid
 * @return     SYS_CONFIG_ERROR_TAG_NOT_SET if the tag hasn't been set
 */
int sys_config_get(uint16_t tag, void ** value)
{
    // Get the address of the configuration data in RAM
    void * data = NULL;
    int return_code = sys_config_get_data_ptr_priv(tag, &data);

    if (return_code < 0)
        return return_code;

    if (tag == SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME)
    {
        // Populate the sys_config values with the current date/time
        syshal_rtc_data_and_time_t date_time;
        syshal_rtc_get_date_and_time(&date_time);

        sys_config.sys_config_rtc_current_date_and_time.hdr.set = true;
        sys_config.sys_config_rtc_current_date_and_time.contents.day = date_time.day;
        sys_config.sys_config_rtc_current_date_and_time.contents.month = date_time.month;
        sys_config.sys_config_rtc_current_date_and_time.contents.year = date_time.year;
        sys_config.sys_config_rtc_current_date_and_time.contents.hours = date_time.hours;
        sys_config.sys_config_rtc_current_date_and_time.contents.minutes = date_time.minutes;
        sys_config.sys_config_rtc_current_date_and_time.contents.seconds = date_time.seconds;
    }

    // If this isn't a read only tag or a special tag check it's set flag
    if (SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME != tag
        && SYS_CONFIG_TAG_LOGGING_FILE_SIZE != tag
        && SYS_CONFIG_TAG_LOGGING_FILE_TYPE != tag
        && SYS_CONFIG_TAG_LOGGING_START_END_SYNC_ENABLE != tag)
    {
        // If this configuration tag hasn't been previously set then return an error
        if (false == ((sys_config_hdr_t *)data)->set)
            return SYS_CONFIG_ERROR_TAG_NOT_SET;
    }

    if (value != NULL)
        (*value) = data + sizeof(sys_config_hdr_t);

    return return_code;
}

/**
 * @brief      Checks to see if the given tag is valid
 *
 * @param[in]  tag   The configuration tag
 *
 * @return     true if valid
 */
bool sys_config_is_valid(uint16_t tag)
{
    for (uint32_t i = 0; i < SYS_CONFIG_TAG_TOTAL_NUMBER; ++i)
    {
        if (tag == sys_config_lookup_priv[i])
            return true;
    }

    return false;
}

/**
 * @brief      Returns the next tag after the given tag. This is used for
 *             iterating through all the tags
 *
 * @warning    This function assumes the configuration tag given is valid and
 *             will produce spurious output if it is not
 *
 * @param[in]  tag         Pointer to tag value to populate
 * @param      last_index  Private variable for maintaining state. For the first
 *                         call pass a pointer to a uint32_t set to 0
 *
 * @return     SYS_CONFIG_NO_ERROR on success
 * @return     SYS_CONFIG_ERROR_NO_MORE_TAGS if there are no more tags after this
 *             one
 */
int sys_config_iterate(uint16_t * tag, uint16_t * last_index)
{
    uint16_t idx = *last_index;
    if (idx >= SYS_CONFIG_TAG_TOTAL_NUMBER)
        return SYS_CONFIG_ERROR_NO_MORE_TAGS;

    // Return the next tag in the list
    (*last_index)++;
    *tag = sys_config_lookup_priv[idx];

    return SYS_CONFIG_NO_ERROR;
}