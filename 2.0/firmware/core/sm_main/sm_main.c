/* sm.c - Main state machine
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

#include <string.h>
#include <math.h>
#include "sm_main.h"
//#include "bsp.h"
#include "buffer.h"
#include "cmd.h"
#include "config_if.h"
#include "crc32.h"
#include "debug.h"
#include "exceptions.h"
#include "fs.h"
#include "sm.h"
#include "sys_config.h"
#include "logging.h"
#include "syshal_axl.h"
#include "syshal_batt.h"
#include "syshal_gpio.h"
#include "syshal_gps.h"
#include "syshal_firmware.h"
#include "syshal_flash.h"
#include "syshal_i2c.h"
#include "syshal_pmu.h"
#include "syshal_pressure.h"
#include "syshal_rtc.h"
#include "syshal_spi.h"
#include "syshal_switch.h"
#include "syshal_timer.h"
#include "syshal_uart.h"
#include "syshal_usb.h"
#include "syshal_ble.h"
#include "version.h"
#include "syshal_device.h"

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// MAIN STATES ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void sm_main_boot(sm_handle_t * state_handle);
static void sm_main_battery_charging(sm_handle_t * state_handle);
static void sm_main_battery_level_low(sm_handle_t * state_handle);
static void sm_main_log_file_full(sm_handle_t * state_handle);
static void sm_main_provisioning_needed(sm_handle_t * state_handle);
static void sm_main_provisioning(sm_handle_t * state_handle);
static void sm_main_operational(sm_handle_t * state_handle);

sm_state_func_t sm_main_states[] =
{
    [SM_MAIN_BOOT] = sm_main_boot,
    [SM_MAIN_BATTERY_CHARGING] = sm_main_battery_charging,
    [SM_MAIN_BATTERY_LEVEL_LOW] = sm_main_battery_level_low,
    [SM_MAIN_LOG_FILE_FULL] = sm_main_log_file_full,
    [SM_MAIN_PROVISIONING_NEEDED] = sm_main_provisioning_needed,
    [SM_MAIN_PROVISIONING] = sm_main_provisioning,
    [SM_MAIN_OPERATIONAL] = sm_main_operational
};

#ifndef DEBUG_DISABLED
static const char * sm_main_state_str[] =
{
    [SM_MAIN_BOOT]                 = "SM_MAIN_BOOT",
    [SM_MAIN_BATTERY_CHARGING]     = "SM_MAIN_BATTERY_CHARGING",
    [SM_MAIN_BATTERY_LEVEL_LOW]    = "SM_MAIN_BATTERY_LEVEL_LOW",
    [SM_MAIN_LOG_FILE_FULL]        = "SM_MAIN_LOG_FILE_FULL",
    [SM_MAIN_PROVISIONING_NEEDED]  = "SM_MAIN_PROVISIONING_NEEDED",
    [SM_MAIN_PROVISIONING]         = "SM_MAIN_PROVISIONING",
    [SM_MAIN_OPERATIONAL]          = "SM_MAIN_OPERATIONAL",
};
#endif

////////////////////////////////////////////////////////////////////////////////
////////////////////////////// MESSAGE STATES //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef enum
{
    SM_MESSAGE_STATE_IDLE,
    SM_MESSAGE_STATE_CFG_READ_NEXT,
    SM_MESSAGE_STATE_CFG_WRITE_NEXT,
    SM_MESSAGE_STATE_CFG_WRITE_ERROR,
    SM_MESSAGE_STATE_GPS_WRITE_NEXT,
    SM_MESSAGE_STATE_GPS_READ_NEXT,
    SM_MESSAGE_STATE_BLE_WRITE_NEXT,
    SM_MESSAGE_STATE_BLE_READ_NEXT,
    SM_MESSAGE_STATE_LOG_READ_NEXT,
    SM_MESSAGE_STATE_FW_SEND_IMAGE_NEXT,
} sm_message_state_t;
static sm_message_state_t message_state = SM_MESSAGE_STATE_IDLE;

// State specific context, used for maintaining information between config_if message sub-states
typedef struct
{
    union
    {
        struct
        {
            uint32_t length;
            uint8_t error_code;
            uint8_t buffer[SYS_CONFIG_TAG_MAX_SIZE];
            uint32_t buffer_occupancy;
        } cfg_write;

        struct
        {
            uint8_t * buffer_base;
            uint32_t  length;
            uint32_t  buffer_offset;
            uint16_t  last_index;
        } cfg_read;

        struct
        {
            uint8_t   address;
            uint16_t  length;
        } ble_write;

        struct
        {
            uint8_t   address;
            uint16_t  length;
        } ble_read;

        struct
        {
            uint32_t  length;
        } gps_write;

        struct
        {
            uint32_t  length;
        } gps_read;

        struct
        {
            uint32_t length;
            uint32_t start_offset;
        } log_read;

        struct
        {
            uint8_t image_type;
            uint32_t length;
            uint32_t crc32_supplied;
            uint32_t crc32_calculated;
        } fw_send_image;
    };
} sm_context_t;
static sm_context_t sm_context;

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// GPS STATES ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

typedef enum
{
    SM_GPS_STATE_ASLEEP,
    SM_GPS_STATE_ACQUIRING,
    SM_GPS_STATE_FIXED,
} sm_gps_state_t;

static volatile sm_gps_state_t sm_gps_state; // The current operating state of the GPS

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// GLOBALS /////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define LOG_FILE_FLUSH_PERIOD_SECONDS ( (1 * 60 * 60) - 60 ) // Period in seconds in which to flush the log file to FLASH

// Size of logging buffer that is used to store sensor data before it it written to FLASH
#define LOGGING_BUFFER_SIZE (40)
#define LOGGING_FIFO_DEPTH  (12) // Maximum number of readings that can be stored before a write to the FLASH log must be done

#define USB_ENUMERATION_TIMEOUT_MS (10000) // Time in ms to try for a USB connection interface when VUSB is connected

#define SM_MAIN_INACTIVITY_TIMEOUT_MS (5000) // How many ms until the message state machine reverts back to idle

#define GPS_WATCHDOG_TIME_SECONDS (4) // How many seconds until we deem the GPS unresponsive

#define REED_SWITCH_DEBOUNCE_TIME_S (2) // How many seconds to debounce the reed switch

#define SOFT_WATCHDOG_TIMEOUT_S     (10) // How many seconds to allow before soft watchdog trips

#define KICK_WATCHDOG()   syshal_rtc_soft_watchdog_refresh()


static volatile bool     config_if_tx_pending = false;
static volatile bool     config_if_rx_queued = false;
static bool              syshal_gps_bridging = false;
static bool              syshal_ble_bridging = false;
static bool              system_startup_log_required = true;
static volatile buffer_t config_if_send_buffer;
static volatile buffer_t config_if_receive_buffer;
static volatile buffer_t logging_buffer;
static volatile uint8_t  config_if_send_buffer_pool[SYSHAL_USB_PACKET_SIZE * 2];
static volatile uint8_t  config_if_receive_buffer_pool[SYSHAL_USB_PACKET_SIZE];
static volatile uint8_t  logging_buffer_pool[LOGGING_BUFFER_SIZE * LOGGING_FIFO_DEPTH];
static uint32_t          config_if_message_timeout;
static volatile bool     config_if_connected = false;
static fs_t              file_system;
static volatile bool     tracker_above_water = true; // Is the device above water?
static volatile bool     log_file_created = false; // Does a log file exist?
static volatile bool     gps_ttff_reading_logged = false; // Have we read the most recent gps ttff reading?
static uint8_t           last_battery_reading;
static volatile bool     sensor_logging_enabled = false; // Are sensors currently allowed to log
static uint8_t           ble_state;
static volatile bool     reed_switch_debounce = false; // Debouncing on the reed switch
static bool              gps_waiting_for_first_fix; // Are we waiting for the GPS to achieve a first fix?
static bool              green_led_flashing;

#ifndef GTEST
static fs_handle_t       file_handle = NULL; // The global file handle we have open. Only allow one at once
#else
fs_handle_t file_handle = NULL; // non-static to allow unit tests access to it
#endif

// Timer handles
static timer_handle_t timer_gps_interval;
static timer_handle_t timer_gps_no_fix;
static timer_handle_t timer_gps_maximum_acquisition;
static timer_handle_t timer_gps_very_first_fix_hold_time;
static timer_handle_t timer_gps_watchdog;
static timer_handle_t timer_log_flush;
static timer_handle_t timer_saltwater_switch_hysteresis;
static timer_handle_t timer_reed_switch_hysteresis;
static timer_handle_t timer_pressure_interval;
static timer_handle_t timer_pressure_maximum_acquisition;
static timer_handle_t timer_axl_interval;
static timer_handle_t timer_axl_maximum_acquisition;
static timer_handle_t timer_ble_interval;
static timer_handle_t timer_ble_duration;
static timer_handle_t timer_ble_timeout;

// Timer callbacks
static void timer_gps_interval_callback(void);
static void timer_gps_no_fix_callback(void);
static void timer_gps_maximum_acquisition_callback(void);
static void timer_gps_very_first_fix_hold_time_callback(void);
static void timer_gps_watchdog_callback(void);
static void timer_log_flush_callback(void);
static void timer_saltwater_switch_hysteresis_callback(void);
static void timer_reed_switch_hysteresis_callback(void);
static void timer_pressure_interval_callback(void);
static void timer_pressure_maximum_acquisition_callback(void);
static void timer_axl_interval_callback(void);
static void timer_axl_maximum_acquisition_callback(void);
static void timer_ble_interval_callback(void);
static void timer_ble_duration_callback(void);
static void timer_ble_timeout_callback(void);
static void soft_watchdog_callback(unsigned int);

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// PROTOTYPES ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void config_if_timeout_reset(void);
static void message_set_state(sm_message_state_t s);

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// STARTUP ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void setup_buffers(void)
{
    // Send buffer
    buffer_init_policy(pool, &config_if_send_buffer,
                       (uintptr_t) &config_if_send_buffer_pool[0],
                       sizeof(config_if_send_buffer_pool), 2);

    // Receive buffer
    buffer_init_policy(pool, &config_if_receive_buffer,
                       (uintptr_t) &config_if_receive_buffer_pool[0],
                       sizeof(config_if_receive_buffer_pool), 1);

    // Logging buffer
    buffer_init_policy(pool, &logging_buffer,
                       (uintptr_t) &logging_buffer_pool[0],
                       sizeof(logging_buffer_pool), LOGGING_FIFO_DEPTH);
}

// Set all global varibles to their default values
// this is used to allow unit tests to start from a clean slate
static void set_default_global_values(void)
{
    message_state = SM_MESSAGE_STATE_IDLE;
    config_if_tx_pending = false;
    config_if_rx_queued = false;
    syshal_gps_bridging = false;
    syshal_ble_bridging = false;
    config_if_message_timeout = 0;
    config_if_connected = false;
    tracker_above_water = true;
    log_file_created = false;
    gps_ttff_reading_logged = false;
    last_battery_reading = 0;
    sensor_logging_enabled = false;
    ble_state = 0;
    file_handle = NULL;
    reed_switch_debounce = false;
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////// HELPER FUNCTIONS ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#define MIN(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })


static void soft_watchdog_callback(unsigned int lr)
{
    logging_soft_watchdog_t log_wdog;
    uint32_t bytes_written;

    if (sys_config.sys_config_logging_date_time_stamp_enable.contents.enable)
    {
        syshal_rtc_data_and_time_t current_time;
        logging_date_time_t log_date;

        syshal_rtc_get_date_and_time(&current_time);

        log_date.h.id = LOGGING_DATE_TIME;
        log_date.day = current_time.day;
        log_date.month = current_time.month;
        log_date.year = current_time.year;
        log_date.hours = current_time.hours;
        log_date.minutes = current_time.minutes;
        log_date.seconds = current_time.seconds;
        (void)fs_write(file_handle, &log_date, sizeof(log_date), &bytes_written);
    }

    log_wdog.h.id = LOGGING_SOFT_WDOG;
    log_wdog.watchdog_address = lr;
    (void)fs_write(file_handle, &log_wdog, sizeof(log_wdog), &bytes_written);

    /* Try to clean-up the log file since we are about to reset */
    (void)fs_close(file_handle);

    /* Execute a software reset */
    for (;;)
        syshal_pmu_reset();
}

static void config_if_send_priv(volatile buffer_t * buffer)
{
    if (config_if_tx_pending)
        Throw(EXCEPTION_TX_BUSY);

    uintptr_t addr;
    uint32_t length = buffer_read(buffer, &addr);

    if (length)
    {
        config_if_tx_pending = true;
        config_if_send((uint8_t *) addr, length); // Send response
    }
    else
    {
        Throw(EXCEPTION_TX_BUFFER_FULL);
    }
}

static void config_if_receive_length_priv(uint32_t length)
{
    if (!config_if_rx_queued)
    {
        // Queue receive
        uint8_t * receive_buffer;
        if (!buffer_write(&config_if_receive_buffer, (uintptr_t *)&receive_buffer))
            Throw(EXCEPTION_RX_BUFFER_FULL);

        if (CONFIG_IF_NO_ERROR == config_if_receive(receive_buffer, length))
            config_if_rx_queued = true;
    }
}

static void config_if_receive_priv(void)
{
    config_if_receive_length_priv(SYSHAL_USB_PACKET_SIZE);
}

/**
 * @brief      Determines if any essential configuration tags are not set
 *
 * @return     false if essential configuration tags are not set
 */
static bool check_configuration_tags_set(void)
{
    // Check that all configuration tags are set
    bool tag_not_set = false;
    uint16_t tag, last_index = 0;
    int ret;

    // Conditional branching booleans for checking what configuration tags are required to be set

    // If one of the following tags is not set, default to false
    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_TAG_LOGGING_ENABLE, NULL))
        sys_config.sys_config_logging_enable.contents.enable = false;

    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE, NULL))
        sys_config.sys_config_logging_group_sensor_readings_enable.contents.enable = false;

    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_TAG_LOGGING_START_END_SYNC_ENABLE, NULL))
        sys_config.sys_config_logging_start_end_sync_enable.contents.enable = false;

    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_TAG_LOGGING_DATE_TIME_STAMP_ENABLE, NULL))
        sys_config.sys_config_logging_date_time_stamp_enable.contents.enable = false;

    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_TAG_LOGGING_HIGH_RESOLUTION_TIMER_ENABLE, NULL))
        sys_config.sys_config_logging_high_resolution_timer_enable.contents.enable = false;

    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_TAG_GPS_LOG_POSITION_ENABLE, NULL))
        sys_config.sys_config_gps_log_position_enable.contents.enable = false;

    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_TAG_GPS_LOG_TTFF_ENABLE, NULL))
        sys_config.sys_config_gps_log_ttff_enable.contents.enable = false;

    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_SALTWATER_SWITCH_LOG_ENABLE, NULL))
        sys_config.sys_config_saltwater_switch_log_enable.contents.enable = false;

    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_TAG_TEMP_SENSOR_LOG_ENABLE, NULL))
        sys_config.sys_config_temp_sensor_log_enable.contents.enable = false;

    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_TAG_PRESSURE_SENSOR_LOG_ENABLE, NULL))
        sys_config.sys_config_pressure_sensor_log_enable.contents.enable = false;

    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_TAG_AXL_LOG_ENABLE, NULL))
        sys_config.sys_config_axl_log_enable.contents.enable = false;

    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_TAG_RTC_SYNC_TO_GPS_ENABLE, NULL))
        sys_config.sys_config_rtc_sync_to_gps_enable.contents.enable = false;

    if (SYS_CONFIG_ERROR_TAG_NOT_SET == sys_config_get(SYS_CONFIG_TAG_BATTERY_LOG_ENABLE, NULL))
        sys_config.sys_config_battery_log_enable.contents.enable = false;

    while (!sys_config_iterate(&tag, &last_index))
    {

        // Ignore any non-essential tags
        if (!sys_config.sys_config_logging_enable.contents.enable)
        {
            if (SYS_CONFIG_TAG_LOGGING_ENABLE == tag ||
                SYS_CONFIG_TAG_LOGGING_FILE_SIZE == tag ||
                SYS_CONFIG_TAG_LOGGING_FILE_TYPE == tag ||
                SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE == tag ||
                SYS_CONFIG_TAG_LOGGING_START_END_SYNC_ENABLE == tag ||
                SYS_CONFIG_TAG_LOGGING_DATE_TIME_STAMP_ENABLE == tag ||
                SYS_CONFIG_TAG_LOGGING_HIGH_RESOLUTION_TIMER_ENABLE == tag ||
                SYS_CONFIG_TAG_GPS_LOG_POSITION_ENABLE == tag ||
                SYS_CONFIG_TAG_GPS_LOG_TTFF_ENABLE == tag ||
                SYS_CONFIG_TAG_GPS_TRIGGER_MODE == tag ||
                SYS_CONFIG_TAG_GPS_SCHEDULED_ACQUISITION_INTERVAL == tag ||
                SYS_CONFIG_TAG_GPS_MAXIMUM_ACQUISITION_TIME == tag ||
                SYS_CONFIG_TAG_GPS_SCHEDULED_ACQUISITION_NO_FIX_TIMEOUT == tag ||
                SYS_CONFIG_TAG_GPS_VERY_FIRST_FIX_HOLD_TIME == tag ||
                SYS_CONFIG_TAG_GPS_DEBUG_LOGGING_ENABLE == tag ||
                SYS_CONFIG_SALTWATER_SWITCH_LOG_ENABLE == tag ||
                SYS_CONFIG_SALTWATER_SWITCH_HYSTERESIS_PERIOD == tag ||
                SYS_CONFIG_TAG_AXL_LOG_ENABLE == tag ||
                SYS_CONFIG_TAG_AXL_CONFIG == tag ||
                SYS_CONFIG_TAG_AXL_G_FORCE_HIGH_THRESHOLD == tag ||
                SYS_CONFIG_TAG_AXL_SAMPLE_RATE == tag ||
                SYS_CONFIG_TAG_AXL_MODE == tag ||
                SYS_CONFIG_TAG_PRESSURE_SENSOR_LOG_ENABLE == tag ||
                SYS_CONFIG_TAG_PRESSURE_SAMPLE_RATE == tag ||
                SYS_CONFIG_TAG_PRESSURE_LOW_THRESHOLD == tag ||
                SYS_CONFIG_TAG_PRESSURE_HIGH_THRESHOLD == tag ||
                SYS_CONFIG_TAG_PRESSURE_MODE == tag ||
                SYS_CONFIG_TAG_TEMP_SENSOR_LOG_ENABLE == tag ||
                SYS_CONFIG_TAG_TEMP_SENSOR_SAMPLE_RATE == tag ||
                SYS_CONFIG_TAG_TEMP_SENSOR_LOW_THRESHOLD == tag ||
                SYS_CONFIG_TAG_TEMP_SENSOR_HIGH_THRESHOLD == tag ||
                SYS_CONFIG_TAG_TEMP_SENSOR_MODE == tag)
            {
                continue;
            }
        }

        if (!sys_config.sys_config_gps_log_position_enable.contents.enable)
        {
            if (SYS_CONFIG_TAG_GPS_LOG_POSITION_ENABLE == tag ||
                SYS_CONFIG_TAG_GPS_TRIGGER_MODE == tag ||
                SYS_CONFIG_TAG_GPS_SCHEDULED_ACQUISITION_INTERVAL == tag ||
                SYS_CONFIG_TAG_GPS_MAXIMUM_ACQUISITION_TIME == tag ||
                SYS_CONFIG_TAG_GPS_SCHEDULED_ACQUISITION_NO_FIX_TIMEOUT == tag)
            {
                continue;
            }
        }

        if (!sys_config.sys_config_gps_log_ttff_enable.contents.enable)
        {
            if (SYS_CONFIG_TAG_GPS_LOG_TTFF_ENABLE == tag)
                continue;
        }

        if (!sys_config.sys_config_saltwater_switch_log_enable.contents.enable)
        {
            if (SYS_CONFIG_SALTWATER_SWITCH_LOG_ENABLE == tag)
                continue;
        }
        else
        {
            // If we're in switch only trigger mode, then ignore any options meant for SCHEDULED or HYBRID modes
            if (sys_config.sys_config_gps_trigger_mode.hdr.set &&
                SYS_CONFIG_GPS_TRIGGER_MODE_SWITCH_TRIGGERED == sys_config.sys_config_gps_trigger_mode.contents.mode)
            {
                if (SYS_CONFIG_TAG_GPS_SCHEDULED_ACQUISITION_INTERVAL == tag ||
                    SYS_CONFIG_TAG_GPS_MAXIMUM_ACQUISITION_TIME == tag ||
                    SYS_CONFIG_TAG_GPS_SCHEDULED_ACQUISITION_NO_FIX_TIMEOUT == tag)
                {
                    continue;
                }
            }
        }

        // We don't care about our last GPS location
        if (SYS_CONFIG_TAG_GPS_LAST_KNOWN_POSITION == tag)
            continue;

        // GPS first fix hold time is not required
        if (SYS_CONFIG_TAG_GPS_VERY_FIRST_FIX_HOLD_TIME == tag)
            continue;

        // GPS debug logging not required
        if (SYS_CONFIG_TAG_GPS_DEBUG_LOGGING_ENABLE == tag)
            continue;

        if (!sys_config.sys_config_temp_sensor_log_enable.contents.enable)
        {
            if (SYS_CONFIG_TAG_TEMP_SENSOR_LOG_ENABLE == tag ||
                SYS_CONFIG_TAG_TEMP_SENSOR_SAMPLE_RATE == tag ||
                SYS_CONFIG_TAG_TEMP_SENSOR_LOW_THRESHOLD == tag ||
                SYS_CONFIG_TAG_TEMP_SENSOR_HIGH_THRESHOLD == tag ||
                SYS_CONFIG_TAG_TEMP_SENSOR_MODE == tag)
            {
                continue;
            }
        }

        if (!sys_config.sys_config_pressure_sensor_log_enable.contents.enable)
        {
            if (SYS_CONFIG_TAG_PRESSURE_SENSOR_LOG_ENABLE == tag ||
                SYS_CONFIG_TAG_PRESSURE_SAMPLE_RATE == tag ||
                SYS_CONFIG_TAG_PRESSURE_LOW_THRESHOLD == tag ||
                SYS_CONFIG_TAG_PRESSURE_HIGH_THRESHOLD == tag ||
                SYS_CONFIG_TAG_PRESSURE_MODE == tag ||
                SYS_CONFIG_TAG_PRESSURE_SCHEDULED_ACQUISITION_INTERVAL == tag ||
                SYS_CONFIG_TAG_PRESSURE_MAXIMUM_ACQUISITION_TIME == tag)
            {
                continue;
            }
        }
        else
        {
            // If we're in periodic mode, then skip the low/high threshold tags
            if (sys_config.sys_config_pressure_mode.hdr.set &&
                SYS_CONFIG_PRESSURE_MODE_PERIODIC == sys_config.sys_config_pressure_mode.contents.mode)
            {
                if (SYS_CONFIG_TAG_PRESSURE_LOW_THRESHOLD == tag ||
                    SYS_CONFIG_TAG_PRESSURE_HIGH_THRESHOLD == tag)
                {
                    continue;
                }
            }
        }

        if (!sys_config.sys_config_axl_log_enable.contents.enable)
        {
            if (SYS_CONFIG_TAG_AXL_LOG_ENABLE == tag ||
                SYS_CONFIG_TAG_AXL_CONFIG == tag ||
                SYS_CONFIG_TAG_AXL_G_FORCE_HIGH_THRESHOLD == tag ||
                SYS_CONFIG_TAG_AXL_SAMPLE_RATE == tag ||
                SYS_CONFIG_TAG_AXL_MODE == tag ||
                SYS_CONFIG_TAG_AXL_SCHEDULED_ACQUISITION_INTERVAL == tag ||
                SYS_CONFIG_TAG_AXL_MAXIMUM_ACQUISITION_TIME == tag)
            {
                continue;
            }
        }
        else
        {
            // If we're in periodic mode, then skip the high threshold tag
            if (sys_config.sys_config_axl_mode.hdr.set &&
                SYS_CONFIG_AXL_MODE_PERIODIC == sys_config.sys_config_axl_mode.contents.mode)
            {
                if (SYS_CONFIG_TAG_AXL_G_FORCE_HIGH_THRESHOLD == tag)
                {
                    continue;
                }
            }
        }

        if (!sys_config.sys_config_rtc_sync_to_gps_enable.contents.enable)
        {
            if (SYS_CONFIG_TAG_RTC_SYNC_TO_GPS_ENABLE == tag)
                continue;
        }

        if (SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE == tag)
            continue;

        if (SYS_CONFIG_TAG_LOGGING_START_END_SYNC_ENABLE == tag)
            continue;

        if (SYS_CONFIG_TAG_LOGGING_DATE_TIME_STAMP_ENABLE == tag)
            continue;

        if (SYS_CONFIG_TAG_LOGGING_HIGH_RESOLUTION_TIMER_ENABLE == tag)
            continue;

        if (SYS_CONFIG_TAG_BATTERY_LOG_ENABLE == tag)
            continue;

        // It does not matter if the low battery threshold is set or not
        if (SYS_CONFIG_TAG_BATTERY_LOW_THRESHOLD == tag)
            continue;

        // We don't care about the bluetooth device address
        if (SYS_CONFIG_TAG_BLUETOOTH_DEVICE_ADDRESS == tag)
            continue;

        // We let the BLE device choose advertising interval if not set
        if (SYS_CONFIG_TAG_BLUETOOTH_ADVERTISING_INTERVAL == tag)
            continue;

        // We let the BLE device choose connection interval if not set
        if (SYS_CONFIG_TAG_BLUETOOTH_CONNECTION_INTERVAL == tag)
            continue;

        // We let the BLE device choose PHY if not set
        if (SYS_CONFIG_TAG_BLUETOOTH_PHY_MODE == tag)
            continue;

        ret = sys_config_get(tag, NULL);

        if (SYS_CONFIG_ERROR_TAG_NOT_SET == ret)
        {
            tag_not_set = true;
            //DEBUG_PR_WARN("Configuration tag 0x%04X not set", tag);
        }
    }

    return !tag_not_set;
}

void logging_add_to_buffer(uint8_t * data, uint32_t size)
{
    uint32_t length = 0;
    uint8_t * buf_ptr;
    if (!buffer_write(&logging_buffer, (uintptr_t *)&buf_ptr))
    {
        DEBUG_PR_ERROR("LOG BUFFER FULL");
        return; // If our logging buffer is full then just ignore this data
    }

    static syshal_rtc_data_and_time_t last_log_time;

    // Are we supposed to be adding a timestamp with this value?
    if (sys_config.sys_config_logging_date_time_stamp_enable.hdr.set &&
        sys_config.sys_config_logging_date_time_stamp_enable.contents.enable)
    {
        syshal_rtc_data_and_time_t current_time;
        bool log_time = true;

        syshal_rtc_get_date_and_time(&current_time);

        // Are we supposed to be grouping every log entry that happens within the same second together?
        if (sys_config.sys_config_logging_group_sensor_readings_enable.hdr.set &&
            sys_config.sys_config_logging_group_sensor_readings_enable.contents.enable)
        {
            // Has our time changed since the last log entry?
            if (last_log_time.year == current_time.year ||
                last_log_time.month == current_time.month ||
                last_log_time.day == current_time.day ||
                last_log_time.hours == current_time.hours ||
                last_log_time.minutes == current_time.minutes ||
                last_log_time.seconds == current_time.seconds)
            {
                last_log_time = current_time;
                log_time = false; // Time has not changed, so do not log it
            }

        }

        if (log_time)
        {
            logging_date_time_t * date_time = (logging_date_time_t *) buf_ptr;

            LOGGING_SET_HDR(date_time, LOGGING_DATE_TIME);

            date_time->year = current_time.year;
            date_time->month = current_time.month;
            date_time->day = current_time.day;
            date_time->hours = current_time.hours;
            date_time->minutes = current_time.minutes;
            date_time->seconds = current_time.seconds;

            buf_ptr += sizeof(logging_date_time_t);
            length += sizeof(logging_date_time_t);
        }
    }

    if (sys_config.sys_config_logging_high_resolution_timer_enable.contents.enable)
    {
        DEBUG_PR_ERROR("logging_high_resolution_timer NOT IMPLEMENTED");
    }

    // Add the supplied data to the buffer
    memcpy(buf_ptr, data, size);
    length += size;

    buffer_write_advance(&logging_buffer, length);
}

void GPS_on(void)
{
    // Start GPS watchdog
    syshal_timer_set(timer_gps_watchdog, one_shot, GPS_WATCHDOG_TIME_SECONDS);

    if (SM_GPS_STATE_ASLEEP != sm_gps_state)
        return; // GPS already awake

    sm_gps_state = SM_GPS_STATE_ACQUIRING;
    gps_ttff_reading_logged = false;

    syshal_gps_wake_up();

    // Log the GPS switched on event
    if (sys_config.sys_config_gps_debug_logging_enable.hdr.set &&
        sys_config.sys_config_gps_debug_logging_enable.contents.enable)
    {
        logging_log_gps_on_t gps_on_log;
        LOGGING_SET_HDR(&gps_on_log, LOGGING_GPS_ON);
        logging_add_to_buffer((uint8_t *) &gps_on_log, sizeof(gps_on_log));
    }
}

void GPS_off(void)
{
    if (SM_GPS_STATE_ASLEEP == sm_gps_state)
        return; // GPS already asleep

    // Stop GPS watchdog
    syshal_timer_cancel(timer_gps_watchdog);

    syshal_gps_shutdown();

    sm_gps_state = SM_GPS_STATE_ASLEEP;

    // Log the GPS switched off event
    if (sys_config.sys_config_gps_debug_logging_enable.hdr.set &&
        sys_config.sys_config_gps_debug_logging_enable.contents.enable)
    {
        logging_log_gps_off_t gps_off_log;
        LOGGING_SET_HDR(&gps_off_log, LOGGING_GPS_OFF);
        logging_add_to_buffer((uint8_t *) &gps_off_log, sizeof(gps_off_log));
    }
}

void GPS_off_no_log(void)
{
    if (SM_GPS_STATE_ASLEEP == sm_gps_state)
        return; // GPS already asleep

    // Stop GPS watchdog
    syshal_timer_cancel(timer_gps_watchdog);

    syshal_gps_shutdown();

    sm_gps_state = SM_GPS_STATE_ASLEEP;
}

// Start or stop BLE based on ble_state triggers
void manage_ble(void)
{
    // Should we start out BLE scheduled timer?
    if (sys_config.sys_config_tag_bluetooth_scheduled_interval.hdr.set &&
        sys_config.sys_config_tag_bluetooth_scheduled_duration.hdr.set &&
        sys_config.sys_config_tag_bluetooth_trigger_control.hdr.set &&
        sys_config.sys_config_tag_bluetooth_trigger_control.contents.flags & SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_SCHEDULED)
    {
        if (sys_config.sys_config_tag_bluetooth_scheduled_interval.contents.seconds)
        {
            if (!syshal_timer_running(timer_ble_interval))
            {
                syshal_timer_set(timer_ble_interval, periodic, sys_config.sys_config_tag_bluetooth_scheduled_interval.contents.seconds);
            }
        }
        else
        {
            ble_state |= SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_SCHEDULED; // As the scheduled interval = 0 this is a special case to mean bluetooth is always on
        }
    }
    else
    {
        syshal_timer_cancel(timer_ble_interval);
    }

    if (ble_state &&
        CONFIG_IF_BACKEND_NOT_SET == config_if_current())
    {
        // Should we log this event
        if (sys_config.sys_config_tag_bluetooth_log_enable.hdr.set &&
            sys_config.sys_config_tag_bluetooth_log_enable.contents.enable)
        {
            logging_ble_enabled_t ble_enabled;
            LOGGING_SET_HDR(&ble_enabled, LOGGING_BLE_ENABLED);

            if (ble_state & SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_REED_SWITCH)
                ble_enabled.cause = LOGGING_BLE_ENABLED_CAUSE_REED_SWITCH;
            else if (ble_state & SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_SCHEDULED)
                ble_enabled.cause = LOGGING_BLE_ENABLED_CAUSE_SCHEDULE_TIMER;
            else if (ble_state & SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_GEOFENCE)
                ble_enabled.cause = LOGGING_BLE_ENABLED_CAUSE_GEOFENCE;

            logging_add_to_buffer((uint8_t *) &ble_enabled, sizeof(ble_enabled));
        }

        config_if_init(CONFIG_IF_BACKEND_BLE);
    }
}

static void setup_GPS_based_on_configuration(void)
{
    // GPS switch activated trigger mode
    if (SYS_CONFIG_GPS_TRIGGER_MODE_SWITCH_TRIGGERED == sys_config.sys_config_gps_trigger_mode.contents.mode)
    {
        if (tracker_above_water)
        {
            GPS_on();

            // Do we have a maximum acquisition time to adhere to?
            if (sys_config.sys_config_gps_maximum_acquisition_time.contents.seconds)
                syshal_timer_set(timer_gps_maximum_acquisition, one_shot, sys_config.sys_config_gps_maximum_acquisition_time.contents.seconds);
        }
        else
        {
            GPS_off();
        }
    }

    // GPS scheduled trigger mode
    if (SYS_CONFIG_GPS_TRIGGER_MODE_SCHEDULED == sys_config.sys_config_gps_trigger_mode.contents.mode)
    {
        if (sys_config.sys_config_gps_scheduled_acquisition_interval.contents.seconds)
        {
            GPS_off();
            syshal_timer_set(timer_gps_interval, periodic, sys_config.sys_config_gps_scheduled_acquisition_interval.contents.seconds);
        }
        else
        {
            GPS_on();
        }
    }

    // GPS hybrid trigger mode
    if (SYS_CONFIG_GPS_TRIGGER_MODE_HYBRID == sys_config.sys_config_gps_trigger_mode.contents.mode)
    {

        if (sys_config.sys_config_gps_scheduled_acquisition_interval.contents.seconds)
        {
            // If we're above the surface than turn on the GPS
            if (tracker_above_water)
            {
                GPS_on();

                // Do we have a maximum acquisition time to adhere to?
                if (sys_config.sys_config_gps_maximum_acquisition_time.contents.seconds)
                    syshal_timer_set(timer_gps_maximum_acquisition, one_shot, sys_config.sys_config_gps_maximum_acquisition_time.contents.seconds);
            }
            else
            {
                GPS_off();
            }

            // Start our interval timer
            syshal_timer_set(timer_gps_interval, periodic, sys_config.sys_config_gps_scheduled_acquisition_interval.contents.seconds);
        }
        else
        {
            GPS_on();
        }

    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////// CALLBACK FUNCTIONS //////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void syshal_axl_callback(syshal_axl_data_t data)
{
    // If accelerometer data logging is disabled
    if (!sys_config.sys_config_axl_log_enable.contents.enable)
    {
        syshal_axl_sleep(); // Sleep the accelerometer device
        return;
    }

    if (!sensor_logging_enabled)
        return;

    switch (sys_config.sys_config_axl_mode.contents.mode)
    {
        case SYS_CONFIG_AXL_MODE_PERIODIC:
            __NOP(); // NOP required to give the switch case an instruction to jump to
            logging_axl_xyz_t axl;
            LOGGING_SET_HDR(&axl, LOGGING_AXL_XYZ);
            axl.x = data.x;
            axl.y = data.y;
            axl.z = data.z;
            logging_add_to_buffer((uint8_t *) &axl, sizeof(axl));
            break;

        case SYS_CONFIG_AXL_MODE_TRIGGER_ABOVE:
            // Calculate vector magnitude
            __NOP(); // NOP required to give the switch case an instruction to jump to
            uint16_t magnitude_squared = (data.x * data.x) + (data.y * data.y) + (data.z * data.z); // WARN uint16_t maybe too small to contain true value
            // Determine if the read data is above the trigger point
            if (magnitude_squared >= sys_config.sys_config_axl_g_force_high_threshold.contents.threshold)
                __NOP(); // FIXME: Log data!
            break;
    }
}

void syshal_pressure_callback(int32_t pressure)
{
    // If pressure logging is disabled
    if (!sys_config.sys_config_pressure_sensor_log_enable.contents.enable)
    {
        syshal_pressure_sleep(); // Sleep the pressure device
        return;
    }

    if (!sensor_logging_enabled)
        return;

    logging_pressure_t pressure_data;
    LOGGING_SET_HDR(&pressure_data, LOGGING_PRESSURE);
    pressure_data.pressure = pressure;
    logging_add_to_buffer((uint8_t *) &pressure_data, sizeof(pressure_data));
}

void syshal_gps_callback(syshal_gps_event_t event)
{
    // Kick the GPS watchdog
    syshal_timer_reset(timer_gps_watchdog);

    // If gps data logging is disabled
    if (!sys_config.sys_config_gps_log_position_enable.contents.enable &&
        !sys_config.sys_config_gps_log_ttff_enable.contents.enable)
    {
        GPS_off();
        return;
    }

    if (!sensor_logging_enabled)
        return;

    switch (event.event_id)
    {
        case SYSHAL_GPS_EVENT_STATUS:
            DEBUG_PR_TRACE("SYSHAL_GPS_EVENT_STATUS - Fix: %u", event.event_data.status.gpsFix);

            if (event.event_data.status.gpsFix > 0)
            {
                if (SM_GPS_STATE_ASLEEP != sm_gps_state) // If we haven't already slept the GPS
                {
                    syshal_timer_cancel(timer_gps_no_fix); // Clear any no fix timer
                    sm_gps_state = SM_GPS_STATE_FIXED;
                }

                // If TTFF logging is enabled then log this
                if (!gps_ttff_reading_logged &&
                    sys_config.sys_config_gps_log_ttff_enable.contents.enable)
                {
                    logging_gps_ttff_t gps_ttff;

                    LOGGING_SET_HDR(&gps_ttff, LOGGING_GPS_TTFF);
                    gps_ttff.ttff = event.event_data.status.ttff;
                    logging_add_to_buffer((uint8_t *) &gps_ttff, sizeof(gps_ttff));

                    gps_ttff_reading_logged = true;
                }

            }
            else
            {
                if (SM_GPS_STATE_ASLEEP != sm_gps_state) // If we haven't already slept the GPS
                {
                    // Have we just lost GPS fix?
                    if (SM_GPS_STATE_FIXED == sm_gps_state)
                    {
                        // Are we supposed to be scheduling a no fix timer?
                        // If our interval time is 0 this is a special case meaning run the GPS forever
                        if (sys_config.sys_config_gps_scheduled_acquisition_interval.contents.seconds)
                        {
                            // If we are and we are either in scheduled mode or hybrid + underwater then start the no-fix timer
                            if ((SYS_CONFIG_GPS_TRIGGER_MODE_SCHEDULED == sys_config.sys_config_gps_trigger_mode.contents.mode) ||
                                ((SYS_CONFIG_GPS_TRIGGER_MODE_HYBRID == sys_config.sys_config_gps_trigger_mode.contents.mode) && (!tracker_above_water)))
                            {
                                if (sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout.contents.seconds) // Don't set a no fix timeout if it's zero as this is a special case
                                    syshal_timer_set(timer_gps_no_fix, one_shot, sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout.contents.seconds);
                            }
                        }
                    }

                    sm_gps_state = SM_GPS_STATE_ACQUIRING;
                }
            }
            break;

        case SYSHAL_GPS_EVENT_POSLLH:
            DEBUG_PR_TRACE("SYSHAL_GPS_EVENT_POSLLH - lat,long: %ld,%ld", event.event_data.location.lat, event.event_data.location.lon);

            // If we have a GPS fix
            if (SM_GPS_STATE_FIXED == sm_gps_state)
            {
                // Store this value into our last known location configuration interface tag
                sys_config.sys_config_gps_last_known_position.hdr.set = true;
                sys_config.sys_config_gps_last_known_position.contents.iTOW = event.event_data.location.iTOW;
                sys_config.sys_config_gps_last_known_position.contents.lon = event.event_data.location.lon;
                sys_config.sys_config_gps_last_known_position.contents.lat = event.event_data.location.lat;
                sys_config.sys_config_gps_last_known_position.contents.height = event.event_data.location.hMSL;
                sys_config.sys_config_gps_last_known_position.contents.hAcc = event.event_data.location.hAcc;
                sys_config.sys_config_gps_last_known_position.contents.vAcc = event.event_data.location.vAcc;

                syshal_rtc_data_and_time_t current_time;
                syshal_rtc_get_date_and_time(&current_time);

                sys_config.sys_config_gps_last_known_position.contents.day = current_time.day;
                sys_config.sys_config_gps_last_known_position.contents.month = current_time.month;
                sys_config.sys_config_gps_last_known_position.contents.year = current_time.year;
                sys_config.sys_config_gps_last_known_position.contents.hours = current_time.hours;
                sys_config.sys_config_gps_last_known_position.contents.minutes = current_time.minutes;
                sys_config.sys_config_gps_last_known_position.contents.seconds = current_time.seconds;

                // Add data to be logged
                logging_gps_position_t position;

                LOGGING_SET_HDR(&position, LOGGING_GPS_POSITION);
                position.iTOW = event.event_data.location.iTOW;
                position.lon = event.event_data.location.lon;
                position.lat = event.event_data.location.lat;
                position.height = event.event_data.location.hMSL;
                position.hAcc = event.event_data.location.hAcc;
                position.vAcc = event.event_data.location.vAcc;

                logging_add_to_buffer((uint8_t *) &position, sizeof(position));
            }
            break;

        default:
            DEBUG_PR_WARN("Unknown GPS event in %s() : %d", __FUNCTION__, event.event_id);
            break;
    }

}

void syshal_switch_callback(syshal_switch_event_id_t event)
{
    switch (event)
    {
        case SYSHAL_SWITCH_EVENT_OPEN:
            syshal_timer_cancel(timer_saltwater_switch_hysteresis);

            // If we're in the operational state and we were previously underwater
            if (sensor_logging_enabled && !tracker_above_water)
            {
                if (sys_config.sys_config_saltwater_switch_log_enable.contents.enable)
                {
                    logging_surfaced_t surfaced;
                    LOGGING_SET_HDR(&surfaced, LOGGING_SURFACED);
                    logging_add_to_buffer((uint8_t *) &surfaced, sizeof(surfaced));
                }

                // Are we supposed to be waking the GPS?
                if (SYS_CONFIG_GPS_TRIGGER_MODE_SWITCH_TRIGGERED == sys_config.sys_config_gps_trigger_mode.contents.mode
                    || SYS_CONFIG_GPS_TRIGGER_MODE_HYBRID == sys_config.sys_config_gps_trigger_mode.contents.mode)
                {
                    GPS_on();

                    // If our maximum acquisition time is not set to forever (0)
                    if (sys_config.sys_config_gps_maximum_acquisition_time.contents.seconds)
                        syshal_timer_set(timer_gps_maximum_acquisition, one_shot, sys_config.sys_config_gps_maximum_acquisition_time.contents.seconds);
                }

                syshal_timer_cancel(timer_gps_no_fix); // We ignore the no fix timeout when on the surface
            }

            tracker_above_water = true;
            break;

        case SYSHAL_SWITCH_EVENT_CLOSED:
            if (sys_config.sys_config_saltwater_switch_hysteresis_period.contents.seconds &&
                sys_config.sys_config_saltwater_switch_hysteresis_period.hdr.set &&
                sensor_logging_enabled)
                syshal_timer_set(timer_saltwater_switch_hysteresis, one_shot, sys_config.sys_config_saltwater_switch_hysteresis_period.contents.seconds);
            else
                timer_saltwater_switch_hysteresis_callback(); // Trigger an instant switch timeout
            break;

        default:
            DEBUG_PR_WARN("Unknown switch event in %s() : %d", __FUNCTION__, event);
            break;
    }
}

static void gpio_reed_sw_callback(void)
{
    DEBUG_PR_TRACE("%s() state: %d", __FUNCTION__, syshal_gpio_get_input(GPIO_REED_SW));

    if (!reed_switch_debounce)
    {
        reed_switch_debounce = true;
        syshal_timer_set(timer_reed_switch_hysteresis, one_shot, REED_SWITCH_DEBOUNCE_TIME_S);

        // Should we be using the reed switch to trigger BLE activation?
        if (sys_config.sys_config_tag_bluetooth_trigger_control.hdr.set &&
            sys_config.sys_config_tag_bluetooth_trigger_control.contents.flags | SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_REED_SWITCH)
        {
            if (!syshal_gpio_get_input(GPIO_REED_SW))
            {
                ble_state |= SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_REED_SWITCH;
            }
            else
            {
                ble_state &= (uint8_t) ~SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_REED_SWITCH;

                // Was the reed switch the only reason the the BLE interface was running?
                if (!ble_state &&
                    CONFIG_IF_BACKEND_BLE == config_if_current())
                {
                    // Should we log this event
                    if (sys_config.sys_config_tag_bluetooth_log_enable.hdr.set &&
                        sys_config.sys_config_tag_bluetooth_log_enable.contents.enable)
                    {
                        logging_ble_enabled_t ble_disabled;
                        LOGGING_SET_HDR(&ble_disabled, LOGGING_BLE_DISABLED);
                        ble_disabled.cause = LOGGING_BLE_DISABLED_CAUSE_REED_SWITCH;
                        logging_add_to_buffer((uint8_t *) &ble_disabled, sizeof(ble_disabled));
                    }

                    // If so then terminate it
                    config_if_term();

                    if (config_if_connected)
                    {
                        // If the BLE device was connected, trigger a disconnect event
                        config_if_event_t disconnectEvent;
                        disconnectEvent.backend = CONFIG_IF_BACKEND_BLE;
                        disconnectEvent.id = CONFIG_IF_EVENT_DISCONNECTED;
                        config_if_callback(&disconnectEvent);
                    }
                }
            }
        }
    }
}

static void timer_gps_interval_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    // If we are and we are either in scheduled mode or hybrid + underwater
    if ((SYS_CONFIG_GPS_TRIGGER_MODE_SCHEDULED == sys_config.sys_config_gps_trigger_mode.contents.mode) ||
        ((SYS_CONFIG_GPS_TRIGGER_MODE_HYBRID == sys_config.sys_config_gps_trigger_mode.contents.mode) && (!tracker_above_water)))
    {
        // Our scheduled interval has elapsed
        // So wake up the GPS
        GPS_on();

        syshal_timer_set(timer_gps_maximum_acquisition, one_shot, sys_config.sys_config_gps_maximum_acquisition_time.contents.seconds);

        if (sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout.contents.seconds) // Don't set a no fix timeout if it's zero as this is a special case
            syshal_timer_set(timer_gps_no_fix, one_shot, sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout.contents.seconds);
    }
}

static void timer_gps_no_fix_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    // If we are and we are either in scheduled mode or hybrid + underwater
    if ((SYS_CONFIG_GPS_TRIGGER_MODE_SCHEDULED == sys_config.sys_config_gps_trigger_mode.contents.mode) ||
        ((SYS_CONFIG_GPS_TRIGGER_MODE_HYBRID == sys_config.sys_config_gps_trigger_mode.contents.mode) && (!tracker_above_water)))
    {
        syshal_timer_cancel(timer_gps_maximum_acquisition);
        // We have been unable to achieve a GPS fix
        // So shutdown the GPS
        GPS_off();
    }
}

static void timer_gps_maximum_acquisition_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    syshal_timer_cancel(timer_gps_no_fix);

    // We have been GPS logging for our maximum allowed time
    // So shutdown the GPS
    GPS_off();
}

static void timer_gps_very_first_fix_hold_time_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    gps_waiting_for_first_fix = false; // First fix hold time is complete
    green_led_flashing = false;
    syshal_gpio_set_output_low(GPIO_LED1_GREEN);
    setup_GPS_based_on_configuration(); // Set the GPS to standard operation
}

static void timer_gps_watchdog_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    if (!sensor_logging_enabled)
        return;

    // The GPS has been unresponsive for a suspiciously long time. Try waking it up again
    if (SM_GPS_STATE_ASLEEP != sm_gps_state)
    {
        // Reset GPS watchdog
        syshal_timer_set(timer_gps_watchdog, one_shot, GPS_WATCHDOG_TIME_SECONDS);

        sm_gps_state = SM_GPS_STATE_ACQUIRING;
        gps_ttff_reading_logged = false;

        // Reset the GPS device
        syshal_gps_shutdown();
        syshal_time_delay_ms(10);
        syshal_gps_wake_up();

        // Log the GPS switched on event
        if (sys_config.sys_config_gps_debug_logging_enable.hdr.set &&
            sys_config.sys_config_gps_debug_logging_enable.contents.enable)
        {
            logging_log_gps_on_t gps_on_log;
            LOGGING_SET_HDR(&gps_on_log, LOGGING_GPS_ON);
            logging_add_to_buffer((uint8_t *) &gps_on_log, sizeof(gps_on_log));
        }

    }
}

static void timer_log_flush_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    // Flush and log data to the FLASH
    if (file_handle)
        fs_flush(file_handle);
}

static void timer_saltwater_switch_hysteresis_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    // The tracker has been underwater for the configured amount of time
    tracker_above_water = false;

    if (sensor_logging_enabled)
    {
        if (sys_config.sys_config_saltwater_switch_log_enable.contents.enable)
        {
            logging_submerged_t submerged;
            LOGGING_SET_HDR(&submerged, LOGGING_SUBMERGED);
            logging_add_to_buffer((uint8_t *) &submerged, sizeof(submerged));
        }

        // Are we supposed to be sleeping the GPS?
        if (SYS_CONFIG_GPS_TRIGGER_MODE_SWITCH_TRIGGERED == sys_config.sys_config_gps_trigger_mode.contents.mode
            || SYS_CONFIG_GPS_TRIGGER_MODE_HYBRID == sys_config.sys_config_gps_trigger_mode.contents.mode)
        {
            // We are no longer aquiring GPS data so cancel any timer for this
            syshal_timer_cancel(timer_gps_maximum_acquisition);
            GPS_off();
        }
    }
}

static void timer_reed_switch_hysteresis_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    reed_switch_debounce = false;

    gpio_reed_sw_callback(); // Check for any state changes
    syshal_timer_cancel(timer_reed_switch_hysteresis);
    reed_switch_debounce = false;
}

static void timer_pressure_interval_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    // Start the max acquisition timer
    syshal_timer_set(timer_pressure_maximum_acquisition, one_shot, sys_config.sys_config_pressure_maximum_acquisition_time.contents.seconds);

    // Start the sampling timer
    syshal_pressure_wake();
}

static void timer_pressure_maximum_acquisition_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    syshal_pressure_sleep(); // Stop the pressure sensor
}

static void timer_axl_interval_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    syshal_timer_set(timer_axl_maximum_acquisition, one_shot, sys_config.sys_config_axl_maximum_acquisition_time.contents.seconds);
    syshal_axl_wake();
}

static void timer_axl_maximum_acquisition_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    syshal_axl_sleep();
}

static void timer_ble_interval_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    // Should we be using the reed switch to trigger BLE activation?
    if (sys_config.sys_config_tag_bluetooth_scheduled_interval.hdr.set &&
        sys_config.sys_config_tag_bluetooth_scheduled_duration.hdr.set &&
        sys_config.sys_config_tag_bluetooth_trigger_control.hdr.set &&
        sys_config.sys_config_tag_bluetooth_trigger_control.contents.flags | SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_SCHEDULED)
    {
        ble_state |= SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_SCHEDULED;
        syshal_timer_set(timer_ble_duration, one_shot, sys_config.sys_config_tag_bluetooth_scheduled_duration.contents.seconds);
    }
}

static void timer_ble_duration_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    ble_state &= (uint8_t) ~SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_SCHEDULED;

    // If we've not managed to connect during the duration period
    if (!config_if_connected &&
        !ble_state) // And there is no other reason to keep the BLE interface running
    {
        // Should we log this event
        if (sys_config.sys_config_tag_bluetooth_log_enable.hdr.set &&
            sys_config.sys_config_tag_bluetooth_log_enable.contents.enable)
        {
            logging_ble_enabled_t ble_disabled;
            LOGGING_SET_HDR(&ble_disabled, LOGGING_BLE_DISABLED);
            ble_disabled.cause = LOGGING_BLE_DISABLED_CAUSE_SCHEDULE_TIMER;
            logging_add_to_buffer((uint8_t *) &ble_disabled, sizeof(ble_disabled));
        }

        // Then terminate it
        config_if_term();
    }
}

static void timer_ble_timeout_callback(void)
{
    DEBUG_PR_TRACE("%s() called", __FUNCTION__);

    // This has been triggered because we have a bluetooth connection but no data has been sent or received for a while

    ble_state &= (uint8_t) ~SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_SCHEDULED;

    if (config_if_connected && // If we are currently connected
        !ble_state && // And there is no other reason to keep the BLE interface running (e.g. reed switch)
        CONFIG_IF_BACKEND_BLE == config_if_current()) // And we are currently using the bluetooth interface
    {
        // Should we log this event
        if (sys_config.sys_config_tag_bluetooth_log_enable.hdr.set &&
            sys_config.sys_config_tag_bluetooth_log_enable.contents.enable)
        {
            logging_ble_enabled_t ble_disabled;
            LOGGING_SET_HDR(&ble_disabled, LOGGING_BLE_DISABLED);
            ble_disabled.cause = LOGGING_BLE_DISABLED_CAUSE_INACTIVITY_TIMEOUT;
            logging_add_to_buffer((uint8_t *) &ble_disabled, sizeof(ble_disabled));
        }

        // If so then terminate it
        config_if_term();

        // And generate a disconnect event
        config_if_event_t disconnectEvent;
        disconnectEvent.backend = CONFIG_IF_BACKEND_BLE;
        disconnectEvent.id = CONFIG_IF_EVENT_DISCONNECTED;
        config_if_callback(&disconnectEvent);
    }
}

static void populate_log_file_size_tag(void)
{
    if (log_file_created)
    {
        fs_stat_t stat;
        int ret = fs_stat(file_system, FS_FILE_ID_LOG, &stat);
        if (FS_NO_ERROR == ret)
            sys_config.sys_config_logging_file_size.contents.file_size = stat.size;
        else
            sys_config.sys_config_logging_file_size.contents.file_size = 0;
    }
    else
    {
        sys_config.sys_config_logging_file_size.contents.file_size = 0;
    }
}

/**
 * @brief      Create the configuration file in FLASH memory
 *
 * @return     @ref FS_NO_ERROR on success.
 * @return     @ref FS_ERROR_FILE_ALREADY_EXISTS if the configuration file has
 *             already been created.
 * @return     @ref FS_ERROR_FLASH_MEDIA if the amount of data written to the file
 *             is not of expected length
 * @return     @ref FS_ERROR_NO_FREE_HANDLE if no file handle could be allocated.
 */
static int fs_create_configuration_data(void)
{
    // The configuration file does not exist so lets make one
    int ret = fs_open(file_system, &file_handle, FS_FILE_ID_CONF, FS_MODE_CREATE, NULL);

    if (FS_NO_ERROR != ret)
        return ret; // An unrecoverable error has occured

    fs_close(file_handle); // Close the newly created file and flush any data

    return FS_NO_ERROR;
}

/**
 * @brief      Deletes our configuration data file in FLASH
 *
 * @return     @ref FS_NO_ERROR on success.
 * @return     @ref FS_ERROR_FILE_NOT_FOUND if the configuration file was not
 *             found.
 * @return     @ref FS_ERROR_FILE_PROTECTED if the configuration file is write
 *             protected
 * @return     @ref FS_ERROR_NO_FREE_HANDLE if no file handle could be allocated.
 */
static int fs_delete_configuration_data(void)
{
    return (fs_delete(file_system, FS_FILE_ID_CONF));
}

/**
 * @brief      Write our configuration data from RAM to FLASH
 *
 * @return     @ref FS_NO_ERROR on success.
 * @return     @ref FS_ERROR_FILE_NOT_FOUND if the configuration file was not
 *             found.
 * @return     @ref FS_ERROR_FILE_PROTECTED if the configuration file is write
 *             protected
 * @return     @ref FS_ERROR_FLASH_MEDIA if the amount of data written to the file
 *             is not of expected length
 * @return     @ref FS_ERROR_NO_FREE_HANDLE if no file handle could be allocated.
 */
static int fs_set_configuration_data(void)
{
    int ret = fs_open(file_system, &file_handle, FS_FILE_ID_CONF, FS_MODE_WRITEONLY, NULL);

    if (FS_NO_ERROR != ret)
        return ret; // An unrecoverable error has occured

    // We have now created and opened our unformatted configuration file
    // So lets write our configuration data to it
    uint32_t bytes_written;

    // Ensure our configuration version flag is set
    sys_config.format_version = SYS_CONFIG_FORMAT_VERSION;

    ret = fs_write(file_handle, &sys_config, sizeof(sys_config), &bytes_written);

    fs_close(file_handle); // Close the file and flush any data

    if (FS_NO_ERROR != ret)
        return ret; // An unrecoverable error has occured

    if (bytes_written != sizeof(sys_config))
    {
        DEBUG_PR_WARN("%s() size mismatch", __FUNCTION__);
        return FS_ERROR_FLASH_MEDIA;
    }

    return FS_NO_ERROR;
}

/**
 * @brief      Load the configuration data from FLASH.
 *
 * @return     @ref FS_NO_ERROR on success.
 * @return     @ref FS_ERROR_FILE_NOT_FOUND if the configuration file was not
 *             found.
 * @return     @ref FS_ERROR_FLASH_MEDIA if the amount of data written to the file
 *             is not of expected length
 * @return     @ref FS_ERROR_NO_FREE_HANDLE if no file handle could be allocated.
 * @return     @ref FS_ERROR_FILE_VERSION_MISMATCH if the configuration file found
 *             is not compatible with this firmware.
 */
static int fs_get_configuration_data(void)
{
    // Attempt to open the configuration file
    int ret = fs_open(file_system, &file_handle, FS_FILE_ID_CONF, FS_MODE_READONLY, NULL);

    if (FS_NO_ERROR != ret)
        return ret; // An unrecoverable error has occured

    // Check the configuration version number matches our firmware one
    uint32_t bytes_read;

    uint8_t format_version = 0;
    ret = fs_read(file_handle, &format_version, sizeof(sys_config.format_version), &bytes_read);

    if (FS_NO_ERROR != ret)
        return ret; // An unrecoverable error has occured

    fs_close(file_handle); // We're done reading from this file

    if (SYS_CONFIG_FORMAT_VERSION != format_version)
    {
        DEBUG_PR_WARN("%s() configuration file is an incompatible format version", __FUNCTION__);
        return FS_ERROR_FILE_VERSION_MISMATCH;
    }

    // Populate our sys_config RAM struct from the FLASH file
    ret  = fs_open(file_system, &file_handle, FS_FILE_ID_CONF, FS_MODE_READONLY, NULL);
    ret |= fs_read(file_handle, &sys_config, sizeof(sys_config), &bytes_read);
    fs_close(file_handle); // We're done reading from this file

    if (FS_NO_ERROR != ret)
        return ret; // An unrecoverable error has occured

    if (bytes_read != sizeof(sys_config))
    {
        DEBUG_PR_WARN("%s() size mismatch", __FUNCTION__);

        // We have an erroneous configuration file, so lets unset all the tags we may have set incorrectly
        uint16_t last_index = 0;
        uint16_t tag;

        while (!sys_config_iterate(&tag, &last_index))
            sys_config_unset(tag);

        return FS_ERROR_FILE_VERSION_MISMATCH;
    }

    return FS_NO_ERROR;
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// CFG_READ ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void cfg_read_populate_next(uint16_t tag, void * src, uint16_t length)
{
    sm_context.cfg_read.buffer_base[sm_context.cfg_read.buffer_offset++] = (uint8_t) (tag);
    sm_context.cfg_read.buffer_base[sm_context.cfg_read.buffer_offset++] = (uint8_t) (tag >> 8);
    memcpy(&sm_context.cfg_read.buffer_base[sm_context.cfg_read.buffer_offset],
           src, length);
    sm_context.cfg_read.buffer_offset += length;
}

void cfg_read_populate_buffer(void)
{
    uint16_t tag;
    int ret;

    /* Iterate configuration tags */
    while (!sys_config_iterate(&tag, &sm_context.cfg_read.last_index))
    {
        void * src;

        // If this is a request for the current log file size
        if (SYS_CONFIG_TAG_LOGGING_FILE_SIZE == tag)
            populate_log_file_size_tag();

        ret = sys_config_get(tag, &src);
        if (ret > 0)
        {
            if ((sm_context.cfg_read.buffer_offset + (uint32_t) ret + sizeof(uint16_t)) > SYSHAL_USB_PACKET_SIZE)
            {
                /* Buffer is full so defer this to the next iteration */
                sm_context.cfg_read.last_index--;
                break;
            }

            cfg_read_populate_next(tag, src, (uint16_t)ret);
        }
    }
}

uint32_t cfg_read_all_calc_length(void)
{
    int ret;
    uint16_t last_index = 0, tag;
    uint32_t length = 0;

    /* Iterate all configuration tags */
    while (!sys_config_iterate(&tag, &last_index))
    {
        void * src;
        ret = sys_config_get(tag, &src);
        if (ret > 0)
            length += ( (uint32_t) ret + sizeof(uint16_t));
    }

    return length;
}

void cfg_read_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_cfg_read_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);

    CMD_SET_HDR(resp, CMD_CFG_READ_RESP);

    /* Allocate buffer for following configuration data */
    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_cfg_read_resp_t));
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&sm_context.cfg_read.buffer_base))
        Throw(EXCEPTION_TX_BUFFER_FULL);

    /* Reset buffer offset to head of buffer */
    sm_context.cfg_read.buffer_offset = 0;

    if (CFG_READ_REQ_READ_ALL == req->p.cmd_cfg_read_req.configuration_tag)
    {
        /* Requested all configuration items */
        resp->p.cmd_cfg_read_resp.error_code = CMD_NO_ERROR;
        resp->p.cmd_cfg_read_resp.length = cfg_read_all_calc_length();
        sm_context.cfg_read.last_index = 0;
        sm_context.cfg_read.length = resp->p.cmd_cfg_read_resp.length;
        if (resp->p.cmd_cfg_read_resp.length > 0)
        {
            cfg_read_populate_buffer();
            buffer_write_advance(&config_if_send_buffer, sm_context.cfg_read.buffer_offset);
        }
    }
    else
    {
        void * src;
        /* Requested a single configuration tag */

        // If this is a request for the current log file size
        if (SYS_CONFIG_TAG_LOGGING_FILE_SIZE == req->p.cmd_cfg_read_req.configuration_tag)
            populate_log_file_size_tag();

        int ret = sys_config_get(req->p.cmd_cfg_read_req.configuration_tag, &src);

        if (ret < 0)
        {
            resp->p.cmd_cfg_read_resp.length = 0;
            if (SYS_CONFIG_ERROR_INVALID_TAG == ret)  // Tag is not valid. Return an error code
            {
                resp->p.cmd_cfg_read_resp.error_code = CMD_ERROR_INVALID_CONFIG_TAG;
            }
            else if (SYS_CONFIG_ERROR_TAG_NOT_SET == ret)  // Tag is not set. Return an error code
            {
                resp->p.cmd_cfg_read_resp.error_code = CMD_ERROR_CONFIG_TAG_NOT_SET;
            }
            else
            {
                DEBUG_PR_ERROR("Failed to retrieve tag 0x%04X, with error: %d", req->p.cmd_cfg_read_req.configuration_tag, ret);
                Throw(EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION);
            }
        }
        else
        {
            cfg_read_populate_next(req->p.cmd_cfg_read_req.configuration_tag, src, (uint16_t)ret);
            resp->p.cmd_cfg_read_resp.error_code = CMD_NO_ERROR;
            resp->p.cmd_cfg_read_resp.length = sm_context.cfg_read.buffer_offset;
            sm_context.cfg_read.length = sm_context.cfg_read.buffer_offset;
            buffer_write_advance(&config_if_send_buffer, sm_context.cfg_read.buffer_offset);
        }
    }

    config_if_send_priv(&config_if_send_buffer); // Send the response

    if (resp->p.cmd_cfg_read_resp.length > 0)
    {
        /* Another buffer must follow the initial response */
        message_set_state(SM_MESSAGE_STATE_CFG_READ_NEXT);
    }
}

void cfg_read_next_state(void)
{
    /* Send the pending buffer and prepare a new buffer */
    config_if_send_priv(&config_if_send_buffer);
    sm_context.cfg_read.length -= sm_context.cfg_read.buffer_offset;

    if (sm_context.cfg_read.length > 0)
    {
        /* Allocate buffer for following configuration data */
        if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&sm_context.cfg_read.buffer_base))
            Throw(EXCEPTION_TX_BUFFER_FULL);

        /* Reset buffer offset to head of buffer */
        sm_context.cfg_read.buffer_offset = 0;
        cfg_read_populate_buffer();

        /* Advance the buffer */
        buffer_write_advance(&config_if_send_buffer, sm_context.cfg_read.buffer_offset);
    }
    else
    {
        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// CFG_WRITE //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_write_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_cfg_write_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    sm_context.cfg_write.length = req->p.cmd_cfg_write_req.length;

    // Length is zero
    if (!sm_context.cfg_write.length)
        Throw(EXCEPTION_PACKET_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);
    resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer); // Send response

    config_if_receive_length_priv(sm_context.cfg_write.length); // Queue a receive

    sm_context.cfg_write.buffer_occupancy = 0;

    message_set_state(SM_MESSAGE_STATE_CFG_WRITE_NEXT);
}

static void cfg_write_next_state(void)
{
    uint32_t bytes_to_copy;
    uint8_t * read_buffer;
    uint32_t length = buffer_read(&config_if_receive_buffer, (uintptr_t *)&read_buffer);

    if (!length)
        return;

    buffer_read_advance(&config_if_receive_buffer, length); // Remove this packet from the receive buffer

    if (length > sm_context.cfg_write.length)
    {
        // We've received more data then we were expecting
        sm_context.cfg_write.error_code = CMD_ERROR_DATA_OVERSIZE;
        message_set_state(SM_MESSAGE_STATE_CFG_WRITE_ERROR);
        Throw(EXCEPTION_PACKET_WRONG_SIZE);
    }

    while (length)
    {

        // Do we have a tag ID in our working buffer?
        if (sm_context.cfg_write.buffer_occupancy < SYS_CONFIG_TAG_ID_SIZE)
        {
            // If not then put as much of our tag ID into our temp buffer as possible
            bytes_to_copy = MIN(length, SYS_CONFIG_TAG_ID_SIZE - sm_context.cfg_write.buffer_occupancy);

            memcpy(&sm_context.cfg_write.buffer[sm_context.cfg_write.buffer_occupancy], read_buffer, bytes_to_copy);

            sm_context.cfg_write.buffer_occupancy += bytes_to_copy;
            read_buffer += bytes_to_copy;
            length -= bytes_to_copy;
        }

        if (sm_context.cfg_write.buffer_occupancy < SYS_CONFIG_TAG_ID_SIZE)
            break; // If we still don't have at least a tag ID then wait for more data

        // Fetch the configuration tag
        uint16_t tag = 0;
        tag |= (uint16_t) sm_context.cfg_write.buffer[0] & 0x00FF;
        tag |= (uint16_t) (sm_context.cfg_write.buffer[1] << 8) & 0xFF00;

        // Determine the size of this configuration tag
        int tag_data_size = sys_config_size(tag);

        // If the tag is invalid
        if (tag_data_size < 0)
        {
            // Then we should exit out and return an error
            DEBUG_PR_ERROR("sys_config_size(0x%04X) returned: %d()", tag, tag_data_size);
            sm_context.cfg_write.error_code = CMD_ERROR_INVALID_CONFIG_TAG;
            message_set_state(SM_MESSAGE_STATE_CFG_WRITE_ERROR);
            Throw(EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION);
        }

        // Then lets put what we have into our working buffer, accounting for what we might already have in our buffer
        bytes_to_copy = MIN( length, MIN((uint32_t) tag_data_size, ((uint32_t) tag_data_size) - sm_context.cfg_write.buffer_occupancy + SYS_CONFIG_TAG_ID_SIZE));
        memcpy(&sm_context.cfg_write.buffer[sm_context.cfg_write.buffer_occupancy], read_buffer, bytes_to_copy);
        sm_context.cfg_write.buffer_occupancy += bytes_to_copy;
        read_buffer += bytes_to_copy;
        length -= bytes_to_copy;

        // Do we have all of the configuration tags data in our working buffer?
        if (sm_context.cfg_write.buffer_occupancy < tag_data_size + SYS_CONFIG_TAG_ID_SIZE)
            break; // Then wait until we do

        // Process the tag
        int ret = sys_config_set(tag, &sm_context.cfg_write.buffer[SYS_CONFIG_TAG_ID_SIZE], tag_data_size); // Set tag value

        if (ret < 0)
        {
            DEBUG_PR_ERROR("sys_config_set(0x%04X) returned: %d()", tag, ret);
            message_set_state(SM_MESSAGE_STATE_IDLE);
            Throw(EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION); // Exit and fail silent
        }

        DEBUG_PR_TRACE("sys_config_set(0x%04X)", tag);

        sm_context.cfg_write.length -= sm_context.cfg_write.buffer_occupancy;
        sm_context.cfg_write.buffer_occupancy = 0;

    }

    if (sm_context.cfg_write.length) // Is there still data to receive?
    {
        config_if_receive_length_priv(sm_context.cfg_write.length); // Queue a receive
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have received all the data
        // Then send a confirmation
        cmd_t * resp;
        if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
            Throw(EXCEPTION_TX_BUFFER_FULL);
        CMD_SET_HDR(resp, CMD_CFG_WRITE_CNF);
        resp->p.cmd_cfg_write_cnf.error_code = CMD_NO_ERROR;

        buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_cfg_write_cnf_t));
        config_if_send_priv(&config_if_send_buffer); // Send response

        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

static void cfg_write_error_state(void)
{
    // Return an error code
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_CFG_WRITE_CNF);
    resp->p.cmd_cfg_write_cnf.error_code = sm_context.cfg_write.error_code;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_cfg_write_cnf_t));
    config_if_send_priv(&config_if_send_buffer); // Send response

    message_set_state(SM_MESSAGE_STATE_IDLE);
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// CFG_SAVE ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_save_req(cmd_t * req, uint32_t size)
{
    UNUSED(req);

    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    int ret = fs_delete_configuration_data(); // Must first delete our configuration data

    switch (ret)
    {
        case FS_ERROR_FILE_NOT_FOUND: // If there is no configuration file, then make one
        case FS_NO_ERROR:

            ret = fs_create_configuration_data(); // Re/Create the file
            if (FS_NO_ERROR != ret)
                Throw(EXCEPTION_FS_ERROR);

            ret = fs_set_configuration_data(); // Flush our RAM configuration data to the file
            if (FS_NO_ERROR != ret)
                Throw(EXCEPTION_FS_ERROR);

            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
            break;

        case FS_ERROR_FILE_PROTECTED:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_CONFIG_PROTECTED;
            break;

        default:
            Throw(EXCEPTION_FS_ERROR);
            break;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer); // Send confirmation
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// CFG_RESTORE /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_restore_req(cmd_t * req, uint32_t size)
{
    UNUSED(req);

    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    int ret = fs_get_configuration_data();

    switch (ret)
    {
        case FS_NO_ERROR:
            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
            break;

        case FS_ERROR_FILE_NOT_FOUND:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_NOT_FOUND;
            break;

        case FS_ERROR_FILE_VERSION_MISMATCH:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_INCOMPATIBLE;
            break;

        default:
            Throw(EXCEPTION_FS_ERROR);
            break;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer); // Send confirmation
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// CFG_ERASE //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_erase_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_cfg_erase_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    if (CFG_ERASE_REQ_ERASE_ALL == req->p.cmd_cfg_erase_req.configuration_tag) // Erase all configuration tags
    {
        uint16_t last_index = 0;
        uint16_t tag;

        while (!sys_config_iterate(&tag, &last_index))
        {
            sys_config_unset(tag);
        }

        resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
    }
    else
    {
        // Erase just one configuration tag
        int return_code = sys_config_unset(req->p.cmd_cfg_erase_req.configuration_tag);

        switch (return_code)
        {
            case SYS_CONFIG_NO_ERROR:
                resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
                break;

            case SYS_CONFIG_ERROR_INVALID_TAG:
                resp->p.cmd_generic_resp.error_code = CMD_ERROR_INVALID_CONFIG_TAG;
                break;

            default:
                Throw(EXCEPTION_FS_ERROR);
                break;
        }
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer); // Send confirmation
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// CFG_PROTECT //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_protect_req(cmd_t * req, uint32_t size)
{
    UNUSED(req);

    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    int ret = fs_protect(file_system, FS_FILE_ID_CONF);

    switch (ret)
    {
        case FS_NO_ERROR:
            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
            break;

        case FS_ERROR_FILE_NOT_FOUND:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_NOT_FOUND;
            break;

        default:
            Throw(EXCEPTION_FS_ERROR);
            break;
    }

    // Send the response
    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// CFG_UNPROTECT /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void cfg_unprotect_req(cmd_t * req, uint32_t size)
{
    UNUSED(req);

    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    int ret = fs_unprotect(file_system, FS_FILE_ID_CONF);

    switch (ret)
    {
        case FS_NO_ERROR:
            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
            break;

        case FS_ERROR_FILE_NOT_FOUND:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_NOT_FOUND;
            break;

        default:
            Throw(EXCEPTION_FS_ERROR);
            break;
    }

    // Send the response
    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// GPS_WRITE ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void gps_write_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_gps_write_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    // If bridging is enabled
    if (syshal_gps_bridging)
    {
        sm_context.gps_write.length = req->p.cmd_gps_write_req.length;
        resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;

        config_if_receive_length_priv(sm_context.gps_write.length); // Queue a receive

        message_set_state(SM_MESSAGE_STATE_GPS_WRITE_NEXT);
    }
    else
    {
        // Bridging is not enabled so return an error code
        resp->p.cmd_generic_resp.error_code = CMD_ERROR_BRIDGING_DISABLED;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void gps_write_next_state(void)
{
    uint8_t * read_buffer;
    uint32_t length = buffer_read(&config_if_receive_buffer, (uintptr_t *)&read_buffer);

    if (!length)
        return;

    buffer_read_advance(&config_if_receive_buffer, length); // Remove this packet from the receive buffer

    if (length > sm_context.gps_write.length)
    {
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_PACKET_WRONG_SIZE);
    }

    int ret = syshal_gps_send_raw(read_buffer, length);

    // Check send worked
    if (ret < 0)
    {
        // If not we should exit out
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_GPS_SEND_ERROR);
    }

    sm_context.gps_write.length -= length;

    if (sm_context.gps_write.length) // Is there still data to receive?
    {
        config_if_receive_length_priv(sm_context.gps_write.length); // Queue a receive
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have received all the data
        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// GPS_READ ///////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void gps_read_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_gps_read_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GPS_READ_RESP);

    // If bridging is enabled
    if (syshal_gps_bridging)
    {
        // Try to match the requested length, if not return as close to it as we can
        //DEBUG_PR_TRACE("syshal_gps_available_raw() = %lu, req->p.cmd_gps_read_req.length = %lu", syshal_gps_available_raw(), req->p.cmd_gps_read_req.length);
        sm_context.gps_read.length = MIN(syshal_gps_available_raw(), req->p.cmd_gps_read_req.length);

        resp->p.cmd_gps_read_resp.length = sm_context.gps_read.length;
        resp->p.cmd_gps_read_resp.error_code = CMD_NO_ERROR;
    }
    else
    {
        // Bridging is not enabled so return an error code
        resp->p.cmd_gps_read_resp.length = 0;
        resp->p.cmd_gps_read_resp.error_code = CMD_ERROR_BRIDGING_DISABLED;
    }

    // Send response
    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_gps_read_resp_t));
    config_if_send_priv(&config_if_send_buffer);

    if (sm_context.gps_read.length > 0)
        message_set_state(SM_MESSAGE_STATE_GPS_READ_NEXT);
}

static void gps_read_next_state(void)
{
    // Generate and send response
    uint8_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);

    // Don't read more than the maximum packet size
    uint32_t bytes_to_read = MIN(sm_context.gps_read.length, (uint32_t) SYSHAL_USB_PACKET_SIZE);

    // Receive data from the GPS module
    uint32_t bytes_actually_read = syshal_gps_receive_raw(resp, bytes_to_read);

    sm_context.gps_read.length -= bytes_actually_read;

    // Send response
    buffer_write_advance(&config_if_send_buffer, bytes_actually_read);
    config_if_send_priv(&config_if_send_buffer);

    if (sm_context.gps_read.length) // Is there still data to send?
    {
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have sent all the data
        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// GPS_CONFIG_REQ ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void gps_config_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_gps_config_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    syshal_gps_bridging = req->p.cmd_gps_config_req.enable; // Disable or enable GPS bridging

    // If we've just enabled bridging, remove any previous data in the GPS rx buffer
    if (syshal_gps_bridging)
    {
        uint8_t flush;
        while (syshal_gps_receive_raw(&flush, 1))
        {}
    }

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void ble_config_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_ble_config_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    syshal_ble_bridging = req->p.cmd_ble_config_req.enable; // Disable or enable BLE bridging

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void ble_write_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_ble_write_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    // If bridging is enabled
    if (syshal_ble_bridging)
    {
        sm_context.ble_write.address = req->p.cmd_ble_write_req.address;
        sm_context.ble_write.length = req->p.cmd_ble_write_req.length;
        resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;

        config_if_receive_length_priv(sm_context.ble_write.length); // Queue a receive

        message_set_state(SM_MESSAGE_STATE_BLE_WRITE_NEXT);
    }
    else
    {
        // Bridging is not enabled so return an error code
        resp->p.cmd_generic_resp.error_code = CMD_ERROR_BRIDGING_DISABLED;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void ble_write_next_state(void)
{
    uint8_t * read_buffer;
    uint32_t length = buffer_read(&config_if_receive_buffer, (uintptr_t *)&read_buffer);

    if (!length)
        return;

    buffer_read_advance(&config_if_receive_buffer, length); // Remove this packet from the receive buffer

    if (length > sm_context.ble_write.length)
    {
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_PACKET_WRONG_SIZE);
    }

    // Send address
    if (syshal_spi_transfer(SPI_BLE, &sm_context.ble_write.address, NULL, sizeof(sm_context.ble_write.address)))
    {
        // If it failed we should exit
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_SPI_ERROR);
    }

    // Send data
    if (syshal_spi_transfer(SPI_BLE, read_buffer, NULL, length))
    {
        // If it failed we should exit
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_SPI_ERROR);
    }

    sm_context.ble_write.length -= length;

    if (sm_context.ble_write.length) // Is there still data to receive?
    {
        config_if_receive_length_priv(sm_context.ble_write.length); // Queue a receive
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have received all the data
        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

static void ble_read_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_ble_read_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    // If bridging is enabled
    if (syshal_ble_bridging)
    {
        sm_context.ble_read.address = req->p.cmd_ble_read_req.address;
        sm_context.ble_read.length = req->p.cmd_ble_read_req.length;
        resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
    }
    else
    {
        // Bridging is not enabled so return an error code
        sm_context.ble_read.length = 0;
        resp->p.cmd_generic_resp.error_code = CMD_ERROR_BRIDGING_DISABLED;
    }

    // Send response
    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);

    if (sm_context.ble_read.length > 0)
        message_set_state(SM_MESSAGE_STATE_BLE_READ_NEXT);
}

static void ble_read_next_state(void)
{
    // Generate and send response
    uint8_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);

    // Don't read more than the maximum packet size less one byte which shall
    // be used for storing the SPI bus address
    uint32_t bytes_to_read = MIN(sm_context.ble_read.length, SYSHAL_USB_PACKET_SIZE);

    // Send address
    if (syshal_spi_transfer(SPI_BLE, &sm_context.ble_read.address, NULL, sizeof(sm_context.ble_read.address)))
    {
        // If it failed we should exit
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_SPI_ERROR);
    }

    // Read data
    if (syshal_spi_transfer(SPI_BLE, resp, resp, bytes_to_read))
    {
        // If it failed we should exit
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_SPI_ERROR);
    }

    sm_context.ble_read.length -= bytes_to_read;

    // Send response
    buffer_write_advance(&config_if_send_buffer, bytes_to_read);
    config_if_send_priv(&config_if_send_buffer);

    if (sm_context.ble_read.length) // Is there still data to send?
    {
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have sent all the data
        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////// STATUS_REQ //////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void status_req(cmd_t * req, uint32_t size)
{
    UNUSED(req);

    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_STATUS_RESP);

    resp->p.cmd_status_resp.error_code = CMD_NO_ERROR;
    resp->p.cmd_status_resp.stm_firmware_version = STM32_FIRMWARE_VERSION;
    uint32_t version;
    syshal_ble_get_version(&version); // Get BLE version via spi
    resp->p.cmd_status_resp.ble_firmware_version = version;
    resp->p.cmd_status_resp.configuration_format_version = SYS_CONFIG_FORMAT_VERSION;

    // Get unique ID
    syshal_device_id(&(resp->p.cmd_status_resp.mcu_uid));

    // Get battery level
    resp->p.cmd_status_resp.charge_level = syshal_batt_level();

    // Get log file size
    fs_stat_t stat;
    int ret = fs_stat(file_system, FS_FILE_ID_LOG, &stat);
    if (FS_NO_ERROR == ret)
        resp->p.cmd_status_resp.log_file_size = stat.size;
    else
        resp->p.cmd_status_resp.log_file_size = 0;

    // Get which sensors are enabled
    resp->p.cmd_status_resp.pressure_enabled = sys_config.sys_config_pressure_sensor_log_enable.contents.enable;
    resp->p.cmd_status_resp.temp_enabled = sys_config.sys_config_temp_sensor_log_enable.contents.enable;
    resp->p.cmd_status_resp.accel_enabled = sys_config.sys_config_axl_log_enable.contents.enable;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_status_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void fw_send_image_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_fw_send_image_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    // Store variables for use in future
    sm_context.fw_send_image.length = req->p.cmd_fw_send_image_req.length;
    sm_context.fw_send_image.crc32_supplied = req->p.cmd_fw_send_image_req.CRC32;
    sm_context.fw_send_image.crc32_calculated = 0;

    DEBUG_PR_TRACE("Supplied CRC32 = %08x", (unsigned int)sm_context.fw_send_image.crc32_supplied);

    // Check the image type is correct
    sm_context.fw_send_image.image_type = req->p.cmd_fw_send_image_req.image_type;

    if ( (sm_context.fw_send_image.image_type == FS_FILE_ID_STM32_IMAGE)
         || (sm_context.fw_send_image.image_type == FS_FILE_ID_BLE_IMAGE) )
    {
        int ret = fs_delete(file_system, sm_context.fw_send_image.image_type); // Must first delete any current image

        switch (ret)
        {
            case FS_ERROR_FILE_NOT_FOUND: // If there is no image file, then make one
            case FS_NO_ERROR:
                __NOP(); // Instruct for switch case to jump to
                int ret = fs_open(file_system, &file_handle, sm_context.fw_send_image.image_type, FS_MODE_CREATE, NULL);
                if (FS_NO_ERROR != ret)
                    Throw(EXCEPTION_FS_ERROR); // An unrecoverable error has occured

                // FIXME: Check to see if there is sufficient room for the firmware image
                config_if_receive_length_priv(sm_context.fw_send_image.length); // Queue a receive
                resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
                message_set_state(SM_MESSAGE_STATE_FW_SEND_IMAGE_NEXT);
                break;

            case FS_ERROR_FILE_PROTECTED: // We never lock the fw images so this shouldn't occur
                resp->p.cmd_generic_resp.error_code = CMD_ERROR_CONFIG_PROTECTED;
                break;

            default:
                Throw(EXCEPTION_FS_ERROR);
                break;
        }
    }
    else
    {
        resp->p.cmd_generic_resp.error_code = CMD_ERROR_INVALID_FW_IMAGE_TYPE;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void fw_send_image_next_state(void)
{
    uint8_t * read_buffer;
    uint32_t length = buffer_read(&config_if_receive_buffer, (uintptr_t *)&read_buffer);

    if (!length)
        return;

    buffer_read_advance(&config_if_receive_buffer, length); // Remove this packet from the receive buffer

    // Is this packet larger than we were expecting?
    if (length > sm_context.fw_send_image.length)
    {
        // If it is we should exit out
        message_set_state(SM_MESSAGE_STATE_IDLE);
        fs_close(file_handle);
        fs_delete(file_system, sm_context.fw_send_image.image_type);
        Throw(EXCEPTION_PACKET_WRONG_SIZE);
    }

    sm_context.fw_send_image.crc32_calculated = crc32(sm_context.fw_send_image.crc32_calculated, read_buffer, length);
    uint32_t bytes_written = 0;
    int ret = fs_write(file_handle, read_buffer, length, &bytes_written);
    if (FS_NO_ERROR != ret)
    {
        fs_close(file_handle);
        fs_delete(file_system, sm_context.fw_send_image.image_type);
        message_set_state(SM_MESSAGE_STATE_IDLE);
        Throw(EXCEPTION_FS_ERROR);
    }

    sm_context.fw_send_image.length -= length;

    if (sm_context.fw_send_image.length) // Is there still data to receive?
    {
        config_if_receive_length_priv(sm_context.fw_send_image.length); // Queue a receive
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have received all the data
        fs_close(file_handle); // Close the file

        // Then send a confirmation
        cmd_t * resp;
        if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
            Throw(EXCEPTION_TX_BUFFER_FULL);
        CMD_SET_HDR(resp, CMD_FW_SEND_IMAGE_COMPLETE_CNF);

        // Check the CRC32 is correct
        if (sm_context.fw_send_image.crc32_calculated == sm_context.fw_send_image.crc32_supplied)
        {
            resp->p.cmd_fw_send_image_complete_cnf.error_code = CMD_NO_ERROR;
        }
        else
        {
            resp->p.cmd_fw_send_image_complete_cnf.error_code = CMD_ERROR_IMAGE_CRC_MISMATCH;
            fs_delete(file_system, sm_context.fw_send_image.image_type); // Image is invalid, so delete it
        }

        buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_fw_send_image_complete_cnf_t));
        config_if_send_priv(&config_if_send_buffer); // Send response

        message_set_state(SM_MESSAGE_STATE_IDLE);
    }
}

__RAMFUNC void execute_stm32_firmware_upgrade(void)
{
    uint8_t read_buffer[4];
    uint32_t bytes_actually_read;
    int ret;

    fs_open(file_system, &file_handle, FS_FILE_ID_STM32_IMAGE, FS_MODE_READONLY, NULL);

    syshal_firmware_prepare(); // Erase our FLASH

    do
    {
        ret = fs_read(file_handle, &read_buffer, sizeof(read_buffer), &bytes_actually_read);
        syshal_firmware_write(read_buffer, bytes_actually_read);
        syshal_pmu_kick_watchdog();
    }
    while (FS_ERROR_END_OF_FILE != ret);

    syshal_firmware_flush();

    for (;;)
        syshal_pmu_reset();
}

static void fw_apply_image_req(cmd_t * req, uint32_t size)
{
    fs_stat_t stat;

    // Check request size is correct
    if (CMD_SIZE(cmd_fw_apply_image_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    // Check the image type is correct
    uint8_t image_type = req->p.cmd_fw_apply_image_req.image_type;

    if ( (image_type == FS_FILE_ID_STM32_IMAGE)
         || (image_type == FS_FILE_ID_BLE_IMAGE) )
    {
        // Check image exists
        int ret = fs_open(file_system, &file_handle, image_type, FS_MODE_READONLY, NULL);

        switch (ret)
        {
            case FS_NO_ERROR:

                switch (image_type)
                {
                    case FS_FILE_ID_STM32_IMAGE:
                        DEBUG_PR_TRACE("Apply FS_FILE_ID_STM32_IMAGE");

                        fs_close(file_handle); // Close the file

                        // Respond now as we're about to wipe our FLASH and reset
                        resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
                        buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
                        config_if_send_priv(&config_if_send_buffer);

                        // Make sure our response is sent before we attempt any FLASH operations
#ifndef GTEST // Prevent infinite loop in unit test
                        while (config_if_tx_pending)
#endif
                        {
                            config_if_tick();
                        }

#ifndef GTEST // Prevent infinite loop in unit test
                        execute_stm32_firmware_upgrade();
#endif

                        // Program will never reach this point as the firmware upgrade resets the MCU
                        break;

                    case FS_FILE_ID_BLE_IMAGE:
                        DEBUG_PR_TRACE("Apply FS_FILE_ID_BLE_IMAGE");

                        // Respond now as we're about to wipe our FLASH and reset
                        resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
                        buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
                        config_if_send_priv(&config_if_send_buffer);

                        // Make sure our response is sent before we attempt any FLASH operations
#ifndef GTEST // Prevent infinite loop in unit test
                        while (config_if_tx_pending)
#endif
                        {
                            config_if_tick();
                        }

                        fs_stat(file_system, FS_FILE_ID_BLE_IMAGE, &stat);

                        // TODO: The CRC is passed as zero since there is no CRC check on
                        // the nRF52 side
                        syshal_ble_config_fw_upgrade(stat.size, 0);

                        uint32_t toggleLedTime = syshal_time_get_ticks_ms(), crc = 0;
                        const uint32_t ledTogglePeriodMs = 200;

                        do
                        {
                            // Toggle LEDs to indicate status
                            if (syshal_time_get_ticks_ms() - toggleLedTime > ledTogglePeriodMs)
                            {
                                syshal_gpio_set_output_toggle(GPIO_LED1_GREEN);
                                syshal_gpio_set_output_toggle(GPIO_LED2_RED);
                                toggleLedTime = syshal_time_get_ticks_ms();
                            }

                            uint8_t read_buffer[128];
                            uint32_t bytes_actually_read;
                            ret = fs_read(file_handle, &read_buffer, sizeof(read_buffer), &bytes_actually_read);
                            if (bytes_actually_read)
                            {
                                crc = crc32(crc, read_buffer, bytes_actually_read);
                                syshal_ble_fw_send(read_buffer, bytes_actually_read);
                                // WARNING: this busy wait delay is required to avoid doing
                                // the next SPI write during the nRF52 flash page programming
                                // operation.  Don't remove it without understanding the timing
                                // of the nRF52 page programming first!
                                for (volatile unsigned int j = 0; j < 10000; j++)
                                    ;
                            }

                            KICK_WATCHDOG();
                            syshal_pmu_kick_watchdog();
                        }
                        while (FS_ERROR_END_OF_FILE != ret);

                        if (crc != sm_context.fw_send_image.crc32_supplied)
                        {
                            DEBUG_PR_ERROR("Computed CRC (%x) does not match expected CRC (%x)",
                                (unsigned int)crc,
                                (unsigned int)sm_context.fw_send_image.crc32_supplied);
                        }

                        fs_close(file_handle); // Close the file

                        fs_delete(file_system, FS_FILE_ID_BLE_IMAGE);

                        syshal_gpio_set_output_low(GPIO_LED1_GREEN);
                        syshal_gpio_set_output_low(GPIO_LED2_RED);

                        // We've just flashed and reset our bluetooth device
                        // If we were using it as our configuration interface it is now unused
                        if (CONFIG_IF_BACKEND_BLE == config_if_current())
                        {
                            config_if_term();

                            if (config_if_connected)
                            {
                                // If the BLE device was connected, trigger a disconnect event
                                config_if_event_t disconnectEvent;
                                disconnectEvent.backend = CONFIG_IF_BACKEND_BLE;
                                disconnectEvent.id = CONFIG_IF_EVENT_DISCONNECTED;
                                config_if_callback(&disconnectEvent);
                            }
                        }

                        DEBUG_PR_TRACE("Complete FS_FILE_ID_BLE_IMAGE");

                        return;
                        break;
                }

                resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
                break;

            case FS_ERROR_FILE_NOT_FOUND:
                resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_NOT_FOUND;
                break;

            default:
                Throw(EXCEPTION_FS_ERROR);
                break;
        }

    }
    else
    {
        resp->p.cmd_generic_resp.error_code = CMD_ERROR_INVALID_FW_IMAGE_TYPE;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void reset_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_reset_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    bool STM32_going_to_reset = false;

    switch (req->p.cmd_reset_req.reset_type)
    {
        case RESET_REQ_STM32:
            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
            STM32_going_to_reset = true;
            break;

        case RESET_REQ_FLASH_ERASE_ALL:
            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
            fs_format(file_system);
            log_file_created = false;
            break;

        default:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_INVALID_PARAMETER;
            break;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);

    if (STM32_going_to_reset)
    {
        // Wait for response to have been sent
        // Prevent an infinite loop in unit tests
#ifndef GTEST
        while (config_if_tx_pending)
#endif
        {
            config_if_tick();
        }

        syshal_pmu_reset();
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////// BATTERY_STATUS_REQ //////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void battery_status_req(cmd_t * req, uint32_t size)
{
    UNUSED(req);

    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_BATTERY_STATUS_RESP);

#ifdef DUMMY_BATTERY_MONITOR
    DEBUG_PR_WARN("%s() NOT IMPLEMENTED, responding with spoof data", __FUNCTION__);

    resp->p.cmd_battery_status_resp.error_code = CMD_NO_ERROR;
    resp->p.cmd_battery_status_resp.charging_indicator = 1;
    resp->p.cmd_battery_status_resp.charge_level = 100;
#else
    resp->p.cmd_battery_status_resp.error_code = CMD_NO_ERROR;
    resp->p.cmd_battery_status_resp.charging_indicator = syshal_gpio_get_input(GPIO_VUSB);
    resp->p.cmd_battery_status_resp.charge_level = syshal_batt_level();
#endif

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_battery_status_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// LOG_CREATE_REQ ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void log_create_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_log_create_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate and send response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    // Get request's parameters
    uint8_t mode = req->p.cmd_log_create_req.mode;
    uint8_t sync_enable = req->p.cmd_log_create_req.sync_enable;

    // Attempt to create the log file
    if (CMD_LOG_CREATE_REQ_MODE_FILL == mode || CMD_LOG_CREATE_REQ_MODE_CIRCULAR == mode)
    {
        // Convert from create mode to fs_mode_t
        fs_mode_t fs_mode = FS_MODE_CREATE;
        if (CMD_LOG_CREATE_REQ_MODE_FILL == mode)
            fs_mode = FS_MODE_CREATE;
        else if (CMD_LOG_CREATE_REQ_MODE_CIRCULAR == mode)
            fs_mode = FS_MODE_CREATE_CIRCULAR;

        int ret = fs_open(file_system, &file_handle, FS_FILE_ID_LOG, fs_mode, &sync_enable);

        switch (ret)
        {
            case FS_NO_ERROR:
                log_file_created = true;
                resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
                fs_close(file_handle); // Close the file

                // Set the sys_config log file type
                sys_config_logging_file_type_t log_file_type;
                log_file_type.contents.file_type = mode;
                sys_config_set(SYS_CONFIG_TAG_LOGGING_FILE_TYPE, &log_file_type.contents, SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_file_type_t));

                // Set the sys_config log file size
                sys_config_logging_file_size_t log_file_size;
                log_file_size.contents.file_size = 0; // File is currently of zero size
                sys_config_set(SYS_CONFIG_TAG_LOGGING_FILE_SIZE, &log_file_size.contents, SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_file_size_t));

                buffer_reset(&logging_buffer); // Clear anything we were looking to flush to the log file
                break;

            case FS_ERROR_FILE_ALREADY_EXISTS:
                resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_ALREADY_EXISTS;
                break;

            default:
                Throw(EXCEPTION_FS_ERROR);
                break;
        }
    }
    else
    {
        resp->p.cmd_generic_resp.error_code = CMD_ERROR_INVALID_PARAMETER;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// LOG_ERASE_REQ /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void log_erase_req(cmd_t * req, uint32_t size)
{
    UNUSED(req);

    // Check request size is correct
    if (size != CMD_SIZE_HDR)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_GENERIC_RESP);

    int ret = fs_delete(file_system, FS_FILE_ID_LOG);

    switch (ret)
    {
        case FS_NO_ERROR:
            resp->p.cmd_generic_resp.error_code = CMD_NO_ERROR;
            log_file_created = false;
            break;

        case FS_ERROR_FILE_NOT_FOUND:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_FILE_NOT_FOUND;
            break;

        case FS_ERROR_FILE_PROTECTED:
            resp->p.cmd_generic_resp.error_code = CMD_ERROR_CONFIG_PROTECTED;
            break;

        default:
            Throw(EXCEPTION_FS_ERROR);
            break;
    }

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_generic_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////////// LOG_READ_REQ /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void log_read_req(cmd_t * req, uint32_t size)
{
    // Check request size is correct
    if (CMD_SIZE(cmd_log_read_req_t) != size)
        Throw(EXCEPTION_REQ_WRONG_SIZE);

    // Generate response
    cmd_t * resp;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&resp))
        Throw(EXCEPTION_TX_BUFFER_FULL);
    CMD_SET_HDR(resp, CMD_LOG_READ_RESP);

    sm_context.log_read.length = 0;

    fs_stat_t stat;
    int ret = fs_stat(file_system, FS_FILE_ID_LOG, &stat);

    switch (ret)
    {
        case FS_NO_ERROR:
            sm_context.log_read.length = req->p.cmd_log_read_req.length;
            sm_context.log_read.start_offset = req->p.cmd_log_read_req.start_offset;

            // Check if both parameters are zero. If they are the client is requesting a full log file
            if ((0 == sm_context.log_read.length) && (0 == sm_context.log_read.start_offset))
                sm_context.log_read.length = stat.size;

            if (sm_context.log_read.start_offset > stat.size) // Is the offset beyond the end of the file?
            {
                resp->p.cmd_log_read_resp.error_code = CMD_ERROR_INVALID_PARAMETER;
            }
            else
            {
                // Do we have this amount of data ready to read?
                if ((sm_context.log_read.length + sm_context.log_read.start_offset) > stat.size)
                {
                    // If the length requested is greater than what we have then reduce the length
                    sm_context.log_read.length = stat.size - sm_context.log_read.start_offset;
                }

                // Open the file
                ret = fs_open(file_system, &file_handle, FS_FILE_ID_LOG, FS_MODE_READONLY, NULL);

                if (FS_NO_ERROR == ret)
                {
                    resp->p.cmd_log_read_resp.error_code = CMD_NO_ERROR;
                    if (sm_context.log_read.length)
                    {
                        message_set_state(SM_MESSAGE_STATE_LOG_READ_NEXT);

                        // Move to the offset position
                        while (sm_context.log_read.start_offset)
                        {
                            uint8_t dummyData;
                            uint32_t bytes_actually_read;
                            // If so move to this location in packet sized chunks
                            ret = fs_read(file_handle, &dummyData, sizeof(dummyData), &bytes_actually_read);
                            if (FS_NO_ERROR != ret)
                            {
                                Throw(EXCEPTION_FS_ERROR);
                            }

                            sm_context.log_read.start_offset -= bytes_actually_read;
                        }
                    }
                    else
                        fs_close(file_handle);
                }
                else
                {
                    Throw(EXCEPTION_FS_ERROR);
                }
            }
            break;

        case FS_ERROR_FILE_NOT_FOUND:
            resp->p.cmd_log_read_resp.error_code = CMD_ERROR_FILE_NOT_FOUND;
            break;

        default:
            Throw(EXCEPTION_FS_ERROR);
            break;
    }

    resp->p.cmd_log_read_resp.length = sm_context.log_read.length;

    buffer_write_advance(&config_if_send_buffer, CMD_SIZE(cmd_log_read_resp_t));
    config_if_send_priv(&config_if_send_buffer);
}

static void log_read_next_state()
{
    uint32_t bytes_to_read;
    uint32_t bytes_actually_read;
    int ret;

    //DEBUG_PR_TRACE("Bytes left to write: %lu", sm_context.log_read.length);

    // Get write buffer
    uint8_t * read_buffer;
    if (!buffer_write(&config_if_send_buffer, (uintptr_t *)&read_buffer))
        Throw(EXCEPTION_TX_BUFFER_FULL);

    // Read data out
    bytes_to_read = MIN(sm_context.log_read.length, (uint32_t) SYSHAL_USB_PACKET_SIZE);
    ret = fs_read(file_handle, read_buffer, bytes_to_read, &bytes_actually_read);
    if (FS_NO_ERROR != ret)
    {
        Throw(EXCEPTION_FS_ERROR);
    }

    sm_context.log_read.length -= bytes_actually_read;

    buffer_write_advance(&config_if_send_buffer, bytes_actually_read);
    config_if_send_priv(&config_if_send_buffer);

    if (sm_context.log_read.length) // Is there still data to send?
    {
        config_if_timeout_reset(); // Reset the message timeout counter
    }
    else
    {
        // We have sent all the data
        fs_close(file_handle); // Close the file
        message_set_state(SM_MESSAGE_STATE_IDLE);
    }

}

////////////////////////////////////////////////////////////////////////////////
////////////////////////// MESSAGE STATE EXECUTION CODE ////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void config_if_session_cleanup(void)
{
    buffer_reset(&config_if_send_buffer);
    buffer_reset(&config_if_receive_buffer);
    config_if_tx_pending = false;
    config_if_rx_queued = false; // Setting this to false does not mean a receive is still not queued!

    // Close any open files
    if (file_handle)
        fs_close(file_handle);
    file_handle = NULL;
}

int config_if_callback(config_if_event_t * event)
{
    // This is called from an interrupt so we'll keep it short
    switch (event->id)
    {

        case CONFIG_IF_EVENT_SEND_COMPLETE:
            buffer_read_advance(&config_if_send_buffer, event->send.size); // Remove it from the buffer
            config_if_tx_pending = false;

            // Should we be resetting a BLE inactivity timer?
            if (syshal_timer_running(timer_ble_timeout))
                syshal_timer_reset(timer_ble_timeout);
            break;

        case CONFIG_IF_EVENT_RECEIVE_COMPLETE:
            buffer_write_advance(&config_if_receive_buffer, event->receive.size); // Store it in the buffer
            config_if_rx_queued = false;

            // Should we be resetting a BLE inactivity timer?
            if (syshal_timer_running(timer_ble_timeout))
                syshal_timer_reset(timer_ble_timeout);
            break;

        case CONFIG_IF_EVENT_CONNECTED:
            DEBUG_PR_TRACE("CONFIG_IF_EVENT_CONNECTED");

            // Was this a bluetooth connection?
            if (CONFIG_IF_BACKEND_BLE == event->backend)
            {
                // Should we log this event
                if (sys_config.sys_config_tag_bluetooth_log_enable.hdr.set &&
                    sys_config.sys_config_tag_bluetooth_log_enable.contents.enable)
                {
                    logging_ble_connected_t ble_connected;
                    LOGGING_SET_HDR(&ble_connected, LOGGING_BLE_CONNECTED);
                    logging_add_to_buffer((uint8_t *) &ble_connected, sizeof(ble_connected));
                }

                // Should we be starting a BLE inactivity timer?
                if (sys_config.sys_config_tag_bluetooth_connection_inactivity_timeout.hdr.set &&
                    sys_config.sys_config_tag_bluetooth_connection_inactivity_timeout.contents.seconds)
                {
                    syshal_timer_set(timer_ble_timeout, one_shot, sys_config.sys_config_tag_bluetooth_connection_inactivity_timeout.contents.seconds);
                }
            }

            config_if_session_cleanup(); // Clean up any previous session
            config_if_timeout_reset(); // Reset our timeout counter
            config_if_connected = true;
            break;

        case CONFIG_IF_EVENT_DISCONNECTED:
            DEBUG_PR_TRACE("CONFIG_IF_EVENT_DISCONNECTED");

            // Should we log this event
            if (CONFIG_IF_BACKEND_BLE == event->backend)
                if (sys_config.sys_config_tag_bluetooth_log_enable.hdr.set &&
                    sys_config.sys_config_tag_bluetooth_log_enable.contents.enable)
                {
                    logging_ble_disconnected_t ble_disconnected;
                    LOGGING_SET_HDR(&ble_disconnected, LOGGING_BLE_DISCONNECTED);
                    logging_add_to_buffer((uint8_t *) &ble_disconnected, sizeof(ble_disconnected));
                }

            syshal_timer_cancel(timer_ble_timeout);
            // Clear all pending transmissions/receptions
            config_if_session_cleanup();
            config_if_connected = false;
            syshal_gps_bridging = false;
            break;

    }

    return CONFIG_IF_NO_ERROR;
}

static void message_idle_state(void)
{
    cmd_t * req;
    uint32_t length = buffer_read(&config_if_receive_buffer, (uintptr_t *)&req);
    if (length) // Is a message waiting to be processed
    {

        // Then process any pending event here, outside of an interrupt

        // Mark this message as received. This maybe a bit preemptive but it
        // is assumed the request handlers will handle the message appropriately
        buffer_read_advance(&config_if_receive_buffer, length);

        switch (req->h.cmd)
        {
            case CMD_CFG_READ_REQ:
                DEBUG_PR_INFO("CFG_READ_REQ");
                cfg_read_req(req, length);
                break;

            case CMD_CFG_WRITE_REQ:
                DEBUG_PR_INFO("CFG_WRITE_REQ");
                cfg_write_req(req, length);
                break;

            case CMD_CFG_SAVE_REQ:
                DEBUG_PR_INFO("CFG_SAVE_REQ");
                cfg_save_req(req, length);
                break;

            case CMD_CFG_RESTORE_REQ:
                DEBUG_PR_INFO("CFG_RESTORE_REQ");
                cfg_restore_req(req, length);
                break;

            case CMD_CFG_ERASE_REQ:
                DEBUG_PR_INFO("CFG_ERASE_REQ");
                cfg_erase_req(req, length);
                break;

            case CMD_CFG_PROTECT_REQ:
                DEBUG_PR_INFO("CFG_PROTECT_REQ");
                cfg_protect_req(req, length);
                break;

            case CMD_CFG_UNPROTECT_REQ:
                DEBUG_PR_INFO("CFG_UNPROTECT_REQ");
                cfg_unprotect_req(req, length);
                break;

            case CMD_GPS_WRITE_REQ:
                DEBUG_PR_INFO("GPS_WRITE_REQ");
                gps_write_req(req, length);
                break;

            case CMD_GPS_READ_REQ:
                DEBUG_PR_INFO("GPS_READ_REQ");
                gps_read_req(req, length);
                break;

            case CMD_GPS_CONFIG_REQ:
                DEBUG_PR_INFO("GPS_CONFIG_REQ");
                gps_config_req(req, length);
                break;

            case CMD_BLE_CONFIG_REQ:
                DEBUG_PR_INFO("BLE_CONFIG_REQ");
                ble_config_req(req, length);
                break;

            case CMD_BLE_WRITE_REQ:
                DEBUG_PR_INFO("BLE_WRITE_REQ");
                ble_write_req(req, length);
                break;

            case CMD_BLE_READ_REQ:
                DEBUG_PR_INFO("BLE_READ_REQ");
                ble_read_req(req, length);
                break;

            case CMD_STATUS_REQ:
                DEBUG_PR_INFO("STATUS_REQ");
                status_req(req, length);
                break;

            case CMD_FW_SEND_IMAGE_REQ:
                DEBUG_PR_INFO("FW_SEND_IMAGE_REQ");
                fw_send_image_req(req, length);
                break;

            case CMD_FW_APPLY_IMAGE_REQ:
                DEBUG_PR_INFO("FW_APPLY_IMAGE_REQ");
                fw_apply_image_req(req, length);
                break;

            case CMD_RESET_REQ:
                DEBUG_PR_INFO("RESET_REQ");
                reset_req(req, length);
                break;

            case CMD_BATTERY_STATUS_REQ:
                DEBUG_PR_INFO("BATTERY_STATUS_REQ");
                battery_status_req(req, length);
                break;

            case CMD_LOG_CREATE_REQ:
                DEBUG_PR_INFO("LOG_CREATE_REQ");
                log_create_req(req, length);
                break;

            case CMD_LOG_ERASE_REQ:
                DEBUG_PR_INFO("LOG_ERASE_REQ");
                log_erase_req(req, length);
                break;

            case CMD_LOG_READ_REQ:
                DEBUG_PR_INFO("LOG_READ_REQ");
                log_read_req(req, length);
                break;

            default:
                DEBUG_PR_WARN("Unhandled command: id %d", req->h.cmd);
                // Don't return an error. Fail silent
                break;
        }

    }
    else
    {
        config_if_receive_priv();
    }
}

void state_message_exception_handler(CEXCEPTION_T e)
{
    switch (e)
    {
        case EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION:
            DEBUG_PR_ERROR("EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION");
            break;

        case EXCEPTION_REQ_WRONG_SIZE:
            DEBUG_PR_ERROR("EXCEPTION_REQ_WRONG_SIZE");
            break;

        case EXCEPTION_TX_BUFFER_FULL:
            DEBUG_PR_ERROR("EXCEPTION_TX_BUFFER_FULL");
            break;

        case EXCEPTION_TX_BUSY:
            DEBUG_PR_ERROR("EXCEPTION_TX_BUSY");
            break;

        case EXCEPTION_RX_BUFFER_EMPTY:
            DEBUG_PR_ERROR("EXCEPTION_RX_BUFFER_EMPTY");
            break;

        case EXCEPTION_RX_BUFFER_FULL:
            DEBUG_PR_ERROR("EXCEPTION_RX_BUFFER_FULL");
            break;

        case EXCEPTION_PACKET_WRONG_SIZE:
            DEBUG_PR_ERROR("EXCEPTION_PACKET_WRONG_SIZE");
            break;

        case EXCEPTION_GPS_SEND_ERROR:
            DEBUG_PR_ERROR("EXCEPTION_GPS_SEND_ERROR");
            break;

        case EXCEPTION_FS_ERROR:
            DEBUG_PR_ERROR("EXCEPTION_FS_ERROR");
            break;

        default:
            DEBUG_PR_ERROR("Unknown message exception");
            break;
    }
}

static inline void config_if_timeout_reset(void)
{
    config_if_message_timeout = syshal_time_get_ticks_ms();
}

static void message_set_state(sm_message_state_t s)
{
    config_if_timeout_reset();
    message_state = s;
}

static void handle_config_if_messages(void)
{
    CEXCEPTION_T e = CEXCEPTION_NONE;

    // Has a message timeout occured?
    if ((syshal_time_get_ticks_ms() - config_if_message_timeout) > SM_MAIN_INACTIVITY_TIMEOUT_MS)
    {
        if (SM_MESSAGE_STATE_IDLE != message_state) // Our idle state can't timeout
        {
            DEBUG_PR_WARN("State: %d, MESSAGE TIMEOUT", message_state);
            message_set_state(SM_MESSAGE_STATE_IDLE); // Return to the idle state
            config_if_session_cleanup(); // Clear any pending messages
        }
    }

    // Don't allow the processing of anymore messages until we have a free transmit buffer
    if (config_if_tx_pending)
        return;

    Try
    {

        switch (message_state)
        {
            case SM_MESSAGE_STATE_IDLE: // No message is currently being handled
                message_idle_state();
                config_if_timeout_reset(); // Reset the timeout counter
                break;

            case SM_MESSAGE_STATE_CFG_READ_NEXT:
                cfg_read_next_state();
                break;

            case SM_MESSAGE_STATE_CFG_WRITE_NEXT:
                cfg_write_next_state();
                break;

            case SM_MESSAGE_STATE_CFG_WRITE_ERROR:
                cfg_write_error_state();
                break;

            case SM_MESSAGE_STATE_GPS_WRITE_NEXT:
                gps_write_next_state();
                break;

            case SM_MESSAGE_STATE_GPS_READ_NEXT:
                gps_read_next_state();
                break;

            case SM_MESSAGE_STATE_BLE_READ_NEXT:
                ble_read_next_state();
                break;

            case SM_MESSAGE_STATE_BLE_WRITE_NEXT:
                ble_write_next_state();
                break;

            case SM_MESSAGE_STATE_LOG_READ_NEXT:
                log_read_next_state();
                break;

            case SM_MESSAGE_STATE_FW_SEND_IMAGE_NEXT:
                fw_send_image_next_state();
                break;

            default:
                // TODO: add an illegal error state here for catching
                // invalid state changes
                break;
        }

    } Catch (e)
    {
        state_message_exception_handler(e);
    }
}

static void log_system_startup_event(void)
{
    while (system_startup_log_required)
    {
        system_startup_log_required = false;

        // Try to log startup event
        int ret = fs_open(file_system, &file_handle, FS_FILE_ID_LOG, FS_MODE_WRITEONLY, NULL); // Open the log file
        if (FS_NO_ERROR != ret)
        {
            // Unable to log this entry, so skip
            file_handle = NULL;
            break;
        }

        logging_startup_t log_start;
        uint32_t bytes_written;

        if (sys_config.sys_config_logging_date_time_stamp_enable.contents.enable)
        {
            syshal_rtc_data_and_time_t current_time;
            logging_date_time_t log_date;

            syshal_rtc_get_date_and_time(&current_time);

            log_date.h.id = LOGGING_DATE_TIME;
            log_date.day = current_time.day;
            log_date.month = current_time.month;
            log_date.year = current_time.year;
            log_date.hours = current_time.hours;
            log_date.minutes = current_time.minutes;
            log_date.seconds = current_time.seconds;
            (void)fs_write(file_handle, &log_date, sizeof(log_date), &bytes_written);
        }

        log_start.h.id = LOGGING_STARTUP;
        log_start.cause = syshal_pmu_get_startup_status();
        (void)fs_write(file_handle, &log_start, sizeof(log_start), &bytes_written);

        // Always flush this log entry so we can always see when a resets occurs
        fs_close(file_handle);
        file_handle = NULL;
    }
}

////////////////////////////////////////////////////////////////////////////////
///////////////////////////// STATE EXECUTION CODE /////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static void sm_main_boot(sm_handle_t * state_handle)
{
    set_default_global_values(); // Set all our global static variables to their default values
    // This is done to ensure individual unit tests all start in the same state

    syshal_time_init();

    setup_buffers();

    // Initialize all configured peripherals
    syshal_rtc_init();

    syshal_gpio_init(GPIO_LED1_GREEN);
    syshal_gpio_init(GPIO_LED2_RED);
    syshal_gpio_init(GPIO_VUSB);
    syshal_gpio_init(GPIO_SPI1_CS_BT);
    syshal_gpio_set_output_high(GPIO_SPI1_CS_BT);

    syshal_gpio_init(GPIO_REED_SW);
    syshal_gpio_enable_interrupt(GPIO_REED_SW, gpio_reed_sw_callback);

    syshal_uart_init(UART_1);
    syshal_uart_init(UART_2);
//    syshal_uart_init(UART_3);

    // Init timers
    syshal_timer_init(&timer_gps_interval, timer_gps_interval_callback);
    syshal_timer_init(&timer_gps_no_fix, timer_gps_no_fix_callback);
    syshal_timer_init(&timer_gps_maximum_acquisition, timer_gps_maximum_acquisition_callback);
    syshal_timer_init(&timer_gps_very_first_fix_hold_time, timer_gps_very_first_fix_hold_time_callback);
    syshal_timer_init(&timer_gps_watchdog, timer_gps_watchdog_callback);
    syshal_timer_init(&timer_log_flush, timer_log_flush_callback);
    syshal_timer_init(&timer_saltwater_switch_hysteresis, timer_saltwater_switch_hysteresis_callback);
    syshal_timer_init(&timer_reed_switch_hysteresis, timer_reed_switch_hysteresis_callback);
    syshal_timer_init(&timer_pressure_interval, timer_pressure_interval_callback);
    syshal_timer_init(&timer_pressure_maximum_acquisition, timer_pressure_maximum_acquisition_callback);
    syshal_timer_init(&timer_axl_interval, timer_axl_interval_callback);
    syshal_timer_init(&timer_axl_maximum_acquisition, timer_axl_maximum_acquisition_callback);
    syshal_timer_init(&timer_ble_interval, timer_ble_interval_callback);
    syshal_timer_init(&timer_ble_duration, timer_ble_duration_callback);
    syshal_timer_init(&timer_ble_timeout, timer_ble_timeout_callback);

    syshal_spi_init(SPI_1);
    syshal_spi_init(SPI_2);

    syshal_i2c_init(I2C_1);
    syshal_i2c_init(I2C_2);

    syshal_flash_init(0, SPI_FLASH);

    syshal_batt_init();

    // Re/Set global vars
    syshal_gps_bridging = false;
    syshal_ble_bridging = false;

    // Print General System Info
    DEBUG_PR_SYS("Arribada Tracker Device");
    DEBUG_PR_SYS("Version:  %s", GIT_VERSION);
    DEBUG_PR_SYS("Compiled: %s %s With %s", COMPILE_DATE, COMPILE_TIME, COMPILER_NAME);

    // Start the soft watchdog timer
    syshal_rtc_soft_watchdog_enable(SOFT_WATCHDOG_TIMEOUT_S, soft_watchdog_callback);

    // Load the file system
    fs_init(FS_DEVICE);
    fs_mount(FS_DEVICE, &file_system);

    // Determine if a log file exists or not
    int ret = fs_open(file_system, &file_handle, FS_FILE_ID_LOG, FS_MODE_READONLY, NULL);

    if (FS_NO_ERROR == ret)
    {
        log_file_created = true;
        fs_close(file_handle);
    }
    else
    {
        log_file_created = false;
    }

    ret = fs_get_configuration_data();

    if (!(FS_NO_ERROR == ret || FS_ERROR_FILE_NOT_FOUND == ret || FS_ERROR_FILE_VERSION_MISMATCH == ret))
        Throw(EXCEPTION_FS_ERROR);

    // Attempt to log system startup event into the log file
    log_system_startup_event();

    // Delete any firmware images we may have
    fs_delete(file_system, FS_FILE_ID_STM32_IMAGE);
    fs_delete(file_system, FS_FILE_ID_BLE_IMAGE);

    // Init the peripheral devices after configuration data has been collected
    syshal_gps_init();
    sm_gps_state = SM_GPS_STATE_ASLEEP; // GPS starts off
    sys_config.sys_config_gps_last_known_position.hdr.set = false; // Invalidate any prior last location

    syshal_switch_init();
    tracker_above_water = !syshal_switch_get();

    if (syshal_gpio_get_input(GPIO_VUSB))
    {
        // Branch to Battery Charging state if VUSB is present
        sm_set_next_state(state_handle, SM_MAIN_BATTERY_CHARGING);
    }
    else if (check_configuration_tags_set() && log_file_created)
    {
        // Branch to Operational state if log file exists and configuration tags are set
        sm_set_next_state(state_handle, SM_MAIN_OPERATIONAL);
    }
    else
    {
        // Branch to Provisioning Needed state
        sm_set_next_state(state_handle, SM_MAIN_PROVISIONING_NEEDED);
    }
}

static void sm_main_operational(sm_handle_t * state_handle)
{
    static uint32_t led_flashing_start_time;
    static uint32_t led_last_flash_time;

    KICK_WATCHDOG();

    if (sm_is_first_entry(state_handle))
    {
        DEBUG_PR_INFO("Entered state %s from %s",
                      sm_main_state_str[sm_get_current_state(state_handle)],
                      sm_main_state_str[sm_get_last_state(state_handle)]);

        GPS_off_no_log();

        int ret = fs_open(file_system, &file_handle, FS_FILE_ID_LOG, FS_MODE_WRITEONLY, NULL); // Open the log file

        if (FS_NO_ERROR != ret)
        {
            // Fatal error
            file_handle = NULL;
            Throw(EXCEPTION_FS_ERROR);
        }

        if (system_startup_log_required)
        {
            logging_startup_t log_start;
            uint32_t bytes_written;

            system_startup_log_required = false;

            if (sys_config.sys_config_logging_date_time_stamp_enable.contents.enable)
            {
                syshal_rtc_data_and_time_t current_time;
                logging_date_time_t log_date;

                syshal_rtc_get_date_and_time(&current_time);

                log_date.h.id = LOGGING_DATE_TIME;
                log_date.day = current_time.day;
                log_date.month = current_time.month;
                log_date.year = current_time.year;
                log_date.hours = current_time.hours;
                log_date.minutes = current_time.minutes;
                log_date.seconds = current_time.seconds;
                (void)fs_write(file_handle, &log_date, sizeof(log_date), &bytes_written);
            }

            log_start.h.id = LOGGING_STARTUP;
            log_start.cause = syshal_pmu_get_startup_status();
            (void)fs_write(file_handle, &log_start, sizeof(log_start), &bytes_written);

            // Always flush this log entry so we can always see when a resets occurs
            fs_flush(file_handle);
        }

        green_led_flashing = true;
        led_last_flash_time = syshal_time_get_ticks_ms();
        led_flashing_start_time = 0;

        // Start the log file flushing timer
        syshal_timer_set(timer_log_flush, periodic, LOG_FILE_FLUSH_PERIOD_SECONDS);

        gps_ttff_reading_logged = false; // Make sure we read/log the first TTFF reading
        last_battery_reading = 0xFF; // Ensure the first battery reading is logged

        if (sys_config.sys_config_logging_enable.contents.enable)
            sensor_logging_enabled = true;

        if (sys_config.sys_config_gps_log_position_enable.contents.enable ||
            sys_config.sys_config_gps_log_ttff_enable.contents.enable)
        {
            // Clear the GPS buffer
            uint8_t flush;
            while (syshal_gps_receive_raw(&flush, 1))
            {}

            // AG-176: We must ensure that we don't re-enter very first fix state e.g.,
            // if a IWDG or soft reset occurs.  This state is reserved only for the
            // situation when we have just finished provisioning.
            if (sys_config.sys_config_gps_very_first_fix_hold_time.hdr.set &&
                sys_config.sys_config_gps_very_first_fix_hold_time.contents.seconds &&
                (sm_get_last_state(state_handle) == SM_MAIN_PROVISIONING ||
                    sm_get_last_state(state_handle) == SM_MAIN_BATTERY_CHARGING))
            {
                // A very first fix timeout is present so we need to attempt to achieve a fix now
                gps_waiting_for_first_fix = true;

                // Wake the GPS if it is asleep
                GPS_on();
            }
            else
            {
                gps_waiting_for_first_fix = false;
                setup_GPS_based_on_configuration(); // Set the GPS to standard operation
            }
        }
        else
        {
            GPS_off();
        }

        // Should we be logging pressure data?
        if (sys_config.sys_config_pressure_sensor_log_enable.contents.enable)
        {
            syshal_pressure_init();
            if (SYS_CONFIG_PRESSURE_MODE_PERIODIC == sys_config.sys_config_pressure_mode.contents.mode)
            {
                // If SYS_CONFIG_TAG_PRESSURE_SCHEDULED_ACQUISITION_INTERVAL = 0 then this is a special case meaning to always run the pressure sensor
                if (sys_config.sys_config_pressure_scheduled_acquisition_interval.contents.seconds)
                    syshal_timer_set(timer_pressure_interval, periodic, sys_config.sys_config_pressure_scheduled_acquisition_interval.contents.seconds);
                else
                    syshal_pressure_wake();
            }
        }

        // Should we be logging axl data?
        if (sys_config.sys_config_axl_log_enable.contents.enable)
        {
            syshal_axl_init();
            if (SYS_CONFIG_AXL_MODE_PERIODIC == sys_config.sys_config_axl_mode.contents.mode)
            {
                // If SYS_CONFIG_TAG_AXL_SCHEDULED_ACQUISITION_INTERVAL = 0 then this is a special case meaning to always run the AXL sensor
                if (sys_config.sys_config_axl_scheduled_acquisition_interval.contents.seconds)
                    syshal_timer_set(timer_axl_interval, periodic, sys_config.sys_config_axl_scheduled_acquisition_interval.contents.seconds);
                else
                    syshal_axl_wake();
            }
        }
    }

    if (green_led_flashing)
    {
        if (syshal_time_get_ticks_ms() > led_last_flash_time + 200)
        {
            syshal_gpio_set_output_toggle(GPIO_LED1_GREEN);
            led_last_flash_time += 200;
        }

        if (!gps_waiting_for_first_fix)
        {
            if (!led_flashing_start_time)
                led_flashing_start_time = syshal_time_get_ticks_ms();

            if (syshal_time_get_ticks_ms() > led_flashing_start_time + 5000)
            {
                green_led_flashing = false;
                syshal_gpio_set_output_low(GPIO_LED1_GREEN);
            }
        }
    }

    // If GPS bridging is disabled and GPS logging enabled
    if ( (!syshal_gps_bridging) && // NOTE: Is this GPS bridging check needed in the operational state?
         (sys_config.sys_config_gps_log_position_enable.contents.enable ||
          sys_config.sys_config_gps_log_ttff_enable.contents.enable) )
    {
        syshal_gps_tick(); // Process GPS messages

        if (gps_waiting_for_first_fix)
        {
            if (SM_GPS_STATE_FIXED == sm_gps_state)
            {
                if (!syshal_timer_running(timer_gps_very_first_fix_hold_time)) // The GPS is locked. So start the first fix hold timer
                {
                    syshal_timer_set(timer_gps_very_first_fix_hold_time, one_shot, sys_config.sys_config_gps_very_first_fix_hold_time.contents.seconds);
                }
                syshal_gpio_set_output_high(GPIO_LED1_GREEN); // Have the LED be solid green to show a GPS fix has been achieved
            }
            else
            {
                syshal_timer_cancel(timer_gps_very_first_fix_hold_time); // We're not locked so don't start the first fix hold timer
            }
        }
    }

    if (sys_config.sys_config_pressure_sensor_log_enable.contents.enable)
        syshal_pressure_tick();

    if (sys_config.sys_config_axl_log_enable.contents.enable)
        syshal_axl_tick();

    // Determine how deep a sleep we should take
    if (!syshal_pressure_awake() && !syshal_axl_awake() && !green_led_flashing)
    {
        if (SM_GPS_STATE_ASLEEP == sm_gps_state)
            syshal_pmu_set_level(POWER_STOP);
        else
            syshal_pmu_set_level(POWER_SLEEP);
    }

    // Get the battery level state
    int level = syshal_batt_level();
    if (level >= 0) // If we've read the battery level successfully
    {
        // Has our battery level decreased
        if (last_battery_reading > level)
        {
            // Should we log this?
            if (sys_config.sys_config_battery_log_enable.hdr.set &&
                sys_config.sys_config_battery_log_enable.contents.enable)
            {
                // Log the battery level
                logging_battery_t battery_log;

                LOGGING_SET_HDR(&battery_log, LOGGING_BATTERY);
                battery_log.charge = (uint8_t) level;
                logging_add_to_buffer((uint8_t *) &battery_log, sizeof(battery_log));
            }

            // Should we check to see if we should enter a low power state?
            if (sys_config.sys_config_battery_low_threshold.hdr.set &&
                level <= sys_config.sys_config_battery_low_threshold.contents.threshold)
                sm_set_next_state(state_handle, SM_MAIN_BATTERY_LEVEL_LOW);

            last_battery_reading = (uint8_t) level;
        }
    }

    // Is global logging enabled?
    if (sys_config.sys_config_logging_enable.contents.enable)
    {
        // Is there any data waiting to be written to the log file?
        uint8_t * read_buffer;
        uint32_t length = buffer_read(&logging_buffer, (uintptr_t *)&read_buffer);

        while (length) // Then write all of it
        {
            uint32_t bytes_written;
            int ret = fs_write(file_handle, read_buffer, length, &bytes_written);

#ifndef DEBUG_DISABLED
            DEBUG_PR_TRACE("Writing to Log File");
            printf("Contents: ");
            for (uint32_t i = 0; i < length; ++i)
                printf("%02X ", read_buffer[i]);
            printf("\r\n");
#endif

            if (FS_NO_ERROR == ret)
            {
                buffer_read_advance(&logging_buffer, length);
            }
            else if (FS_ERROR_FILESYSTEM_FULL == ret)
            {
                // Branch to Log File Full state if log file is full
                sm_set_next_state(state_handle, SM_MAIN_LOG_FILE_FULL);
                break;
            }
            else
            {
                Throw(EXCEPTION_FS_ERROR);
            }

            length = buffer_read(&logging_buffer, (uintptr_t *)&read_buffer);
        }
    }

    syshal_timer_tick();

    // Branch to Battery Charging if VUSB is present
    if (syshal_gpio_get_input(GPIO_VUSB))
        sm_set_next_state(state_handle, SM_MAIN_BATTERY_CHARGING);

    manage_ble();

    config_if_tick();

    // Branch to Provisioning state if config_if has connected
    if (config_if_connected)
        sm_set_next_state(state_handle, SM_MAIN_PROVISIONING);

    // Are we about to leave this state?
    if (sm_is_last_entry(state_handle))
    {
        // Close any open files
        if (file_handle)
        {
            fs_close(file_handle);
            file_handle = NULL;
        }

        syshal_axl_term();
        syshal_pressure_term();

        // Sleep the GPS to save power
        GPS_off();

        // Turn off the green LED incase we've left it on
        syshal_gpio_set_output_low(GPIO_LED1_GREEN);

        // Stop any unnecessary timers
        syshal_timer_cancel(timer_gps_interval);
        syshal_timer_cancel(timer_gps_no_fix);
        syshal_timer_cancel(timer_gps_maximum_acquisition);
        syshal_timer_cancel(timer_gps_very_first_fix_hold_time);
        syshal_timer_cancel(timer_gps_watchdog);
        syshal_timer_cancel(timer_log_flush);
        syshal_timer_cancel(timer_pressure_interval);
        syshal_timer_cancel(timer_pressure_maximum_acquisition);
        syshal_timer_cancel(timer_axl_interval);
        syshal_timer_cancel(timer_axl_maximum_acquisition);

        sensor_logging_enabled = false; // Prevent any sensors from logging but still other logs eg. BLE connection events
    }
}

static void sm_main_log_file_full(sm_handle_t * state_handle)
{
    if (sm_is_first_entry(state_handle))
    {
        DEBUG_PR_INFO("Entered state %s from %s",
                      sm_main_state_str[sm_get_current_state(state_handle)],
                      sm_main_state_str[sm_get_last_state(state_handle)]);
    }

    KICK_WATCHDOG();

    syshal_timer_tick();

    manage_ble();

    config_if_tick();

    syshal_pmu_set_level(POWER_STOP);

    // Branch to Provisioning state if config_if has connected
    if (config_if_connected)
        sm_set_next_state(state_handle, SM_MAIN_PROVISIONING);

    // Branch to Battery Charging if VUSB is present
    if (syshal_gpio_get_input(GPIO_VUSB))
        sm_set_next_state(state_handle, SM_MAIN_BATTERY_CHARGING);

    // Branch to Battery Low state if battery is beneath threshold
    int level = syshal_batt_level();
    if (level >= 0)
        if (sys_config.sys_config_battery_low_threshold.hdr.set &&
            level <= sys_config.sys_config_battery_low_threshold.contents.threshold)
            sm_set_next_state(state_handle, SM_MAIN_BATTERY_LEVEL_LOW);
}

static void sm_main_battery_charging(sm_handle_t * state_handle)
{
    static uint32_t usb_enumeration_timeout;

    KICK_WATCHDOG();

    if (sm_is_first_entry(state_handle))
    {
        DEBUG_PR_INFO("Entered state %s from %s",
                      sm_main_state_str[sm_get_current_state(state_handle)],
                      sm_main_state_str[sm_get_last_state(state_handle)]);

        // If we've just entered the charging state try to enumerate for USB_ENUMERATION_TIMEOUT seconds
        if (CONFIG_IF_BACKEND_USB != config_if_current())
        {
            config_if_term();
            config_if_init(CONFIG_IF_BACKEND_USB);
            usb_enumeration_timeout = syshal_time_get_ticks_ms();
        }
    }

    manage_ble();

    config_if_tick();

    syshal_timer_tick();

    // Branch to Provisioning state if config_if has connected
    if (config_if_connected)
        sm_set_next_state(state_handle, SM_MAIN_PROVISIONING);

    // Has our USB enumeration attempt timed out?
    if (syshal_time_get_ticks_ms() - usb_enumeration_timeout >= USB_ENUMERATION_TIMEOUT_MS)
        if (CONFIG_IF_BACKEND_USB == config_if_current())
            config_if_term();

    if (!syshal_gpio_get_input(GPIO_VUSB))
    {
        // Our charging voltage has been removed

        // Branch to Operational state if log file exists and configuration tags are set
        if (check_configuration_tags_set() && log_file_created)
            sm_set_next_state(state_handle, SM_MAIN_OPERATIONAL);
        else
            // Branch to Provisioning Needed state if configuration tags are not set OR log file doesn't exist
            sm_set_next_state(state_handle, SM_MAIN_PROVISIONING_NEEDED);

        // Branch to Battery Low state if battery is beneath threshold
        int level = syshal_batt_level();
        if (level >= 0)
            // If we've read the battery level successfully
            if (sys_config.sys_config_battery_low_threshold.hdr.set &&
                level <= sys_config.sys_config_battery_low_threshold.contents.threshold)
                sm_set_next_state(state_handle, SM_MAIN_BATTERY_LEVEL_LOW);

        // Terminate any attempt to connect over VUSB
        if (CONFIG_IF_BACKEND_USB == config_if_current())
            config_if_term();
    }
}

static void sm_main_battery_level_low(sm_handle_t * state_handle)
{
    KICK_WATCHDOG();

    if (sm_is_first_entry(state_handle))
    {
        DEBUG_PR_INFO("Entered state %s from %s",
                      sm_main_state_str[sm_get_current_state(state_handle)],
                      sm_main_state_str[sm_get_last_state(state_handle)]);

        config_if_term();

        // Sleep the GPS to save power
            GPS_off();
    }

    syshal_pmu_set_level(POWER_STOP);

    // Branch to Battery Charging if VUSB is present
    if (syshal_gpio_get_input(GPIO_VUSB))
        sm_set_next_state(state_handle, SM_MAIN_BATTERY_CHARGING);
}

static void sm_main_provisioning_needed(sm_handle_t * state_handle)
{
    KICK_WATCHDOG();

    if (sm_is_first_entry(state_handle))
    {
        DEBUG_PR_INFO("Entered state %s from %s",
                      sm_main_state_str[sm_get_current_state(state_handle)],
                      sm_main_state_str[sm_get_last_state(state_handle)]);

        // Sleep the GPS to save power
            GPS_off();
    }

    // Blink an LED to indicate this state
    static uint32_t blinkTimer = 0;
    const uint32_t blinkTimeMs = 300;

    if (syshal_time_get_ticks_ms() >= (blinkTimeMs + blinkTimer))
    {
        syshal_gpio_set_output_high(GPIO_LED2_RED);
        syshal_time_delay_ms(50);
        syshal_gpio_set_output_low(GPIO_LED2_RED);
        blinkTimer = syshal_time_get_ticks_ms();
    }

    manage_ble();

    config_if_tick();

    syshal_timer_tick();

    // Branch to Battery Charging if VUSB is present
    if (syshal_gpio_get_input(GPIO_VUSB))
        sm_set_next_state(state_handle, SM_MAIN_BATTERY_CHARGING);

    // Branch to Provisioning state if config_if has connected
    if (config_if_connected)
        sm_set_next_state(state_handle, SM_MAIN_PROVISIONING);

    // Branch to Battery Low state if battery is beneath threshold
    int level = syshal_batt_level();
    if (level >= 0)
        // If we've read the battery level successfully
        if (sys_config.sys_config_battery_low_threshold.hdr.set &&
            level <= sys_config.sys_config_battery_low_threshold.contents.threshold)
            sm_set_next_state(state_handle, SM_MAIN_BATTERY_LEVEL_LOW);
}

static void sm_main_provisioning(sm_handle_t * state_handle)
{
    KICK_WATCHDOG();

    if (sm_is_first_entry(state_handle))
    {
        DEBUG_PR_INFO("Entered state %s from %s",
                      sm_main_state_str[sm_get_current_state(state_handle)],
                      sm_main_state_str[sm_get_last_state(state_handle)]);

        // Wake the GPS so the configuration interface can communicate with it
        GPS_on();
    }

    bool ready_for_operational_state = check_configuration_tags_set() && log_file_created;

    // Show if device is ready for operation by using GREEN LED for yes and RED LED for no
    if (ready_for_operational_state)
    {
        syshal_gpio_set_output_low(GPIO_LED2_RED);
        syshal_gpio_set_output_high(GPIO_LED1_GREEN);
    }
    else
    {
        syshal_gpio_set_output_high(GPIO_LED2_RED);
        syshal_gpio_set_output_low(GPIO_LED1_GREEN);
    }

    manage_ble();

    config_if_tick();

    syshal_timer_tick();

    if (config_if_connected)
    {
        handle_config_if_messages();
    }
    else
    {
        // Our configuration interface has been disconnected

        if (ready_for_operational_state)
            // Branch to Operational state if log file exists AND configuration tags are set
            sm_set_next_state(state_handle, SM_MAIN_OPERATIONAL);
        else
            // Branch to Provisioning Needed state if configuration tags are not set OR log file doesn't exist
            sm_set_next_state(state_handle, SM_MAIN_PROVISIONING_NEEDED);

        // Branch to Battery Low state if battery is beneath threshold
        int level = syshal_batt_level();
        if (level >= 0)
            // If we've read the battery level successfully
            if (sys_config.sys_config_battery_low_threshold.hdr.set &&
                level <= sys_config.sys_config_battery_low_threshold.contents.threshold)
                sm_set_next_state(state_handle, SM_MAIN_BATTERY_LEVEL_LOW);

        // Branch to Battery Charging if VUSB is present
        if (syshal_gpio_get_input(GPIO_VUSB))
            sm_set_next_state(state_handle, SM_MAIN_BATTERY_CHARGING);
    }

    // Are we about to leave this state?
    if (sm_is_last_entry(state_handle))
    {
        message_set_state(SM_MESSAGE_STATE_IDLE); // Return the message handler to the idle state
        config_if_session_cleanup(); // Clear any pending messages

        // Close any open files
        if (file_handle)
        {
            fs_close(file_handle);
            file_handle = NULL;
        }

        // Close the configuration interface if it's USB
        if (CONFIG_IF_BACKEND_USB == config_if_current())
            config_if_term();
    }
}

////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// STATE HANDLERS ////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void sm_main_exception_handler(CEXCEPTION_T e)
{
    switch (e)
    {
        case EXCEPTION_REQ_WRONG_SIZE:
            DEBUG_PR_ERROR("EXCEPTION_REQ_WRONG_SIZE");
            break;

        case EXCEPTION_RESP_TX_PENDING:
            DEBUG_PR_ERROR("EXCEPTION_RESP_TX_PENDING");
            break;

        case EXCEPTION_TX_BUFFER_FULL:
            DEBUG_PR_ERROR("EXCEPTION_TX_BUFFER_FULL");
            break;

        case EXCEPTION_TX_BUSY:
            DEBUG_PR_ERROR("EXCEPTION_TX_BUSY");
            break;

        case EXCEPTION_RX_BUFFER_EMPTY:
            DEBUG_PR_ERROR("EXCEPTION_RX_BUFFER_EMPTY");
            break;

        case EXCEPTION_RX_BUFFER_FULL:
            DEBUG_PR_ERROR("EXCEPTION_RX_BUFFER_FULL");
            break;

        case EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION:
            DEBUG_PR_ERROR("EXCEPTION_BAD_SYS_CONFIG_ERROR_CONDITION");
            break;

        case EXCEPTION_PACKET_WRONG_SIZE:
            DEBUG_PR_ERROR("EXCEPTION_PACKET_WRONG_SIZE");
            break;

        case EXCEPTION_GPS_SEND_ERROR:
            DEBUG_PR_ERROR("EXCEPTION_GPS_SEND_ERROR");
            break;

        case EXCEPTION_FS_ERROR:
            DEBUG_PR_ERROR("EXCEPTION_FS_ERROR");
            break;

        case EXCEPTION_SPI_ERROR:
            DEBUG_PR_ERROR("EXCEPTION_SPI_ERROR");
            break;

        default:
            DEBUG_PR_ERROR("Unknown state exception %d", e);
            break;
    }
}

void syshal_flash_busy_handler(uint32_t drive)
{
    /* Kick the software watchdog */
    KICK_WATCHDOG();

    /* We must also kick the hardware watchdog here */
    syshal_pmu_kick_watchdog();
}
