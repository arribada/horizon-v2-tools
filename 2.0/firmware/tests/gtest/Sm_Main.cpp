// Sm_Main.cpp - Main statemachine unit tests
//
// Copyright (C) 2018 Arribada
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

extern "C" {
#include "unity.h"
#include <assert.h>
#include <stdint.h>
#include "Mocksyshal_axl.h"
#include "Mocksyshal_batt.h"
#include "Mocksyshal_ble.h"
#include "Mocksyshal_gpio.h"
#include "Mocksyshal_gps.h"
#include "Mocksyshal_flash.h"
#include "Mocksyshal_uart.h"
#include "Mocksyshal_spi.h"
#include "Mocksyshal_switch.h"
#include "Mocksyshal_i2c.h"
#include "Mocksyshal_rtc.h"
#include "Mocksyshal_time.h"
#include "Mocksyshal_pressure.h"
#include "Mocksyshal_pmu.h"
#include "Mockconfig_if.h"
#include "syshal_timer.h"
#include "logging.h"
#include "fs_priv.h"
#include "fs.h"
#include "crc32.h"
#include "cmd.h"

#include "sys_config.h"
#include "sm.h"
#include "sm_main.h"
#include <stdlib.h>
}

#include "googletest.h"

#include <ctime>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include <utility>
#include <queue>
#include <vector>


// config_if
typedef std::vector<uint8_t> message_t;

config_if_backend_t config_if_current_interface;

int config_if_init_GTest(config_if_backend_t backend, int cmock_num_calls)
{
    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current_interface); // We probably shouldn't be calling init when a backend is already in use

    config_if_current_interface = backend;

    return CONFIG_IF_NO_ERROR;
}

int config_if_term_GTest(int cmock_num_calls)
{
    config_if_current_interface = CONFIG_IF_BACKEND_NOT_SET;
    return CONFIG_IF_NO_ERROR;
}

config_if_backend_t config_if_current_GTest(int cmock_num_calls)
{
    return config_if_current_interface;
}

void config_if_tick_GTest(int cmock_num_calls) {}

uint8_t * config_if_receive_buffer;
uint32_t config_if_receive_buffer_size;
bool config_if_receive_queued = false;

static int config_if_receive_GTest(uint8_t * data, uint32_t size, int cmock_num_calls)
{
    if (config_if_receive_queued)
        return CONFIG_IF_ERROR_BUSY;

    config_if_receive_queued = true;

    config_if_receive_buffer = data;
    config_if_receive_buffer_size = size;

    return CONFIG_IF_NO_ERROR;
}

std::vector< std::vector<uint8_t> > config_if_transmitted_data;

static int config_if_send_GTest(uint8_t * data, uint32_t size, int cmock_num_calls)
{
    config_if_transmitted_data.push_back( std::vector<uint8_t> (data, data + size) );

    return CONFIG_IF_NO_ERROR;
}

// Send config_if message to the state machine
void send_message(uint8_t * message, uint32_t size)
{
    // We should have a configuration interface init before sending a message
    ASSERT_NE(CONFIG_IF_BACKEND_NOT_SET, config_if_current_interface);

    ASSERT_LE(size, config_if_receive_buffer_size); // Message is less than or equal to expected size
    ASSERT_NE(nullptr, config_if_receive_buffer);

    // Copy message into receive buffer
    memcpy(config_if_receive_buffer, message, size);

    // Generate a receive complete event
    config_if_event_t event;
    event.id = CONFIG_IF_EVENT_RECEIVE_COMPLETE;
    event.backend = config_if_current_interface;
    event.receive.size = size;

    config_if_receive_queued = false;
    config_if_callback(&event);
}

// Send config_if message to the state machine
void send_message(cmd_t * message, uint32_t size)
{
    send_message((uint8_t *)message, size);
}

void receive_message(uint8_t * message)
{
    ASSERT_GT(config_if_transmitted_data.size(), 0); // We must at least have a message to receive

    // Prepare a transmit complete event
    config_if_event_t event;
    event.id = CONFIG_IF_EVENT_SEND_COMPLETE;
    event.backend = config_if_current_interface;
    event.send.size = config_if_transmitted_data.size();

    // Copy message from send buffer
    for (unsigned int i = 0; i < config_if_transmitted_data[0].size(); ++i)
        message[i] = config_if_transmitted_data.back()[i];

    // Remove this buffer from the vector
    config_if_transmitted_data.pop_back();

    config_if_callback(&event); // Generate the transmit complete event
}

void receive_message(cmd_t * message)
{
    receive_message( (uint8_t *) message);
}

// syshal_time

int syshal_time_init_GTest(int cmock_num_calls) {return SYSHAL_TIME_NO_ERROR;}

uint32_t syshal_time_get_ticks_ms_value;
uint32_t syshal_time_get_ticks_ms_GTest(int cmock_num_calls)
{
    return syshal_time_get_ticks_ms_value;
}

void syshal_time_delay_us_GTest(uint32_t us, int cmock_num_calls) {}
void syshal_time_delay_ms_GTest(uint32_t ms, int cmock_num_calls) {}

// syshal_gpio
bool GPIO_pin_state[GPIO_TOTAL_NUMBER];
void (*GPIO_callback_function[GPIO_TOTAL_NUMBER])(void);

int syshal_gpio_init_GTest(uint32_t pin, int cmock_num_calls) {return SYSHAL_GPIO_NO_ERROR;}
bool syshal_gpio_get_input_GTest(uint32_t pin, int cmock_num_calls) {return GPIO_pin_state[pin];}
void syshal_gpio_set_output_low_GTest(uint32_t pin, int cmock_num_calls) {GPIO_pin_state[pin] = 0;}
void syshal_gpio_set_output_high_GTest(uint32_t pin, int cmock_num_calls) {GPIO_pin_state[pin] = 1;}
void syshal_gpio_set_output_toggle_GTest(uint32_t pin, int cmock_num_calls) {GPIO_pin_state[pin] = !GPIO_pin_state[pin];}
void syshal_gpio_enable_interrupt_GTest(uint32_t pin, void (*callback_function)(void), int cmock_num_calls) {GPIO_callback_function[pin] = callback_function;}

// syshal_ble
const uint32_t syshal_ble_version = 0xABCD0123;

int syshal_ble_get_version_GTest(uint32_t * version, int cmock_num_calls)
{
    *version = syshal_ble_version;
    return SYSHAL_BLE_NO_ERROR;
};

static std::vector<uint8_t> syshal_ble_write_register_address;
static std::vector<uint8_t> syshal_ble_write_register_data;
static std::vector<uint16_t> syshal_ble_write_register_length;
int syshal_ble_write_register_GTest(uint8_t address, uint8_t * data, uint16_t length, int cmock_num_calls)
{
    syshal_ble_write_register_address.push_back(address);

    for (auto i = 0; i < length; ++i)
        syshal_ble_write_register_data.push_back(data[i]);

    syshal_ble_write_register_length.push_back(length);

    return SYSHAL_BLE_NO_ERROR;
}

static std::vector<uint8_t> syshal_ble_read_register_address;
static std::vector<uint8_t> syshal_ble_read_register_data;
static std::vector<uint16_t> syshal_ble_read_register_length;
int syshal_ble_read_register_GTest(uint8_t address, uint8_t * data, uint16_t length, int cmock_num_calls)
{
    syshal_ble_read_register_address.push_back(address);

    for (auto i = 0; i < length; ++i)
    {
        data[i] = syshal_ble_read_register_data.back();
        syshal_ble_read_register_data.pop_back();
    }

    syshal_ble_read_register_length.push_back(length);

    return SYSHAL_BLE_NO_ERROR;
}

// syshal_rtc
time_t current_date_time = time(0);
uint32_t current_milliseconds = 0;

int syshal_rtc_init_GTest(int cmock_num_calls) {return SYSHAL_RTC_NO_ERROR;}

int syshal_rtc_set_date_and_time_GTest(syshal_rtc_data_and_time_t date_time, int cmock_num_calls)
{
    struct tm * timeinfo = localtime(&current_date_time);

    timeinfo->tm_sec = date_time.seconds; // seconds of minutes from 0 to 61
    timeinfo->tm_min = date_time.minutes; // minutes of hour from 0 to 59
    timeinfo->tm_hour = date_time.hours;  // hours of day from 0 to 24
    timeinfo->tm_mday = date_time.day;    // day of month from 1 to 31
    timeinfo->tm_mon = date_time.month;   // month of year from 0 to 11
    timeinfo->tm_year = date_time.year;   // year since 1900

    current_date_time = mktime(timeinfo);

    return SYSHAL_RTC_NO_ERROR;
}

int syshal_rtc_get_date_and_time_GTest(syshal_rtc_data_and_time_t * date_time, int cmock_num_calls)
{
    struct tm * timeinfo = localtime(&current_date_time);

    date_time->milliseconds = current_milliseconds;
    date_time->seconds = timeinfo->tm_sec; // seconds of minutes from 0 to 61
    date_time->minutes = timeinfo->tm_min; // minutes of hour from 0 to 59
    date_time->hours = timeinfo->tm_hour;  // hours of day from 0 to 24
    date_time->day = timeinfo->tm_mday;    // day of month from 1 to 31
    date_time->month = timeinfo->tm_mon;   // month of year from 0 to 11
    date_time->year = timeinfo->tm_year;   // year since 1900

    return SYSHAL_RTC_NO_ERROR;
}

// syshal_batt
int battery_level;

int syshal_batt_level_GTest(int cmock_num_calls)
{
    return battery_level;
}

// syshal_uart
int syshal_uart_init_GTest(uint32_t instance, int cmock_num_calls) {return SYSHAL_UART_NO_ERROR;}

// syshal_spi
int syshal_spi_init_GTest(uint32_t instance, int cmock_num_calls) {return SYSHAL_SPI_NO_ERROR;}

// syshal_i2c
int syshal_i2c_init_GTest(uint32_t instance, int cmock_num_calls) {return SYSHAL_I2C_NO_ERROR;}

// syshal_flash
#define FLASH_SIZE          (FS_PRIV_SECTOR_SIZE * FS_PRIV_MAX_SECTORS)
#define ASCII(x)            ((x) >= 32 && (x) <= 127) ? (x) : '.'

static bool fs_trace;
char flash_ram[FLASH_SIZE];

int syshal_flash_init_GTest(uint32_t drive, uint32_t device, int cmock_num_calls) {return SYSHAL_FLASH_NO_ERROR;}

int syshal_flash_read_GTest(uint32_t device, void * dest, uint32_t address, uint32_t size, int cmock_num_calls)
{
    //printf("syshal_flash_read(%08x,%u)\n", address, size);
    for (unsigned int i = 0; i < size; i++)
        ((char *)dest)[i] = flash_ram[address + i];

    return 0;
}

int syshal_flash_write_GTest(uint32_t device, const void * src, uint32_t address, uint32_t size, int cmock_num_calls)
{
    if (fs_trace)
        printf("syshal_flash_write(%08x, %u)\n", address, size);
    for (unsigned int i = 0; i < size; i++)
    {
        /* Ensure no new bits are being set */
        if ((((char *)src)[i] & flash_ram[address + i]) ^ ((char *)src)[i])
        {
            printf("syshal_flash_write: Can't set bits from 0 to 1 (%08x: %02x => %02x)\n", address + i,
                   (uint8_t)flash_ram[address + i], (uint8_t)((char *)src)[i]);
            assert(0);
        }
        flash_ram[address + i] = ((char *)src)[i];
    }

    return 0;
}

int syshal_flash_erase_GTest(uint32_t device, uint32_t address, uint32_t size, int cmock_num_calls)
{
    /* Make sure address is sector aligned */
    if (address % FS_PRIV_SECTOR_SIZE || size % FS_PRIV_SECTOR_SIZE)
    {
        printf("syshal_flash_erase: Non-aligned address %08x", address);
        assert(0);
    }

    for (unsigned int i = 0; i < size; i++)
        flash_ram[address + i] = 0xFF;

    return 0;
}

// syshal_gps

// syshal_gps_send_raw callback function
uint8_t gps_write_buffer[2048];
int syshal_gps_send_raw_GTest(uint8_t * data, uint32_t size, int cmock_num_calls)
{
    if (size > sizeof(gps_write_buffer))
        assert(0);

    memcpy(&gps_write_buffer[0], data, size);

    return SYSHAL_GPS_NO_ERROR;
}

// syshal_gps_receive_raw callback function
std::queue<uint8_t> gps_receive_buffer;
int syshal_gps_receive_raw_GTest(uint8_t * data, uint32_t size, int cmock_num_calls)
{
    if (size > gps_receive_buffer.size())
        size = gps_receive_buffer.size();

    for (unsigned int i = 0; i < size; ++i)
    {
        data[i] = gps_receive_buffer.front();
        gps_receive_buffer.pop();
    }

    return size;
}

// syshal_switch
bool syshal_switch_state;

int syshal_switch_init_GTest(int cmock_num_calls) {return SYSHAL_SWITCH_NO_ERROR;}
bool syshal_switch_get_GTest(int cmock_num_calls) {return syshal_switch_state;}

// syshal_pressure
bool syshal_pressure_awake_GTest(int cmock_num_calls) {return false;}

// syshal_axl
bool syshal_axl_awake_GTest(int cmock_num_calls) {return false;}

// syshal_gps
bool syshal_gps_on;
void syshal_gps_wake_up_GTest(int cmock_num_calls) {syshal_gps_on = true;}
void syshal_gps_shutdown_GTest(int cmock_num_calls) {syshal_gps_on = false;}

class Sm_MainTest : public ::testing::Test
{

    virtual void SetUp()
    {
        srand(time(NULL));

        // config_if
        Mockconfig_if_Init();

        config_if_init_StubWithCallback(config_if_init_GTest);
        config_if_term_StubWithCallback(config_if_term_GTest);
        config_if_current_StubWithCallback(config_if_current_GTest);
        config_if_send_StubWithCallback(config_if_send_GTest);
        config_if_receive_StubWithCallback(config_if_receive_GTest);
        config_if_tick_StubWithCallback(config_if_tick_GTest);

        config_if_current_interface = CONFIG_IF_BACKEND_NOT_SET;

        // syshal_axl
        Mocksyshal_axl_Init();

        syshal_axl_awake_StubWithCallback(syshal_axl_awake_GTest);

        // syshal_gpio
        Mocksyshal_gpio_Init();

        syshal_gpio_init_StubWithCallback(syshal_gpio_init_GTest);
        syshal_gpio_get_input_StubWithCallback(syshal_gpio_get_input_GTest);
        syshal_gpio_set_output_low_StubWithCallback(syshal_gpio_set_output_low_GTest);
        syshal_gpio_set_output_high_StubWithCallback(syshal_gpio_set_output_high_GTest);
        syshal_gpio_set_output_toggle_StubWithCallback(syshal_gpio_set_output_toggle_GTest);
        syshal_gpio_enable_interrupt_StubWithCallback(syshal_gpio_enable_interrupt_GTest);

        // Set all gpio pin states to low
        for (unsigned int i = 0; i < GPIO_TOTAL_NUMBER; ++i)
            GPIO_pin_state[i] = 0;

        // Clear all gpio interrupts
        for (unsigned int i = 0; i < GPIO_TOTAL_NUMBER; ++i)
            GPIO_callback_function[i] = nullptr;

        // syshal_time
        Mocksyshal_time_Init();

        syshal_time_init_StubWithCallback(syshal_time_init_GTest);
        syshal_time_delay_us_StubWithCallback(syshal_time_delay_us_GTest);
        syshal_time_delay_ms_StubWithCallback(syshal_time_delay_ms_GTest);
        syshal_time_get_ticks_ms_StubWithCallback(syshal_time_get_ticks_ms_GTest);
        syshal_time_get_ticks_ms_value = 0;

        // syshal_ble
        Mocksyshal_ble_Init();
        syshal_ble_get_version_StubWithCallback(syshal_ble_get_version_GTest);

        syshal_ble_write_register_address.clear();
        syshal_ble_write_register_data.clear();
        syshal_ble_write_register_length.clear();
        syshal_ble_read_register_address.clear();
        syshal_ble_read_register_data.clear();
        syshal_ble_read_register_length.clear();

        // syshal_rtc
        Mocksyshal_rtc_Init();

        syshal_rtc_init_StubWithCallback(syshal_rtc_init_GTest);
        syshal_rtc_set_date_and_time_StubWithCallback(syshal_rtc_set_date_and_time_GTest);
        syshal_rtc_get_date_and_time_StubWithCallback(syshal_rtc_get_date_and_time_GTest);
        syshal_rtc_soft_watchdog_enable_IgnoreAndReturn(SYSHAL_RTC_NO_ERROR);
        syshal_rtc_soft_watchdog_refresh_IgnoreAndReturn(SYSHAL_RTC_NO_ERROR);

        // syshal_batt
        Mocksyshal_batt_Init();

        syshal_batt_init_IgnoreAndReturn(SYSHAL_BATT_NO_ERROR);
        syshal_batt_level_StubWithCallback(syshal_batt_level_GTest);

        battery_level = 100;

        // sys_config
        unset_all_configuration_tags_RAM();

        // syshal_uart
        Mocksyshal_uart_Init();
        syshal_uart_init_StubWithCallback(syshal_uart_init_GTest);

        // syshal_spi
        Mocksyshal_spi_Init();
        syshal_spi_init_StubWithCallback(syshal_spi_init_GTest);

        // syshal_i2c
        Mocksyshal_i2c_Init();
        syshal_i2c_init_StubWithCallback(syshal_i2c_init_GTest);

        // syshal_flash
        Mocksyshal_flash_Init();
        syshal_flash_init_StubWithCallback(syshal_flash_init_GTest);
        syshal_flash_read_StubWithCallback(syshal_flash_read_GTest);
        syshal_flash_write_StubWithCallback(syshal_flash_write_GTest);
        syshal_flash_erase_StubWithCallback(syshal_flash_erase_GTest);

        // Clear FLASH contents
        for (auto i = 0; i < FLASH_SIZE; ++i)
            flash_ram[i] = 0xFF;

        fs_trace = false; // turn FS trace off

        // syshal_gps
        Mocksyshal_gps_Init();
        syshal_gps_on = false;
        syshal_gps_init_Ignore();
        syshal_gps_tick_Ignore();
        syshal_gps_wake_up_StubWithCallback(syshal_gps_wake_up_GTest);
        syshal_gps_shutdown_StubWithCallback(syshal_gps_shutdown_GTest);
        syshal_gps_send_raw_StubWithCallback(syshal_gps_send_raw_GTest);
        syshal_gps_receive_raw_StubWithCallback(syshal_gps_receive_raw_GTest);

        // syshal_switch
        Mocksyshal_switch_Init();
        syshal_switch_init_StubWithCallback(syshal_switch_init_GTest);
        syshal_switch_get_StubWithCallback(syshal_switch_get_GTest);

        // syshal_pressure
        Mocksyshal_pressure_Init();
        syshal_pressure_awake_StubWithCallback(syshal_pressure_awake_GTest);

        // syshal_pmu
        Mocksyshal_pmu_Init();

        syshal_pmu_set_level_Ignore();

        // Setup main state machine
        sm_init(&state_handle, sm_main_states);

        sys_config.format_version = SYS_CONFIG_FORMAT_VERSION;
    }

    virtual void TearDown()
    {
        // Reset all syshal timers
        for (timer_handle_t i = 0; i < SYSHAL_TIMER_NUMBER_OF_TIMERS; ++i)
            syshal_timer_term(i);

        Mockconfig_if_Verify();
        Mockconfig_if_Destroy();
        Mocksyshal_axl_Verify();
        Mocksyshal_axl_Destroy();
        Mocksyshal_gpio_Verify();
        Mocksyshal_gpio_Destroy();
        Mocksyshal_time_Verify();
        Mocksyshal_time_Destroy();
        Mocksyshal_ble_Verify();
        Mocksyshal_ble_Destroy();
        Mocksyshal_rtc_Verify();
        Mocksyshal_rtc_Destroy();
        Mocksyshal_batt_Verify();
        Mocksyshal_batt_Destroy();
        Mocksyshal_uart_Verify();
        Mocksyshal_uart_Destroy();
        Mocksyshal_spi_Verify();
        Mocksyshal_spi_Destroy();
        Mocksyshal_i2c_Verify();
        Mocksyshal_i2c_Destroy();
        Mocksyshal_flash_Verify();
        Mocksyshal_flash_Destroy();
        Mocksyshal_gps_Verify();
        Mocksyshal_gps_Destroy();
        Mocksyshal_pressure_Verify();
        Mocksyshal_pressure_Destroy();
        Mocksyshal_pmu_Verify();
        Mocksyshal_pmu_Destroy();
    }

public:

    sm_handle_t state_handle;

    void BootTagsNotSet(void)
    {
        sm_set_current_state(&state_handle, SM_MAIN_BOOT);

        sm_tick(&state_handle);
    }

    void BootTagsSetAndLogFileCreated(void)
    {
        set_all_configuration_tags_RAM();
        SetBatteryLowThreshold(10);

        // Create the log file
        fs_t file_system;
        fs_handle_t file_system_handle;

        EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
        EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
        EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
        EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_CREATE, NULL));
        EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

        sm_set_current_state(&state_handle, SM_MAIN_BOOT);

        sm_tick(&state_handle);
    }


    void CreateEmptyLogfile(void)
    {
        fs_t file_system;
        fs_handle_t file_system_handle;

        EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
        EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
        EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
        EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_CREATE, NULL));
        EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));
    }

    void SetVUSB(bool state)
    {
        SetGPIOPin(GPIO_VUSB, state);
    }

    void SetBatteryPercentage(int level)
    {
        battery_level = level;
    }

    void SetBatteryLowThreshold(uint8_t level)
    {
        sys_config.sys_config_battery_low_threshold.hdr.set = true;
        sys_config.sys_config_battery_low_threshold.contents.threshold = level;
    }

    void SetGPIOPin(uint32_t pin, bool state)
    {
        GPIO_pin_state[pin] = state;

        // If there's any interrupt on this pin, call it
        if (GPIO_callback_function[pin])
            GPIO_callback_function[pin]();
    }

    static void IncrementMilliseconds(uint32_t milliseconds)
    {
        struct tm * timeinfo = localtime(&current_date_time);

        timeinfo->tm_sec += (current_milliseconds + milliseconds) / 1000;

        current_date_time = mktime(timeinfo);

        current_milliseconds = (current_milliseconds + milliseconds) % 1000;

        syshal_time_get_ticks_ms_value += milliseconds;
    }

    static void IncrementSeconds(uint32_t seconds)
    {
        IncrementMilliseconds(seconds * 1000);
    }

    // Message handling //

    static void BLEConnectionEvent(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_CONNECTED;
        event.backend = CONFIG_IF_BACKEND_BLE;
        config_if_callback(&event);
    }

    static void BLEDisconnectEvent(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_DISCONNECTED;
        event.backend = CONFIG_IF_BACKEND_BLE;
        config_if_callback(&event);
    }

    static void BLETriggeredOnReedSwitchEnable(void)
    {
        sys_config.sys_config_tag_bluetooth_trigger_control.hdr.set = true;
        sys_config.sys_config_tag_bluetooth_trigger_control.contents.flags |= SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_REED_SWITCH;
    }

    static void BLETriggeredOnSchedule(uint32_t interval, uint32_t duration, uint32_t timeout)
    {
        sys_config.sys_config_tag_bluetooth_trigger_control.hdr.set = true;
        sys_config.sys_config_tag_bluetooth_trigger_control.contents.flags |= SYS_CONFIG_TAG_BLUETOOTH_TRIGGER_CONTROL_SCHEDULED;

        sys_config.sys_config_tag_bluetooth_scheduled_interval.hdr.set = true;
        sys_config.sys_config_tag_bluetooth_scheduled_interval.contents.seconds = interval;

        sys_config.sys_config_tag_bluetooth_scheduled_duration.hdr.set = true;
        sys_config.sys_config_tag_bluetooth_scheduled_duration.contents.seconds = duration;

        sys_config.sys_config_tag_bluetooth_connection_inactivity_timeout.hdr.set = true;
        sys_config.sys_config_tag_bluetooth_connection_inactivity_timeout.contents.seconds = timeout;
    }

    static void USBConnectionEvent(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_CONNECTED;
        event.backend = CONFIG_IF_BACKEND_USB;
        config_if_callback(&event);
    }

    static void USBDisconnectEvent(void)
    {
        config_if_event_t event;
        event.id = CONFIG_IF_EVENT_DISCONNECTED;
        event.backend = CONFIG_IF_BACKEND_USB;
        config_if_callback(&event);
    }

    static void set_all_configuration_tags_RAM()
    {
        uint8_t config_if_dummy_data[SYS_CONFIG_MAX_DATA_SIZE] = {0};

        uint16_t tag, last_index = 0;
        uint32_t length = 0;

        while (!sys_config_iterate(&tag, &last_index))
        {
            int ret;
            do
            {
                ret = sys_config_set(tag, &config_if_dummy_data, length++);
            }
            while (SYS_CONFIG_ERROR_WRONG_SIZE == ret);

            length = 0;

            if (SYS_CONFIG_ERROR_NO_MORE_TAGS == ret)
                break;
        }
    }

    static void unset_all_configuration_tags_RAM()
    {
        // Set all configuration tag data to random values
        // This is to ensure the tests properly ignore them
        for (auto i = 0; i < sizeof(sys_config); ++i)
            ((uint8_t *)&sys_config)[i] = rand();

        // Unset all the tags
        uint16_t last_index = 0;
        uint16_t tag;

        while (!sys_config_iterate(&tag, &last_index))
        {
            sys_config_unset(tag);
        }
    }

};

//////////////////////////////////////////////////////////////////
/////////////////////////// Boot State ///////////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, BootNoTags)
{
    BootTagsNotSet();

    EXPECT_EQ(SM_MAIN_PROVISIONING_NEEDED, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BootNoTagsVUSB)
{
    SetVUSB(true);

    BootTagsNotSet();

    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BootOperationalState)
{
    BootTagsSetAndLogFileCreated();

    EXPECT_EQ(SM_MAIN_OPERATIONAL, sm_get_current_state(&state_handle));
}

//////////////////////////////////////////////////////////////////
/////////////////////// Battery Low State ////////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, BatteryLowToBatteryChargingNoVUSB)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_LEVEL_LOW);

    SetVUSB(false);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_LEVEL_LOW, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryLowToBatteryChargingVUSB)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_LEVEL_LOW);

    SetVUSB(true);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

//////////////////////////////////////////////////////////////////
///////////////////// Battery Charging State /////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, BatteryChargingNoVUSBBatteryLowNoThreshold)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    SetVUSB(false);
    SetBatteryPercentage(0);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING_NEEDED, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryChargingNoVUSBOperationalState)
{
    BootTagsSetAndLogFileCreated();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    SetVUSB(false);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_OPERATIONAL, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryChargingNoVUSBBatteryLow)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    SetVUSB(false);
    SetBatteryPercentage(0);

    SetBatteryLowThreshold(10);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_LEVEL_LOW, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryChargingUSBTimeout)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    SetVUSB(true);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));

    // Jump forward 5 seconds
    IncrementSeconds(5);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));

    // Jump forward 100 seconds
    IncrementSeconds(100);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryChargingBLERunningButNotConnected)
{
    // This should realise the BLE stack is running but is not connected
    // So it should terminate it and start the USB stack
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    config_if_init(CONFIG_IF_BACKEND_BLE);
    SetVUSB(true);

    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryChargingUSBConnected)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    SetVUSB(true);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));

    USBConnectionEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryChargingBLEReedSwitchWithUSBTimeout)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    SetVUSB(true);
    BLETriggeredOnReedSwitchEnable(); // Ensure the BLE should be on by setting the Reed switch activation
    SetGPIOPin(GPIO_REED_SW, 0); // Trigger the reed switch

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));

    // Jump forward 5 seconds
    IncrementSeconds(5);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));

    // Jump forward 100 seconds
    IncrementSeconds(100);

    sm_tick(&state_handle);
    sm_tick(&state_handle);

    // USB timed out but BLE should be running
    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, BatteryChargingBLEScheduledWithUSBTimeout)
{
    const uint32_t interval = 15;
    const uint32_t duration = 10;

    SetVUSB(true);
    BootTagsNotSet();
    BLETriggeredOnSchedule(15, 5, 0);

    sm_set_current_state(&state_handle, SM_MAIN_BATTERY_CHARGING);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));

    // Jump forward 5 seconds
    IncrementSeconds(5);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));

    // Jump forward 15 seconds
    IncrementSeconds(15);

    sm_tick(&state_handle);
    sm_tick(&state_handle);

    // USB timed out but BLE should be running
    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());
    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

//////////////////////////////////////////////////////////////////
////////////////////// Log File Full State ///////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, LogFileFullToBatteryCharging)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_LOG_FILE_FULL);

    EXPECT_EQ(SM_MAIN_LOG_FILE_FULL, sm_get_current_state(&state_handle));

    SetVUSB(true);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, LogFileFullToLowBattery)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_LOG_FILE_FULL);

    EXPECT_EQ(SM_MAIN_LOG_FILE_FULL, sm_get_current_state(&state_handle));

    SetBatteryPercentage(0);
    SetBatteryLowThreshold(10);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_LEVEL_LOW, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, LogFileFullBLEConnection)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_LOG_FILE_FULL);

    EXPECT_EQ(SM_MAIN_LOG_FILE_FULL, sm_get_current_state(&state_handle));

    config_if_init(CONFIG_IF_BACKEND_BLE);
    BLETriggeredOnReedSwitchEnable(); // Ensure the BLE should be on by setting the Reed switch activation
    SetGPIOPin(GPIO_REED_SW, 0); // Trigger the reed switch
    BLEConnectionEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, LogFileFullBLEReedSwitchToggle)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_LOG_FILE_FULL);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());

    // Enable reed switch activation of BLE
    BLETriggeredOnReedSwitchEnable();

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    SetGPIOPin(GPIO_REED_SW, 0); // Trigger the reed switch

    IncrementSeconds(5); // Progress time to allow for switch debouncing

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());
    SetGPIOPin(GPIO_REED_SW, 1); // Release the reed switch

    IncrementSeconds(5); // Progress time to allow for switch debouncing

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    EXPECT_EQ(SM_MAIN_LOG_FILE_FULL, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, LogFileFullBLEScheduled)
{
    const uint32_t interval = 15;
    const uint32_t duration = 5;

    BootTagsNotSet();
    BLETriggeredOnSchedule(15, 5, 0);

    sm_set_current_state(&state_handle, SM_MAIN_LOG_FILE_FULL);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());

    IncrementSeconds(interval);

    sm_tick(&state_handle);
    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());

    IncrementSeconds(duration);

    sm_tick(&state_handle);
    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    EXPECT_EQ(SM_MAIN_LOG_FILE_FULL, sm_get_current_state(&state_handle));
}

//////////////////////////////////////////////////////////////////
//////////////////// Provisioning Needed State ///////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, ProvisioningNeededToBatteryCharging)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING_NEEDED);

    SetVUSB(true);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, ProvisioningNeededToBatteryLow)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING_NEEDED);

    SetVUSB(false);
    SetBatteryPercentage(0);
    SetBatteryLowThreshold(10);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_LEVEL_LOW, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, ProvisioningNeededToProvisioning)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING_NEEDED);

    config_if_init(CONFIG_IF_BACKEND_BLE);
    BLETriggeredOnReedSwitchEnable(); // Ensure the BLE should be on by setting the Reed switch activation
    SetGPIOPin(GPIO_REED_SW, 0); // Trigger the reed switch
    BLEConnectionEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, ProvisioningNeededBLEReedSwitchToggle)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING_NEEDED);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());

    // Enable reed switch activation of BLE
    BLETriggeredOnReedSwitchEnable();

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    SetGPIOPin(GPIO_REED_SW, 0); // Trigger the reed switch

    IncrementSeconds(5); // Progress time to allow for switch debouncing

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());
    SetGPIOPin(GPIO_REED_SW, 1); // Release the reed switch

    IncrementSeconds(5); // Progress time to allow for switch debouncing

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    EXPECT_EQ(SM_MAIN_PROVISIONING_NEEDED, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, ProvisioningNeededBLEReedSwitchToggleButDisabled)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING_NEEDED);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    SetGPIOPin(GPIO_REED_SW, 0); // Trigger the reed switch

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    SetGPIOPin(GPIO_REED_SW, 1); // Release the reed switch

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    EXPECT_EQ(SM_MAIN_PROVISIONING_NEEDED, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, ProvisioningNeededBLEScheduled)
{
    const uint32_t interval = 15;
    const uint32_t duration = 5;

    BootTagsNotSet();
    BLETriggeredOnSchedule(interval, duration, 0);

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING_NEEDED);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());

    IncrementSeconds(interval);

    sm_tick(&state_handle);
    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());

    IncrementSeconds(duration);

    sm_tick(&state_handle);
    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    EXPECT_EQ(SM_MAIN_PROVISIONING_NEEDED, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, ProvisioningNeededBLEScheduledConnectionInactivityTimeout)
{
    const uint32_t interval = 30;
    const uint32_t duration = 15;
    const uint32_t timeout = 5;

    BootTagsNotSet();
    BLETriggeredOnSchedule(interval, duration, timeout);

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING_NEEDED);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());

    sm_tick(&state_handle);
    sm_tick(&state_handle);

    IncrementSeconds(interval);

    BLEConnectionEvent();

    sm_tick(&state_handle);
    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());

    IncrementSeconds(1);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());
    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));

    IncrementSeconds(timeout);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    EXPECT_EQ(SM_MAIN_PROVISIONING_NEEDED, sm_get_current_state(&state_handle));
}

//////////////////////////////////////////////////////////////////
/////////////////////// Provisioning State ///////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, ProvisioningToProvisioningNeeded)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_BLE);
    BLETriggeredOnReedSwitchEnable(); // Ensure the BLE should be on by setting the Reed switch activation
    SetGPIOPin(GPIO_REED_SW, 0); // Trigger the reed switch
    BLEConnectionEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));

    BLEDisconnectEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING_NEEDED, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, ProvisioningToCharging)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    SetVUSB(true);
    USBConnectionEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));

    USBDisconnectEvent();

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, ProvisioningToOperationalState)
{
    BootTagsSetAndLogFileCreated();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_OPERATIONAL, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, ProvisioningBLEReedSwitchDisable)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    // Enable reed switch activation of BLE
    BLETriggeredOnReedSwitchEnable();
    SetGPIOPin(GPIO_REED_SW, 0); // Trigger the reed switch

    config_if_init(CONFIG_IF_BACKEND_BLE);
    BLEConnectionEvent();

    IncrementSeconds(5); // Progress time to allow for switch debouncing

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());

    SetGPIOPin(GPIO_REED_SW, 1); // Release the reed switch

    IncrementSeconds(5); // Progress time to allow for switch debouncing

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    EXPECT_EQ(SM_MAIN_PROVISIONING_NEEDED, sm_get_current_state(&state_handle));
}

//////////////////////////////////////////////////////////////////
/////////////////////// Operational State ////////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, OperationalToBatteryLow)
{
    BootTagsSetAndLogFileCreated();

    sm_set_current_state(&state_handle, SM_MAIN_OPERATIONAL);

    SetBatteryPercentage(0);
    SetBatteryLowThreshold(10);

    // TODO: handle these ignored functions properly
    syshal_axl_term_IgnoreAndReturn(0);
    syshal_pressure_term_IgnoreAndReturn(0);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_LEVEL_LOW, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, OperationalToBatteryCharging)
{
    BootTagsSetAndLogFileCreated();

    sm_set_current_state(&state_handle, SM_MAIN_OPERATIONAL);

    SetVUSB(true);

    // TODO: handle these ignored functions properly
    syshal_axl_term_IgnoreAndReturn(0);
    syshal_pressure_term_IgnoreAndReturn(0);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_BATTERY_CHARGING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, OperationalToProvisioning)
{
    BootTagsSetAndLogFileCreated();

    sm_set_current_state(&state_handle, SM_MAIN_OPERATIONAL);

    config_if_init(CONFIG_IF_BACKEND_BLE);
    BLETriggeredOnReedSwitchEnable(); // Ensure the BLE should be on by setting the Reed switch activation
    SetGPIOPin(GPIO_REED_SW, 0); // Trigger the reed switch
    BLEConnectionEvent();

    // TODO: handle these ignored functions properly
    syshal_axl_term_IgnoreAndReturn(0);
    syshal_pressure_term_IgnoreAndReturn(0);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, OperationalToLogFileFull)
{
    int32_t pressureReading = rand();

    BootTagsSetAndLogFileCreated();

    // Fill the log file completely
    fs_t file_system;
    fs_handle_t file_system_handle;
    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_WRITEONLY, NULL));

    int writeResult;
    uint8_t dummyData[2048];
    uint32_t bytes_written;

    do
    {
        writeResult = fs_write(file_handle, dummyData, sizeof(dummyData), &bytes_written);
    } while (writeResult == FS_NO_ERROR);

    EXPECT_EQ(FS_ERROR_FILESYSTEM_FULL, writeResult);
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    sm_set_current_state(&state_handle, SM_MAIN_OPERATIONAL);

    // Enable general logging
    sys_config.sys_config_logging_enable.hdr.set = true;
    sys_config.sys_config_logging_enable.contents.enable = true;

    // Enable the pressure sensor
    sys_config.sys_config_pressure_sensor_log_enable.hdr.set = true;
    sys_config.sys_config_pressure_sensor_log_enable.contents.enable = true;

    // TODO: handle these ignored functions properly
    syshal_pressure_init_ExpectAndReturn(SYSHAL_PRESSURE_NO_ERROR);
    syshal_pressure_wake_ExpectAndReturn(SYSHAL_PRESSURE_NO_ERROR);
    syshal_pressure_tick_ExpectAndReturn(SYSHAL_PRESSURE_NO_ERROR);

    sm_tick(&state_handle);

    syshal_pressure_callback(pressureReading);

    syshal_pressure_tick_ExpectAndReturn(SYSHAL_PRESSURE_NO_ERROR);

    syshal_axl_term_IgnoreAndReturn(SYSHAL_AXL_NO_ERROR);
    syshal_pressure_term_IgnoreAndReturn(SYSHAL_PRESSURE_NO_ERROR);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_LOG_FILE_FULL, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, OperationalBLEReedSwitchToggle)
{
    BootTagsSetAndLogFileCreated();

    sm_set_current_state(&state_handle, SM_MAIN_OPERATIONAL);

    // TODO: handle these ignored functions properly
    syshal_axl_term_IgnoreAndReturn(0);
    syshal_pressure_term_IgnoreAndReturn(0);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());

    // Enable reed switch activation of BLE
    BLETriggeredOnReedSwitchEnable();

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    SetGPIOPin(GPIO_REED_SW, 0); // Trigger the reed switch

    IncrementSeconds(5); // Progress time to allow for switch debouncing

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());
    SetGPIOPin(GPIO_REED_SW, 1); // Release the reed switch

    IncrementSeconds(5); // Progress time to allow for switch debouncing

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    EXPECT_EQ(SM_MAIN_OPERATIONAL, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, OperationalPressureLogging)
{
    int32_t pressureReading = rand();

    BootTagsSetAndLogFileCreated();

    sm_set_current_state(&state_handle, SM_MAIN_OPERATIONAL);

    // Enable general logging
    sys_config.sys_config_logging_enable.hdr.set = true;
    sys_config.sys_config_logging_enable.contents.enable = true;

    // Enable the pressure sensor
    sys_config.sys_config_pressure_sensor_log_enable.hdr.set = true;
    sys_config.sys_config_pressure_sensor_log_enable.contents.enable = true;

    // TODO: handle these ignored functions properly
    syshal_pressure_init_ExpectAndReturn(SYSHAL_PRESSURE_NO_ERROR);
    syshal_pressure_wake_ExpectAndReturn(SYSHAL_PRESSURE_NO_ERROR);
    syshal_pressure_tick_ExpectAndReturn(SYSHAL_PRESSURE_NO_ERROR);

    sm_tick(&state_handle);

    syshal_pressure_callback(pressureReading);

    syshal_pressure_tick_ExpectAndReturn(SYSHAL_PRESSURE_NO_ERROR);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_OPERATIONAL, sm_get_current_state(&state_handle));

    EXPECT_EQ(FS_NO_ERROR, fs_flush(file_handle)); // Flush any content that hasn't been written yet

    fs_t file_system;
    fs_handle_t file_system_handle;
    uint32_t bytes_read;
    logging_pressure_t pressureLog;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(file_system_handle, (uint8_t *) &pressureLog, sizeof(logging_pressure_t), &bytes_read));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    EXPECT_EQ(sizeof(logging_pressure_t), bytes_read);
    EXPECT_EQ(LOGGING_PRESSURE, pressureLog.h.id);
    EXPECT_EQ(pressureReading, pressureLog.pressure);
}

TEST_F(Sm_MainTest, OperationalAXLLogging)
{
    syshal_axl_data_t axlReading;
    axlReading.x = rand();
    axlReading.y = rand();
    axlReading.z = rand();

    BootTagsSetAndLogFileCreated();

    sm_set_current_state(&state_handle, SM_MAIN_OPERATIONAL);

    // Enable general logging
    sys_config.sys_config_logging_enable.hdr.set = true;
    sys_config.sys_config_logging_enable.contents.enable = true;

    // Enable the axl sensor
    sys_config.sys_config_axl_log_enable.hdr.set = true;
    sys_config.sys_config_axl_log_enable.contents.enable = true;

    // TODO: handle these ignored functions properly
    syshal_axl_init_ExpectAndReturn(SYSHAL_AXL_NO_ERROR);
    syshal_axl_wake_ExpectAndReturn(SYSHAL_AXL_NO_ERROR);
    syshal_axl_tick_ExpectAndReturn(SYSHAL_AXL_NO_ERROR);

    sm_tick(&state_handle);

    syshal_axl_callback(axlReading);

    syshal_axl_tick_ExpectAndReturn(SYSHAL_AXL_NO_ERROR);

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_OPERATIONAL, sm_get_current_state(&state_handle));

    EXPECT_EQ(FS_NO_ERROR, fs_flush(file_handle)); // Flush any content that hasn't been written yet

    fs_t file_system;
    fs_handle_t file_system_handle;
    uint32_t bytes_read;
    logging_axl_xyz_t axlLog;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(file_system_handle, (uint8_t *) &axlLog, sizeof(logging_axl_xyz_t), &bytes_read));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    EXPECT_EQ(sizeof(logging_axl_xyz_t), bytes_read);
    EXPECT_EQ(LOGGING_AXL_XYZ, axlLog.h.id);
    EXPECT_EQ(axlReading.x, axlLog.x);
    EXPECT_EQ(axlReading.y, axlLog.y);
    EXPECT_EQ(axlReading.z, axlLog.z);
}

TEST_F(Sm_MainTest, OperationalBatteryLogging)
{
    int32_t batteryLevel = rand() % 100;

    BootTagsSetAndLogFileCreated();

    sm_set_current_state(&state_handle, SM_MAIN_OPERATIONAL);

    // Enable general logging
    sys_config.sys_config_logging_enable.hdr.set = true;
    sys_config.sys_config_logging_enable.contents.enable = true;

    // Enable battery level logging
    sys_config.sys_config_battery_log_enable.hdr.set = true;
    sys_config.sys_config_battery_log_enable.contents.enable = true;

    // Disable battery low threshold
    sys_config.sys_config_battery_low_threshold.hdr.set = false;

    SetBatteryPercentage(batteryLevel);

    sm_tick(&state_handle);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_OPERATIONAL, sm_get_current_state(&state_handle));

    EXPECT_EQ(FS_NO_ERROR, fs_flush(file_handle)); // Flush any content that hasn't been written yet

    fs_t file_system;
    fs_handle_t file_system_handle;
    uint32_t bytes_read;
    logging_battery_t batteryCharge;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(file_system_handle, (uint8_t *) &batteryCharge, sizeof(batteryCharge), &bytes_read));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    EXPECT_EQ(sizeof(logging_battery_t), bytes_read);
    EXPECT_EQ(LOGGING_BATTERY, batteryCharge.h.id);
    EXPECT_EQ(batteryLevel, batteryCharge.charge);
}

TEST_F(Sm_MainTest, OperationalBLEScheduled)
{
    const uint32_t interval = 15;
    const uint32_t duration = 5;

    BootTagsSetAndLogFileCreated();
    BLETriggeredOnSchedule(15, 5, 0);

    sm_set_current_state(&state_handle, SM_MAIN_OPERATIONAL);

    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());

    IncrementSeconds(interval);

    sm_tick(&state_handle);
    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_BLE, config_if_current());

    IncrementSeconds(duration);

    sm_tick(&state_handle);
    sm_tick(&state_handle);

    EXPECT_EQ(CONFIG_IF_BACKEND_NOT_SET, config_if_current());
    EXPECT_EQ(SM_MAIN_OPERATIONAL, sm_get_current_state(&state_handle));
}

TEST_F(Sm_MainTest, OperationalStateGPSScheduled)
{
    uint32_t acquisition_interval = 165;
    uint32_t maximum_acquisition = 15;
    uint32_t no_fix_timeout = 0;

    sm_set_current_state(&state_handle, SM_MAIN_BOOT);

    set_all_configuration_tags_RAM();
    CreateEmptyLogfile();

    // Set GPS trigger mode
    sys_config.sys_config_gps_trigger_mode.hdr.set = true;
    sys_config.sys_config_gps_trigger_mode.contents.mode = SYS_CONFIG_GPS_TRIGGER_MODE_SCHEDULED;

    // Set GPS acquisition interval
    sys_config.sys_config_gps_scheduled_acquisition_interval.hdr.set = true;
    sys_config.sys_config_gps_scheduled_acquisition_interval.contents.seconds = acquisition_interval;

    // Set GPS acquisition time
    sys_config.sys_config_gps_maximum_acquisition_time.hdr.set = true;
    sys_config.sys_config_gps_maximum_acquisition_time.contents.seconds = maximum_acquisition;

    // Set GPS no fix timeout
    sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout.hdr.set = true;
    sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout.contents.seconds = no_fix_timeout;

    // Disable GPS first fix hold
    sys_config.sys_config_gps_very_first_fix_hold_time.hdr.set = false;

    // Enable GPS logging
    sys_config.sys_config_gps_log_position_enable.hdr.set = true;
    sys_config.sys_config_gps_log_position_enable.contents.enable = true;

    // Enable Global logging
    sys_config.sys_config_logging_enable.hdr.set = true;
    sys_config.sys_config_logging_enable.contents.enable = true;

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_OPERATIONAL, sm_get_current_state(&state_handle));
    EXPECT_FALSE(syshal_gps_on);

    for (unsigned int i = 0; i < acquisition_interval+1; ++i)
    {
        IncrementSeconds(1);
        sm_tick(&state_handle);
    }

    EXPECT_TRUE(syshal_gps_on);

    for (unsigned int i = 0; i < maximum_acquisition; ++i)
    {
        IncrementSeconds(1);
        sm_tick(&state_handle);
    }

    EXPECT_FALSE(syshal_gps_on);

    for (unsigned int i = 0; i < acquisition_interval - maximum_acquisition; ++i)
    {
        IncrementSeconds(1);
        sm_tick(&state_handle);
    }

    EXPECT_TRUE(syshal_gps_on);

    for (unsigned int i = 0; i < maximum_acquisition; ++i)
    {
        IncrementSeconds(1);
        sm_tick(&state_handle);
    }

    EXPECT_FALSE(syshal_gps_on);
}

TEST_F(Sm_MainTest, OperationalStateGPSScheduledFirstFixHold)
{
    uint32_t acquisition_interval = 165;
    uint32_t maximum_acquisition = 15;
    uint32_t no_fix_timeout = 0;
    uint32_t very_first_fix_hold_time = 60;
    syshal_gps_event_t event;

    sm_set_current_state(&state_handle, SM_MAIN_BOOT);

    set_all_configuration_tags_RAM();
    CreateEmptyLogfile();

    // Set GPS trigger mode
    sys_config.sys_config_gps_trigger_mode.hdr.set = true;
    sys_config.sys_config_gps_trigger_mode.contents.mode = SYS_CONFIG_GPS_TRIGGER_MODE_SCHEDULED;

    // Set GPS acquisition interval
    sys_config.sys_config_gps_scheduled_acquisition_interval.hdr.set = true;
    sys_config.sys_config_gps_scheduled_acquisition_interval.contents.seconds = acquisition_interval;

    // Set GPS acquisition time
    sys_config.sys_config_gps_maximum_acquisition_time.hdr.set = true;
    sys_config.sys_config_gps_maximum_acquisition_time.contents.seconds = maximum_acquisition;

    // Set GPS no fix timeout
    sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout.hdr.set = true;
    sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout.contents.seconds = no_fix_timeout;

    // Set GPS first fix hold
    sys_config.sys_config_gps_very_first_fix_hold_time.hdr.set = true;
    sys_config.sys_config_gps_very_first_fix_hold_time.contents.seconds = very_first_fix_hold_time;

    // Enable GPS logging
    sys_config.sys_config_gps_log_position_enable.hdr.set = true;
    sys_config.sys_config_gps_log_position_enable.contents.enable = true;

    // Enable Global logging
    sys_config.sys_config_logging_enable.hdr.set = true;
    sys_config.sys_config_logging_enable.contents.enable = true;

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_OPERATIONAL, sm_get_current_state(&state_handle));

    sm_tick(&state_handle);
    EXPECT_TRUE(syshal_gps_on);

    // Generate a GPS not fixed event
    event.event_id = SYSHAL_GPS_EVENT_STATUS;
    event.event_data.status.gpsFix = 0;

    // Test to see if GPS remains on with no fix
    for (unsigned int i = 0; i < acquisition_interval + maximum_acquisition + very_first_fix_hold_time + 1; ++i)
    {
        IncrementSeconds(1);
        syshal_gps_callback(event);
        sm_tick(&state_handle);
        ASSERT_TRUE(syshal_gps_on);
    }

    // Generate a GPS fixed event
    event.event_id = SYSHAL_GPS_EVENT_STATUS;
    event.event_data.status.gpsFix = 2;

    for (unsigned int i = 0; i < very_first_fix_hold_time + 1; ++i)
    {
        IncrementSeconds(1);
        syshal_gps_callback(event);
        sm_tick(&state_handle);
    }

    EXPECT_FALSE(syshal_gps_on);

    for (unsigned int i = 0; i < acquisition_interval+1; ++i)
    {
        IncrementSeconds(1);
        syshal_gps_callback(event);
        sm_tick(&state_handle);
    }

    EXPECT_TRUE(syshal_gps_on);

    for (unsigned int i = 0; i < maximum_acquisition; ++i)
    {
        IncrementSeconds(1);
        syshal_gps_callback(event);
        sm_tick(&state_handle);
    }

    EXPECT_FALSE(syshal_gps_on);

    for (unsigned int i = 0; i < acquisition_interval - maximum_acquisition; ++i)
    {
        IncrementSeconds(1);
        syshal_gps_callback(event);
        sm_tick(&state_handle);
    }

    EXPECT_TRUE(syshal_gps_on);

    for (unsigned int i = 0; i < maximum_acquisition; ++i)
    {
        IncrementSeconds(1);
        syshal_gps_callback(event);
        sm_tick(&state_handle);
    }

    EXPECT_FALSE(syshal_gps_on);
}

TEST_F(Sm_MainTest, OperationalStateGPSScheduledFirstFixHoldLostFix)
{
    uint32_t acquisition_interval = 165;
    uint32_t maximum_acquisition = 15;
    uint32_t no_fix_timeout = 0;
    uint32_t very_first_fix_hold_time = 60;
    syshal_gps_event_t event;

    sm_set_current_state(&state_handle, SM_MAIN_BOOT);

    set_all_configuration_tags_RAM();
    CreateEmptyLogfile();

    // Set GPS trigger mode
    sys_config.sys_config_gps_trigger_mode.hdr.set = true;
    sys_config.sys_config_gps_trigger_mode.contents.mode = SYS_CONFIG_GPS_TRIGGER_MODE_SCHEDULED;

    // Set GPS acquisition interval
    sys_config.sys_config_gps_scheduled_acquisition_interval.hdr.set = true;
    sys_config.sys_config_gps_scheduled_acquisition_interval.contents.seconds = acquisition_interval;

    // Set GPS acquisition time
    sys_config.sys_config_gps_maximum_acquisition_time.hdr.set = true;
    sys_config.sys_config_gps_maximum_acquisition_time.contents.seconds = maximum_acquisition;

    // Set GPS no fix timeout
    sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout.hdr.set = true;
    sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout.contents.seconds = no_fix_timeout;

    // Set GPS first fix hold
    sys_config.sys_config_gps_very_first_fix_hold_time.hdr.set = true;
    sys_config.sys_config_gps_very_first_fix_hold_time.contents.seconds = very_first_fix_hold_time;

    // Enable GPS logging
    sys_config.sys_config_gps_log_position_enable.hdr.set = true;
    sys_config.sys_config_gps_log_position_enable.contents.enable = true;

    // Enable Global logging
    sys_config.sys_config_logging_enable.hdr.set = true;
    sys_config.sys_config_logging_enable.contents.enable = true;

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_OPERATIONAL, sm_get_current_state(&state_handle));

    sm_tick(&state_handle);
    EXPECT_TRUE(syshal_gps_on);

    // Test to see if GPS remains on with no fix
    for (unsigned int i = 0; i < acquisition_interval + maximum_acquisition + very_first_fix_hold_time + 1; ++i)
    {
        IncrementSeconds(1);
        sm_tick(&state_handle);
        ASSERT_TRUE(syshal_gps_on);
    }

    // Generate a GPS fixed event
    event.event_id = SYSHAL_GPS_EVENT_STATUS;
    event.event_data.status.gpsFix = 2;
    syshal_gps_callback(event);
    sm_tick(&state_handle);

    for (unsigned int i = 0; i < (very_first_fix_hold_time / 2) + 1; ++i)
    {
        IncrementSeconds(1);
        syshal_gps_callback(event);
        sm_tick(&state_handle);
    }

    EXPECT_TRUE(syshal_gps_on);

    // Generate a GPS fix lost event before the no fix hold time has elapsed
    event.event_id = SYSHAL_GPS_EVENT_STATUS;
    event.event_data.status.gpsFix = 0;
    syshal_gps_callback(event);
    sm_tick(&state_handle);

    for (unsigned int i = 0; i < (very_first_fix_hold_time); ++i)
    {
        IncrementSeconds(1);
        syshal_gps_callback(event);
        sm_tick(&state_handle);
    }

    EXPECT_TRUE(syshal_gps_on); // Check the GPS is still on

    // Generate a GPS fixed event
    event.event_id = SYSHAL_GPS_EVENT_STATUS;
    event.event_data.status.gpsFix = 2;
    syshal_gps_callback(event);
    sm_tick(&state_handle);

    for (unsigned int i = 0; i < very_first_fix_hold_time + 1; ++i)
    {
        IncrementSeconds(1);
        syshal_gps_callback(event);
        sm_tick(&state_handle);
    }

    EXPECT_FALSE(syshal_gps_on); // GPS should now be off
}

TEST_F(Sm_MainTest, OperationalStateGPSScheduledAlwaysOn)
{
    uint32_t acquisition_interval = 0;
    uint32_t maximum_acquisition = 0;
    uint32_t no_fix_timeout = 0;

    sm_set_current_state(&state_handle, SM_MAIN_BOOT);

    set_all_configuration_tags_RAM();
    CreateEmptyLogfile();

    // Set GPS trigger mode
    sys_config.sys_config_gps_trigger_mode.hdr.set = true;
    sys_config.sys_config_gps_trigger_mode.contents.mode = SYS_CONFIG_GPS_TRIGGER_MODE_SCHEDULED;

    // Set GPS acquisition interval
    sys_config.sys_config_gps_scheduled_acquisition_interval.hdr.set = true;
    sys_config.sys_config_gps_scheduled_acquisition_interval.contents.seconds = acquisition_interval;

    // Set GPS acquisition time
    sys_config.sys_config_gps_maximum_acquisition_time.hdr.set = true;
    sys_config.sys_config_gps_maximum_acquisition_time.contents.seconds = maximum_acquisition;

    // Set GPS no fix timeout
    sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout.hdr.set = true;
    sys_config.sys_config_gps_scheduled_acquisition_no_fix_timeout.contents.seconds = no_fix_timeout;

    // Disable GPS first fix hold
    sys_config.sys_config_gps_very_first_fix_hold_time.hdr.set = false;

    // Enable GPS logging
    sys_config.sys_config_gps_log_position_enable.hdr.set = true;
    sys_config.sys_config_gps_log_position_enable.contents.enable = true;

    // Enable Global logging
    sys_config.sys_config_logging_enable.hdr.set = true;
    sys_config.sys_config_logging_enable.contents.enable = true;

    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_OPERATIONAL, sm_get_current_state(&state_handle));

    for (unsigned int i = 0; i < 100; ++i)
    {
        IncrementSeconds(1);
        sm_tick(&state_handle);
        ASSERT_TRUE(syshal_gps_on);
    }
}

//////////////////////////////////////////////////////////////////
//////////////////////// Message Handling ////////////////////////
//////////////////////////////////////////////////////////////////

TEST_F(Sm_MainTest, StatusRequest)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate status request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_STATUS_REQ);
    send_message(&req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_STATUS_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_status_resp.error_code);
    EXPECT_EQ(STM32_FIRMWARE_VERSION, resp.p.cmd_status_resp.stm_firmware_version);
    EXPECT_EQ(syshal_ble_version, resp.p.cmd_status_resp.ble_firmware_version);
    EXPECT_EQ(SYS_CONFIG_FORMAT_VERSION, resp.p.cmd_status_resp.configuration_format_version);
}

TEST_F(Sm_MainTest, ResetRequestSTM32)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate reset request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_RESET_REQ);
    req.p.cmd_reset_req.reset_type = RESET_REQ_STM32;
    send_message(&req, CMD_SIZE(cmd_reset_req_t));

    syshal_pmu_reset_Expect();

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, ResetRequestFlashErase)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate reset request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_RESET_REQ);
    req.p.cmd_reset_req.reset_type = RESET_REQ_FLASH_ERASE_ALL;
    send_message(&req, CMD_SIZE(cmd_reset_req_t));

    CreateEmptyLogfile(); // Create file in the file system

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    fs_t file_system;
    fs_handle_t file_system_handle;
    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));

    // Check the FLASH has been erased
    for (unsigned int id = 0; id <= 255; ++id)
        EXPECT_EQ(FS_ERROR_FILE_NOT_FOUND, fs_open(file_system, &file_system_handle, id, FS_MODE_READONLY, NULL));
}

TEST_F(Sm_MainTest, ResetRequestInvalid)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate reset request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_RESET_REQ);
    req.p.cmd_reset_req.reset_type = 0xAB; // Invalid/unknown type
    send_message(&req, CMD_SIZE(cmd_reset_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_INVALID_PARAMETER, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgWriteOne)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate cfg write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_WRITE_REQ);
    req.p.cmd_cfg_write_req.length = SYS_CONFIG_TAG_ID_SIZE + SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_group_sensor_readings_enable_t);
    send_message(&req, CMD_SIZE(cmd_cfg_write_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Generate cfg tag data packet
    uint8_t tag_data_packet[3];
    tag_data_packet[0] = uint16_t(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE) & 0x00FF;
    tag_data_packet[1] = (uint16_t(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE) & 0xFF00) >> 8;
    tag_data_packet[2] = true; // Enable

    send_message(tag_data_packet, sizeof(tag_data_packet));

    sm_tick(&state_handle); // Process the message

    // Check the response
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_CFG_WRITE_CNF, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_cfg_write_cnf.error_code);

    // Check the tag was correctly set
    EXPECT_TRUE(sys_config.sys_config_logging_group_sensor_readings_enable.contents.enable);
}

TEST_F(Sm_MainTest, CfgWriteOneInvalidTag)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate cfg write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_WRITE_REQ);
    req.p.cmd_cfg_write_req.length = SYS_CONFIG_TAG_ID_SIZE + SYS_CONFIG_TAG_DATA_SIZE(sys_config_logging_group_sensor_readings_enable_t);
    send_message(&req, CMD_SIZE(cmd_cfg_write_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Generate cfg tag data packet
    uint8_t tag_data_packet[3];
    uint16_t invalid_tag_ID = 0xABAB;
    tag_data_packet[0] = invalid_tag_ID & 0x00FF;
    tag_data_packet[1] = (invalid_tag_ID & 0xFF00) >> 8;
    tag_data_packet[2] = true; // Enable

    send_message(tag_data_packet, sizeof(tag_data_packet));

    sm_tick(&state_handle); // Process the message
    sm_tick(&state_handle); // Return the error

    // Check the response
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_CFG_WRITE_CNF, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_INVALID_CONFIG_TAG, resp.p.cmd_cfg_write_cnf.error_code);
}

TEST_F(Sm_MainTest, CfgReadOne)
{
    // Set a tag up for reading later
    sys_config.sys_config_logging_group_sensor_readings_enable.contents.enable = true;
    sys_config.sys_config_logging_group_sensor_readings_enable.hdr.set = true;

    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate cfg read request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_READ_REQ);
    req.p.cmd_cfg_read_req.configuration_tag = SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE;
    send_message(&req, CMD_SIZE(cmd_cfg_read_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_CFG_READ_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_cfg_read_resp.error_code);

    sm_tick(&state_handle); // Send the data packet

    receive_message(&resp);
    uint8_t * message = (uint8_t *) &resp;

    uint16_t tag = 0;
    tag |= (uint16_t) message[0] & 0x00FF;
    tag |= (uint16_t) (message[1] << 8) & 0xFF00;
    EXPECT_EQ(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE, tag);

    // Check the tag was correctly read
    EXPECT_TRUE(message[2]);
}

TEST_F(Sm_MainTest, CfgReadInvalidTag)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate cfg read request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_READ_REQ);
    req.p.cmd_cfg_read_req.configuration_tag = 0xABAB;
    send_message(&req, CMD_SIZE(cmd_cfg_read_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_CFG_READ_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_INVALID_CONFIG_TAG, resp.p.cmd_cfg_read_resp.error_code);
}

TEST_F(Sm_MainTest, CfgSaveSuccess)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_SAVE_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Check the configuration file in FLASH matches the configuration in RAM
    fs_t file_system;
    fs_handle_t file_system_handle;
    uint32_t bytes_read;
    uint8_t flash_config_data[sizeof(sys_config)];

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_CONF, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(file_system_handle, &flash_config_data[0], sizeof(sys_config), &bytes_read));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));
    EXPECT_EQ(sizeof(sys_config), bytes_read);

    // Check RAM and FLASH contents match
    bool RAM_FLASH_mismatch = false;
    uint8_t * sys_config_itr = (uint8_t *) &sys_config;
    for (unsigned int i = 0; i < bytes_read; ++i)
    {
        if (flash_config_data[i] != sys_config_itr[i])
        {
            RAM_FLASH_mismatch = true;
            break;
        }
    }

    EXPECT_FALSE(RAM_FLASH_mismatch);
}

TEST_F(Sm_MainTest, CfgRestoreNoFile)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_RESTORE_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgRestoreSuccess)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate the configuration file in FLASH
    fs_t file_system;
    fs_handle_t file_system_handle;
    uint32_t bytes_written;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_CONF, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(file_system_handle, &sys_config, sizeof(sys_config), &bytes_written));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_RESTORE_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgProtectSuccess)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate the configuration file in FLASH
    fs_t file_system;
    fs_handle_t file_system_handle;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_CONF, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_PROTECT_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgProtectNoFile)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_PROTECT_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgUnprotectSuccess)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate the configuration file in FLASH
    fs_t file_system;
    fs_handle_t file_system_handle;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_CONF, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_UNPROTECT_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgUnprotectNoFile)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate cfg save request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_UNPROTECT_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, CfgEraseAll)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    set_all_configuration_tags_RAM(); // Set all the configuration tags

    // Generate cfg erase request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_ERASE_REQ);
    req.p.cmd_cfg_erase_req.configuration_tag = CFG_ERASE_REQ_ERASE_ALL;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_cfg_erase_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Check all the configuration tags have been unset
    bool all_tags_unset = true;
    uint16_t tag, last_index = 0;
    while (!sys_config_iterate(&tag, &last_index))
    {
        void * src;
        int ret = sys_config_get(tag, &src);

        if (SYS_CONFIG_ERROR_TAG_NOT_SET != ret &&
            SYS_CONFIG_TAG_RTC_CURRENT_DATE_AND_TIME != tag &&
            SYS_CONFIG_TAG_LOGGING_FILE_SIZE != tag &&
            SYS_CONFIG_TAG_LOGGING_FILE_TYPE != tag &&
            SYS_CONFIG_TAG_LOGGING_START_END_SYNC_ENABLE != tag)
        {
            all_tags_unset = false;
            break;
        }
    }

    EXPECT_TRUE(all_tags_unset);
}

TEST_F(Sm_MainTest, CfgEraseOne)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    set_all_configuration_tags_RAM(); // Set all the configuration tags

    // Generate log erase request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_CFG_ERASE_REQ);
    req.p.cmd_cfg_erase_req.configuration_tag = SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_cfg_erase_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Check the configuration tag has been unset
    void * src;
    int ret = sys_config_get(SYS_CONFIG_TAG_LOGGING_GROUP_SENSOR_READINGS_ENABLE, &src);

    bool tag_unset = false;
    if (SYS_CONFIG_ERROR_TAG_NOT_SET == ret)
        tag_unset = true;

    EXPECT_TRUE(tag_unset);
}

TEST_F(Sm_MainTest, LogEraseSuccess)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate the log file in FLASH
    fs_t file_system;
    fs_handle_t file_system_handle;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate log erase request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_ERASE_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, LogEraseNoFile)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate log erase request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_ERASE_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, LogCreateFill)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate log create request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_CREATE_REQ);
    req.p.cmd_log_create_req.mode = CMD_LOG_CREATE_REQ_MODE_FILL;
    req.p.cmd_log_create_req.sync_enable = false;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_log_create_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Check the log file has been created and is of the right mode
    fs_t file_system;
    fs_stat_t file_stats;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_stat(file_system, FS_FILE_ID_LOG, &file_stats));
    EXPECT_FALSE(file_stats.is_circular);
}

TEST_F(Sm_MainTest, LogCreateCircular)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate log create request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_CREATE_REQ);
    req.p.cmd_log_create_req.mode = CMD_LOG_CREATE_REQ_MODE_CIRCULAR;
    req.p.cmd_log_create_req.sync_enable = false;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_log_create_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Check the log file has been created and is of the right mode
    fs_t file_system;
    fs_stat_t file_stats;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_stat(file_system, FS_FILE_ID_LOG, &file_stats));
    EXPECT_TRUE(file_stats.is_circular);
}

TEST_F(Sm_MainTest, LogCreateAlreadyExists)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Create the log file
    fs_t file_system;
    fs_handle_t file_system_handle;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate log create request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_CREATE_REQ);
    req.p.cmd_log_create_req.mode = CMD_LOG_CREATE_REQ_MODE_CIRCULAR;
    req.p.cmd_log_create_req.sync_enable = false;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_log_create_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_ALREADY_EXISTS, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, LogReadNoFile)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate log read request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_READ_REQ);
    req.p.cmd_log_read_req.start_offset = 0;
    req.p.cmd_log_read_req.length = 256;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_log_read_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_LOG_READ_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_log_read_resp.error_code);
}

TEST_F(Sm_MainTest, LogReadSuccess)
{
    const uint32_t log_size = 256;
    uint8_t testData[log_size];
    uint32_t bytes_written = 0;

    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate test data
    for (auto i = 0; i < log_size; ++i)
        testData[i] = rand();

    // Create the log file
    fs_t file_system;
    fs_handle_t file_system_handle;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(file_system_handle, testData, log_size, &bytes_written)); // Load test data into the log file
    EXPECT_EQ(log_size, bytes_written);
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate log read request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_READ_REQ);
    req.p.cmd_log_read_req.start_offset = 0;
    req.p.cmd_log_read_req.length = log_size;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_log_read_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_LOG_READ_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_log_read_resp.error_code);
    EXPECT_EQ(log_size, resp.p.cmd_log_read_resp.length);

    sm_tick(&state_handle); // Process the message

    uint8_t log_data[log_size];
    receive_message(log_data);

    bool log_read_mismatch = false;
    for (auto i = 0; i < log_size; ++i)
    {
        if (log_data[i] != testData[i])
        {
            log_read_mismatch = true;
            break;
        }
    }

    EXPECT_FALSE(log_read_mismatch);
}

TEST_F(Sm_MainTest, LogReadAll)
{
    const uint32_t log_size = 256;
    uint8_t testData[log_size];
    uint32_t bytes_written = 0;

    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate test data
    for (auto i = 0; i < log_size; ++i)
        testData[i] = rand();

    // Create the log file
    fs_t file_system;
    fs_handle_t file_system_handle;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(file_system_handle, testData, log_size, &bytes_written)); // Load test data into the log file
    EXPECT_EQ(log_size, bytes_written);
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate log read request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_READ_REQ);
    req.p.cmd_log_read_req.start_offset = 0; // Both being zero means read all
    req.p.cmd_log_read_req.length = 0;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_log_read_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_LOG_READ_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_log_read_resp.error_code);
    EXPECT_EQ(log_size, resp.p.cmd_log_read_resp.length);

    sm_tick(&state_handle); // Process the message

    uint8_t log_data[log_size];
    receive_message(log_data);

    bool log_read_mismatch = false;
    for (auto i = 0; i < log_size; ++i)
    {
        if (log_data[i] != testData[i])
        {
            log_read_mismatch = true;
            break;
        }
    }

    EXPECT_FALSE(log_read_mismatch);
}

TEST_F(Sm_MainTest, LogReadOffset)
{
    const uint32_t log_size = 256;
    const uint32_t log_read_offset = 128;
    const uint32_t log_read_size = log_size - log_read_offset;
    uint8_t testData[log_size];
    uint32_t bytes_written = 0;

    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate test data
    for (auto i = 0; i < log_size; ++i)
        testData[i] = rand();

    // Create the log file
    fs_t file_system;
    fs_handle_t file_system_handle;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_format(file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, FS_FILE_ID_LOG, FS_MODE_CREATE, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_write(file_system_handle, testData, log_size, &bytes_written)); // Load test data into the log file
    EXPECT_EQ(log_size, bytes_written);
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Generate log read request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_LOG_READ_REQ);
    req.p.cmd_log_read_req.start_offset = log_read_offset; // Both being zero means read all
    req.p.cmd_log_read_req.length = log_read_size;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_log_read_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_LOG_READ_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_log_read_resp.error_code);
    EXPECT_EQ(log_read_size, resp.p.cmd_log_read_resp.length);

    sm_tick(&state_handle); // Process the message

    uint8_t log_data[log_size];
    receive_message(log_data);

    bool log_read_mismatch = false;
    for (auto i = 0; i < log_read_size; ++i)
    {
        if (log_data[i] != testData[i + log_read_offset])
        {
            log_read_mismatch = true;
            break;
        }
    }

    EXPECT_FALSE(log_read_mismatch);
}

TEST_F(Sm_MainTest, BatteryStatus)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate battery status request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_BATTERY_STATUS_REQ);
    send_message((uint8_t *) &req, CMD_SIZE_HDR);

    uint8_t chargePercentage = rand() % 100;

    SetBatteryPercentage(chargePercentage);

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_BATTERY_STATUS_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_battery_status_resp.error_code);
    EXPECT_EQ(true, resp.p.cmd_battery_status_resp.charging_indicator);
    EXPECT_EQ(chargePercentage, resp.p.cmd_battery_status_resp.charge_level);
}

TEST_F(Sm_MainTest, GpsConfig)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate gps config request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_GPS_CONFIG_REQ);
    req.p.cmd_gps_config_req.enable = true;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_gps_config_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_battery_status_resp.error_code);
}

TEST_F(Sm_MainTest, GpsWriteBridgingOff)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate GPS write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_GPS_WRITE_REQ);
    req.p.cmd_gps_write_req.length = 100;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_gps_write_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_BRIDGING_DISABLED, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, GpsWriteSuccess)
{
    uint32_t gps_write_length = 256;

    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate gps config message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_GPS_CONFIG_REQ);
    req.p.cmd_gps_config_req.enable = true;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_gps_config_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Generate GPS write request message
    CMD_SET_HDR((&req), CMD_GPS_WRITE_REQ);
    req.p.cmd_gps_write_req.length = gps_write_length;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_gps_write_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Generate GPS write payload
    uint8_t gps_data_packet[gps_write_length];
    for (unsigned int i = 0; i < sizeof(gps_data_packet); ++i)
        gps_data_packet[i] = i;

    send_message(gps_data_packet, sizeof(gps_data_packet));

    sm_tick(&state_handle); // Process the message

    // Check message wrote is as expected
    bool gps_write_mismatch = false;
    for (unsigned int i = 0; i < gps_write_length; ++i)
    {
        if (gps_write_buffer[i] != gps_data_packet[i])
        {
            gps_write_mismatch = true;
            break;
        }
    }

    EXPECT_FALSE(gps_write_mismatch);
}

TEST_F(Sm_MainTest, GpsReadBridgingOff)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate GPS read request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_GPS_READ_REQ);
    req.p.cmd_gps_read_req.length = 100;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_gps_read_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GPS_READ_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_BRIDGING_DISABLED, resp.p.cmd_gps_read_resp.error_code);
}

TEST_F(Sm_MainTest, GpsReadSuccess)
{
    uint32_t gps_read_length = 256;

    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate gps config message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_GPS_CONFIG_REQ);
    req.p.cmd_gps_config_req.enable = true;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_gps_config_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Generate GPS read request message
    CMD_SET_HDR((&req), CMD_GPS_READ_REQ);
    req.p.cmd_gps_read_req.length = gps_read_length;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_gps_read_req_t));

    syshal_gps_available_raw_ExpectAndReturn(gps_read_length);

    sm_tick(&state_handle); // Process the message

    // Check the response
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GPS_READ_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Load the GPS SPI buffer with test data
    for (unsigned int i = 0; i < gps_read_length; ++i)
        gps_receive_buffer.push(i);

    sm_tick(&state_handle);

    uint8_t received_data[gps_read_length];
    receive_message(received_data);

    // Look for mismatch between data received on SPI and data transmitted on config_if
    bool SPI_and_config_if_mismatch = false;
    for (unsigned int i = 0; i < gps_read_length; ++i)
    {
        if (received_data[i] != i)
        {
            SPI_and_config_if_mismatch = true;
            break;
        }
    }

    EXPECT_FALSE(SPI_and_config_if_mismatch);
}

TEST_F(Sm_MainTest, BleConfig)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate ble config request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_BLE_CONFIG_REQ);
    req.p.cmd_ble_config_req.enable = true;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_ble_config_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_battery_status_resp.error_code);
}

TEST_F(Sm_MainTest, BleWriteBridgingOff)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate BLE write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_BLE_WRITE_REQ);
    req.p.cmd_ble_write_req.length = 100;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_ble_write_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_BRIDGING_DISABLED, resp.p.cmd_generic_resp.error_code);
}

//TEST_F(Sm_MainTest, BleWriteSuccess)
//{
//    uint32_t ble_write_length = 256;
//    uint8_t ble_address = rand();
//
//    BootTagsNotSet();
//
//    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);
//
//    config_if_init(CONFIG_IF_BACKEND_USB);
//    USBConnectionEvent();
//
//    SetVUSB(true);
//    sm_tick(&state_handle);
//
//    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
//    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
//
//    // Generate ble config message
//    cmd_t req;
//    CMD_SET_HDR((&req), CMD_BLE_CONFIG_REQ);
//    req.p.cmd_ble_config_req.enable = true;
//    send_message((uint8_t *) &req, CMD_SIZE(cmd_ble_config_req_t));
//
//    sm_tick(&state_handle); // Process the message
//
//    // Check the response
//    cmd_t resp;
//    receive_message(&resp);
//    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
//    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
//    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
//
//    // Generate BLE write request message
//    CMD_SET_HDR((&req), CMD_BLE_WRITE_REQ);
//    req.p.cmd_ble_write_req.address = ble_address;
//    req.p.cmd_ble_write_req.length = ble_write_length;
//    send_message(&req, CMD_SIZE(cmd_ble_write_req_t));
//
//    sm_tick(&state_handle); // Process the message
//
//    // Check the response
//    receive_message(&resp);
//    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
//    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
//    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
//
//    // Generate BLE write payload
//    uint8_t ble_data_packet[ble_write_length];
//    for (unsigned int i = 0; i < sizeof(ble_data_packet); ++i)
//        ble_data_packet[i] = i;
//
//    send_message(ble_data_packet, sizeof(ble_data_packet));
//
//    sm_tick(&state_handle); // Process the message
//
//    EXPECT_EQ(ble_address, syshal_ble_write_register_address.back());
//    EXPECT_EQ(ble_write_length, syshal_ble_write_register_length.back());
//    ASSERT_EQ(ble_write_length, syshal_ble_write_register_data.size());
//
//    // Check message wrote is as expected
//    bool ble_write_mismatch = false;
//    for (unsigned int i = 0; i < sizeof(ble_data_packet); ++i)
//    {
//        if (syshal_ble_write_register_data[i] != ble_data_packet[i])
//        {
//            ble_write_mismatch = true;
//            break;
//        }
//    }
//
//    EXPECT_FALSE(ble_write_mismatch);
//}

TEST_F(Sm_MainTest, BleReadBridgingOff)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate BLE read request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_BLE_READ_REQ);
    req.p.cmd_ble_read_req.length = 100;
    req.p.cmd_ble_read_req.address = 0xAA;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_ble_read_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_BRIDGING_DISABLED, resp.p.cmd_generic_resp.error_code);
}

//TEST_F(Sm_MainTest, BleReadSuccess)
//{
//    uint32_t ble_read_length = 256;
//    uint8_t ble_address = rand();
//
//    BootTagsNotSet();
//
//    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);
//
//    config_if_init(CONFIG_IF_BACKEND_USB);
//    USBConnectionEvent();
//
//    SetVUSB(true);
//    sm_tick(&state_handle);
//
//    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
//    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());
//
//    // Generate ble config message
//    cmd_t req;
//    CMD_SET_HDR((&req), CMD_BLE_CONFIG_REQ);
//    req.p.cmd_ble_config_req.enable = true;
//    send_message((uint8_t *) &req, CMD_SIZE(cmd_ble_config_req_t));
//
//    sm_tick(&state_handle); // Process the message
//
//    // Check the response
//    cmd_t resp;
//    receive_message(&resp);
//    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
//    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
//    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
//
//    // Generate BLE write request message
//    CMD_SET_HDR((&req), CMD_BLE_READ_REQ);
//    req.p.cmd_ble_write_req.address = ble_address;
//    req.p.cmd_ble_write_req.length = ble_read_length;
//    send_message(&req, CMD_SIZE(cmd_ble_write_req_t));
//
//    sm_tick(&state_handle); // Process the message
//
//    // Check the response
//    receive_message(&resp);
//    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
//    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
//    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
//
//    // Generate BLE write payload
//    uint8_t ble_data_packet[ble_read_length];
//    for (unsigned int i = 0; i < sizeof(ble_data_packet); ++i)
//        ble_data_packet[i] = i;
//
//    send_message(ble_data_packet, sizeof(ble_data_packet));
//
//    sm_tick(&state_handle); // Process the message
//
//    EXPECT_EQ(ble_address, syshal_ble_write_register_address.back());
//    EXPECT_EQ(ble_write_length, syshal_ble_write_register_length.back());
//    ASSERT_EQ(ble_write_length, syshal_ble_write_register_data.size());
//
//    // Check message wrote is as expected
//    bool ble_write_mismatch = false;
//    for (unsigned int i = 0; i < sizeof(ble_data_packet); ++i)
//    {
//        if (syshal_ble_write_register_data[i] != ble_data_packet[i])
//        {
//            ble_write_mismatch = true;
//            break;
//        }
//    }
//
//    EXPECT_FALSE(ble_write_mismatch);
//}

TEST_F(Sm_MainTest, FwWriteWrongImageType)
{
    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // FS_FILE_ID_STM32_IMAGE    (1)  // STM32 application image
    // FS_FILE_ID_BLE_APP_IMAGE  (2)  // BLE application image
    // FS_FILE_ID_BLE_SOFT_IMAGE (3)  // BLE soft-device image

    // Generate FW write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_FW_SEND_IMAGE_REQ);
    req.p.cmd_fw_send_image_req.image_type = 0xAA; // Invalid image type
    req.p.cmd_fw_send_image_req.length = 100;
    req.p.cmd_fw_send_image_req.CRC32 = 0;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_fw_send_image_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_INVALID_FW_IMAGE_TYPE, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, FwWriteInvalidCRC32)
{
    const uint32_t fw_size = 100;
    uint8_t fw_image[fw_size];

    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate the FW image to be sent
    for (uint32_t i = 0; i < fw_size; ++i)
        fw_image[i] = rand();

    // FS_FILE_ID_STM32_IMAGE    (1)  // STM32 application image
    // FS_FILE_ID_BLE_APP_IMAGE  (2)  // BLE application image
    // FS_FILE_ID_BLE_SOFT_IMAGE (3)  // BLE soft-device image

    // Generate FW write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_FW_SEND_IMAGE_REQ);
    req.p.cmd_fw_send_image_req.image_type = 1; // STM32 application image
    req.p.cmd_fw_send_image_req.length = fw_size;
    req.p.cmd_fw_send_image_req.CRC32 = 0;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_fw_send_image_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Send the firmware image
    send_message(fw_image, sizeof(fw_image));

    sm_tick(&state_handle); // Process the image

    // Check the response
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_FW_SEND_IMAGE_COMPLETE_CNF, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_IMAGE_CRC_MISMATCH, resp.p.cmd_fw_send_image_complete_cnf.error_code);
}

TEST_F(Sm_MainTest, FwWriteSingle)
{
    const uint32_t fw_size = 100;
    uint8_t fw_image[fw_size];
    uint32_t fw_crc32 = 0;

    // FS_FILE_ID_STM32_IMAGE    (1)  // STM32 application image
    // FS_FILE_ID_BLE_APP_IMAGE  (2)  // BLE application image
    // FS_FILE_ID_BLE_SOFT_IMAGE (3)  // BLE soft-device image

    uint8_t fw_type = 1;

    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate the FW image to be sent
    for (uint32_t i = 0; i < fw_size; ++i)
        fw_image[i] = rand();

    // Calculate the crc
    fw_crc32 = crc32(fw_crc32, fw_image, fw_size);

    // Generate FW write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_FW_SEND_IMAGE_REQ);
    req.p.cmd_fw_send_image_req.image_type = fw_type; // STM32 application image
    req.p.cmd_fw_send_image_req.length = fw_size;
    req.p.cmd_fw_send_image_req.CRC32 = fw_crc32;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_fw_send_image_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Send the firmware image
    send_message(fw_image, fw_size);

    sm_tick(&state_handle); // Process the image

    // Check the response
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_FW_SEND_IMAGE_COMPLETE_CNF, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_fw_send_image_complete_cnf.error_code);

    // Check the image that has been written to the FLASH
    fs_t file_system;
    fs_handle_t file_system_handle;
    uint8_t flash_fw_data[fw_size];
    uint32_t bytes_read;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, fw_type, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(file_system_handle, &flash_fw_data[0], fw_size, &bytes_read));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Look for differences between the two
    bool flash_and_image_match = true;
    for (uint32_t i = 0; i < fw_size; ++i)
    {
        if (flash_fw_data[i] != fw_image[i])
        {
            flash_and_image_match = false;
            break;
        }
    }

    EXPECT_TRUE(flash_and_image_match);
}

TEST_F(Sm_MainTest, FwWriteMultiple)
{
    const uint32_t packet_number = 20;
    const uint32_t packet_size = 512;
    const uint32_t fw_size = packet_number * packet_size;
    uint8_t fw_image[fw_size];
    uint32_t fw_crc32 = 0;

    // FS_FILE_ID_STM32_IMAGE    (1)  // STM32 application image
    // FS_FILE_ID_BLE_APP_IMAGE  (2)  // BLE application image
    // FS_FILE_ID_BLE_SOFT_IMAGE (3)  // BLE soft-device image

    uint8_t fw_type = 1;

    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate the FW image to be sent
    for (uint32_t i = 0; i < fw_size; ++i)
        fw_image[i] = rand();

    // Calculate the crc
    fw_crc32 = crc32(fw_crc32, fw_image, fw_size);

    // Generate FW write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_FW_SEND_IMAGE_REQ);
    req.p.cmd_fw_send_image_req.image_type = fw_type; // STM32 application image
    req.p.cmd_fw_send_image_req.length = fw_size;
    req.p.cmd_fw_send_image_req.CRC32 = fw_crc32;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_fw_send_image_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Send the full firmware image in discrete packets
    for (uint32_t i = 0; i < packet_number; ++i)
    {
        send_message(fw_image + (i * packet_size), packet_size);
        sm_tick(&state_handle); // Process the image
    }

    // Check the response
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_FW_SEND_IMAGE_COMPLETE_CNF, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_fw_send_image_complete_cnf.error_code);

    // Check the image that has been written to the FLASH
    fs_t file_system;
    fs_handle_t file_system_handle;
    uint8_t flash_fw_data[fw_size];
    uint32_t bytes_read;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, fw_type, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(file_system_handle, &flash_fw_data[0], fw_size, &bytes_read));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Look for differences between the two
    bool flash_and_image_match = true;
    for (uint32_t i = 0; i < fw_size; ++i)
    {
        if (flash_fw_data[i] != fw_image[i])
        {
            flash_and_image_match = false;
            break;
        }
    }

    EXPECT_TRUE(flash_and_image_match);
}

TEST_F(Sm_MainTest, FwApplyImageCorrect)
{
    const uint32_t packet_number = 20;
    const uint32_t packet_size = 512;
    const uint32_t fw_size = packet_number * packet_size;
    uint8_t fw_image[fw_size];
    uint32_t fw_crc32 = 0;

    // FS_FILE_ID_STM32_IMAGE    (1)  // STM32 application image
    // FS_FILE_ID_BLE_APP_IMAGE  (2)  // BLE application image
    // FS_FILE_ID_BLE_SOFT_IMAGE (3)  // BLE soft-device image

    uint8_t fw_type = 1; // STM32 application image

    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate the FW image to be sent
    for (uint32_t i = 0; i < fw_size; ++i)
        fw_image[i] = rand();

    // Calculate the crc
    fw_crc32 = crc32(fw_crc32, fw_image, fw_size);

    // Generate FW write request message
    cmd_t req;
    CMD_SET_HDR((&req), CMD_FW_SEND_IMAGE_REQ);
    req.p.cmd_fw_send_image_req.image_type = fw_type;
    req.p.cmd_fw_send_image_req.length = fw_size;
    req.p.cmd_fw_send_image_req.CRC32 = fw_crc32;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_fw_send_image_req_t));

    sm_tick(&state_handle); // Process the message

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);

    // Send the full firmware image in discrete packets
    for (uint32_t i = 0; i < packet_number; ++i)
    {
        send_message(fw_image + (i * packet_size), packet_size);
        sm_tick(&state_handle); // Process the image
    }

    // Check the response
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_FW_SEND_IMAGE_COMPLETE_CNF, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_fw_send_image_complete_cnf.error_code);

    // Check the image that has been written to the FLASH
    fs_t file_system;
    fs_handle_t file_system_handle;
    uint8_t flash_fw_data[fw_size];
    uint32_t bytes_read;

    EXPECT_EQ(FS_NO_ERROR, fs_init(FS_DEVICE));
    EXPECT_EQ(FS_NO_ERROR, fs_mount(FS_DEVICE, &file_system));
    EXPECT_EQ(FS_NO_ERROR, fs_open(file_system, &file_system_handle, fw_type, FS_MODE_READONLY, NULL));
    EXPECT_EQ(FS_NO_ERROR, fs_read(file_system_handle, &flash_fw_data[0], fw_size, &bytes_read));
    EXPECT_EQ(FS_NO_ERROR, fs_close(file_system_handle));

    // Look for differences between the two
    bool flash_and_image_match = true;
    for (uint32_t i = 0; i < fw_size; ++i)
    {
        if (flash_fw_data[i] != fw_image[i])
        {
            flash_and_image_match = false;
            break;
        }
    }

    EXPECT_TRUE(flash_and_image_match);

    // Generate an apple image request
    CMD_SET_HDR((&req), CMD_FW_APPLY_IMAGE_REQ);
    req.p.cmd_fw_apply_image_req.image_type = fw_type;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_fw_apply_image_req_t));

    sm_tick(&state_handle); // Process the request

    // Check the response
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_NO_ERROR, resp.p.cmd_generic_resp.error_code);
}

TEST_F(Sm_MainTest, FwApplyImageNotFound)
{
    // FS_FILE_ID_STM32_IMAGE    (1)  // STM32 application image
    // FS_FILE_ID_BLE_APP_IMAGE  (2)  // BLE application image
    // FS_FILE_ID_BLE_SOFT_IMAGE (3)  // BLE soft-device image

    uint8_t fw_type = 1; // STM32 application image

    BootTagsNotSet();

    sm_set_current_state(&state_handle, SM_MAIN_PROVISIONING);

    config_if_init(CONFIG_IF_BACKEND_USB);
    USBConnectionEvent();

    SetVUSB(true);
    sm_tick(&state_handle);

    EXPECT_EQ(SM_MAIN_PROVISIONING, sm_get_current_state(&state_handle));
    EXPECT_EQ(CONFIG_IF_BACKEND_USB, config_if_current());

    // Generate an apply image request
    cmd_t req;
    CMD_SET_HDR((&req), CMD_FW_APPLY_IMAGE_REQ);
    req.p.cmd_fw_apply_image_req.image_type = fw_type;
    send_message((uint8_t *) &req, CMD_SIZE(cmd_fw_apply_image_req_t));

    sm_tick(&state_handle); // Process the request

    // Check the response
    cmd_t resp;
    receive_message(&resp);
    EXPECT_EQ(CMD_SYNCWORD, resp.h.sync);
    EXPECT_EQ(CMD_GENERIC_RESP, resp.h.cmd);
    EXPECT_EQ(CMD_ERROR_FILE_NOT_FOUND, resp.p.cmd_generic_resp.error_code);
}