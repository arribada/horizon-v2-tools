/* iot.c - Internet of things abstraction layer
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
#include "syshal_cellular.h"
#include "syshal_pmu.h"
#include "logging.h"
#include "aws.h"
#include "iot.h"
#include "debug.h"

#define JSON_WORKING_BUF_SIZE (2048)
#define CELLULAR_SET_RAT_TIMEOUT_MS (10000)

static iot_init_t config;

static char logging_topic_path_full[sizeof(config.iot_cellular_aws_config->contents.thing_name) + sizeof(config.iot_cellular_aws_config->contents.logging_topic_path)];
static char device_shadow_path_full[sizeof(config.iot_cellular_aws_config->contents.thing_name) + sizeof(config.iot_cellular_aws_config->contents.device_shadow_path)];

static int last_error;

static struct
{
    iot_radio_type_t radio_type;
    bool radio_on;
    bool connected;
} internal_status;

static int cellular_error_mapping(int error_code)
{
    switch (error_code)
    {
        case SYSHAL_CELLULAR_ERROR_TIMEOUT:
            return IOT_ERROR_TIMEOUT;
        case SYSHAL_CELLULAR_ERROR_HTTP:
            return IOT_ERROR_HTTP;
        default:
            return IOT_ERROR_BACKEND;
    }
}

static int fs_error_mapping(int error_code)
{
    switch (error_code)
    {
        case FS_ERROR_FILESYSTEM_CORRUPTED:
            return IOT_ERROR_FS_FILE_CORRUPTED;
        case FS_ERROR_FILE_NOT_FOUND:
            return IOT_ERROR_FILE_NOT_FOUND;
        default:
            return IOT_ERROR_FS;
    }
}

static scan_mode_t scan_mode_mapping(uint32_t mode)
{
    switch (mode)
    {
        default:
        case SYS_CONFIG_TAG_IOT_CELLULAR_CONNECTION_MODE_AUTO:
            return SCAN_MODE_AUTO;
        case SYS_CONFIG_TAG_IOT_CELLULAR_CONNECTION_MODE_2_G:
            return SCAN_MODE_2G;
        case SYS_CONFIG_TAG_IOT_CELLULAR_CONNECTION_MODE_3_G:
            return SCAN_MODE_3G;
    }
}

static bool replace_hash(char * dest, const char * source, const char * replacement)
{
    size_t source_len = strlen(source);
    size_t replacement_len = strlen(replacement);

    char * hash_location = strchr(source, '#');
    if (!hash_location)
    {
        // No hash found so just use original string
        memcpy(dest, source, source_len);
        return false;
    }

    // Copy up to '#' from source
    size_t first_size = hash_location - source;
    memcpy(dest, source, first_size);
    dest += first_size;

    // Append replacement string
    memcpy(dest, replacement, replacement_len);
    dest += replacement_len;

    // Append string after '#' from source
    size_t remaining_size = source_len - first_size - 1;
    memcpy(dest, source + first_size + 1, remaining_size);
    dest += remaining_size;

    // Null terminate the string
    *(dest) = '\0';

    return true;
}

int iot_get_last_error(void)
{
    return last_error;
}

int iot_init(iot_init_t init)
{
    // Make a local copy of the init params
    config = init;

    // Replace character '#' with thing_name in aws paths if one exists, and IoT cellular AWS is enabled
    if (config.iot_config->contents.enable && config.iot_config->hdr.set &&
        config.iot_cellular_config->contents.enable && config.iot_cellular_config->hdr.set)
    {
        if (config.iot_cellular_aws_config->contents.thing_name[0] == '\0')
        {
            // Use the system device name if a thing_name is not given
            replace_hash(device_shadow_path_full, config.iot_cellular_aws_config->contents.device_shadow_path, config.system_device_identifier->contents.name);
            replace_hash(logging_topic_path_full, config.iot_cellular_aws_config->contents.logging_topic_path, config.system_device_identifier->contents.name);
        }
        else
        {
            replace_hash(device_shadow_path_full, config.iot_cellular_aws_config->contents.device_shadow_path, config.iot_cellular_aws_config->contents.thing_name);
            replace_hash(logging_topic_path_full, config.iot_cellular_aws_config->contents.logging_topic_path, config.iot_cellular_aws_config->contents.thing_name);
        }
    }

    internal_status.radio_on = false;
    internal_status.connected = false;

    return IOT_NO_ERROR;
}

int iot_power_on(iot_radio_type_t radio_type)
{
    iot_imsi_t imsi;
    int ret;

    if (!config.iot_config->hdr.set || !config.iot_config->contents.enable)
        return IOT_ERROR_NOT_ENABLED;

    if (internal_status.radio_on)
        return IOT_ERROR_RADIO_ALREADY_ON;

    switch (radio_type)
    {
        case IOT_RADIO_CELLULAR:
            if (!config.iot_cellular_config->hdr.set || !config.iot_cellular_config->contents.enable)
                return IOT_ERROR_NOT_ENABLED;

            DEBUG_PR_TRACE("Powering cellular on");
            if (syshal_cellular_power_on())
                return IOT_ERROR_BACKEND;
            
            DEBUG_PR_TRACE("Syncing cellular comms");
            if (syshal_cellular_sync_comms())
                return IOT_ERROR_BACKEND;

            DEBUG_PR_TRACE("Checking cellular sim");
            ret = syshal_cellular_check_sim((uint8_t *) &imsi);
            if (SYSHAL_CELLULAR_ERROR_SIM_CARD_NOT_FOUND == ret)
                return IOT_ERROR_NO_SIM_FOUND;
            else if (ret)
                return IOT_ERROR_BACKEND;

            DEBUG_PR_TRACE("Creating secure profile");
            if (syshal_cellular_create_secure_profile())
                return IOT_ERROR_BACKEND;

            DEBUG_PR_TRACE("Setting connection preferences");
            if (syshal_cellular_set_rat(CELLULAR_SET_RAT_TIMEOUT_MS, scan_mode_mapping(config.iot_cellular_config->contents.connection_mode)))
                return IOT_ERROR_BACKEND;
            internal_status.radio_type = radio_type;
            internal_status.radio_on = true;
            break;

        case IOT_RADIO_SATELLITE:
            DEBUG_PR_ERROR("IOT_RADIO_SATELLITE not implemented!");
            internal_status.radio_type = radio_type;
            internal_status.radio_on = true;
            break;

        default:
            return IOT_INVALID_PARAM;
            break;
    }

    return IOT_NO_ERROR;
}

int iot_power_off(void)
{
    // We don't check if the Iot is enabled here as we want to be 100% sure the radio is powered off when this is called
    switch (internal_status.radio_type)
    {
        case IOT_RADIO_CELLULAR:
            DEBUG_PR_TRACE("Powering cellular off");
            syshal_cellular_power_off();
            break;
        case IOT_RADIO_SATELLITE:
            DEBUG_PR_ERROR("IOT_RADIO_SATELLITE not implemented!");
            break;
        default:
            return IOT_INVALID_PARAM; // Shouldn't be possible
            break;
    }

    internal_status.radio_on = false;
    internal_status.connected = false;

    return IOT_NO_ERROR;
}

int iot_connect(uint32_t timeout_ms)
{
    int return_code;

    if (!config.iot_config->hdr.set || !config.iot_config->contents.enable)
        return IOT_ERROR_NOT_ENABLED;

    if (!internal_status.radio_on)
        return IOT_ERROR_RADIO_NOT_ON;

    switch (internal_status.radio_type)
    {
        case IOT_RADIO_CELLULAR:
            if (!config.iot_cellular_config->hdr.set || !config.iot_cellular_config->contents.enable)
                return IOT_ERROR_NOT_ENABLED;

            DEBUG_PR_TRACE("Scanning for cellular network");
            return_code = syshal_cellular_scan(timeout_ms);
            if (return_code == SYSHAL_CELLULAR_ERROR_TIMEOUT)
                return IOT_ERROR_NOT_RADIO_COVERAGE;

            DEBUG_PR_TRACE("Attaching to cellular network");
            return_code = syshal_cellular_attach(timeout_ms);
            if (return_code == SYSHAL_CELLULAR_ERROR_TIMEOUT)
                return IOT_ERROR_NOT_RADIO_COVERAGE;

            DEBUG_PR_TRACE("Activating pdp");
            return_code = syshal_cellular_activate_pdp(&config.iot_cellular_apn->contents.apn, timeout_ms);
            if (return_code)
                return cellular_error_mapping(return_code);
            break;
        case IOT_RADIO_SATELLITE:
            DEBUG_PR_ERROR("IOT_RADIO_SATELLITE not implemented!");
            break;
        default:
            return IOT_INVALID_PARAM; // Shouldn't be possible
            break;
    }

    internal_status.connected = true;
    return IOT_NO_ERROR;
}

int iot_check_sim(iot_imsi_t * imsi)
{
    if (!internal_status.radio_on)
        return IOT_ERROR_RADIO_NOT_ON;

    if (!internal_status.radio_type == IOT_RADIO_CELLULAR)
        return IOT_ERROR_NOT_SUPPORTED;

    int ret = syshal_cellular_check_sim((uint8_t *) imsi);
    if (SYSHAL_CELLULAR_ERROR_SIM_CARD_NOT_FOUND == ret)
        return IOT_ERROR_NO_SIM_FOUND;
    else if (ret)
        return IOT_ERROR_BACKEND;

    return IOT_NO_ERROR;
}

int iot_calc_prepass(iot_prepass_result_t * result)
{
    DEBUG_PR_ERROR("%s not implemented!", __FUNCTION__);
    return -100;
}

int iot_fetch_device_shadow(uint32_t timeout_ms, iot_device_shadow_t * shadow)
{
    int return_code;
    uint32_t bytes_written;
    uint32_t http_return_code;

    if (!internal_status.radio_on)
        return IOT_ERROR_RADIO_NOT_ON;

    if (!internal_status.radio_type == IOT_RADIO_CELLULAR)
        return IOT_ERROR_NOT_SUPPORTED;

    if (!internal_status.connected)
        return IOT_ERROR_NOT_CONNECTED;

    DEBUG_PR_TRACE("Fetching device shadow from: %s:%u%s", config.iot_cellular_aws_config->contents.arn, config.iot_cellular_aws_config->contents.port, device_shadow_path_full);

    return_code = syshal_cellular_https_get(timeout_ms, config.iot_cellular_aws_config->contents.arn, config.iot_cellular_aws_config->contents.port, device_shadow_path_full);
    if (return_code)
        return cellular_error_mapping(return_code);

    uint8_t file_working_buffer[JSON_WORKING_BUF_SIZE] = {0};

    return_code = syshal_cellular_read_from_file_to_buffer(file_working_buffer, sizeof(file_working_buffer), &bytes_written, &http_return_code);
    if (return_code)
    {
        last_error = http_return_code;
        if (SYSHAL_CELLULAR_ERROR_HTTP == return_code)
            DEBUG_PR_ERROR("HTTP connection failed with %d", last_error);
        return cellular_error_mapping(return_code);
    }

    DEBUG_PR_TRACE("Device shadow: %.*s", JSON_WORKING_BUF_SIZE, file_working_buffer);

    if (aws_json_gets_device_shadow((char *) file_working_buffer, shadow, bytes_written))
        return IOT_ERROR_BACKEND;

    return IOT_NO_ERROR;
}

int iot_send_device_status(uint32_t timeout_ms, const iot_device_status_t * device_status)
{
    int return_code;
    int json_length;

    if (!internal_status.radio_on)
        return IOT_ERROR_RADIO_NOT_ON;

    if (!internal_status.connected)
        return IOT_ERROR_NOT_CONNECTED;

    uint8_t file_working_buffer[JSON_WORKING_BUF_SIZE] = {0};

    DEBUG_PR_TRACE("Sending device status to: %s:%u%s", config.iot_cellular_aws_config->contents.arn, config.iot_cellular_aws_config->contents.port, device_shadow_path_full);

    json_length = aws_json_dumps_device_status(device_status, (char *) file_working_buffer, sizeof(file_working_buffer));
    if (json_length < 0)
        return IOT_ERROR_BACKEND;

    DEBUG_PR_TRACE("Device status: %.*s", JSON_WORKING_BUF_SIZE, file_working_buffer);

    return_code = syshal_cellular_write_from_buffer_to_file(file_working_buffer, json_length);
    if (return_code)
        return cellular_error_mapping(return_code);

    return_code = syshal_cellular_https_post(timeout_ms, config.iot_cellular_aws_config->contents.arn, config.iot_cellular_aws_config->contents.port, device_shadow_path_full);
    if (return_code)
        return cellular_error_mapping(return_code);

    return IOT_NO_ERROR;
}

// Returns IOT_NO_ERROR if the log file is valid
// Returns IOT_ERROR_FS_FILE_CORRUPTED if the log file is corrupted
static int is_log_file_corrupt(fs_t fs, uint32_t local_file_id, uint32_t log_file_read_pos, uint32_t length)
{
    fs_handle_t handle;
    int return_code;
    uint32_t bytes_actually_read;

    return_code = fs_open(fs, &handle, local_file_id, FS_MODE_READONLY, NULL);
    if (return_code)
        return fs_error_mapping(return_code);

    return_code = fs_seek(handle, log_file_read_pos);
    if (return_code)
    {
        fs_close(handle);
        return fs_error_mapping(return_code);
    }

    while (length)
    {
        // Read the tag value
        uint8_t tag_id;
        return_code = fs_read(handle, &tag_id, sizeof(tag_id), &bytes_actually_read);
        if (return_code || !bytes_actually_read)
        {
            fs_close(handle);
            return fs_error_mapping(return_code);
        }

        // Is this tag valid?
        size_t tag_size;
        if (logging_tag_size(tag_id, &tag_size))
        {
            fs_close(handle);
            return IOT_ERROR_FS_FILE_CORRUPTED;
        }

        // Tag is valid but is all its data present?
        if (tag_size > length)
        {
            fs_close(handle);
            return IOT_ERROR_FS_FILE_CORRUPTED;
        }

        length -= tag_size;

        if (length)
        {
            return_code = fs_seek(handle, tag_size - sizeof(tag_id));
            if (return_code)
            {
                fs_close(handle);
                return fs_error_mapping(return_code);
            }
        }
    }

    fs_close(handle);
    return IOT_NO_ERROR;
}

/**
 * @brief      Send a log file using IoT
 *
 * @param[in]  timeout_ms         The timeout_ms
 * @param[in]  fs                 The file system
 * @param[in]  local_file_id      The local file identifier
 * @param[in]  log_file_read_pos  The log file read position
 *
 * @return     Number of bytes sent on success
 * @return     < 0 on failure
 */
int iot_send_logging(uint32_t timeout_ms, fs_t fs, uint32_t local_file_id, uint32_t log_file_read_pos)
{
    uint32_t total_bytes_to_send, bytes_to_send;
    uint8_t buffer[IOT_AWS_MAX_FILE_SIZE]; // WARN: This is big. Make sure the stack can handle it
    fs_handle_t handle;
    fs_stat_t stat;
    int return_code;

    if (!internal_status.radio_on)
        return IOT_ERROR_RADIO_NOT_ON;

    if (!internal_status.radio_type == IOT_RADIO_CELLULAR)
        return IOT_ERROR_NOT_SUPPORTED;

    if (!internal_status.connected)
        return IOT_ERROR_NOT_CONNECTED;

    return_code = fs_stat(fs, local_file_id, &stat);
    if (return_code)
        return fs_error_mapping(return_code);

    if (stat.size <= log_file_read_pos)
        return 0; // There is no data to send so return

    total_bytes_to_send = stat.size - log_file_read_pos;

    DEBUG_PR_TRACE("Sending log file data of length %lu starting at position %lu", total_bytes_to_send, log_file_read_pos);

    return_code = is_log_file_corrupt(fs, local_file_id, log_file_read_pos, total_bytes_to_send);
    if (return_code)
        return return_code;

    return_code = fs_open(fs, &handle, local_file_id, FS_MODE_READONLY, NULL);
    if (return_code)
        return fs_error_mapping(return_code);

    return_code = fs_seek(handle, log_file_read_pos);
    if (return_code)
        return fs_error_mapping(return_code);

    uint32_t bytes_in_buffer = 0;

    bytes_to_send = total_bytes_to_send;
    while (bytes_to_send)
    {
        // Attempt to read the maximum amount we can to fill our working buffer
        uint32_t bytes_actually_read;
        return_code = fs_read(handle, &buffer[bytes_in_buffer], sizeof(buffer) - bytes_in_buffer, &bytes_actually_read);
        if (return_code)
        {
            fs_close(handle);
            return fs_error_mapping(return_code);
        }

        bytes_in_buffer += bytes_actually_read;

        if (bytes_in_buffer >= bytes_to_send)
        {
            // We have reached the end of the file so write whatever data we have to the AWS file
            return_code = syshal_cellular_write_from_buffer_to_file(buffer, bytes_in_buffer);
            if (return_code)
            {
                fs_close(handle);
                return cellular_error_mapping(return_code);
            }

            return_code = syshal_cellular_https_post(timeout_ms, config.iot_cellular_aws_config->contents.arn, config.iot_cellular_aws_config->contents.port, logging_topic_path_full);
            if (return_code)
            {
                fs_close(handle);
                return cellular_error_mapping(return_code);
            }

            bytes_to_send -= bytes_in_buffer;
        }
        else
        {
            // If there is still data remaining then we need to ensure we don't send a partial log entry
            // if we did this we would break the AWS lamda functions causing data loss/corruption

            // To do this we must scan through all the log entries and find the last one that will fit

            uint32_t byte_idx = 0;
            while (byte_idx < sizeof(buffer))
            {
                size_t tag_size;
                if (logging_tag_size(buffer[byte_idx], &tag_size))
                {
                    fs_close(handle);
                    return IOT_ERROR_FS_FILE_CORRUPTED;
                }

                if (byte_idx + tag_size > sizeof(buffer))
                {
                    // If this is a partial tag then exit
                    break;
                }

                byte_idx += tag_size;
            }

            // Send data up to and including the last full tag
            return_code = syshal_cellular_write_from_buffer_to_file(buffer, byte_idx);
            if (return_code)
            {
                fs_close(handle);
                return cellular_error_mapping(return_code);
            }

            return_code = syshal_cellular_https_post(timeout_ms, config.iot_cellular_aws_config->contents.arn, config.iot_cellular_aws_config->contents.port, logging_topic_path_full);
            if (return_code)
            {
                fs_close(handle);
                return cellular_error_mapping(return_code);
            }

            // Move any partial tags to the start of the buffer
            bytes_to_send -= byte_idx;
            bytes_in_buffer = sizeof(buffer) - byte_idx;
            memcpy(buffer, &buffer[byte_idx], bytes_in_buffer);
        }

        iot_busy_handler();

    }

    fs_close(handle);
    return total_bytes_to_send;
}

int iot_download_file(uint32_t timeout_ms, const iot_url_t * url, fs_t fs, uint32_t local_file_id, uint32_t *file_size)
{
    fs_handle_t handle;
    int return_code;
    uint32_t http_return_code;

    DEBUG_PR_TRACE("%s()", __FUNCTION__);

    if (!internal_status.radio_on)
        return IOT_ERROR_RADIO_NOT_ON;

    if (!internal_status.radio_type == IOT_RADIO_CELLULAR)
        return IOT_ERROR_NOT_SUPPORTED;

    if (!internal_status.connected)
        return IOT_ERROR_NOT_CONNECTED;
    return_code = syshal_cellular_https_get(timeout_ms, url->domain, url->port, url->path);
    if (return_code)
        return cellular_error_mapping(return_code);

    if (FS_ERROR_FILE_NOT_FOUND != fs_delete(fs, local_file_id))
        return IOT_ERROR_FS;

    if (fs_open(fs, &handle, local_file_id, FS_MODE_CREATE, NULL))
        return IOT_ERROR_FS;

    return_code = syshal_cellular_read_from_file_to_fs(handle, &http_return_code, file_size);
    if (return_code)
    {
        last_error = http_return_code;
        if (SYSHAL_CELLULAR_ERROR_HTTP == return_code)
            DEBUG_PR_ERROR("HTTP connection failed with %d", last_error);
        fs_close(handle);
        return cellular_error_mapping(return_code);
    }

    fs_close(handle);
    return IOT_NO_ERROR;
}

/*! \brief IoT busy handler
 *
 * This handler function can be used to perform useful work
 * whilst the IoT layer is busy doing a potentially long task
 *
 */
__attribute__((weak)) void iot_busy_handler(void)
{
    /* Do not modify -- override with your own handler function */
}