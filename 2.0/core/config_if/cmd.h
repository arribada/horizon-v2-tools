/* cmd.h - Configuration interface commands
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

#ifndef _CMD_H_
#define _CMD_H_

#include <stdint.h>

#define CMD_SYNCWORD        (0x7E)

#define CMD_SIZE_HDR        (sizeof(cmd_hdr_t))
#define CMD_MIN_SIZE        (CMD_SIZE_HDR)
#define CMD_MAX_SIZE        (512)
#define CMD_MAX_PAYLOAD     (CMD_MAX_SIZE - sizeof(cmd_hdr_t))

#define CMD_CFG_TAG_ALL (0xFFFF) // A special tag to denote a read of all configuration values from RAM

#define CMD_SET_HDR(p, i)  \
    p->h.sync  = CMD_SYNCWORD; \
    p->h.cmd   = i;

#define CMD_SIZE(i)  (sizeof(cmd_hdr_t) + sizeof(i))

#define CFG_READ_REQ_READ_ALL (0xFFFF) // Read all configuration tags
#define CFG_ERASE_REQ_ERASE_ALL (CFG_READ_REQ_READ_ALL) // Erase all configuration tags

#define RESET_REQ_STM32           (0) // Reset the STM32 device
#define RESET_REQ_FLASH_ERASE_ALL (1) // Erase all of the FLASH

typedef struct __attribute__((__packed__))
{
    uint8_t sync; // Start of command synchronization byte
    uint8_t cmd;
} cmd_hdr_t;

enum
{
    CMD_LOG_CREATE_REQ_MODE_FILL,
    CMD_LOG_CREATE_REQ_MODE_CIRCULAR
};

typedef enum
{
    CMD_GENERIC_RESP, // Generic response message sent where only an error response is needed

    ///////////////// Configuration /////////////////
    CMD_CFG_READ_REQ,      // Read single configuration tag or all configuration tags
    CMD_CFG_WRITE_REQ,     // Write new configuration items as a series of tag/value pairs
    CMD_CFG_SAVE_REQ,      // Save all current configuration settings to flash memory
    CMD_CFG_RESTORE_REQ,   // Restore all current configuration settings from flash memory
    CMD_CFG_ERASE_REQ,     // Erase a single or all configuration tags in flash memory
    CMD_CFG_PROTECT_REQ,   // Protect the configuration file in flash memory
    CMD_CFG_UNPROTECT_REQ, // Unprotect the configuration file in flash memory
    CMD_CFG_READ_RESP,
    CMD_CFG_WRITE_CNF,     // Confirmation message sent after the entire CFG_WRITE_REQ payload has been received

    ////////////////// GPS Bridge ///////////////////
    CMD_GPS_WRITE_REQ,      // Send UBX commands directly to the GPS module
    CMD_GPS_READ_REQ,       // Receive UBX command responses directly from the GPS module
    CMD_GPS_READ_RESP,      // The response from the GPS module
    CMD_GPS_CONFIG_REQ,     // Allow a GPS IRQ events to be generated and sent over the USB interrupt endpoint.  This shall be used to indicate that data is available to be read from the internal FIFO

    ////////////////// BLE Bridge ///////////////////
    CMD_BLE_CONFIG_REQ,     // Allow BLE IRQ events to be generated and sent over the USB interrupt endpoint
    CMD_BLE_WRITE_REQ,      // Initiate a write to the BLE module at Address with data of Length
    CMD_BLE_READ_REQ,       // Initiate a read from the BLE module from Address for data of Length

    //////////////////// System /////////////////////
    CMD_STATUS_REQ,                 // Request firmware status
    CMD_STATUS_RESP,                // Firmware status response
    CMD_FW_SEND_IMAGE_REQ,          // Request to send a new firmware image and store temporarily in local flash memory
    CMD_FW_SEND_IMAGE_COMPLETE_CNF, // This shall be sent by the server to the client to indicate all bytes have been received and stored
    CMD_FW_APPLY_IMAGE_REQ,         // Request to apply an existing firmware image in temporary storage to the target
    CMD_RESET_REQ,                  // Request to reset the system

    //////////////////// Battery /////////////////////
    CMD_BATTERY_STATUS_REQ,  // Request battery status
    CMD_BATTERY_STATUS_RESP, // Error response carrying the battery status information

    //////////////////// Logging /////////////////////
    CMD_LOG_CREATE_REQ, // Request to create a log file of the specified operation mode and length
    CMD_LOG_ERASE_REQ,  // Request to erase the current log file
    CMD_LOG_READ_REQ,   // Read the log file from the starting offset for the given number of bytes
    CMD_LOG_READ_RESP,  // Response for a read request

} cmd_id_t;

// Error codes
typedef enum
{
    CMD_NO_ERROR,                    // Successful completion of the command.
    CMD_ERROR_FILE_NOT_FOUND,        // File associated with the operation could not be found.
    CMD_ERROR_FILE_ALREADY_EXISTS,   // Unable to create a file that already exists.
    CMD_ERROR_INVALID_CONFIG_TAG,    // Invalid configuration tag found in the tag stream.
    CMD_ERROR_GPS_COMMS,             // GPS module communications error e.g., attempt to do a GPS read/write when not bridging.
    CMD_ERROR_TIMEOUT,               // A timeout happened waiting on the byte stream to be received.
    CMD_ERROR_CONFIG_PROTECTED,      // Configuration operation not permitted as it is protected.
    CMD_ERROR_CONFIG_TAG_NOT_SET,    // Configuration tag has not been set.
    CMD_ERROR_BRIDGING_DISABLED,     // Bridging is currently disabled for this module/device
    CMD_ERROR_DATA_OVERSIZE,         // We've received more data then we were expecting
    CMD_ERROR_INVALID_PARAMETER,     // An invalid parameter has been provided
    CMD_ERROR_INVALID_FW_IMAGE_TYPE, // An invalid image type was received in a CMD_FW_SEND_IMAGE_REQ
    CMD_ERROR_IMAGE_CRC_MISMATCH,    // The firmware images CRC does not match one in flash
    CMD_ERROR_FILE_INCOMPATIBLE,     // This file is incompatible with this firmware version
} cmd_error_t;

// Exposed functions
uint32_t cmd_size_of_command(cmd_id_t command);

// Generic response message
typedef struct __attribute__((__packed__))
{
    uint8_t error_code;
} cmd_generic_resp_t;

///////////////// Configuration /////////////////
typedef struct __attribute__((__packed__))
{
    uint16_t configuration_tag;
} cmd_cfg_read_req_t;

typedef struct __attribute__((__packed__))
{
    uint32_t length;
} cmd_cfg_write_req_t;

typedef struct __attribute__((__packed__))
{
    uint16_t configuration_tag;
} cmd_cfg_erase_req_t;

typedef struct __attribute__((__packed__))
{
    uint8_t error_code;
    uint32_t length;
} cmd_cfg_read_resp_t;

typedef struct __attribute__((__packed__))
{
    uint8_t error_code;
} cmd_cfg_write_cnf_t;

////////////////// GPS Bridge ///////////////////
typedef struct __attribute__((__packed__))
{
    uint32_t length;
} cmd_gps_write_req_t;

typedef struct __attribute__((__packed__))
{
    uint32_t length;
} cmd_gps_read_req_t;

typedef struct __attribute__((__packed__))
{
    uint8_t error_code;
    uint32_t length;
} cmd_gps_read_resp_t;

typedef struct __attribute__((__packed__))
{
    uint8_t enable;
} cmd_gps_config_req_t;

////////////////// BLE Bridge ///////////////////
typedef struct __attribute__((__packed__))
{
    uint8_t enable;
} cmd_ble_config_req_t;

typedef struct __attribute__((__packed__))
{
    uint8_t address;
    uint16_t length;
} cmd_ble_write_req_t;

typedef struct __attribute__((__packed__))
{
    uint8_t address;
    uint16_t length;
} cmd_ble_read_req_t;

//////////////////// System /////////////////////
typedef struct __attribute__((__packed__))
{
    uint8_t error_code;
    uint32_t stm_firmware_version;
    uint32_t ble_firmware_version;
    uint32_t configuration_format_version;
} cmd_status_resp_t;

typedef struct __attribute__((__packed__))
{
    uint8_t image_type;
    uint32_t length;
    uint32_t CRC32;
} cmd_fw_send_image_req_t;

typedef struct __attribute__((__packed__))
{
    uint8_t error_code;
} cmd_fw_send_image_complete_cnf_t;

typedef struct __attribute__((__packed__))
{
    uint8_t image_type;
} cmd_fw_apply_image_req_t;

typedef struct __attribute__((__packed__))
{
    uint8_t reset_type;
} cmd_reset_req_t;

//////////////////// Battery /////////////////////
typedef struct __attribute__((__packed__))
{
    uint8_t error_code;
    uint8_t charging_indicator;
    uint8_t charge_level; // Charge level in percent
} cmd_battery_status_resp_t;

//////////////////// Logging /////////////////////
typedef struct __attribute__((__packed__))
{
    uint8_t mode;
    uint8_t sync_enable;
} cmd_log_create_req_t;

typedef struct __attribute__((__packed__))
{
    uint32_t start_offset;
    uint32_t length;
} cmd_log_read_req_t;

typedef struct __attribute__((__packed__))
{
    uint8_t error_code;
    uint32_t length;
} cmd_log_read_resp_t;

typedef struct __attribute__((__packed__))
{
    cmd_hdr_t h;
    union
    {
        cmd_generic_resp_t                  cmd_generic_resp;
        cmd_cfg_read_req_t                  cmd_cfg_read_req;
        cmd_cfg_write_req_t                 cmd_cfg_write_req;
        cmd_cfg_erase_req_t                 cmd_cfg_erase_req;
        cmd_cfg_read_resp_t                 cmd_cfg_read_resp;
        cmd_cfg_write_cnf_t                 cmd_cfg_write_cnf;
        cmd_gps_write_req_t                 cmd_gps_write_req;
        cmd_gps_read_req_t                  cmd_gps_read_req;
        cmd_gps_read_resp_t                 cmd_gps_read_resp;
        cmd_gps_config_req_t                cmd_gps_config_req;
        cmd_ble_config_req_t                cmd_ble_config_req;
        cmd_ble_write_req_t                 cmd_ble_write_req;
        cmd_ble_read_req_t                  cmd_ble_read_req;
        cmd_status_resp_t                   cmd_status_resp;
        cmd_fw_send_image_req_t             cmd_fw_send_image_req;
        cmd_fw_send_image_complete_cnf_t    cmd_fw_send_image_complete_cnf;
        cmd_fw_apply_image_req_t            cmd_fw_apply_image_req;
        cmd_reset_req_t                     cmd_reset_req;
        cmd_battery_status_resp_t           cmd_battery_status_resp;
        cmd_log_create_req_t                cmd_log_create_req;
        cmd_log_read_req_t                  cmd_log_read_req;
        cmd_log_read_resp_t                 cmd_log_read_resp;
    } p;
} cmd_t;

#endif /* _CMD_H_ */
