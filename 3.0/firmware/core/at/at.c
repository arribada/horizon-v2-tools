/* at.c - AT library for handling AT command thought UART Interface
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

#include "at.h"
#include "bsp.h"
#include "syshal_uart.h"
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include "fs.h"
#include "debug.h"

/* Constants */

#define END_CHARACTER            ('\r')

#define INTERNAL_BUFFER_SIZE        (10) // uint32_t maximum digit
#define MICROSECOND                 (1000000)
#define MILLI_TO_MICROSECONDS(x)    (x * 1000)
#define BITS_PER_CHARACTER          (10) // start/stop bit + 8 data bits

/* Macros */


/* Types */

/* Static Data */
static uint32_t uart_device_instance;

/*Internal Timeout for waiting after the first characted is read  in microseconds*/
static uint32_t char_timeout_microsecond;
static uint32_t char_timeout_millisecond;


/* Static Functions */



/*! \brief Internal utoa
 *
 * Conver uint32_t iput into an uint8_t, with a specific format.
 * It doesn't add the '\0' character to the destination buffer.
 *
 * \param dest[out] Destination buffer.
 * \param value[in]  value to convert.
 * \return count Number of byte written into the destination buffer.
 */
static uint32_t at_utoa(uint32_t value , uint8_t *dest)
{
    uint32_t count = 0;
    uint32_t temp;

    if (dest == NULL)
        return 0;

    if (value == 0)
    {
        *dest = '0';
        count ++;
        return count;
    }

    for (temp = value; temp > 0; temp /= 10, count++);

    for (int i = count - 1; i >= 0; i--)
    {
        *(dest + i) = (value % 10) + '0';
        value /= 10;
    }
    return count;
}

/*! \brief Internal atou
 *
 * Conver uint8_t[] input into an uint32_t, with a specific format.
 *
 *
 * \param src[in] source buffer.
 * \param value[in]  value converted.
 * \return count Number of byte written into the destination buffer.
 */
static uint32_t at_atou(uint32_t *value , uint8_t *src)
{
    uint32_t count = 0;
    uint32_t internal_value = 0;
    uint8_t *temp = src;
    internal_value = 0;
    if (src == NULL)
        return 0;

    for (temp = src; *temp != '\0'; temp++)
    {
        internal_value *= 10;
        internal_value += (*temp - '0');
        count++;
    }
    *value = internal_value;
    return count;
}



/*! \brief Internal strcpy
 *
 * copy the content from a char[] into a uint8_t[] until '\0' character.
 * It doesn't add the '\0' character to the destination buffer.
 *
 * \param dest[out] Destination buffer.
 * \param src[in]  Source buffer.
 * \return count Number of byte written into the destination buffer.
 */
static uint32_t at_strcpy(uint8_t* dest, char* src)
{
    uint32_t count = 0;
    char* temp;
    for (temp = src; *temp != '\0'; temp ++)
    {
        *dest++ = *temp;
        count++;
    }

    return count;
}
/*! \brief Internal function to compose a format command
 *
 * Variadic function to format a message
 * Variadic args has to be initializated by using va_start(args, last_argument_before_...)
 *
 *
 * \param format[in] Destination buffer.
 * \param command[out]  Destination buffer.
 * \param len_in[in]  Destination buffer.
 * \param len_out[out]  Destination buffer.
 * \param arg[out]  Destination buffer.
 * \return \ref AT_NO_ERROR on success.
 * \return \ref AT_ERROR_DEVICE error in fs or UART doesn't match with the expected one.
 * \return \ref AT_ERROR_FORMAT_NOT_SUPPORTED on not supported special character i.e. %d
 */

static inline int create_command(const uint8_t *format, uint8_t *command, uint32_t len_in, uint32_t *len_out, va_list args)
{
    /* Read until end character */
    while (*format != '\0')
    {
        /* Regular Character */
        if (*format != '%')
        {
            *(command + len_in) = *format++;
            len_in++;
        }
        /* Special Character */
        else
        {
            format++;
            char c_type = *format++;
            switch (c_type)
            {
                case 'u' :
                    /* Convert and adapt format from unsigned to uint8_t[]  */
                    len_in += at_utoa(va_arg(args, uint32_t), command + len_in );
                    break;

                case 's':
                    /* Convert and adapt format from char[] to uint8_t[]  */
                    len_in += at_strcpy(command + len_in, va_arg(args, char*));
                    break;
                default:
                    return AT_ERROR_FORMAT_NOT_SUPPORTED;
                    break;
            }
        }
    }

    *(command + len_in)  = END_CHARACTER;
    len_in++;
    *len_out = len_in;
    return AT_NO_ERROR;
}
/* Exported Functions */

/*! \brief Set UART device.
 *
 * Set a specific UART interface for the AT module
 *
 * \param uart_device[in] number of the selected UART interface.
 * \return \ref AT_NO_ERROR on success.
 * \return \ref AT_ERROR_DEVICE error in fs or UART.
 * doesn't match with the expected one.
 */
int at_init(uint32_t uart_device)
{
    /* Uart not present in the device */
    if (uart_device > UART_TOTAL_NUMBER)
    {
        return AT_ERROR_DEVICE;
    }

    uart_device_instance = uart_device;

    /*time between character (ms) */
    uint32_t baud;

    if (syshal_uart_get_baud(uart_device_instance, &baud) != SYSHAL_UART_NO_ERROR)
    {
        return AT_ERROR_DEVICE;
    }
    else
    {
        /* Calc time between character  */
        char_timeout_microsecond = (BITS_PER_CHARACTER * TIMES_CHAR_TIMEOUT * 1000000) / baud;
        char_timeout_millisecond = (BITS_PER_CHARACTER * TIMES_CHAR_TIMEOUT * 1000) / baud;
        if (char_timeout_millisecond == 0)
            char_timeout_millisecond = 1;

        return AT_NO_ERROR;
    }
}

/*! \brief Write AT COMMAND to AT Module
 *
 * Write variadic string to the AT Module
 * adding AT+ string
 *
 * \param format[in] variadic format input.
 * \param ...[in] rest of parameters especificated by format parameter.
 * \return \ref AT_NO_ERROR on success.
 * \return \ref AT_ERROR_DEVICE error in fs or UART.
 * doesn't match with the expected one.
 * \return \ref AT_ERROR_FORMAT_NOT_SUPPORTED on not supported special character i.e. %d
 */
int at_send(const uint8_t *format, ...)
{
    va_list args;
    int status;
    uint8_t command[MAX_AT_COMMAND_SIZE];

    /* Start variadic list */
    va_start(args, format);

    /* Start with AT+ string */
    uint32_t len = at_strcpy(command, "AT+");

    /* Internal function for creating format message */
    status = create_command(format, command, len, &len, args);

    DEBUG_PR_TRACE("at_send: %.*s\r\n", (int) len, command);

    va_end(args);

    if (status)
        return status;

    /* Close variadic list */
    status = syshal_uart_send(uart_device_instance, command, len);
    if (status)
        return AT_ERROR_DEVICE;

    return AT_NO_ERROR;
}


/*! \brief Write raw operation to AT Module
 *
 * Write variadic string to the AT Module
 * without adding AT+ string
 *
 * \param format[in] variadic format input.
 * \param ...[in] rest of parameters especificated by format parameter.
 * \return \ref AT_NO_ERROR on success.
 * \return \ref AT_ERROR_DEVICE error in fs or UART.
 * doesn't match with the expected one.
 */
int at_send_raw_with_cr(const uint8_t *buffer, uint32_t length)
{
    uint8_t last_character = '\r';
    int status;

    status = syshal_uart_send(uart_device_instance, (uint8_t *) buffer, length);
    if (status == SYSHAL_UART_ERROR_TIMEOUT)
        return AT_ERROR_TIMEOUT;
    else if (status != SYSHAL_UART_NO_ERROR)
        return AT_ERROR_DEVICE;

    status = syshal_uart_send(uart_device_instance, &last_character, 1);
    if (status == SYSHAL_UART_ERROR_TIMEOUT)
        return AT_ERROR_TIMEOUT;
    else if (status != SYSHAL_UART_NO_ERROR)
        return AT_ERROR_DEVICE;

    return AT_NO_ERROR;
}


/*! \brief Read an expected transaction for AT module
 *
 * Read an especific message from the AT module.
 * The message read has to start with the same sequence
 * than is expected if not, an error response will be sent.
 * it has a maximum time for waiting the read operation from UART
 * module.
 * This funciton works as a variadic function.
 *
 * \param format[in] variadic format input.
 * \param timeout[in] maximum time for waiting UART Command in milliseconds.
 * \param bytes_read[out] total amount of bytes read.
 * \param ...[in] rest of parameters especificated by format parameter.
 * \return \ref AT_NO_ERROR on success.
 * \return \ref AT_ERROR_TIMEOUT on read timeout error.
 * \return \ref AT_ERROR_UNEXPECTED_RESPONSE message received
 * doesn't match with the expected one.
 * \return \ref AT_ERROR_DEVICE error in fs or UART.
 */
int at_expect(const uint8_t *format, uint32_t timeout_ms, uint32_t *bytes_read, ...)
{

    uint32_t bytes_received;
    uint32_t total_bytes_read;
    int status;


    uint8_t buffer;
    /* Add a final character at the end */
    uint8_t internal_buffer_num[INTERNAL_BUFFER_SIZE + 1];

    const uint8_t *current_format = format;

    va_list args;

    /* Start variadic list */
    va_start(args, bytes_read);
    bytes_received = 0;
    total_bytes_read = 0;
    status = syshal_uart_read_timeout(uart_device_instance, &buffer, sizeof(buffer), MILLI_TO_MICROSECONDS(timeout_ms), char_timeout_microsecond, &bytes_received);

    if (status == SYSHAL_UART_ERROR_TIMEOUT)
    {
        va_end(args);
        return AT_ERROR_TIMEOUT;
    }
    else if (status != SYSHAL_UART_NO_ERROR)
    {
        va_end(args);
        return AT_ERROR_DEVICE;
    }
    total_bytes_read += bytes_received;

#ifndef DEBUG_DISABLED
    if (g_debug_level >= DEBUG_TRACE)
        printf("AT Expect: %c", buffer);
#endif

    /* Read until end character */
    while (*current_format != '\0')
    {
        /* Regular Character */
        if (*current_format != '%')
        {
            if (buffer != *current_format++)
            {
                va_end(args);
                /* Start variadic list */
                va_start(args, bytes_read);
                current_format = format;
            }
            if (*current_format != '\0')
            {
                status = syshal_uart_read_timeout(uart_device_instance, &buffer, sizeof(buffer), char_timeout_microsecond, char_timeout_microsecond, &bytes_received);
                if (status == SYSHAL_UART_ERROR_TIMEOUT)
                {
                    va_end(args);
                    return AT_ERROR_TIMEOUT;
                }
                else if (status != SYSHAL_UART_NO_ERROR)
                {
                    va_end(args);
                    return AT_ERROR_DEVICE;
                }

#ifndef DEBUG_DISABLED
                if (g_debug_level >= DEBUG_TRACE)
                    printf("%c", buffer);
#endif

                total_bytes_read += bytes_received;
            }
        }

        /* Special Character */
        else
        {
            current_format++;
            char c_type = *current_format++;
            char termination_char;
            char *src;
            uint32_t len_src;
            uint32_t index = 0;
            switch (c_type)
            {
                case 'u' :
                    /* read until non-alphanumeric character will be found */
                    while ((buffer >= '0') && (buffer <= '9') && (index < INTERNAL_BUFFER_SIZE))
                    {

                        internal_buffer_num[index] = buffer;
                        index++;
                        status = syshal_uart_read_timeout(uart_device_instance, &buffer, sizeof(buffer), char_timeout_microsecond, char_timeout_microsecond, &bytes_received);
                        if (status == SYSHAL_UART_ERROR_TIMEOUT)
                        {
                            va_end(args);
                            return AT_ERROR_TIMEOUT;
                        }
                        else if (status != SYSHAL_UART_NO_ERROR)
                        {
                            va_end(args);
                            return AT_ERROR_DEVICE;
                        }
#ifndef DEBUG_DISABLED
                        if (g_debug_level >= DEBUG_TRACE)
                            printf("%c", buffer);
#endif
                        total_bytes_read += bytes_received;
                    }
                    if (index == INTERNAL_BUFFER_SIZE && ((buffer >= '0') && (buffer <= '9') ))
                    {
                        va_end(args);
                        return AT_ERROR_BUFFER_OVERFLOW;
                    }
                    internal_buffer_num[index] = '\0';
                    at_atou(va_arg(args, uint32_t*), internal_buffer_num);
                    break;

                case 's':
                    termination_char = *current_format;
                    src = va_arg(args, char*);
                    len_src = va_arg(args, uint32_t);
                    while ((termination_char != buffer) && (index < len_src ))
                    {

                        index++;
                        *src++ = buffer;
                        status = syshal_uart_read_timeout(uart_device_instance, &buffer, sizeof(buffer), char_timeout_microsecond, char_timeout_microsecond, &bytes_received);

                        if (status == SYSHAL_UART_ERROR_TIMEOUT)
                        {
                            va_end(args);
                            return AT_ERROR_TIMEOUT;
                        }
                        else if (status != SYSHAL_UART_NO_ERROR)
                        {
                            va_end(args);
                            return AT_ERROR_DEVICE;
                        }
#ifndef DEBUG_DISABLED
                        if (g_debug_level >= DEBUG_TRACE)
                            printf("%c", buffer);
#endif
                        total_bytes_read += bytes_received;
                    }

                    *src = '\0';
                    break;
                default:
                    va_end(args);
                    return AT_ERROR_FORMAT_NOT_SUPPORTED;
                    break;
            }
        }
    }
    /* Close variadic list */
    va_end(args);
    *bytes_read = total_bytes_read;

#ifndef DEBUG_DISABLED
    if (g_debug_level >= DEBUG_TRACE)
        printf("\r\n");
#endif

    return AT_NO_ERROR;
}

/*! \brief Read raw data from UART and store in fa ile.
 *
 * The requested number of bytes shall be read from AT Device.
 * And store it in the file passed with a handle, the maximum
 * lenght allowed is specify by lenght param
 *
 * \param handle[in] file handle to read.
 * \param timeout[in] maximum time for waiting UART Command in seconds.
 * \param length[in] expected length for reading.
 * \return \ref AT_NO_ERROR on success.
 * \return \ref AT_ERROR_TIMEOUT on read timeout error.
 * \return \ref AT_ERROR_DEVICE error in fs or UART.
 */
int at_read_raw_to_fs(uint32_t timeout_ms, uint32_t length, fs_handle_t handle)
{
    uint8_t buffer[MAX_AT_BUFFER_SIZE];
    int status;
    uint32_t len_written, bytes_received;
    uint32_t current_length;
    uint32_t remain_data = length;

    /* Number of times for reading from UART */
    while (remain_data > 0)
    {
        if (remain_data >= MAX_AT_BUFFER_SIZE)
            current_length = MAX_AT_BUFFER_SIZE;
        else
            current_length = remain_data;

        status = syshal_uart_read_timeout(uart_device_instance, buffer, current_length, MILLI_TO_MICROSECONDS(timeout_ms), char_timeout_microsecond , &bytes_received);

        remain_data -= bytes_received;
        if (status == SYSHAL_UART_ERROR_TIMEOUT)
            return AT_ERROR_TIMEOUT;
        else if (status != SYSHAL_UART_NO_ERROR)
            return AT_ERROR_DEVICE;

        if (fs_write(handle, buffer, bytes_received, &len_written) != FS_NO_ERROR)
            return AT_ERROR_DEVICE;

        if ( len_written != bytes_received)
            return AT_ERROR_DEVICE;

        at_busy_handler();
    }

    return AT_NO_ERROR;
}



/*! \brief Read raw data from UART and store in a buffer.
 *
 * The requested number of bytes shall be read from AT Device.
 * And store it in the buffer passed with a handle, the maximum
 * lenght allowed is specify by lenght param
 *
 * \param timeout[in] maximum time for waiting UART Command in seconds.
 * \param length[in] expected length for reading.
 * \param handle[in] buffer for storing data.
 * \return \ref AT_NO_ERROR on success.
 * \return \ref AT_ERROR_TIMEOUT on read timeout error.
 * \return \ref AT_ERROR_DEVICE error in fs or UART.
 */
int at_read_raw_to_buffer(uint32_t timeout_ms, uint32_t length, uint8_t *buffer)
{
    int status;
    uint32_t bytes_received;

    status = syshal_uart_read_timeout(uart_device_instance, buffer, length, MILLI_TO_MICROSECONDS(timeout_ms), char_timeout_microsecond, &bytes_received);

    if (status == SYSHAL_UART_ERROR_TIMEOUT)
    {
        return AT_ERROR_TIMEOUT;
    }
    else if (status != SYSHAL_UART_NO_ERROR)
    {
        return AT_ERROR_DEVICE;
    }

    return AT_NO_ERROR;
}

/*! \brief Flush UART Buffer.
*
* Flush UART buffer by reading a large amount of data from
* The UART SARA Interface
*
* \return \ref AT_NO_ERROR on success.
* \return \ref AT_ERROR_DEVICE error in fs or UART.
*/
int at_flush(void)
{
    int status = syshal_uart_flush(uart_device_instance);
    /* Read MAX_AT_COMMAND_SIZE */
    if (status != SYSHAL_UART_NO_ERROR)
    {
        return AT_ERROR_DEVICE;
    }
    return AT_NO_ERROR;

}

//UNTESTED UNTIL CHECK IF IT USED
int at_send_raw_fs(fs_handle_t handle, uint32_t length)
{
    DEBUG_PR_WARN("%s NOT TESTED!!", __FUNCTION__);

    uint8_t read_buffer[MAX_AT_BUFFER_SIZE];
    uint32_t bytes_actually_read;
    uint32_t bytes_remain = length;
    uint32_t current_bytes;
    while (bytes_remain > 0)
    {
        if (bytes_remain > MAX_AT_BUFFER_SIZE)
        {
            current_bytes = MAX_AT_COMMAND_SIZE;
        }
        else
        {
            current_bytes = bytes_remain;
        }

        if (fs_read(handle, read_buffer, current_bytes, &bytes_actually_read) != FS_NO_ERROR)
        {
            return AT_ERROR_DEVICE;
        }
        bytes_remain -= bytes_actually_read;
        if (syshal_uart_send(uart_device_instance, read_buffer, bytes_actually_read) != SYSHAL_UART_NO_ERROR)
        {
            return AT_ERROR_DEVICE;
        }
    }
    return AT_NO_ERROR;
}


/*! \brief Read raw data from UART and store in a file.
 *
 * The requested number of bytes shall be read from AT Device.
 * And store it in the file passed with a handle, the maximum
 * lenght allowed is specify by lenght param
 *
 * \param handle[in] file handle to read.
 * \param timeout[in] maximum time for waiting UART Command.
 * \param length[in] expected length for reading.
 * \return \ref AT_NO_ERROR on success.
 * \return \ref AT_ERROR_TIMEOUT on read timeout error.
 * \return \ref AT_ERROR_DEVICE error in fs or UART.
 * \return \ref AT_ERROR_HTTP if a HTTP error was returned
 */
int at_expect_http_header(uint32_t *length, uint32_t *http_code)
{
    uint32_t total_bytes_header = 0;
    uint32_t bytes_received;
    int status;

    uint8_t HTTP_expect[] = "HTTP/1.1 %u";
    uint8_t final_string[] = {'\r', '\n', '\r', '\n', '\0'};

    /* Read HTTP code */
    status = at_expect(HTTP_expect, char_timeout_millisecond , &bytes_received, http_code);
    if (status)
        return status;

    total_bytes_header += bytes_received;

    if (*http_code != HTTP_CODE_SUCCESS_NO_ERROR)
    {
        *length = total_bytes_header;
        return AT_ERROR_HTTP;
    }

    /* Read until the end of the header */
    status = at_expect(final_string, char_timeout_millisecond , &bytes_received);
    if (status)
        return status;

    total_bytes_header += bytes_received;

    *length = total_bytes_header;

    return AT_NO_ERROR;
}

/*! \brief Discard length UART Characters
 *
 * Read from the UART BUFFER the number of characters specified by
 * length param or an error has happened
 *
 * \param handle[in] number of character for discarding.
 * \return \ref AT_NO_ERROR on success.
 * \return \ref AT_ERROR_TIMEOUT on read timeout error.
 * \return \ref AT_ERROR_DEVICE error in fs or UART.
 */
int at_discard(uint32_t length)
{
    uint32_t internal_counter = 0;
    uint32_t bytes_received;
    int status = AT_NO_ERROR;
    uint8_t buffer;
    while (status == AT_NO_ERROR && internal_counter < length)
    {
        status = syshal_uart_read_timeout(uart_device_instance, &buffer, sizeof(buffer), char_timeout_microsecond, char_timeout_microsecond, &bytes_received);
        internal_counter += bytes_received;
    }

    if (status == SYSHAL_UART_ERROR_TIMEOUT)
    {
        return AT_ERROR_TIMEOUT;
    }
    else if (status != SYSHAL_UART_NO_ERROR)
    {
        return AT_ERROR_DEVICE;
    }

    return AT_NO_ERROR;
}

/*! \brief At busy handler
 *
 * This handler function can be used to perform useful work
 * whilst the AT layer is busy doing a potentially long task
 *
 */
__attribute__((weak)) void at_busy_handler(void)
{
    /* Do not modify -- override with your own handler function */
}