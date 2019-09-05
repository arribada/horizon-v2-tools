/* Copyright (C) 2018 Arribada
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

#include "bsp.h"

///////////////////////////////// GPIO definitions ////////////////////////////////
GPIO_InitTypeDefAndPort_t GPIO_Inits[GPIO_TOTAL_NUMBER] =
{
    // Port, Pin, Mode, Pull, Speed, Alternate
    {GPIOC, {GPIO_PIN_10, GPIO_MODE_IT_RISING_FALLING, GPIO_PULLDOWN, GPIO_SPEED_FREQ_HIGH, 0}},      // GPIO_USB_ID

    {GPIOA, {GPIO_PIN_0,  GPIO_MODE_EVT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0}},               // GPIO_BUTTON1

    {GPIOC, {GPIO_PIN_6,  GPIO_MODE_OUTPUT_PP,  GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0}},               // GPIO_LED3
    {GPIOC, {GPIO_PIN_8,  GPIO_MODE_OUTPUT_PP,  GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0}},               // GPIO_LED4
    {GPIOC, {GPIO_PIN_9,  GPIO_MODE_OUTPUT_PP,  GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0}},               // GPIO_LED5
    {GPIOC, {GPIO_PIN_7,  GPIO_MODE_OUTPUT_PP,  GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, 0}},               // GPIO_LED6

    {GPIOA, {GPIO_PIN_9,  GPIO_MODE_AF_PP,      GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_USART1}}, // GPIO_UART1_TX
    {GPIOA, {GPIO_PIN_10, GPIO_MODE_AF_PP,      GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_USART1}}, // GPIO_UART1_RX
    {GPIOC, {GPIO_PIN_4,  GPIO_MODE_AF_PP,      GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_USART3}}, // GPIO_UART3_TX
    {GPIOC, {GPIO_PIN_11, GPIO_MODE_AF_PP,      GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_USART3}}, // GPIO_UART3_RX
    {GPIOA, {GPIO_PIN_1,  GPIO_MODE_AF_PP,      GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF4_USART4}}, // GPIO_UART4_TX
    {GPIOC, {GPIO_PIN_11, GPIO_MODE_AF_PP,      GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_USART4}}, // GPIO_UART4_RX

    {GPIOB, {GPIO_PIN_6,  GPIO_MODE_AF_OD,      GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_I2C1}},   // GPIO_I2C1_SCL
    {GPIOB, {GPIO_PIN_7,  GPIO_MODE_AF_OD,      GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_I2C1}},   // GPIO_I2C1_SDA
    {GPIOB, {GPIO_PIN_10, GPIO_MODE_AF_OD,      GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_I2C2}},   // GPIO_I2C2_SCL
    {GPIOB, {GPIO_PIN_11, GPIO_MODE_AF_OD,      GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH, GPIO_AF1_I2C2}},   // GPIO_I2C2_SDA

    {GPIOA, {GPIO_PIN_5,  GPIO_MODE_AF_PP,      GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI1}},   // GPIO_SPI1_SCK
    {GPIOB, {GPIO_PIN_4,  GPIO_MODE_AF_PP,      GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI1}},   // GPIO_SPI1_MISO
    {GPIOB, {GPIO_PIN_5,  GPIO_MODE_AF_PP,      GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI1}},   // GPIO_SPI1_MOSI
    {GPIOB, {GPIO_PIN_13, GPIO_MODE_AF_PP,      GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI2}},   // GPIO_SPI2_SCK
    {GPIOB, {GPIO_PIN_14, GPIO_MODE_AF_PP,      GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI2}},   // GPIO_SPI2_MISO
    {GPIOB, {GPIO_PIN_15, GPIO_MODE_AF_PP,      GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, GPIO_AF0_SPI2}},   // GPIO_SPI2_MOSI
};

///////////////////////////////// SPI definitions /////////////////////////////////
const SPI_InitTypeDefAndInst_t SPI_Inits[SPI_TOTAL_NUMBER] =
{
    // Instance, Mode, Direction, DataSize, CLKPolarity, CLKPhase, NSS, BaudRatePrescaler, FirstBit, TIMode, CRCCalculation, CRCPolynomial, CRCLength, NSSPMode
    {SPI1, {SPI_MODE_MASTER, SPI_DIRECTION_2LINES, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_NSS_SOFT, SPI_BAUDRATEPRESCALER_4, SPI_FIRSTBIT_MSB, SPI_TIMODE_DISABLE, SPI_CRCCALCULATION_DISABLE, 7, SPI_CRC_LENGTH_DATASIZE, SPI_NSS_PULSE_ENABLE}},
    {SPI2, {SPI_MODE_MASTER, SPI_DIRECTION_2LINES, SPI_DATASIZE_8BIT, SPI_POLARITY_LOW, SPI_PHASE_1EDGE, SPI_NSS_SOFT, SPI_BAUDRATEPRESCALER_4, SPI_FIRSTBIT_MSB, SPI_TIMODE_DISABLE, SPI_CRCCALCULATION_DISABLE, 7, SPI_CRC_LENGTH_DATASIZE, SPI_NSS_PULSE_ENABLE}},
};

///////////////////////////////// I2C definitions /////////////////////////////////
const I2C_InitTypeDefAndInst_t I2C_Inits[I2C_TOTAL_NUMBER] =
{
    // Instance, Timing, OwnAddress1, AddressingMode, DualAddressMode, OwnAddress2, OwnAddress2Masks, GeneralCallMode, NoStretchMode
    {I2C1, {0x2000090E, 0, I2C_ADDRESSINGMODE_7BIT, I2C_DUALADDRESS_DISABLE, 0, I2C_OA2_NOMASK, I2C_GENERALCALL_DISABLE, I2C_NOSTRETCH_DISABLE}},
    {I2C2, {0x2000090E, 0, I2C_ADDRESSINGMODE_7BIT, I2C_DUALADDRESS_DISABLE, 0, I2C_OA2_NOMASK, I2C_GENERALCALL_DISABLE, I2C_NOSTRETCH_DISABLE}},
};

///////////////////////////////// UART definitions ////////////////////////////////
const UART_InitTypeDefAndInst_t UART_Inits[UART_TOTAL_NUMBER] =
{
    // Instance,BaudRate,WordLength,StopBits,Parity,Mode,HwFlowCtl,OverSampling,OneBitSampling
    {USART1, {3000000, UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, UART_ONE_BIT_SAMPLE_DISABLE, UART_ADVFEATURE_NO_INIT}},
    {USART3, {9600,   UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, UART_ONE_BIT_SAMPLE_DISABLE, UART_ADVFEATURE_NO_INIT}},
    {USART4, {9600,   UART_WORDLENGTH_8B, UART_STOPBITS_1, UART_PARITY_NONE, UART_MODE_TX_RX, UART_HWCONTROL_NONE, UART_ONE_BIT_SAMPLE_DISABLE, UART_ADVFEATURE_NO_INIT}},
};