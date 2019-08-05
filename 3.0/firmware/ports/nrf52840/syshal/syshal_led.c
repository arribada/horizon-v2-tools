/* syshal_led.c - HAL for LED
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
#include "syshal_led.h"
#include "bsp.h"
#include "syshal_time.h"
#include "nrfx_timer.h"
#include "nrfx_pwm.h"
#include "app_util_platform.h" // CRITICAL_REGION def

#define SYSHAL_LED_ON (1)
#define SYSHAL_LED_OFF (0)

#define LED_BLINK_POOL_US   (1000) // Every 1 ms
#define TIMER_FREQ          (125000)
#define COUNT_1MS           (TIMER_FREQ / 1000)
#define TICKS_PER_OVERFLOW  (16777216) // 16777216 = 2 ^ 24
#define MAX_VALUE           (TICKS_PER_OVERFLOW / COUNT_1MS) // 134 seconds

#define UINT32_COLOUR_TO_UINT15_RED(x)   (((x >> 16) & 0xFF) << 7)
#define UINT32_COLOUR_TO_UINT15_GREEN(x) (((x >> 8) & 0xFF) << 7)
#define UINT32_COLOUR_TO_UINT15_BLUE(x)  ((x & 0xFF) << 7)

static enum
{
    SOLID,
    BLINK,
    SEQUENCE,
    OFF,
} current_type;

static volatile uint32_t current_colour = SYSHAL_LED_COLOUR_OFF;
static volatile uint8_t last_state;
static syshal_led_sequence_t current_sequence;
static nrf_pwm_values_individual_t pwm_values;

static inline void set_colour(uint32_t colour)
{
    if (!colour)
    {
        if (!nrfx_pwm_is_stopped(&PWM_Inits[PWM_LED].pwm))
        {
            nrfx_pwm_stop(&PWM_Inits[PWM_LED].pwm, true);
            // Set the pins to high impedance to reduce current draw
            nrf_gpio_cfg_default(GPIO_Inits[GPIO_LED_RED].pin_number);
            nrf_gpio_cfg_default(GPIO_Inits[GPIO_LED_GREEN].pin_number);
            nrf_gpio_cfg_default(GPIO_Inits[GPIO_LED_BLUE].pin_number);
        }
        return;
    }

    if (nrfx_pwm_is_stopped(&PWM_Inits[PWM_LED].pwm))
    {
        // Set our pins back to being outputs
        nrf_gpio_cfg_output(GPIO_Inits[GPIO_LED_RED].pin_number);
        nrf_gpio_cfg_output(GPIO_Inits[GPIO_LED_GREEN].pin_number);
        nrf_gpio_cfg_output(GPIO_Inits[GPIO_LED_BLUE].pin_number);
    }

    pwm_values.channel_0 = UINT32_COLOUR_TO_UINT15_RED(colour);
    pwm_values.channel_1 = UINT32_COLOUR_TO_UINT15_GREEN(colour);
    pwm_values.channel_2 = UINT32_COLOUR_TO_UINT15_BLUE(colour);

    nrf_pwm_sequence_t const pwm_sequence =
    {
        .values.p_individual = &pwm_values,
        .length              = NRF_PWM_VALUES_LENGTH(pwm_values),
        .repeats             = 0,
        .end_delay           = 0
    };

    nrfx_pwm_simple_playback(&PWM_Inits[PWM_LED].pwm, &pwm_sequence, 1, NRFX_PWM_FLAG_LOOP);
}

static void timer_evt_handler(nrf_timer_event_t event_type, void * p_context)
{
    switch (current_type)
    {
        case BLINK:
            if (last_state == SYSHAL_LED_ON)
            {
                last_state = SYSHAL_LED_OFF;
                set_colour(SYSHAL_LED_COLOUR_OFF);
            }
            else
            {
                last_state = SYSHAL_LED_ON;
                set_colour(current_colour);
            }
            break;

        case SEQUENCE:
            switch (current_sequence)
            {
                case RED_GREEN_BLUE:
                    switch (current_colour)
                    {
                        case SYSHAL_LED_COLOUR_RED:
                            current_colour = SYSHAL_LED_COLOUR_GREEN;
                            break;
                        case SYSHAL_LED_COLOUR_GREEN:
                            current_colour = SYSHAL_LED_COLOUR_BLUE;
                            break;
                        case SYSHAL_LED_COLOUR_BLUE:
                            current_colour = SYSHAL_LED_COLOUR_RED;
                            break;
                        default:
                            current_colour = SYSHAL_LED_COLOUR_RED;
                            break;
                    }
                    set_colour(current_colour);
                    break;
                default:
                    break;
            }
        default:
            break;
    }
}

int syshal_led_init(void)
{
    // Setup the LED pwm instance
    const nrfx_pwm_config_t pwm_config =
    {
        .output_pins  = {
            GPIO_Inits[GPIO_LED_RED].pin_number   | NRFX_PWM_PIN_INVERTED,
            GPIO_Inits[GPIO_LED_GREEN].pin_number | NRFX_PWM_PIN_INVERTED,
            GPIO_Inits[GPIO_LED_BLUE].pin_number  | NRFX_PWM_PIN_INVERTED,
            NRFX_PWM_PIN_NOT_USED,
        },
        .irq_priority = PWM_Inits[PWM_LED].irq_priority,
        .base_clock   = NRF_PWM_CLK_16MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 0x7FFF, // 15 bit counter
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO,
    };

    nrfx_pwm_init(&PWM_Inits[PWM_LED].pwm, &pwm_config, NULL);

    const nrfx_timer_config_t timer_config =
    {
        .frequency          = NRF_TIMER_FREQ_125kHz,
        .mode               = NRF_TIMER_MODE_TIMER,
        .bit_width          = NRF_TIMER_BIT_WIDTH_24,
        .interrupt_priority = TIMER_Inits[TIMER_UART_TIMEOUT].irq_priority,
        .p_context          = NULL
    };

    // Setup a timer to fire every LED_BLINK_POOL_US
    if (nrfx_timer_init(&TIMER_Inits[TIMER_LED].timer, &timer_config, timer_evt_handler) != NRFX_SUCCESS)
        return SYSHAL_LED_ERROR_INIT;

    return SYSHAL_LED_NO_ERROR;
}

int syshal_led_set_solid(uint32_t colour)
{
    if (nrfx_timer_is_enabled(&TIMER_Inits[TIMER_LED].timer))
        nrfx_timer_disable(&TIMER_Inits[TIMER_LED].timer);

    current_colour = colour;
    current_type = SOLID;
    set_colour(colour);
    return SYSHAL_LED_NO_ERROR;
}

int syshal_led_set_blinking(uint32_t colour, uint32_t time_ms)
{
    if (nrfx_timer_is_enabled(&TIMER_Inits[TIMER_LED].timer))
        nrfx_timer_disable(&TIMER_Inits[TIMER_LED].timer);

    current_colour = colour;
    current_type = BLINK;
    uint32_t counter_value;

    if (time_ms > MAX_VALUE)
        counter_value = MAX_VALUE;
    else
        counter_value = COUNT_1MS * time_ms;

    nrfx_timer_extended_compare(&TIMER_Inits[TIMER_LED].timer, NRF_TIMER_CC_CHANNEL0, counter_value, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrfx_timer_clear(&TIMER_Inits[TIMER_LED].timer);
    last_state = SYSHAL_LED_ON;
    set_colour(colour);

    nrfx_timer_enable(&TIMER_Inits[TIMER_LED].timer);

    return SYSHAL_LED_NO_ERROR;
}

int syshal_led_get(uint32_t * colour, bool * is_blinking)
{
    if (current_type == OFF)
        return SYSHAL_LED_ERROR_LED_OFF;

    if (*colour)
        *colour = current_colour;

    if (is_blinking)
        *is_blinking = (current_type == BLINK);

    return SYSHAL_LED_NO_ERROR;
}

int syshal_led_set_sequence(syshal_led_sequence_t sequence, uint32_t time_ms)
{
    if (nrfx_timer_is_enabled(&TIMER_Inits[TIMER_LED].timer))
        nrfx_timer_disable(&TIMER_Inits[TIMER_LED].timer);

    if (time_ms > MAX_VALUE)
        return SYSHAL_LED_ERROR_COUNT_OVERFLOW;

    switch (sequence)
    {
        case RED_GREEN_BLUE:
            current_type = SEQUENCE;
            current_sequence = RED_GREEN_BLUE;
            current_colour = SYSHAL_LED_COLOUR_RED;
            set_colour(current_colour);

            nrfx_timer_clear(&TIMER_Inits[TIMER_LED].timer);
            nrfx_timer_extended_compare(&TIMER_Inits[TIMER_LED].timer, NRF_TIMER_CC_CHANNEL0, COUNT_1MS * time_ms, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
            nrfx_timer_enable(&TIMER_Inits[TIMER_LED].timer);
            break;

        default:
            return SYSHAL_LED_ERROR_SEQUENCE_NOT_DEFINE;
            break;
    }

    return SYSHAL_LED_NO_ERROR;
}

int syshal_led_off(void)
{
    if (nrfx_timer_is_enabled(&TIMER_Inits[TIMER_LED].timer))
        nrfx_timer_disable(&TIMER_Inits[TIMER_LED].timer);

    current_type = OFF;
    set_colour(SYSHAL_LED_COLOUR_OFF);

    return SYSHAL_LED_NO_ERROR;
}

bool syshal_led_is_active(void)
{
    return (current_type != OFF);
}