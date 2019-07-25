/* syshal_gpio.h - HAL for GPIO management
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

#ifndef _SYSHAL_GPIO_H_
#define _SYSHAL_GPIO_H_

#include <stdint.h>
#include <stdbool.h>

#define SYSHAL_GPIO_NO_ERROR (0)

int syshal_gpio_init(uint32_t pin);
void syshal_gpio_term(uint32_t pin);
void syshal_gpio_enable_interrupt(uint32_t pin, void (*callback_function)(void));
void syshal_gpio_disable_interrupt(uint32_t pin);
void syshal_gpio_set_output_low(uint32_t pin);
void syshal_gpio_set_output_high(uint32_t pin);
void syshal_gpio_set_output_toggle(uint32_t pin);
bool syshal_gpio_get_input(uint32_t pin);

#endif /* _SYSHAL_GPIO_H_ */