/*
 * Copyright (C) 2022  Brendan Doherty (2bndy5)
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
/**
 * @file sdk_to_arduino_core_bridge.h
 * 
 * This file is only meant to provide a bridge between functionality specific to 
 * the nRF5 SDK that is not present in the nRF52 Arduino core's resources (as provided
 * by Adafruit).
 */ 
#include <nrf_error.h>
#include <verify.h>

#define VERIFY_TRUE(cond, err_code) \
    if (!(cond))                    \
        return (err_code);

#define VERIFY_FALSE(cond, err) \
    if ((cond))                 \
        return (err);

#define VERIFY_PARAM_NOT_NULL(param) \
    if ((param) == NULL)             \
        return NRF_ERROR_NULL;

#define STATIC_ASSERT VERIFY_STATIC
