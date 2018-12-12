/*
 * Copyright 2016 Dominic Spill <dominicgs@gmail.com>
 *
 * This file is part of HackRF.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#ifndef __OPERACAKE_ROTATOR_H
#define __OPERACAKE_ROTATOR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

void rotator_init();
void operacake_rotator_set_timing(uint32_t dwell_time);
void operacake_rotator_set_antenna_timings(uint32_t ant1_dwell, uint32_t ant2_dwell,
	uint32_t ant3_dwell, uint32_t ant4_dwell);
void operacake_rotator_set_default_sameside(bool sameside);
void operacake_rotator_rotate_crossover_enable(bool rot_crossover_en);
void operacake_rotator_stop();
void operacake_rotator_reset_state();


#ifdef __cplusplus
}
#endif

#endif /* __OPERACAKE_ROTATOR_H */
