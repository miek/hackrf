/*
 * Copyright 2016 Schuyler St. Leger
 *
 * This file is part of GreatFET.
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

#include "usb_api_counter.h"
#include "usb_queue.h"

#include <operacake_rotator.h>
#include <hackrf_core.h>

#include <stddef.h>

#include <libopencm3/lpc43xx/sgpio.h>
#include <libopencm3/lpc43xx/rgu.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/gima.h>


#undef GREATFET_OPERACAKE

usb_request_status_t usb_vendor_request_counter_start(
		usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage) {
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		rotator_init();

		// Value is the delay for each antenna
		if (endpoint->setup.value) {
			operacake_rotator_set_timing(endpoint->setup.value);
		}

		// Index specifies if same side should be enabled
		if(endpoint->setup.index) {
			operacake_rotator_rotate_crossover_enable(true);
		} else {
			operacake_rotator_rotate_crossover_enable(false);
		}

		usb_transfer_schedule_ack(endpoint->in);
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_counter_stop(
		usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage) {
	if (stage == USB_TRANSFER_STAGE_SETUP) {

		// value == 1 to enable sameside, value == 0 to do nothing, and
		// value != 1 && value != 0 to disable sameside
		// Call reset_state after setting side to update output.
		if (endpoint->setup.value == 1) {
			operacake_rotator_set_default_sameside(true);
			operacake_rotator_reset_state();
		} else if (endpoint->setup.value != 0) {
			operacake_rotator_set_default_sameside(false);
			operacake_rotator_reset_state();
		}

		operacake_rotator_stop();
		usb_transfer_schedule_ack(endpoint->in);
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_counter_set(
	usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage) {
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		// Value is the delay for each antenna
		if (endpoint->setup.value) {
			operacake_rotator_set_timing(endpoint->setup.value);
		}

		// Index specifies if same side should be enabled
		if(endpoint->setup.index) {
			operacake_rotator_rotate_crossover_enable(true);
		} else {
			operacake_rotator_rotate_crossover_enable(false);
		}

		usb_transfer_schedule_ack(endpoint->in);
	}
	return USB_REQUEST_STATUS_OK;
}
