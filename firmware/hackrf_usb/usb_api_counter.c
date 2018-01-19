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
#include "sct.h"

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

		RESET_CTRL1 = RESET_CTRL1_SCT_RST;

		// Delay to allow reset sigal to be processed
		// The reset generator uses a 12MHz clock (IRC)
		// The difference between this and the core clock (CCLK)
		// determines this value (CCLK/IRC).
		// The value used here is a bit shorter than would be required, since
		// there are additional commands that fill the time
		delay(8);

		#ifdef GREATFET_OPERACAKE
		// U2CTRL0
		scu_pinmux(P4_3, SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_EHS_FAST | SCU_CONF_FUNCTION1);
		// U2CTRL1
		scu_pinmux(P4_6, SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_EHS_FAST | SCU_CONF_FUNCTION1);
		// U3CTRL0
		scu_pinmux(P4_4, SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_EHS_FAST | SCU_CONF_FUNCTION1);
		// U3CTRL1
		scu_pinmux(P4_2, SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_EHS_FAST | SCU_CONF_FUNCTION1);
		// U1CTRL
		scu_pinmux(P4_5, SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_EHS_FAST | SCU_CONF_FUNCTION1);
		#endif

		#ifndef GREATFET_OPERACAKE
		// U2CTRL0
		scu_pinmux(P7_4, SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_EHS_FAST | SCU_CONF_FUNCTION1);
		// U2CTRL1
		scu_pinmux(P7_5, SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_EHS_FAST | SCU_CONF_FUNCTION1);
		// U3CTRL0
		scu_pinmux(P7_6, SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_EHS_FAST | SCU_CONF_FUNCTION1);
		// U3CTRL1
		scu_pinmux(P7_7, SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_EHS_FAST | SCU_CONF_FUNCTION1);
		// U1CTRL
		scu_pinmux(P7_0, SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_EHS_FAST | SCU_CONF_FUNCTION1);
		#endif


		/* SGPIO bits ------------- */
		SGPIO_OUT_MUX_CFG12 =
			SGPIO_OUT_MUX_CFG_P_OUT_CFG(0x08) | // clkout output mode
			SGPIO_OUT_MUX_CFG_P_OE_CFG(0);      // gpio_oe

		SGPIO_GPIO_OENREG |= BIT12;


		// GIMA Configuration
		GIMA_CTIN_1_IN = 0x2 << 4;  // Route SGPIO12 to SCTIN1


#define SGPIO_CLK_PRESCALE

		SCT_CONFIG |= SCT_CONFIG_UNIFY_32_BIT
#ifdef SGPIO_CLK_PRESCALE
		| SCT_CONFIG_CLKMODE_PRESCALED_BUS_CLOCK
		| SCT_CONFIG_CKSEL_RISING_EDGES_ON_INPUT_1
#endif
;

		SCT_CTRL = SCT_CTRL_HALT_L(1);

		// Prescaler
		SCT_CTRL |= SCT_CTRL_PRE_L(0);

		// Maximum counter value
		if (endpoint->setup.value) {
			SCT_MATCH3 = endpoint->setup.value;
			SCT_MATCHREL3 = endpoint->setup.value;
		} else {
			SCT_MATCH3 = 1024 * 4 - 1;
			SCT_MATCHREL3 = 1024 * 4 - 1;
		}

		// Dwell time
		if (endpoint->setup.index) {
			uint32_t dwell_time = endpoint->setup.index;
			SCT_MATCH0 = dwell_time - 1;
			SCT_MATCHREL0 = dwell_time - 1;
			SCT_MATCH1 = dwell_time * 2 - 1;
			SCT_MATCHREL1 = dwell_time * 2 - 1;
			SCT_MATCH2 = dwell_time * 3 - 1;
			SCT_MATCHREL2 = dwell_time * 3 - 1;
		} else {
			uint32_t dwell_time = 1024 - 1;
			SCT_MATCH0 = dwell_time - 1;
			SCT_MATCHREL0 = dwell_time - 1;
			SCT_MATCH1 = dwell_time * 2 - 1;
			SCT_MATCHREL1 = dwell_time * 2 - 1;
			SCT_MATCH2 = dwell_time * 3 - 1;
			SCT_MATCHREL2 = dwell_time * 3 - 1;
		}

		// Event 0 triggered by MATCH0. Event 1 triggered by MATCH1.
		SCT_EV0_CTRL = SCT_EV0_CTRL_COMBMODE_MATCH | SCT_EV0_CTRL_MATCHSEL(0);
		SCT_EV1_CTRL = SCT_EV1_CTRL_COMBMODE_MATCH | SCT_EV1_CTRL_MATCHSEL(1);
		SCT_EV2_CTRL = SCT_EV2_CTRL_COMBMODE_MATCH | SCT_EV2_CTRL_MATCHSEL(2);
		SCT_EV3_CTRL = SCT_EV3_CTRL_COMBMODE_MATCH | SCT_EV3_CTRL_MATCHSEL(3);

		#ifdef GREATFET_OPERACAKE
		#define U1CTRL_SET SCT_OUT5_SET
		#define U1CTRL_CLR SCT_OUT5_CLR
		#define U2CTRL0_SET SCT_OUT3_SET
		#define U2CTRL0_CLR SCT_OUT3_CLR
		#define U2CTRL1_SET SCT_OUT4_SET
		#define U2CTRL1_CLR SCT_OUT4_CLR
		#define U3CTRL0_SET SCT_OUT2_SET
		#define U3CTRL0_CLR SCT_OUT2_CLR
		#define U3CTRL1_SET SCT_OUT0_SET
		#define U3CTRL1_CLR SCT_OUT0_CLR
		#endif

		#ifndef GREATFET_OPERACAKE
		#define U1CTRL_SET SCT_OUT14_SET
		#define U1CTRL_CLR SCT_OUT14_CLR
		#define U2CTRL0_SET SCT_OUT13_SET
		#define U2CTRL0_CLR SCT_OUT13_CLR
		#define U2CTRL1_SET SCT_OUT12_SET
		#define U2CTRL1_CLR SCT_OUT12_CLR
		#define U3CTRL0_SET SCT_OUT11_SET
		#define U3CTRL0_CLR SCT_OUT11_CLR
		#define U3CTRL1_SET SCT_OUT8_SET
		#define U3CTRL1_CLR SCT_OUT8_CLR
		#endif

		// Event 0, 2 sets output 3, 2. Event 1, 3 clears output 3, 2.
		U2CTRL0_SET = SCT_OUT13_SET_SET0(1) | SCT_OUT13_SET_SET2(1);
		U2CTRL0_CLR = SCT_OUT13_CLR_CLR1(1) | SCT_OUT13_CLR_CLR3(1);

		U3CTRL0_SET = SCT_OUT11_SET_SET0(1) | SCT_OUT11_SET_SET2(1);
		U3CTRL0_CLR = SCT_OUT11_CLR_CLR1(1) | SCT_OUT11_CLR_CLR3(1);


		// Event 1, 2 sets output 4, 0. Event 0, 3 clears output 4, 0.
		U2CTRL1_SET = SCT_OUT12_SET_SET1(1) | SCT_OUT12_SET_SET2(1);
		U2CTRL1_CLR = SCT_OUT12_CLR_CLR0(1) | SCT_OUT12_CLR_CLR3(1);

		U3CTRL1_SET = SCT_OUT8_SET_SET1(1) | SCT_OUT8_SET_SET2(1);
		U3CTRL1_CLR = SCT_OUT8_CLR_CLR0(1) | SCT_OUT8_CLR_CLR3(1);

		// Event 3 toggles output 5.
		#define ONESIDE // Use only 4 antennas, and stay on the same side; comment for 8 antennas
#ifndef ONESIDE
		U1CTRL_SET = SCT_OUT14_SET_SET3(1);
		U1CTRL_CLR = SCT_OUT14_CLR_CLR3(1);
		SCT_RES = SCT_RES_O14RES_TOGGLE_OUTPUT;
#else
		SCT_OUTPUT = BIT14; // Set this to 0 to have A and B crossed statically
#endif


		// Events 0:3 can occur in state 0
		SCT_EV0_STATE = SCT_EV0_STATE_STATEMSK0(1);
		SCT_EV1_STATE = SCT_EV1_STATE_STATEMSK0(1);
		SCT_EV2_STATE = SCT_EV2_STATE_STATEMSK0(1);
		SCT_EV3_STATE = SCT_EV3_STATE_STATEMSK0(1);

		// Event 3 resets the counter to 0
		SCT_LIMIT = BIT3;
		
		// Begin starting
		SCT_STATE = 0;
		
		SCT_CTRL &= ~(SCT_CTRL_STOP_L(1) | SCT_CTRL_HALT_L(1));

		usb_transfer_schedule_ack(endpoint->in);
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_counter_stop(
		usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage) {
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint16_t index = endpoint->setup.index;
		if (index > 0) {
			if (index & BIT0) {
				SGPIO_OUT_MUX_CFG12 =
				  SGPIO_OUT_MUX_CFG_P_OUT_CFG(0x00) | // 1-bit output mode
				  SGPIO_OUT_MUX_CFG_P_OE_CFG(0);      // gpio_oe
			} else {
				SGPIO_OUT_MUX_CFG12 =
				  SGPIO_OUT_MUX_CFG_P_OUT_CFG(0x08) | // clkout output mode
				  SGPIO_OUT_MUX_CFG_P_OE_CFG(0);      // gpio_oe
			}
		} else {
			// Stop timer
			SCT_CTRL |= SCT_CTRL_HALT_L(1);
		}

		usb_transfer_schedule_ack(endpoint->in);
	}
	return USB_REQUEST_STATUS_OK;
}

/* wValue = Freq; wIndex = Duty Cycle */
usb_request_status_t usb_vendor_request_counter_set(
	usb_endpoint_t* const endpoint, const usb_transfer_stage_t stage) {
	if (stage == USB_TRANSFER_STAGE_SETUP) {

		// SCT_CTRL |= SCT_CTRL_HALT_L(1) | SCT_CTRL_CLRCTR_L(1);

		// Maximum counter value
		if (endpoint->setup.value) {
			SCT_MATCHREL3 = endpoint->setup.value;
		}

		// Dwell time
		if (endpoint->setup.index) {
			uint32_t dwell_time = endpoint->setup.index;
			SCT_MATCHREL0 = dwell_time - 1;
			SCT_MATCHREL1 = dwell_time * 2 - 1;
			SCT_MATCHREL2 = dwell_time * 3 - 1;
		}

		// SCT_STATE = 0;

		// SCT_CTRL &= ~(SCT_CTRL_STOP_L(1) | SCT_CTRL_HALT_L(1));

		usb_transfer_schedule_ack(endpoint->in);
	}
	return USB_REQUEST_STATUS_OK;
}

void reset_operacake_counter_state() {
	SCT_CTRL |= SCT_CTRL_HALT_L(1) | SCT_CTRL_CLRCTR_L(1);
	SCT_STATE = 0;
	SCT_OUTPUT = BIT14;
	SCT_CTRL &= ~(SCT_CTRL_HALT_L(1) | SCT_CTRL_CLRCTR_L(1));
}
