/*
 * Copyright 2018 Schuyler St. Leger
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

#include "operacake_rotator.h"
#include "sct.h"

#include <hackrf_core.h>

#include <stddef.h>

#include <libopencm3/lpc43xx/sgpio.h>
#include <libopencm3/lpc43xx/rgu.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/gima.h>


#undef GREATFET_OPERACAKE

#define SGPIO_CLK_PRESCALE

#ifdef GREATFET_OPERACAKE
#define U1CTRL_SET SCT_OUT5_SET
#define U1CTRL_CLR SCT_OUT5_CLR
#define U1CTRL_RES_TOGGLE SCT_RES_O5RES_TOGGLE_OUTPUT
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
#define U1CTRL_RES_TOGGLE SCT_RES_O14RES_TOGGLE_OUTPUT
#define U2CTRL0_SET SCT_OUT13_SET
#define U2CTRL0_CLR SCT_OUT13_CLR
#define U2CTRL1_SET SCT_OUT12_SET
#define U2CTRL1_CLR SCT_OUT12_CLR
#define U3CTRL0_SET SCT_OUT11_SET
#define U3CTRL0_CLR SCT_OUT11_CLR
#define U3CTRL1_SET SCT_OUT8_SET
#define U3CTRL1_CLR SCT_OUT8_CLR
#endif


/**
 * Configure the SCTimer to rotate the antennas with the Operacake in phase with
 * the sample clock. This will configure the SCTimer to output to the pins for
 * GPIO control of the Operacake, however the Operacake must be configured for
 * GPIO control, by disabling the I2C IO expander (done with PA=0 PB=0). The 
 * timing is not set in this function.
 *
 * To trigger the antenna switching synchronously with the sample clock, the
 * SGPIO is configured to output its clock (f=2 * sample clock) to the SCTimer.
 */
void rotator_init() {
	// We start by resetting the SCTimer
	RESET_CTRL1 = RESET_CTRL1_SCT_RST;

	// Delay to allow reset sigal to be processed
	// The reset generator uses a 12MHz clock (IRC)
	// The difference between this and the core clock (CCLK)
	// determines this value (CCLK/IRC).
	// The value used here is a bit shorter than would be required, since
	// there are additional instructions that fill the time. If the duration of
	// the actions from here to the first access to the SCTimer is changed, then
	// this delay may need to be increased.
	delay(8);

	// Pin definitions for the GreatFET
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

	// Pin definitions for the HackRF
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


	// Configure the SGPIO to output the clock (f=2 * sample clock) on pin 12
	SGPIO_OUT_MUX_CFG12 =
		SGPIO_OUT_MUX_CFG_P_OUT_CFG(0x08) | // clkout output mode
		SGPIO_OUT_MUX_CFG_P_OE_CFG(0);      // gpio_oe
	SGPIO_GPIO_OENREG |= BIT12;

	// Use the GIMA to connect the SGPIO clock to the SCTimer
	GIMA_CTIN_1_IN = 0x2 << 4;  // Route SGPIO12 to SCTIN1

	// We configure this register first, because the user manual says to
	SCT_CONFIG |= SCT_CONFIG_UNIFY_32_BIT
#ifdef SGPIO_CLK_PRESCALE
	| SCT_CONFIG_CLKMODE_PRESCALED_BUS_CLOCK
	| SCT_CONFIG_CKSEL_RISING_EDGES_ON_INPUT_1
#endif
;

	// Halt the SCTimer [to enable it to be configured?]
	SCT_CTRL = SCT_CTRL_HALT_L(1);

	// Prescaler
	SCT_CTRL |= SCT_CTRL_PRE_L(0);

	// Event 0 triggered by MATCH0. Event 1 triggered by MATCH1.
	SCT_EV0_CTRL = SCT_EVn_CTRL_COMBMODE_MATCH | SCT_EVn_CTRL_MATCHSEL(0);
	SCT_EV1_CTRL = SCT_EVn_CTRL_COMBMODE_MATCH | SCT_EVn_CTRL_MATCHSEL(1);
	SCT_EV2_CTRL = SCT_EVn_CTRL_COMBMODE_MATCH | SCT_EVn_CTRL_MATCHSEL(2);
	SCT_EV3_CTRL = SCT_EVn_CTRL_COMBMODE_MATCH | SCT_EVn_CTRL_MATCHSEL(3);

	// Events 0, 2 set outputs 3, 2. Events 1, 3 clear outputs 3, 2.
	U2CTRL0_SET = SCT_OUTn_SET_SET0(1) | SCT_OUTn_SET_SET2(1);
	U2CTRL0_CLR = SCT_OUTn_CLR_CLR1(1) | SCT_OUTn_CLR_CLR3(1);

	U3CTRL0_SET = SCT_OUTn_SET_SET0(1) | SCT_OUTn_SET_SET2(1);
	U3CTRL0_CLR = SCT_OUTn_CLR_CLR1(1) | SCT_OUTn_CLR_CLR3(1);


	// Events 1, 2 set outputs 4, 0. Events 0, 3 clear outputs 4, 0.
	U2CTRL1_SET = SCT_OUTn_SET_SET1(1) | SCT_OUTn_SET_SET2(1);
	U2CTRL1_CLR = SCT_OUTn_CLR_CLR0(1) | SCT_OUTn_CLR_CLR3(1);

	U3CTRL1_SET = SCT_OUTn_SET_SET1(1) | SCT_OUTn_SET_SET2(1);
	U3CTRL1_CLR = SCT_OUTn_CLR_CLR0(1) | SCT_OUTn_CLR_CLR3(1);

	// Toggle on confict for U1CTRL output. Used for multisided operation.
	SCT_RES = U1CTRL_RES_TOGGLE;

	// Events 0-3 can occur in state 0
	SCT_EV0_STATE = SCT_EVn_STATE_STATEMSK0(1);
	SCT_EV1_STATE = SCT_EVn_STATE_STATEMSK0(1);
	SCT_EV2_STATE = SCT_EVn_STATE_STATEMSK0(1);
	SCT_EV3_STATE = SCT_EVn_STATE_STATEMSK0(1);

	// Event 3 resets the counter to 0
	SCT_LIMIT = BIT3;

	// Reset state
	SCT_STATE = 0;

	// Enable the SCTimer
	SCT_CTRL &= ~SCT_CTRL_HALT_L(1);
}

void operacake_rotator_set_timing(uint32_t dwell_time) {
	operacake_rotator_set_antenna_timings(
		dwell_time,
		dwell_time * 2,
		dwell_time * 3,
		dwell_time * 4);
}

/**
 * Dwell time in samples for each antenna port. The values are not cumulative,
 * so to obtain an even dwell time t on each antenna, use
 * operacake_rotator_set_timing(uint32_t dwell_time) or these values
 * ant1_dwell = t, ant2_dwell = 2*t, ant3_dwell = 3*t, and ant4_dwell = 4*t
 * 
 * @param ant1_dwell Dwell time for PA1 and PB1
 * @param ant2_dwell Dwell time for PA2 and PB2
 * @param ant3_dwell Dwell time for PA3 and PB3
 * @param ant4_dwell Dwell time for PA4 and PB4
 */
void operacake_rotator_set_antenna_timings(uint32_t ant1_dwell, uint32_t ant2_dwell,
	uint32_t ant3_dwell, uint32_t ant4_dwell) {
	SCT_MATCHREL0 = ant1_dwell - 1;
	SCT_MATCHREL1 = ant2_dwell - 1;
	SCT_MATCHREL2 = ant3_dwell - 1;
	SCT_MATCHREL3 = ant4_dwell - 1;
}

static bool default_sameside = true;
void operacake_rotator_set_default_sameside(bool sameside) {
	default_sameside = sameside;
}

void operacake_rotator_rotate_crossover_enable(bool rot_crossover_en) {
	if (rot_crossover_en) {
		U1CTRL_SET = SCT_OUTn_SET_SET3(1);
		U1CTRL_CLR = SCT_OUTn_CLR_CLR3(1);
	} else {
		U1CTRL_SET = 0;
		U1CTRL_CLR = 0;
		// Set the crossover switch to the correct value, since it may not be in the
		// correct state from the SCTimer events.
		SCT_OUTPUT = default_sameside ? BIT14 : 0;
	}
}

void operacake_rotator_stop() {
	// Halt timer
	SCT_CTRL |= SCT_CTRL_HALT_L(1);
}

/**
 * This will reset the state of the SCTimer, but retains its configuration.
 * This reset will return the selected antenna to 1 and samesided. This is
 * called by sgpio_cpld_stream_disable so the HackRF starts capturing with the
 * same antenna selected each time (ignoring USB sample losses).
 */
void operacake_rotator_reset_state() {
	// Halt the counter to permit resetting the state and clear the counter value
	SCT_CTRL |= SCT_CTRL_HALT_L(1) | SCT_CTRL_CLRCTR_L(1);

	// Reset the state
	SCT_STATE = 0;
	SCT_OUTPUT = default_sameside ? BIT14 : 0;

	// Reenable the SCTimer to ready it for the next capture
	SCT_CTRL &= ~(SCT_CTRL_HALT_L(1) | SCT_CTRL_CLRCTR_L(1));
}
