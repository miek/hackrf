/*
 * Copyright 2012 Jared Boone
 * Copyright 2013 Benjamin Vernoux
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

#include <gpdma.h>
#include <streaming.h>

#include <libopencm3/lpc43xx/m4/nvic.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/gpdma.h>
#include <libopencm3/lpc43xx/rgu.h>
#include <libopencm3/lpc43xx/sgpio.h>

// TODO: probably shouldn't include this here
#include "../hackrf_usb/usb_bulk_buffer.h"

#define BUFFER_COUNT 2
#define BUFFER_SIZE 0x4000
#define TRANSFER_SIZE 0x800
#define TRANSFER_SIZE_BYTES (TRANSFER_SIZE * 4)
#define TRANSFER_COUNT (BUFFER_SIZE / TRANSFER_SIZE_BYTES)
#define LLI_COUNT (BUFFER_COUNT * TRANSFER_COUNT)

gpdma_lli_t lli_loop[LLI_COUNT];

#define SGPIO_GPDMA_CHANNEL 0

static uint32_t cconfig(bool transmit) {
	return \
		GPDMA_CCONFIG_E(0) |\
		GPDMA_CCONFIG_SRCPERIPHERAL(0) |\
		GPDMA_CCONFIG_DESTPERIPHERAL(0) |\
		GPDMA_CCONFIG_FLOWCNTRL(transmit ? 0x01 : 0x02) |\
		GPDMA_CCONFIG_IE(1) |\
		GPDMA_CCONFIG_ITC(1) |\
		GPDMA_CCONFIG_L(0) |\
		GPDMA_CCONFIG_A(0) |\
		GPDMA_CCONFIG_H(0);
}

static uint32_t ccontrol(bool transmit, bool generate_interrupt) {
	return \
		GPDMA_CCONTROL_TRANSFERSIZE(TRANSFER_SIZE) |\
		GPDMA_CCONTROL_SBSIZE(0) |\
		GPDMA_CCONTROL_DBSIZE(0) |\
		GPDMA_CCONTROL_SWIDTH(2) |\
		GPDMA_CCONTROL_DWIDTH(2) |\
		GPDMA_CCONTROL_S(transmit ? 1 : 0) |\
		GPDMA_CCONTROL_D(transmit ? 0 : 1) |\
		GPDMA_CCONTROL_SI(transmit ? 1 : 0) |\
		GPDMA_CCONTROL_DI(transmit ? 0 : 1) |\
		GPDMA_CCONTROL_PROT1(0) |\
		GPDMA_CCONTROL_PROT2(0) |\
		GPDMA_CCONTROL_PROT3(0) |\
		GPDMA_CCONTROL_I(generate_interrupt);
}

static void lli_setup() {
	for (int buffer_index = 0; buffer_index < BUFFER_COUNT; buffer_index++) {
		for (int transfer_index = 0; transfer_index < TRANSFER_COUNT; transfer_index++) {
			gpdma_lli_t* lli = &lli_loop[buffer_index * BUFFER_COUNT + transfer_index];
			lli->csrcaddr = (uint32_t*)&SGPIO_REG_SS0;
			lli->cdestaddr = &usb_bulk_buffer[buffer_index * BUFFER_SIZE + transfer_index * TRANSFER_SIZE_BYTES];
			lli->ccontrol = ccontrol(0, transfer_index == (TRANSFER_COUNT-1));
		}
	}
}

void baseband_streaming_enable(sgpio_config_t* const sgpio_config) {
	nvic_set_priority(NVIC_DMA_IRQ, 0);
	nvic_enable_irq(NVIC_DMA_IRQ);
	CREG_DMAMUX = CREG_DMAMUX_DMAMUXPER0(0x2); // DMA peripheral0 = SGPIO14
	RESET_CTRL0 = RESET_CTRL0_DMA_RST;
	while ((RESET_ACTIVE_STATUS0 & RESET_ACTIVE_STATUS0_DMA_RST) == 0) {}
	GPDMA_INTERRCLR = 0xff;
	GPDMA_INTTCCLEAR = 0xff;
	gpdma_controller_enable();
	gpdma_lli_create_loop(lli_loop, LLI_COUNT);
	lli_setup();

	GPDMA_CSRCADDR(SGPIO_GPDMA_CHANNEL)  = (uint32_t)lli_loop[0].csrcaddr;
	GPDMA_CDESTADDR(SGPIO_GPDMA_CHANNEL) = (uint32_t)lli_loop[0].cdestaddr;
	GPDMA_CLLI(SGPIO_GPDMA_CHANNEL)      = lli_loop[0].clli;
	GPDMA_CCONTROL(SGPIO_GPDMA_CHANNEL)  = lli_loop[0].ccontrol;

	GPDMA_CCONFIG(SGPIO_GPDMA_CHANNEL) = cconfig(0);
	gpdma_channel_enable(SGPIO_GPDMA_CHANNEL);

	SGPIO_SET_EN_1 = (1 << SGPIO_SLICE_A);

	sgpio_cpld_stream_enable(sgpio_config);
}

void baseband_streaming_disable(sgpio_config_t* const sgpio_config) {
	sgpio_cpld_stream_disable(sgpio_config);
}
