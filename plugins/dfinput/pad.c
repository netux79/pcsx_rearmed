/*
 * Copyright (c) 2009, Wei Mingzhi <whistler@openoffice.org>.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses>.
 *
 * this is only pure emulation code to handle analogs,
 * extracted from dfinput.
 */

#include <stdint.h>
#include <string.h>

#include "psemu_plugin_defs.h"
#include "externals.h"

enum {
	ANALOG_LEFT = 0,
	ANALOG_RIGHT,

	ANALOG_TOTAL
};

enum {
	CMD_READ_DATA_AND_VIBRATE = 0x42,
	CMD_CONFIG_MODE = 0x43,
	CMD_SET_MODE_AND_LOCK = 0x44,
	CMD_QUERY_MODEL_AND_MODE = 0x45,
	CMD_QUERY_ACT = 0x46, // ??
	CMD_QUERY_COMB = 0x47, // ??
	CMD_QUERY_MODE = 0x4C, // QUERY_MODE ??
	CMD_VIBRATION_TOGGLE = 0x4D,
};

static struct {
	uint8_t PadMode;
	uint8_t PadID;
	uint8_t ConfigMode;
	PadDataS pad;
} padstate[2];

static uint8_t stdpar[2][8] = {
	{0xFF, 0x5A, 0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80},
	{0xFF, 0x5A, 0xFF, 0xFF, 0x80, 0x80, 0x80, 0x80}
};

static uint8_t unk46[2][8] = {
	{0xFF, 0x5A, 0x00, 0x00, 0x01, 0x02, 0x00, 0x0A},
	{0xFF, 0x5A, 0x00, 0x00, 0x01, 0x02, 0x00, 0x0A}
};

static uint8_t unk47[2][8] = {
	{0xFF, 0x5A, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00},
	{0xFF, 0x5A, 0x00, 0x00, 0x02, 0x00, 0x01, 0x00}
};

static uint8_t unk4c[2][8] = {
	{0xFF, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

static uint8_t unk4d[2][8] = { 
	{0xFF, 0x5A, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
	{0xFF, 0x5A, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}
};

static uint8_t stdcfg[2][8]   = { 
	{0xFF, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

static uint8_t stdmode[2][8]  = { 
	{0xFF, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
	{0xFF, 0x5A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

static uint8_t stdmodel[2][8] = { 
	{0xFF,
	 0x5A,
	 0x01, // 03 - dualshock2, 01 - dualshock
	 0x02, // number of modes
	 0x01, // current mode: 01 - analog, 00 - digital
	 0x02,
	 0x01,
	 0x00},
	{0xFF, 
	 0x5A,
	 0x01, // 03 - dualshock2, 01 - dualshock
	 0x02, // number of modes
	 0x01, // current mode: 01 - analog, 00 - digital
	 0x02,
	 0x01,
	 0x00}
};

static uint8_t *pad_buf;
static unsigned char gcon_buf[8];
unsigned char CurPad, CurByte, CurCmd, CmdLen;

/* get button state and pad type from main emu */
extern void *PAD1_startPoll, *PAD1_poll;
extern void *PAD2_startPoll, *PAD2_poll;
extern long (*PAD1_readPort1)(PadDataS *pad);
extern long (*PAD2_readPort2)(PadDataS *pad);

static uint8_t do_cmd(void)
{
	PadDataS *pad = &padstate[CurPad].pad;
	int pad_num = CurPad;

	CmdLen = 8;
	switch (CurCmd) {
		case CMD_SET_MODE_AND_LOCK:
			pad_buf = stdmode[pad_num];
			return 0xF3;

		case CMD_QUERY_MODEL_AND_MODE:
			pad_buf = stdmodel[pad_num];
			pad_buf[4] = padstate[pad_num].PadMode;
			return 0xF3;

		case CMD_QUERY_ACT:
			pad_buf = unk46[pad_num];
			return 0xF3;

		case CMD_QUERY_COMB:
			pad_buf = unk47[pad_num];
			return 0xF3;

		case CMD_QUERY_MODE:
			pad_buf = unk4c[pad_num];
			return 0xF3;

		case CMD_VIBRATION_TOGGLE:
			pad_buf = unk4d[pad_num];
			return 0xF3;

		case CMD_CONFIG_MODE:
			if (padstate[pad_num].ConfigMode) {
				pad_buf = stdcfg[pad_num];
				return 0xF3;
			}
			// else FALLTHROUGH

		case CMD_READ_DATA_AND_VIBRATE:
		default:
			pad_buf = stdpar[pad_num];

			pad_buf[2] = pad->buttonStatus;
			pad_buf[3] = pad->buttonStatus >> 8;

			if (padstate[pad_num].PadMode == 1) {
				pad_buf[4] = pad->rightJoyX;
				pad_buf[5] = pad->rightJoyY;
				pad_buf[6] = pad->leftJoyX;
				pad_buf[7] = pad->leftJoyY;
			} else {
				CmdLen = 4;
			}

			return padstate[pad_num].PadID;
	}
}

static void do_cmd2(unsigned char value)
{
	switch (CurCmd) {
		case CMD_CONFIG_MODE:
			padstate[CurPad].ConfigMode = value;
			break;

		case CMD_SET_MODE_AND_LOCK:
			padstate[CurPad].PadMode = value;
			padstate[CurPad].PadID = value ? 0x73 : 0x41;
			break;

		case CMD_QUERY_ACT:
			switch (value) {
				case 0: // default
					pad_buf[5] = 0x02;
					pad_buf[6] = 0x00;
					pad_buf[7] = 0x0A;
					break;

				case 1: // Param std conf change
					pad_buf[5] = 0x01;
					pad_buf[6] = 0x01;
					pad_buf[7] = 0x14;
					break;
			}
			break;

		case CMD_QUERY_MODE:
			switch (value) {
				case 0: // mode 0 - digital mode
					pad_buf[5] = PSE_PAD_TYPE_STANDARD;
					break;

				case 1: // mode 1 - analog mode
					pad_buf[5] = PSE_PAD_TYPE_ANALOGPAD;
					break;
			}
			break;
	}
}

static void do_vibration(unsigned char value)
{
    int changed = 0;
    int i;

    switch (CurCmd) {
        case CMD_READ_DATA_AND_VIBRATE:
            for (i = 0; i < 2; i++) {
                if (padstate[CurPad].pad.Vib[i] == CurByte
                     && padstate[CurPad].pad.VibF[i] != value) {
                    padstate[CurPad].pad.VibF[i] = value;
                    changed = 1;
                }
            }

            if (!in_enable_vibration || !changed)
                break;

            plat_trigger_vibrate(CurPad,
                                 padstate[CurPad].pad.VibF[0],
                                 padstate[CurPad].pad.VibF[1]);
            break;
        case CMD_VIBRATION_TOGGLE:
            for (i = 0; i < 2; i++) {
                if (padstate[CurPad].pad.Vib[i] == CurByte)
                    pad_buf[CurByte] = 0;
            }
            if (value < 2) {
                padstate[CurPad].pad.Vib[value] = CurByte;
                if((padstate[CurPad].PadID & 0x0f) < (CurByte - 1) / 2) {
                    padstate[CurPad].PadID = (padstate[CurPad].PadID & 0xf0) + (CurByte - 1) / 2;
                }
            }
            break;
    }
}

unsigned char PADpoll_pad(unsigned char value) {
	if (CurByte == 0) {
		CurCmd = value;
		CurByte++;

		// Don't enable Analog/Vibration for a standard pad
		if (padstate[CurPad].pad.controllerType != PSE_PAD_TYPE_ANALOGPAD)
			CurCmd = CMD_READ_DATA_AND_VIBRATE;

		return do_cmd();
	}

	if (CurByte >= CmdLen)
		return 0xff;	// verified

	if (CurByte == 2)
		do_cmd2(value);

	if (padstate[CurPad].pad.controllerType == PSE_PAD_TYPE_ANALOGPAD)
		do_vibration(value);

	return pad_buf[CurByte++];
}

unsigned char PADstartPoll_pad(int pad) {
	CurPad = pad - 1;
	CurByte = 0;

	if (pad == 1)
		PAD1_readPort1(&padstate[0].pad);
	else
		PAD2_readPort2(&padstate[1].pad);

	return 0xFF;
}

void pad_init(void) {
	int i;

	PAD1_readPort1(&padstate[0].pad);
	PAD2_readPort2(&padstate[1].pad);

	for (i = 0; i < 2; i++) {
		padstate[i].PadID = padstate[i].pad.controllerType == PSE_PAD_TYPE_ANALOGPAD ? 0x73 : 0x41;
		padstate[i].PadMode = padstate[i].pad.controllerType == PSE_PAD_TYPE_ANALOGPAD;
	}
}

unsigned char PADpoll_guncon(unsigned char value)
{
	if (CurByte == 0) {
		CurCmd = value;
		CurByte++;
		return 0x63;	// regardless of cmd
	}

	if (CurCmd != 0x42 || CurByte >= 8)
		return 0xff;	// verified

	return gcon_buf[CurByte++];
}

unsigned char PADstartPoll_guncon(int pad)
{
	int x, y, xn = 0, yn = 0, in = 0, xres = 256, yres = 240;
	CurByte = 0;

	gcon_buf[2] = gcon_buf[3] = 0xff;
	pl_update_gun(&xn, &yn, &xres, &yres, &in);

	// while y = const + line counter, what is x?
	// for 256 mode, hw dumped offsets x, y: 0x5a, 0x20
	//x = 0x5a + (356 * xn >> 10);
	x = 0x5a - (xres - 256) / 3 + (((xres - 256) / 3 + 356) * xn >> 10);
	y = 0x20 + (yres * yn >> 10);

	if (in & GUNIN_TRIGGER)
		gcon_buf[3] &= ~0x20;
	if (in & GUNIN_BTNA)
		gcon_buf[2] &= ~0x08;
	if (in & GUNIN_BTNB)
		gcon_buf[3] &= ~0x40;
	if (in & GUNIN_TRIGGER2) {
		gcon_buf[3] &= ~0x20;
		x = 1;
		y = 10;
	}
	gcon_buf[4] = x;
	gcon_buf[5] = x >> 8;
	gcon_buf[6] = y;
	gcon_buf[7] = y >> 8;

	return 0xff;
}

void guncon_init(void)
{
	memset(gcon_buf, 0xff, sizeof(gcon_buf));
	gcon_buf[1] = 0x5a;
}

static int old_controller_type1 = -1, old_controller_type2 = -1;

#define select_pad(n) \
	if (pad.controllerType != old_controller_type##n) \
	{ \
		switch (pad.controllerType) \
		{ \
		case PSE_PAD_TYPE_ANALOGPAD: \
			PAD##n##_startPoll = PADstartPoll_pad; \
			PAD##n##_poll = PADpoll_pad; \
			pad_init(); \
			break; \
		case PSE_PAD_TYPE_GUNCON: \
			PAD##n##_startPoll = PADstartPoll_guncon; \
			PAD##n##_poll = PADpoll_guncon; \
			guncon_init(); \
			break; \
		case PSE_PAD_TYPE_NEGCON: \
		case PSE_PAD_TYPE_GUN: \
		default: \
			PAD##n##_startPoll = PADstartPoll_pad; \
			PAD##n##_poll = PADpoll_pad; \
			break; \
		} \
		old_controller_type##n = pad.controllerType; \
	}

void dfinput_activate(void)
{
	PadDataS pad;

	PAD1_readPort1(&pad);
	select_pad(1);

	PAD2_readPort2(&pad);
	select_pad(2);
}
