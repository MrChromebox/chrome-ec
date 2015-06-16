/* Copyright (c) 2013 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */
/* IT8380 development board configuration */

#include "common.h"
#include "console.h"
#include "gpio.h"
#include "hooks.h"
#include "registers.h"
#include "task.h"
#include "util.h"
#include "pwm.h"
#include "pwm_chip.h"
#include "adc.h"
#include "adc_chip.h"
#include "ec2i_chip.h"
#include "power_button.h"
#include "lid_switch.h"
#include "keyboard_scan.h"
#include "timer.h"
#include "lpc.h"
#include "intc.h"

/* Test GPIO interrupt function that toggles one LED. */
void test_interrupt(enum gpio_signal signal)
{
	static int busy_state;

	/* toggle LED */
	busy_state = !busy_state;
	gpio_set_level(GPIO_BUSY_LED, busy_state);
}

#include "gpio_list.h"

/* PWM channels. Must be in the exactly same order as in enum pwm_channel. */
const struct pwm_t pwm_channels[] = {
	{0, 0},
	{1, PWM_CONFIG_ACTIVE_LOW},
	{2, 0},
	{3, PWM_CONFIG_ACTIVE_LOW},
	{4, 0},
	{5, PWM_CONFIG_ACTIVE_LOW},
	{6, 0},
	{7, PWM_CONFIG_ACTIVE_LOW},
};

BUILD_ASSERT(ARRAY_SIZE(pwm_channels) == PWM_CH_COUNT);

/* PNPCFG settings */
const struct ec2i_t pnpcfg_settings[] = {
	/* Select logical device 06h(keyboard) */
	{HOST_INDEX_LDN, LDN_KBC_KEYBOARD},
	/* Set IRQ=01h for logical device */
	{HOST_INDEX_IRQNUMX, 0x01},
	/* Enable logical device */
	{HOST_INDEX_LDA, 0x01},

	/* Select logical device 05h(mouse) */
	{HOST_INDEX_LDN, LDN_KBC_MOUSE},
	/* Set IRQ=0Ch for logical device */
	{HOST_INDEX_IRQNUMX, 0x0C},
	/* Enable logical device */
	{HOST_INDEX_LDA, 0x01},

	/* Select logical device 11h(PM1 ACPI) */
	{HOST_INDEX_LDN, LDN_PMC1},
	/* Set IRQ=00h for logical device */
	{HOST_INDEX_IRQNUMX, 0x00},
	/* Enable logical device */
	{HOST_INDEX_LDA, 0x01},

	/* Select logical device 12h(PM2) */
	{HOST_INDEX_LDN, LDN_PMC2},
	/* I/O Port Base Address 200h/204h */
	{HOST_INDEX_IOBAD0_MSB, 0x02},
	{HOST_INDEX_IOBAD0_LSB, 0x00},
	{HOST_INDEX_IOBAD1_MSB, 0x02},
	{HOST_INDEX_IOBAD1_LSB, 0x04},
	/* Set IRQ=00h for logical device */
	{HOST_INDEX_IRQNUMX, 0x00},
	/* Enable logical device */
	{HOST_INDEX_LDA, 0x01},

	/* Select logical device 0Fh(SMFI) */
	{HOST_INDEX_LDN, LDN_SMFI},
	/* H2RAM LPC I/O cycle Dxxx */
	{HOST_INDEX_DSLDC6, 0x00},
	/* Enable H2RAM LPC I/O cycle */
	{HOST_INDEX_DSLDC7, 0x01},
	/* Enable logical device */
	{HOST_INDEX_LDA, 0x01},

	/* Select logical device 17h(PM3) */
	{HOST_INDEX_LDN, LDN_PMC3},
	/* I/O Port Base Address 80h */
	{HOST_INDEX_IOBAD0_MSB, 0x00},
	{HOST_INDEX_IOBAD0_LSB, 0x80},
	{HOST_INDEX_IOBAD1_MSB, 0x00},
	{HOST_INDEX_IOBAD1_LSB, 0x00},
	/* Set IRQ=00h for logical device */
	{HOST_INDEX_IRQNUMX, 0x00},
	/* Enable logical device */
	{HOST_INDEX_LDA, 0x01},
};
BUILD_ASSERT(ARRAY_SIZE(pnpcfg_settings) == EC2I_SETTING_COUNT);

/* Initialize board. */
static void board_init(void)
{

}
DECLARE_HOOK(HOOK_INIT, board_init, HOOK_PRIO_DEFAULT);

/* ADC channels. Must be in the exactly same order as in enum adc_channel. */
const struct adc_t adc_channels[] = {
	/* Convert to mV (3000mV/1024). */
	{"adc_ch0", 3000, 1024, 0, 0},
	{"adc_ch1", 3000, 1024, 0, 1},
	{"adc_ch2", 3000, 1024, 0, 2},
	{"adc_ch3", 3000, 1024, 0, 3},
	{"adc_ch4", 3000, 1024, 0, 4},
	{"adc_ch5", 3000, 1024, 0, 5},
	{"adc_ch6", 3000, 1024, 0, 6},
	{"adc_ch7", 3000, 1024, 0, 7},
};
BUILD_ASSERT(ARRAY_SIZE(adc_channels) == ADC_CH_COUNT);

/* Keyboard scan setting */
struct keyboard_scan_config keyscan_config = {
	.output_settle_us = 35,
	.debounce_down_us = 5 * MSEC,
	.debounce_up_us = 40 * MSEC,
	.scan_period_us = 3 * MSEC,
	.min_post_scan_delay_us = 1000,
	.poll_timeout_us = 100 * MSEC,
	.actual_key_mask = {
		0x14, 0xff, 0xff, 0xff, 0xff, 0xf5, 0xff,
		0xa4, 0xff, 0xfe, 0x55, 0xfa, 0xca  /* full set */
	},
};

/*****************************************************************************/
/* Console commands */

void display_7seg(uint8_t val)
{
	int i;
	static const uint8_t digits[16] = {
		0xc0, 0xf9, 0xa8, 0xb0,
		0x99, 0x92, 0x82, 0xf8,
		0x80, 0x98, 0x88, 0x83,
		0xc6, 0xa1, 0x86, 0x8e,
	};

	for (i = 0; i < 7; i++)
		gpio_set_level(GPIO_H_LED0 + i, digits[val >> 4] & (1 << i));
	for (i = 0; i < 7; i++)
		gpio_set_level(GPIO_L_LED0 + i, digits[val & 0xf] & (1 << i));
}

static int command_7seg(int argc, char **argv)
{
	uint8_t val;
	char *e;

	if (argc != 2)
		return EC_ERROR_PARAM_COUNT;

	val = strtoi(argv[1], &e, 16);
	if (*e)
		return EC_ERROR_PARAM1;

	ccprintf("display 0x%02x\n", val);
	display_7seg(val);

	return EC_SUCCESS;
}
DECLARE_CONSOLE_COMMAND(seg7, command_7seg,
			"<hex>",
			"Print 8-bit value on 7-segment display",
			NULL);
