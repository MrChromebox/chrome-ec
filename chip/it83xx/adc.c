/* Copyright (c) 2014 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* IT83xx ADC module for Chrome EC */

#include "adc.h"
#include "adc_chip.h"
#include "clock.h"
#include "console.h"
#include "common.h"
#include "gpio.h"
#include "hooks.h"
#include "registers.h"
#include "task.h"
#include "timer.h"
#include "util.h"

/* Global variables */
static struct mutex adc_lock;
static int adc_init_done;
static volatile task_id_t task_waiting;

/* Data structure of ADC channel control registers. */
const struct adc_ctrl_t adc_ctrl_regs[] = {
	{&IT83XX_ADC_VCH0CTL, &IT83XX_ADC_VCH0DATM, &IT83XX_ADC_VCH0DATL,
		&IT83XX_GPIO_GPCRI0},
	{&IT83XX_ADC_VCH1CTL, &IT83XX_ADC_VCH1DATM, &IT83XX_ADC_VCH1DATL,
		&IT83XX_GPIO_GPCRI1},
	{&IT83XX_ADC_VCH2CTL, &IT83XX_ADC_VCH2DATM, &IT83XX_ADC_VCH2DATL,
		&IT83XX_GPIO_GPCRI2},
	{&IT83XX_ADC_VCH3CTL, &IT83XX_ADC_VCH3DATM, &IT83XX_ADC_VCH3DATL,
		&IT83XX_GPIO_GPCRI3},
	{&IT83XX_ADC_VCH4CTL, &IT83XX_ADC_VCH4DATM, &IT83XX_ADC_VCH4DATL,
		&IT83XX_GPIO_GPCRI4},
	{&IT83XX_ADC_VCH5CTL, &IT83XX_ADC_VCH5DATM, &IT83XX_ADC_VCH5DATL,
		&IT83XX_GPIO_GPCRI5},
	{&IT83XX_ADC_VCH6CTL, &IT83XX_ADC_VCH6DATM, &IT83XX_ADC_VCH6DATL,
		&IT83XX_GPIO_GPCRI6},
	{&IT83XX_ADC_VCH7CTL, &IT83XX_ADC_VCH7DATM, &IT83XX_ADC_VCH7DATL,
		&IT83XX_GPIO_GPCRI7},
};

static void adc_enable_channel(int ch)
{
	if (ch < 4)
		/*
		 * for channel 0, 1, 2, and 3
		 * bit4 ~ bit0 : indicates voltage channel[x]
		 *               input is selected for measurement (enable)
		 * bit5 : data valid interrupt of adc.
		 * bit7 : W/C data valid flag
		 */
		*adc_ctrl_regs[ch].adc_ctrl = 0xa0 + ch;
	else
		/*
		 * for channel 4, 5, 6, and 7
		 * bit4 : voltage channel enable (ch 4~7 only)
		 * bit5 : data valid interrupt of adc.
		 * bit7 : W/C data valid flag
		 */
		*adc_ctrl_regs[ch].adc_ctrl = 0xb0;

	task_clear_pending_irq(IT83XX_IRQ_ADC);
	task_enable_irq(IT83XX_IRQ_ADC);

	/* bit 0 : adc module enable */
	IT83XX_ADC_ADCCFG |= 0x01;
}

static void adc_disable_channel(int ch)
{
	if (ch < 4)
		/*
		 * for channel 0, 1, 2, and 3
		 * bit4 ~ bit0 : indicates voltage channel[x]
		 *               input is selected for measurement (disable)
		 * bit 7 : W/C data valid flag
		 */
		*adc_ctrl_regs[ch].adc_ctrl = 0x9F;
	else
		/*
		 * for channel 4, 5, 6, and 7
		 * bit4 : voltage channel disable (ch 4~7 only)
		 * bit7 : W/C data valid flag
		 */
		*adc_ctrl_regs[ch].adc_ctrl = 0x80;

	/* bit 0 : adc module disable */
	IT83XX_ADC_ADCCFG &= ~0x01;

	task_disable_irq(IT83XX_IRQ_ADC);
}

int adc_read_channel(enum adc_channel ch)
{
	uint32_t events;
	/* voltage 0 ~ 3v = adc data register raw data 0 ~ 3FFh (10-bit ) */
	uint16_t adc_raw_data;
	int valid = 0;
	int adc_ch, mv;

	if (!adc_init_done)
		return ADC_READ_ERROR;

	mutex_lock(&adc_lock);

	task_waiting = task_get_current();
	adc_ch = adc_channels[ch].channel;
	adc_enable_channel(adc_ch);
	/* Wait for interrupt */
	events = task_wait_event_mask(TASK_EVENT_ADC_DONE, ADC_TIMEOUT_US);
	task_waiting = TASK_ID_INVALID;

	if (events & TASK_EVENT_ADC_DONE) {
		/* data valid of adc channel[x] */
		if (IT83XX_ADC_ADCDVSTS & (1 << adc_ch)) {
			/* read adc raw data msb and lsb */
			adc_raw_data = (*adc_ctrl_regs[adc_ch].adc_datm << 8) +
				*adc_ctrl_regs[adc_ch].adc_datl;

			/* W/C data valid flag */
			IT83XX_ADC_ADCDVSTS = (1 << adc_ch);

			mv = adc_raw_data * adc_channels[ch].factor_mul /
				adc_channels[ch].factor_div +
				adc_channels[ch].shift;
			valid = 1;
		}
	}
	adc_disable_channel(adc_ch);

	mutex_unlock(&adc_lock);

	return valid ? mv : ADC_READ_ERROR;
}

void adc_interrupt(void)
{
	/*
	 * Clear the interrupt status.
	 *
	 * NOTE:
	 * The ADC interrupt pending flag won't be cleared unless
	 * we W/C data valid flag of ADC module as well.
	 * (If interrupt type setting is high-level triggered)
	 */
	task_clear_pending_irq(IT83XX_IRQ_ADC);
	/*
	 * We disable ADC interrupt here, because current setting of
	 * interrupt type is high-level triggered.
	 * The interrupt will be triggered again and again until
	 * we W/C data valid flag if we don't disable it.
	 */
	task_disable_irq(IT83XX_IRQ_ADC);
	/* Wake up the task which was waiting for the interrupt */
	if (task_waiting != TASK_ID_INVALID)
		task_set_event(task_waiting, TASK_EVENT_ADC_DONE, 0);
}

/*
 * ADC analog accuracy initialization (only once after VSTBY power on)
 *
 * Write 1 to this bit and write 0 to this bit immediately once and
 * only once during the firmware initialization and do not write 1 again
 * after initialization since IT83xx takes much power consumption
 * if this bit is set as 1
 */
static void adc_accuracy_initialization(void)
{
	/* bit3 : start adc accuracy initialization */
	IT83XX_ADC_ADCSTS |= 0x08;
	/* short delay for adc accuracy initialization */
	IT83XX_GCTRL_WNCKR = 0;
	/* bit3 : stop adc accuracy initialization */
	IT83XX_ADC_ADCSTS &= ~0x08;
}

/* ADC module Initialization */
static void adc_init(void)
{
	int index;
	int ch;

	/* ADC analog accuracy initialization */
	adc_accuracy_initialization();

	for (index = 0; index < ADC_CH_COUNT; index++) {
		ch = adc_channels[index].channel;

		/* enable adc channel[x] function pin */
		*adc_ctrl_regs[ch].adc_pin_ctrl = 0x00;
	}
	/*
	 * bit7@ADCSTS     : ADCCTS1 = 0
	 * bit5@ADCCFG     : ADCCTS0 = 0
	 * bit[5-0]@ADCCTL : SCLKDIV
	 * The ADC channel conversion time is 30.8*(SCLKDIV+1) us.
	 * (Current setting is 61.6us)
	 *
	 * NOTE: A sample time delay (60us) also need to be included in
	 * conversion time, so the final result is ~= 121.6us.
	 */
	IT83XX_ADC_ADCSTS &= ~(1 << 7);
	IT83XX_ADC_ADCCFG &= ~(1 << 5);
	IT83XX_ADC_ADCCTL = 1;

	task_waiting = TASK_ID_INVALID;
	/* disable adc interrupt */
	task_disable_irq(IT83XX_IRQ_ADC);

	adc_init_done = 1;
}
DECLARE_HOOK(HOOK_INIT, adc_init, HOOK_PRIO_INIT_ADC);
