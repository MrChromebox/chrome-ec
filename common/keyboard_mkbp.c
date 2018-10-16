/* Copyright (c) 2013 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * MKBP keyboard protocol
 */

#include "atomic.h"
#include "chipset.h"
#include "console.h"
#include "gpio.h"
#include "host_command.h"
#include "keyboard_config.h"
#include "keyboard_protocol.h"
#include "keyboard_raw.h"
#include "keyboard_scan.h"
#include "keyboard_test.h"
#include "mkbp_event.h"
#include "system.h"
#include "task.h"
#include "timer.h"
#include "util.h"

/* Console output macros */
#define CPUTS(outstr) cputs(CC_KEYBOARD, outstr)
#define CPRINTS(format, args...) cprints(CC_KEYBOARD, format, ## args)

/*
 * Keyboard FIFO depth.  This needs to be big enough not to overflow if a
 * series of keys is pressed in rapid succession and the kernel is too busy
 * to read them out right away.
 *
 * RAM usage is (depth * #cols); see kb_fifo[][] below.  A 16-entry FIFO will
 * consume 16x13=208 bytes, which is non-trivial but not horrible.
 */
#define KB_FIFO_DEPTH 16

/* Changes to col,row here need to also be reflected in kernel.
 * drivers/input/mkbp.c ... see KEY_BATTERY.
 */
#define BATTERY_KEY_COL 0
#define BATTERY_KEY_ROW 7
#define BATTERY_KEY_ROW_MASK (1 << BATTERY_KEY_ROW)

static uint32_t kb_fifo_start;		/* first entry */
static uint32_t kb_fifo_end;		/* last entry */
static uint32_t kb_fifo_entries;	/* number of existing entries */
static uint8_t kb_fifo[KB_FIFO_DEPTH][KEYBOARD_COLS];
static struct mutex fifo_mutex;

/* Button and switch state. */
static uint32_t mkbp_button_state;
static uint32_t mkbp_switch_state;
#ifndef HAS_TASK_KEYSCAN
/* Keys simulated-pressed */
static uint8_t __bss_slow simulated_key[KEYBOARD_COLS_MAX];
uint8_t keyboard_cols = KEYBOARD_COLS_MAX;
#endif /* !defined(HAS_TASK_KEYSCAN) */

/* Config for mkbp protocol; does not include fields from scan config */
struct ec_mkbp_protocol_config {
	uint32_t valid_mask;	/* valid fields */
	uint8_t flags;		/* some flags (enum mkbp_config_flags) */
	uint8_t valid_flags;	/* which flags are valid */

	/* maximum depth to allow for fifo (0 = no keyscan output) */
	uint8_t fifo_max_depth;
} __packed;

static struct ec_mkbp_protocol_config config = {
	.valid_mask = EC_MKBP_VALID_SCAN_PERIOD | EC_MKBP_VALID_POLL_TIMEOUT |
		EC_MKBP_VALID_MIN_POST_SCAN_DELAY |
		EC_MKBP_VALID_OUTPUT_SETTLE | EC_MKBP_VALID_DEBOUNCE_DOWN |
		EC_MKBP_VALID_DEBOUNCE_UP | EC_MKBP_VALID_FIFO_MAX_DEPTH,
	.valid_flags = EC_MKBP_FLAGS_ENABLE,
	.flags = EC_MKBP_FLAGS_ENABLE,
	.fifo_max_depth = KB_FIFO_DEPTH,
};

static int get_data_size(enum ec_mkbp_event e)
{
	switch (e) {
	case EC_MKBP_EVENT_KEY_MATRIX:
		return keyboard_cols;

	case EC_MKBP_EVENT_HOST_EVENT:
	case EC_MKBP_EVENT_BUTTON:
	case EC_MKBP_EVENT_SWITCH:
		return sizeof(uint32_t);
	default:
		/* For unknown types, say it's 0. */
		return 0;
	}
}

/**
 * Pop keyboard state from FIFO
 *
 * @return EC_SUCCESS if entry popped, EC_ERROR_UNKNOWN if FIFO is empty
 */
static int kb_fifo_remove(uint8_t *buffp)
{
	if (!kb_fifo_entries) {
		/* no entry remaining in FIFO : return last known state */
		int last = (kb_fifo_start + KB_FIFO_DEPTH - 1) % KB_FIFO_DEPTH;
		memcpy(buffp, kb_fifo[last], KEYBOARD_COLS);

		/*
		 * Bail out without changing any FIFO indices and let the
		 * caller know something strange happened. The buffer will
		 * will contain the last known state of the keyboard.
		 */
		return EC_ERROR_UNKNOWN;
	}
	memcpy(buffp, kb_fifo[kb_fifo_start], KEYBOARD_COLS);

	kb_fifo_start = (kb_fifo_start + 1) % KB_FIFO_DEPTH;

	atomic_sub(&kb_fifo_entries, 1);

	return EC_SUCCESS;
}

/**
 * Assert host keyboard interrupt line.
 */
static void set_host_interrupt(int active)
{
	/* interrupt host by using active low EC_INT signal */
	gpio_set_level(GPIO_EC_INT_L, !active);
}

/*****************************************************************************/
/* Interface */

void keyboard_clear_buffer(void)
{
	int i;

	CPRINTS("clearing KB fifo");

	kb_fifo_start = 0;
	kb_fifo_end = 0;
	kb_fifo_entries = 0;
	for (i = 0; i < KB_FIFO_DEPTH; i++)
		memset(kb_fifo[i], 0, KEYBOARD_COLS);
}

test_mockable int keyboard_fifo_add(const uint8_t *buffp)
{
	int ret = EC_SUCCESS;

	/*
	 * If keyboard protocol is not enabled, don't save the state to the
	 * FIFO or trigger an interrupt.
	 */
	if (!(config.flags & EC_MKBP_FLAGS_ENABLE))
		return EC_SUCCESS;

	if (kb_fifo_entries >= config.fifo_max_depth) {
		CPRINTS("KB FIFO depth %d reached",
			config.fifo_max_depth);
		ret = EC_ERROR_OVERFLOW;
		goto kb_fifo_push_done;
	}

	mutex_lock(&fifo_mutex);
	memcpy(kb_fifo[kb_fifo_end], buffp, KEYBOARD_COLS);
	kb_fifo_end = (kb_fifo_end + 1) % KB_FIFO_DEPTH;
	atomic_add(&kb_fifo_entries, 1);
	mutex_unlock(&fifo_mutex);

kb_fifo_push_done:

	if (ret == EC_SUCCESS) {
#ifdef CONFIG_MKBP_EVENT
		mkbp_send_event(EC_MKBP_EVENT_KEY_MATRIX);
#else
		set_host_interrupt(1);
#endif
	}

	return ret;
}

#ifdef CONFIG_MKBP_EVENT
static int keyboard_get_next_event(uint8_t *out)
{
	if (!kb_fifo_entries)
		return -1;

	kb_fifo_remove(out);

	/* Keep sending events if FIFO is not empty */
	if (kb_fifo_entries)
		mkbp_send_event(EC_MKBP_EVENT_KEY_MATRIX);

	return KEYBOARD_COLS;
}
DECLARE_EVENT_SOURCE(EC_MKBP_EVENT_KEY_MATRIX, keyboard_get_next_event);
#endif

void keyboard_send_battery_key(void)
{
	uint8_t state[KEYBOARD_COLS_MAX];

	/* Copy debounced state and add battery pseudo-key */
	memcpy(state, keyboard_scan_get_state(), sizeof(state));
	state[BATTERY_KEY_COL] ^= BATTERY_KEY_ROW_MASK;

	/* Add to FIFO only if AP is on or else it will wake from suspend */
	if (chipset_in_state(CHIPSET_STATE_ON))
		keyboard_fifo_add(state);
}

void clear_typematic_key(void)
{ }

/*****************************************************************************/
/* Host commands */

static int keyboard_get_scan(struct host_cmd_handler_args *args)
{
	kb_fifo_remove(args->response);
	/* if CONFIG_MKBP_EVENT is enabled, we still reset the interrupt
	 * (even if there is an another pending event)
	 * to be backward compatible with firmware using the old API to poll
	 * the keyboard, other software should use EC_CMD_GET_NEXT_EVENT
	 * instead of this command
	 */
	if (!kb_fifo_entries)
		set_host_interrupt(0);

	args->response_size = KEYBOARD_COLS;

	return EC_RES_SUCCESS;
}
DECLARE_HOST_COMMAND(EC_CMD_MKBP_STATE,
		     keyboard_get_scan,
		     EC_VER_MASK(0));

static int mkbp_get_info(struct host_cmd_handler_args *args)
{
	const struct ec_params_mkbp_info *p = args->params;

	if (args->params_size == 0 || p->info_type == EC_MKBP_INFO_KBD) {
		struct ec_response_mkbp_info *r = args->response;

		/* Version 0 just returns info about the keyboard. */
		r->rows = KEYBOARD_ROWS;
		r->cols = keyboard_cols;
		/* This used to be "switches" which was previously 0. */
		r->reserved = 0;

		args->response_size = sizeof(struct ec_response_mkbp_info);
	} else {
		union ec_response_get_next_data *r = args->response;

		/* Version 1 (other than EC_MKBP_INFO_KBD) */
		switch (p->info_type) {
		case EC_MKBP_INFO_SUPPORTED:
			switch (p->event_type) {
			case EC_MKBP_EVENT_BUTTON:
				r->buttons = get_supported_buttons();
				args->response_size = sizeof(r->buttons);
				break;

			case EC_MKBP_EVENT_SWITCH:
				r->switches = get_supported_switches();
				args->response_size = sizeof(r->switches);
				break;

			default:
				/* Don't care for now for other types. */
				return EC_RES_INVALID_PARAM;
			}
			break;

		case EC_MKBP_INFO_CURRENT:
			switch (p->event_type) {
			case EC_MKBP_EVENT_KEY_MATRIX:
				memcpy(r->key_matrix, keyboard_scan_get_state(),
				       sizeof(r->key_matrix));
				args->response_size = sizeof(r->key_matrix);
				break;

			case EC_MKBP_EVENT_HOST_EVENT:
				r->host_event = host_get_events();
				args->response_size = sizeof(r->host_event);
				break;

			case EC_MKBP_EVENT_BUTTON:
				r->buttons = mkbp_button_state;
				args->response_size = sizeof(r->buttons);
				break;

			case EC_MKBP_EVENT_SWITCH:
				r->switches = mkbp_switch_state;
				args->response_size = sizeof(r->switches);
				break;

			default:
				/* Doesn't make sense for other event types. */
				return EC_RES_INVALID_PARAM;
			}
			break;

		default:
			/* Unsupported query. */
			return EC_RES_ERROR;
		}
	}
	return EC_RES_SUCCESS;
}
DECLARE_HOST_COMMAND(EC_CMD_MKBP_INFO,
		     keyboard_get_info,
		     EC_VER_MASK(0));

#ifndef HAS_TASK_KEYSCAN
/* For boards without a keyscan task, try and simulate keyboard presses. */
static void simulate_key(int row, int col, int pressed)
{
	if ((simulated_key[col] & (1 << row)) == ((pressed ? 1 : 0) << row))
		return;  /* No change */

	simulated_key[col] &= ~(1 << row);
	if (pressed)
		simulated_key[col] |= (1 << row);

	keyboard_fifo_add(simulated_key);
}

static int command_mkbp_keyboard_press(int argc, char **argv)
{
	if (argc == 1) {
		int i, j;

		ccputs("Simulated keys:\n");
		for (i = 0; i < keyboard_cols; ++i) {
			if (simulated_key[i] == 0)
				continue;
			for (j = 0; j < KEYBOARD_ROWS; ++j)
				if (simulated_key[i] & (1 << j))
					ccprintf("\t%d %d\n", i, j);
		}

	} else if (argc == 3 || argc == 4) {
		int r, c, p;
		char *e;

		c = strtoi(argv[1], &e, 0);
		if (*e || c < 0 || c >= keyboard_cols)
			return EC_ERROR_PARAM1;

		r = strtoi(argv[2], &e, 0);
		if (*e || r < 0 || r >= KEYBOARD_ROWS)
			return EC_ERROR_PARAM2;

		if (argc == 3) {
			/* Simulate a press and release */
			simulate_key(r, c, 1);
			simulate_key(r, c, 0);
		} else {
			p = strtoi(argv[3], &e, 0);
			if (*e || p < 0 || p > 1)
				return EC_ERROR_PARAM3;

			simulate_key(r, c, p);
		}
	}

	return EC_SUCCESS;
}
DECLARE_CONSOLE_COMMAND(kbpress, command_mkbp_keyboard_press,
			"[col row [0 | 1]]",
			"Simulate keypress");
#endif /* !defined(HAS_TASK_KEYSCAN) */

#ifdef HAS_TASK_KEYSCAN
static void set_keyscan_config(const struct ec_mkbp_config *src,
			       struct ec_mkbp_protocol_config *dst,
			       uint32_t valid_mask, uint8_t new_flags)
{
	struct keyboard_scan_config *ksc = keyboard_scan_get_config();

	if (valid_mask & EC_MKBP_VALID_SCAN_PERIOD)
		ksc->scan_period_us = src->scan_period_us;

	if (valid_mask & EC_MKBP_VALID_POLL_TIMEOUT)
		ksc->poll_timeout_us = src->poll_timeout_us;

	if (valid_mask & EC_MKBP_VALID_MIN_POST_SCAN_DELAY) {
		/*
		 * Key scanning is high priority, so we should require at
		 * least 100us min delay here. Setting this to 0 will cause
		 * watchdog events. Use 200 to be safe.
		 */
		ksc->min_post_scan_delay_us =
			MAX(src->min_post_scan_delay_us, 200);
	}

	if (valid_mask & EC_MKBP_VALID_OUTPUT_SETTLE)
		ksc->output_settle_us = src->output_settle_us;

	if (valid_mask & EC_MKBP_VALID_DEBOUNCE_DOWN)
		ksc->debounce_down_us = src->debounce_down_us;

	if (valid_mask & EC_MKBP_VALID_DEBOUNCE_UP)
		ksc->debounce_up_us = src->debounce_up_us;

	/*
	 * If we just enabled key scanning, kick the task so that it will
	 * fall out of the task_wait_event() in keyboard_scan_task().
	 */
	if ((new_flags & EC_MKBP_FLAGS_ENABLE) &&
			!(dst->flags & EC_MKBP_FLAGS_ENABLE))
		task_wake(TASK_ID_KEYSCAN);
}

static void get_keyscan_config(struct ec_mkbp_config *dst)
{
	const struct keyboard_scan_config *ksc = keyboard_scan_get_config();

	/* Copy fields from keyscan config to mkbp config */
	dst->output_settle_us = ksc->output_settle_us;
	dst->debounce_down_us = ksc->debounce_down_us;
	dst->debounce_up_us = ksc->debounce_up_us;
	dst->scan_period_us = ksc->scan_period_us;
	dst->min_post_scan_delay_us = ksc->min_post_scan_delay_us;
	dst->poll_timeout_us = ksc->poll_timeout_us;
}

/**
 * Copy keyscan configuration from one place to another according to flags
 *
 * This is like a structure copy, except that only selected fields are
 * copied.
 *
 * @param src		Source config
 * @param dst		Destination config
 * @param valid_mask	Bits representing which fields to copy - each bit is
 *			from enum mkbp_config_valid
 * @param valid_flags	Bit mask controlling flags to copy. Any 1 bit means
 *			that the corresponding bit in src->flags is copied
 *			over to dst->flags
 */
static void keyscan_copy_config(const struct ec_mkbp_config *src,
				 struct ec_mkbp_protocol_config *dst,
				 uint32_t valid_mask, uint8_t valid_flags)
{
	uint8_t new_flags;

	if (valid_mask & EC_MKBP_VALID_FIFO_MAX_DEPTH) {
		/* Sanity check for fifo depth */
		dst->fifo_max_depth = MIN(src->fifo_max_depth,
					  KB_FIFO_DEPTH);
	}

	new_flags = dst->flags & ~valid_flags;
	new_flags |= src->flags & valid_flags;

	set_keyscan_config(src, dst, valid_mask, new_flags);
	dst->flags = new_flags;
}

static int host_command_mkbp_set_config(struct host_cmd_handler_args *args)
{
	const struct ec_params_mkbp_set_config *req = args->params;

	keyscan_copy_config(&req->config, &config,
			    config.valid_mask & req->config.valid_mask,
			    config.valid_flags & req->config.valid_flags);

	return EC_RES_SUCCESS;
}
DECLARE_HOST_COMMAND(EC_CMD_MKBP_SET_CONFIG,
		     host_command_mkbp_set_config,
		     EC_VER_MASK(0));

static int host_command_mkbp_get_config(struct host_cmd_handler_args *args)
{
	struct ec_response_mkbp_get_config *resp = args->response;
	struct ec_mkbp_config *dst = &resp->config;

	memcpy(&resp->config, &config, sizeof(config));

	/* Copy fields from mkbp protocol config to mkbp config */
	dst->valid_mask = config.valid_mask;
	dst->flags = config.flags;
	dst->valid_flags = config.valid_flags;
	dst->fifo_max_depth = config.fifo_max_depth;

	get_keyscan_config(dst);

	args->response_size = sizeof(*resp);

	return EC_RES_SUCCESS;
}
DECLARE_HOST_COMMAND(EC_CMD_MKBP_GET_CONFIG,
		     host_command_mkbp_get_config,
		     EC_VER_MASK(0));
