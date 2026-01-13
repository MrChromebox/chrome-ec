/* Copyright (c) 2013 The Chromium OS Authors. All rights reserved.
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* Power button module for Chrome EC */

#include "chipset.h"
#include "common.h"
#include "console.h"
#include "gpio.h"
#include "hooks.h"
#include "host_command.h"
#include "keyboard_scan.h"
#include "lid_switch.h"
#include "power_button.h"
#include "system.h"
#include "task.h"
#include "timer.h"
#include "util.h"

/* Console output macros */
#define CPUTS(outstr) cputs(CC_SWITCH, outstr)
#define CPRINTS(format, args...) cprints(CC_SWITCH, format, ## args)

/* By default the power button is active low */
#ifndef CONFIG_POWER_BUTTON_ACTIVE_STATE
#define CONFIG_POWER_BUTTON_ACTIVE_STATE 0
#endif

#define PWRBTN_DEBOUNCE_US (30 * MSEC)  /* Debounce time for power button */

static int debounced_power_pressed;	/* Debounced power button state */
static int simulate_power_pressed;
static volatile int power_button_is_stable = 1;

/**
 * Return non-zero if power button signal asserted at hardware input.
 *
 */
int power_button_signal_asserted(void)
{
	return !!(gpio_get_level(GPIO_POWER_BUTTON_L)
		 == CONFIG_POWER_BUTTON_ACTIVE_STATE);
}

/**
 * Get raw power button signal state.
 *
 * @return 1 if power button is pressed, 0 if not pressed.
 */
static int raw_power_button_pressed(void)
{
	if (simulate_power_pressed)
		return 1;

#ifndef CONFIG_POWER_BUTTON_IGNORE_LID
	/*
	 * Always indicate power button released if the lid is closed.
	 * This prevents waking the system if the device is squashed enough to
	 * press the power button through the closed lid.
	 */
	if (!lid_is_open())
		return 0;
#endif

	return power_button_signal_asserted();
}

int power_button_is_pressed(void)
{
	return debounced_power_pressed;
}

/**
 * Wait for the power button to be released
 *
 * @param timeout_us Timeout in microseconds, or -1 to wait forever
 * @return EC_SUCCESS if ok, or
 *         EC_ERROR_TIMEOUT if power button failed to release
 */
int power_button_wait_for_release(int timeout_us)
{
	timestamp_t deadline;
	timestamp_t now = get_time();

	deadline.val = now.val + timeout_us;

	while (!power_button_is_stable || power_button_is_pressed()) {
		now = get_time();
		if (timeout_us < 0) {
			task_wait_event(-1);
		} else if (timestamp_expired(deadline, &now) ||
			(task_wait_event(deadline.val - now.val) ==
			TASK_EVENT_TIMER)) {
			CPRINTS("power button not released in time");
			return EC_ERROR_TIMEOUT;
		}
	}

	CPRINTS("power button released in time");
	return EC_SUCCESS;
}

/**
 * Handle power button initialization.
 */
static void power_button_init(void)
{
	if (raw_power_button_pressed())
		debounced_power_pressed = 1;

	/* Enable interrupts, now that we've initialized */
	gpio_enable_interrupt(GPIO_POWER_BUTTON_L);
}
DECLARE_HOOK(HOOK_INIT, power_button_init, HOOK_PRIO_INIT_POWER_BUTTON);

/* After G3 State - enabling will disable legacy OFF reset flag handling. */
#ifdef CONFIG_AFTER_G3_STATE

static const char *ag3s_title = "After G3 State";

/*
 * Value set here (enum ec_after_g3_state) represents the default after G3
 * behavior, which can be later changed via command/coreboot.
 */
#define AG3S_DEFAULT_STATE (EC_AFTER_G3_STATE_PREVIOUS)

static enum ec_after_g3_state ag3s_state = AG3S_DEFAULT_STATE;

/* Order should match enum ec_after_g3_state, starting from value 0. */
static const char *ag3s_name_table[] = { "off", "on", "previous" };

static inline const char *ag3s_get_state_name(enum ec_after_g3_state state)
{
	return ((state >= EC_AFTER_G3_STATE_OFF &&
		 state <= EC_AFTER_G3_STATE_PREVIOUS) ?
		ag3s_name_table[state] : NULL);
}

static enum ec_after_g3_state ag3s_find_state_by_name(const char *name)
{
	int i, len;
	for (i = 0, len = ARRAY_SIZE(ag3s_name_table); i < len; ++i) {
		if (!strcasecmp(name, ag3s_name_table[i]))
			return i;
	}
	return EC_AFTER_G3_STATE_UNKNOWN;
}

static inline enum ec_after_g3_state ag3s_get_state(void)
{
	return ag3s_state;
}

static inline void ag3s_sync(void);

static int ag3s_set_state(enum ec_after_g3_state state)
{
	const char *state_name = ag3s_get_state_name(state);
	if (!state_name)
		return 0;

	CPRINTS("%s: Updating to '%s'", ag3s_title, state_name);

	ag3s_state = state;

	ag3s_sync();

	return 1;
}

/* Clearing operation always take precedence over saving for the same target. */
enum ag3s_reset_flags_op_flags {
	AG3S_RFO_SET_BBRM = (1 << 0),
	AG3S_RFO_SET_SYS  = (1 << 1),
	AG3S_RFO_CLR_BBRM = (1 << 2),
	AG3S_RFO_CLR_SYS  = (1 << 3)
};

/* Reset flags used to keep the device off. */
#define AG3S_RESET_FLAG_OFF      (RESET_FLAG_AP_OFF)
#define AG3S_RESET_FLAG_OFF_NAME "AP_OFF"

static void ag3s_update_off_reset_flags(int rfo_flags)
{
	static const char op_set = '+';
	static const char op_clr = '-';
	static const char op_eq  = '=';

	char op_bbram, op_sys;

	if (!rfo_flags)
		return;

	if (rfo_flags & AG3S_RFO_CLR_BBRM) {
		chip_save_reset_flags(chip_read_reset_flags() &
				      ~AG3S_RESET_FLAG_OFF);
		op_bbram = op_clr;
	} else if (rfo_flags & AG3S_RFO_SET_BBRM) {
		chip_save_reset_flags(chip_read_reset_flags() |
				      AG3S_RESET_FLAG_OFF);
		op_bbram = op_set;
	} else {
		op_bbram = op_eq;
	}

	if (rfo_flags & AG3S_RFO_CLR_SYS) {
		system_clear_reset_flags(AG3S_RESET_FLAG_OFF);
		op_sys = op_clr;
	} else if (rfo_flags & AG3S_RFO_SET_SYS) {
		system_set_reset_flags(AG3S_RESET_FLAG_OFF);
		op_sys = op_set;
	} else {
		op_sys = op_eq;
	}

	CPRINTS("%s: " AG3S_RESET_FLAG_OFF_NAME " flag updated"
	        " (%cbbram, %csys)", ag3s_title, op_bbram, op_sys);
}

static inline int ag3s_are_any_off_bbram_reset_flags_set(void)
{
	return !!(chip_read_reset_flags() & AG3S_RESET_FLAG_OFF);
}

static inline int ag3s_are_any_non_off_bbram_reset_flags_set(void)
{
	return !!(chip_read_reset_flags() & ~AG3S_RESET_FLAG_OFF);
}

enum ag3s_sync_hook {
	/* Reserved for non-hook sync request. */
	AG3S_SYNC_HOOK_NONE = -1,
	/*
	 * HOOK_INIT with HOOK_PRIO_DEFAULT-1.
	 *
	 * Called before power button initial state is configured (on HOOK_INIT
	 * with HOOK_PRIO_DEFAULT). Changing OFF system reset flags at this
	 * point can either make the device remain off or auto power it on.
	 */
	AG3S_SYNC_HOOK_INIT_BEFORE_PB_ISTATE,
	/*
	 * HOOK_INIT with HOOK_PRIO_DEFAULT+1.
	 *
	 * Called after power button initial state is configured.
	 */
	AG3S_SYNC_HOOK_INIT_AFTER_PB_ISTATE,
	/*
	 * HOOK_CHIPSET_STARTUP with HOOK_PRIO_DEFAULT.
	 *
	 * Called on chipset startup.
	 */
	AG3S_SYNC_HOOK_CHIPSET_STARTUP,
	/*
	 * HOOK_CHIPSET_SHUTDOWN with HOOK_PRIO_DEFAULT-1.
	 *
	 * Called when chipset is shutting down. Priority is slightly higher
	 * than handle_pending_reboot() (HOOK_PRIO_DEFAULT).
	 *
	 * Chipset is still NOT OFF at this point, trying to check for
	 * CHIPSET_STATE_ANY_OFF will evaluate to FALSE.
	 */
	AG3S_SYNC_HOOK_CHIPSET_SHUTDOWN,
	/*
	 * HOOK_SYSJUMP with HOOK_PRIO_LAST.
	 *
	 * Called before making a jump to another image, as late as possible.
	 *
	 * Attempting to modify system reset flags from this hook has no effect,
	 * as these flags are already saved to jump data prior to calling
	 * HOOK_SYSJUMP.
	 *
	 * This hook requires explicit console flushing to avoid truncated
	 * output, since it is called right before jumping.
	 *
	 * Only used to fix FIZZ EC bug. See AG3S_SYNC_HOOK_SYSJUMP case in
	 * ag3s_sync_on_hook() for more info.
	 */
	AG3S_SYNC_HOOK_SYSJUMP
};

static void ag3s_sync_on_hook(enum ag3s_sync_hook sync_hook);

#define AG3S_SYNC_ON_HOOK(d_sync_hook_enum, d_hook, d_hook_priority) \
	static void ag3s_sync_on_hook_##d_sync_hook_enum(void) \
	{ \
		ag3s_sync_on_hook(d_sync_hook_enum); \
	} \
	DECLARE_HOOK(d_hook, ag3s_sync_on_hook_##d_sync_hook_enum, \
		     d_hook_priority)

/* Sync on all valid hooks in enum ag3s_sync_hook. */
AG3S_SYNC_ON_HOOK(AG3S_SYNC_HOOK_INIT_BEFORE_PB_ISTATE, HOOK_INIT,
		  HOOK_PRIO_DEFAULT-1);
AG3S_SYNC_ON_HOOK(AG3S_SYNC_HOOK_INIT_AFTER_PB_ISTATE, HOOK_INIT,
		  HOOK_PRIO_DEFAULT+1);
AG3S_SYNC_ON_HOOK(AG3S_SYNC_HOOK_CHIPSET_STARTUP, HOOK_CHIPSET_STARTUP,
		  HOOK_PRIO_DEFAULT);
AG3S_SYNC_ON_HOOK(AG3S_SYNC_HOOK_CHIPSET_SHUTDOWN, HOOK_CHIPSET_SHUTDOWN,
		  HOOK_PRIO_DEFAULT-1);
AG3S_SYNC_ON_HOOK(AG3S_SYNC_HOOK_SYSJUMP, HOOK_SYSJUMP, HOOK_PRIO_LAST);

static void ag3s_sync_on_hook(enum ag3s_sync_hook sync_hook)
{
	int rfo_flags = 0;
	int is_device_on;
	enum ec_after_g3_state state;
	const char *state_name;

	/*
	 * CHIPSET_STATE_ANY_OFF will not evaluate to TRUE on chipset shutdown,
	 * so instead we always assume the device is going off.
	 */
	is_device_on = (sync_hook != AG3S_SYNC_HOOK_CHIPSET_SHUTDOWN &&
			!chipset_in_state(CHIPSET_STATE_ANY_OFF));

	state = ag3s_get_state();
	state_name = ag3s_get_state_name(state);
	if (!state_name) {
		/* Fallback to default state if state is invalid. */
		state = AG3S_DEFAULT_STATE;
		state_name = "x";
	}

	CPRINTS("%s: Sync on hook %d (%s)", ag3s_title, sync_hook, state_name);

	/*
	 * Some hooks may require special handling of reset flags. These hooks
	 * take precedence over proper flag sync.
	 */
	switch(sync_hook) {
	case AG3S_SYNC_HOOK_INIT_BEFORE_PB_ISTATE:
		if (is_device_on) {
			/*
			 * Unset OFF system reset flags if device is on,
			 * otherwise power button initial state setup may cause
			 * abnormal state.
			 */
			rfo_flags = AG3S_RFO_CLR_SYS;
		} else if (ag3s_are_any_off_bbram_reset_flags_set()) {
			/*
			 * Presence of OFF bbram reset flags, while the device
			 * is off, indicates that it should remain off. Set OFF
			 * system reset flags to ensure power button initial
			 * state setup will prevent auto power on.
			 */
			rfo_flags = AG3S_RFO_SET_SYS;
		}

		/* Proper flag sync will be done after power button init. */
		goto update_off_reset_flags;
	case AG3S_SYNC_HOOK_CHIPSET_SHUTDOWN:
		if (state == EC_AFTER_G3_STATE_PREVIOUS &&
		    chipset_get_shutdown_reason() ==
		    CHIPSET_SHUTDOWN_POWERFAIL) {
			/*
			 * Skip saving OFF bbram reset flags for "Previous
			 * State" behavior if shutting down due to power
			 * failure. This ensures that the device will remain off
			 * when power is restored only if shutdown was graceful,
			 * otherwise it will auto power on.
			 */
			return;
		}
		break;
	case AG3S_SYNC_HOOK_SYSJUMP:
		/*
		 * In the bugged image, the "system_reset == 0" condition
		 * (system reset flags == 0) in system_common_pre_init() will
		 * cause jump data to be discarded. This makes system reset
		 * flags reflect what was passed in bbram reset flags at the
		 * time of the jump instead of system reset flags passed via
		 * jump data. These flags can then get carried to power button
		 * initial state setup and cause issues.
		 */
		if (ag3s_are_any_non_off_bbram_reset_flags_set()) {
			/*
			 * If non-OFF bbram reset flags are present, then the
			 * jump will be bugged and system reset flags will
			 * reflect bbram reset flags at the time of the jump.
			 * Adjust OFF bbram reset flags to match the device
			 * state to ensure no issues in power button initial
			 * state setup after the jump...
			 */
			if (is_device_on) {
				/* The device is on, prevent abnormal state. */
				rfo_flags = AG3S_RFO_CLR_BBRM;
			} else {
				/* The device is off, prevent auto power on. */
				rfo_flags = AG3S_RFO_SET_BBRM;
			}
		} else {
			/*
			 * If non-OFF bbram reset flags are not set, then we
			 * just unset OFF bbram reset flags to make sure there
			 * are no bbram reset flags at all. This allows to
			 * avoid triggering the bug.
			 */
			rfo_flags = AG3S_RFO_CLR_BBRM;
		}

		/*
		 * This hook requires explicit console flushing to avoid
		 * truncated output, since it is called right before jumping.
		 */
		ag3s_update_off_reset_flags(rfo_flags);
		cflush();

		/* Proper flag sync skipped due to the pre-jump flag fix. */
		return;
	case AG3S_SYNC_HOOK_INIT_AFTER_PB_ISTATE:
	case AG3S_SYNC_HOOK_CHIPSET_STARTUP:
	case AG3S_SYNC_HOOK_NONE:
	default:
		/* No special action needed, just do proper flag sync. */
		break;
	}

	/* Sync OFF bbram reset flags to match the current after G3 behavior. */
	switch (state) {
	case EC_AFTER_G3_STATE_OFF:
		/* Keep the device off when power is restored. */
		rfo_flags = AG3S_RFO_SET_BBRM;
		break;
	case EC_AFTER_G3_STATE_ON:
		/* Auto power on the device when power is restored. */
		rfo_flags = AG3S_RFO_CLR_BBRM;
		break;
	case EC_AFTER_G3_STATE_PREVIOUS:
	default: /* Fallback to "Previous State" if state is invalid. */
		/* Match the device state when power is restored... */
		if (is_device_on) {
			/* The device is on, auto power it on. */
			rfo_flags = AG3S_RFO_CLR_BBRM;
		} else {
			/* The device is off, keep the device off. */
			rfo_flags = AG3S_RFO_SET_BBRM;
		}
		break;
	}

	/*
	 * Sync OFF system reset flags to match the device state, otherwise
	 * these flags can get carried via jump data to power button initial
	 * state setup and cause issues after any potential future jump...
	 */
	 if (is_device_on) {
		 /* The device is on, prevent abnormal state. */
		 rfo_flags |= AG3S_RFO_CLR_SYS;
	 } else {
		 /* The device is off, prevent auto power on. */
		 rfo_flags |= AG3S_RFO_SET_SYS;
	 }

update_off_reset_flags:
	ag3s_update_off_reset_flags(rfo_flags);
}

static inline void ag3s_sync(void)
{
	ag3s_sync_on_hook(AG3S_SYNC_HOOK_NONE);
}

static int host_command_after_g3_state_set(struct host_cmd_handler_args *args)
{
	const struct ec_params_after_g3_state_set *p = args->params;

	if (!ag3s_set_state(p->state))
		return EC_RES_ERROR;

	return EC_RES_SUCCESS;
}
DECLARE_HOST_COMMAND(EC_CMD_AFTER_G3_STATE_SET, host_command_after_g3_state_set,
		     EC_VER_MASK(0));

static int host_command_after_g3_state_get(struct host_cmd_handler_args *args)
{
	struct ec_response_after_g3_state_get *r = args->response;

	r->state = (int8_t)ag3s_get_state();
	args->response_size = sizeof(*r);

	return EC_RES_SUCCESS;
}
DECLARE_HOST_COMMAND(EC_CMD_AFTER_G3_STATE_GET, host_command_after_g3_state_get,
		     EC_VER_MASK(0));

static int command_after_g3_state(int argc, char **argv)
{
	enum ec_after_g3_state state;
	const char *state_name;

	if (argc < 2) {
		state = ag3s_get_state();
	} else if (argc == 2) {
		state = ag3s_find_state_by_name(argv[1]);
		if (state == EC_AFTER_G3_STATE_UNKNOWN)
			return EC_ERROR_PARAM1;

		if (!ag3s_set_state(state))
			return EC_ERROR_UNKNOWN;
	} else {
		return EC_ERROR_PARAM_COUNT;
	}

	state_name = ag3s_get_state_name(state);
	if (!state_name)
		return EC_ERROR_UNKNOWN;

	ccprintf("%s: %s\n", ag3s_title, state_name);

	return EC_SUCCESS;
}
DECLARE_CONSOLE_COMMAND(afterg3state, command_after_g3_state,
			"[off|on|previous]",
			"Get or set After G3 State value");

#else /* !CONFIG_AFTER_G3_STATE */

#ifdef CONFIG_POWER_BUTTON_INIT_IDLE
/*
 * Set/clear AP_IDLE flag. It's set when the system gracefully shuts down and
 * it's cleared when the system boots up. The result is the system tries to
 * go back to the previous state upon AC plug-in. If the system uncleanly
 * shuts down, it boots immediately. If the system shuts down gracefully,
 * it'll stay at S5 and wait for power button press.
 */
static void pb_chipset_startup(void)
{
	chip_save_reset_flags(chip_read_reset_flags() & ~EC_RESET_FLAG_AP_IDLE);
	system_clear_reset_flags(EC_RESET_FLAG_AP_IDLE);
	CPRINTS("Cleared AP_IDLE flag");
}
DECLARE_HOOK(HOOK_CHIPSET_STARTUP, pb_chipset_startup, HOOK_PRIO_DEFAULT);

static void pb_chipset_shutdown(void)
{
	/* Don't set AP_IDLE if shutting down due to power failure. */
	if (chipset_get_shutdown_reason() == CHIPSET_SHUTDOWN_POWERFAIL)
		return;

	chip_save_reset_flags(chip_read_reset_flags() | EC_RESET_FLAG_AP_IDLE);
	system_set_reset_flags(EC_RESET_FLAG_AP_IDLE);
	CPRINTS("Saved AP_IDLE flag");
}
DECLARE_HOOK(HOOK_CHIPSET_SHUTDOWN, pb_chipset_shutdown,
	     /*
	      * Slightly higher than handle_pending_reboot because
	      * it may clear AP_IDLE flag.
	      */
	     HOOK_PRIO_DEFAULT - 1);
#endif

#endif /* CONFIG_AFTER_G3_STATE */

/**
 * Handle debounced power button changing state.
 */
static void power_button_change_deferred(void)
{
	const int new_pressed = raw_power_button_pressed();

	/* Re-enable keyboard scanning if power button is no longer pressed */
	if (!new_pressed)
		keyboard_scan_enable(1, KB_SCAN_DISABLE_POWER_BUTTON);

	/* If power button hasn't changed state, nothing to do */
	if (new_pressed == debounced_power_pressed) {
		power_button_is_stable = 1;
		return;
	}

	debounced_power_pressed = new_pressed;
	power_button_is_stable = 1;

	CPRINTS("power button %s", new_pressed ? "pressed" : "released");
#ifdef CONFIG_VBOOT_EFS
	if (chipset_in_state(CHIPSET_STATE_ANY_OFF) &&
			new_pressed && !system_is_in_rw()) {
		CPRINTS("cold reset");
		cflush();
		system_reset(0);
		while (1)
			;
	}
#endif
	/* Call hooks */
	hook_notify(HOOK_POWER_BUTTON_CHANGE);

	/* Notify host if power button has been pressed */
	if (new_pressed)
		host_set_single_event(EC_HOST_EVENT_POWER_BUTTON);
}
DECLARE_DEFERRED(power_button_change_deferred);

void power_button_interrupt(enum gpio_signal signal)
{
	/*
	 * If power button is pressed, disable the matrix scan as soon as
	 * possible to reduce the risk of false-reboot triggered by those keys
	 * on the same column with refresh key.
	 */
	if (raw_power_button_pressed())
		keyboard_scan_enable(0, KB_SCAN_DISABLE_POWER_BUTTON);

	/* Reset power button debounce time */
	power_button_is_stable = 0;
	hook_call_deferred(&power_button_change_deferred_data,
			   PWRBTN_DEBOUNCE_US);
}

/*****************************************************************************/
/* Console commands */

static int command_powerbtn(int argc, char **argv)
{
	int ms = 200;  /* Press duration in ms */
	char *e;

	if (argc > 1) {
		ms = strtoi(argv[1], &e, 0);
		if (*e)
			return EC_ERROR_PARAM1;
	}

	ccprintf("Simulating %d ms power button press.\n", ms);
	simulate_power_pressed = 1;
	power_button_is_stable = 0;
	hook_call_deferred(&power_button_change_deferred_data, 0);

	msleep(ms);

	ccprintf("Simulating power button release.\n");
	simulate_power_pressed = 0;
	power_button_is_stable = 0;
	hook_call_deferred(&power_button_change_deferred_data, 0);

	return EC_SUCCESS;
}
DECLARE_CONSOLE_COMMAND(powerbtn, command_powerbtn,
			"[msec]",
			"Simulate power button press");

