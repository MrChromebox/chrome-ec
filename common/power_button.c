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

/*
 *====================
 * After G3 State
 *====================
 *
 * Required headers:
 *   #include "chipset.h"
 *   #include "common.h"
 *   #include "console.h"
 *   #include "hooks.h"
 *   #include "host_command.h"
 *   #include "system.h"
 *   #include "util.h"
 *
 * Only useful for boards with dedicated 3V coin battery to sustain bbram, and
 * only for EC images compiled with CONFIG_POWER_BUTTON_INIT_IDLE, as this
 * ensures that OFF bbram reset flags are retained.
 *
 * Available "board/NAME/board.h" config macros:
 *
 *==========
 *== Enable After G3 State, otherwise legacy OFF flag handling will be used.
 *==========
 * #define CONFIG_AFTER_G3_STATE
 *
 *==========
 *== Use RESET_FLAG_AP_OFF/EC_RESET_FLAG_AP_OFF as OFF flag, instead of using
 *== EC_RESET_FLAG_AP_IDLE as OFF flag.
 *==
 *== FIZZ, and older boards, do not support AP_IDLE, and should use AP_OFF as
 *== OFF flag. PUFF, and newer boards, have been migrated to AP_IDLE as OFF
 *== flag, and should use AP_IDLE.
 *==========
 * #define CONFIG_AFTER_G3_STATE_USE_AP_OFF_AS_OFF_FLAG
 *
 *==========
 *== Enables fix for bbram bug in older EC code, where presence of *any* bbram
 *== reset flags after a jump will discard jump data, and not mark image state
 *== as jumped to.
 *==
 *== Bug is caused by the fact that when EC image is executed after a jump, it
 *== first restores system reset flags from bbram reset flags via pathway:
 *==   main()->gpio_pre_init()->system_is_reboot_warm()->
 *==     system_check_reset_cause()
 *== Then it overwrites system reset flags with jump data reset flags via
 *== pathway:
 *==   main()->system_common_pre_init()
 *==
 *== In bugged firmware system_common_pre_init() will only restore jump data if
 *== condition "system_reset == 0" (system reset flags == 0) evaluates to TRUE.
 *==
 *== This creates a risk of causing auto power on if device is off, or risk of
 *== inducing abnormal state (initializing power button as off) if device is
 *== running.
 *==
 *== Upstream fixed this is newer EC code by dropping the problematic condition:
 *==   https://github.com/MrChromebox/chrome-ec/commit/e833bdec81b9
 *== However, this is mainly suitable for fixing RW (jump RO->RW), as fixing RO
 *== would require overwriting factory RO, which poses a risk of bricking
 *== device, with no easy way to recover.
 *==
 *== Fixing RO (jump RW->RO), without actually changing RO, has two pathways:
 *==   1) Non-OFF bbram reset flags are NOT set when jumping. Fix will unset
 *==      any set OFF bbram reset flags. This has the following potential side
 *==      effects:
 *==        - If EC_AFTER_G3_STATE_OFF is set, while device is off/running, then
 *==          losing power before returning to RW will auto power on device when
 *==          power is restored.
 *==        - If EC_AFTER_G3_STATE_OFF/EC_AFTER_G3_STATE_PREVIOUS is set, while
 *==          device is off, then losing power power before returning to RW will
 *==          auto power on device when power is restored.
 *==   2) Non-OFF bbram reset flags are set when jumping. This means that jump
 *==      will be bugged anyway, since we do not want to touch non-OFF bbram
 *==      reset flags. Fix will adjust OFF bbram reset flags in a way to ensure
 *==      no abnormal state after jump. This has the following potential side
 *==      effects:
 *==        - If EC_AFTER_G3_STATE_OFF is set, while device is running, then
 *==          losing power before returning to RW will auto power on device when
 *==          power is restored.
 *==        - If EC_AFTER_G3_STATE_ON is set, while device is off, then losing
 *==          power before returning to RW will keep device off when power is
 *==          restored.
 *==
 *== Useless for PUFF, and newer boards, as these should not be affected.
 *==========
 * #define CONFIG_AFTER_G3_STATE_SYSJUMP_BBRAM_BUGFIX
 *
 *==========
 *== DEDEDE, and newer boards, do not pass OFF system reset flags via jump data
 *== when jumping, even if set prior to jumping. If RO is unmodified, then
 *== jumping to RO (RW->RO), while device is off, will always cause auto power
 *== on. Jump to RW (RO->RW) is not affected, since RW can restore OFF system
 *== reset flags based on checking existing OFF bbram reset flags.
 *==
 *== Mitigation requires changing code for RW (only if RO is unmodified):
 *== common/system.c: int system_run_image_copy_with_flags(enum ec_image copy,
 *==                                                  uint32_t add_reset_flags)
 *==     [...]
 *==     system_set_reset_flags(add_reset_flags);
 *==     [...]
 *==   ->
 *==     [...]
 *== #ifdef CONFIG_AFTER_G3_STATE_SYSJUMP_PRESERVE_JDATA_AP_IDLE_IF_OFF
 *==      * Preserve system reset flag AP_IDLE (if set) before jumping, while
 *==      * device is off, to pass it via jump data, preventing auto powering on
 *==      * after jumping.
 *==     if (chipset_in_state(CHIPSET_STATE_ANY_OFF))
 *==         add_reset_flags |= system_get_reset_flags() &
 *==                            EC_RESET_FLAG_AP_IDLE;
 *== #endif
 *==     system_set_reset_flags(add_reset_flags);
 *==     [...]
 *==
 *== And defining the intent to use the fix:
 *==========
 * #define CONFIG_AFTER_G3_STATE_SYSJUMP_PRESERVE_JDATA_AP_IDLE_IF_OFF
 */
#ifdef CONFIG_AFTER_G3_STATE

static const char *ag3s_title = "After G3 State";

/*
 * Value set here (enum ec_after_g3_state) represents the default after G3
 * behavior, which can be later changed via command/coreboot.
 */
#define AG3S_DEFAULT_STATE  (EC_AFTER_G3_STATE_PREVIOUS)

static enum ec_after_g3_state ag3s_state = AG3S_DEFAULT_STATE;

/*
 * Order should match enum ec_after_g3_state, starting from value 1.
 * Special values are excluded.
 */
static const char *ag3s_name_table[] = { "off", "on", "previous" };

static inline const char *ag3s_get_state_name(enum ec_after_g3_state state)
{
	return ((state >= EC_AFTER_G3_STATE_OFF &&
	         state <= EC_AFTER_G3_STATE_PREVIOUS) ?
			ag3s_name_table[state - 1] : NULL);
}

static enum ec_after_g3_state ag3s_find_state_by_name(const char *name)
{
	int i, len;
	for (i = 0, len = ARRAY_SIZE(ag3s_name_table); i < len; ++i) {
		if (!strcasecmp(name, ag3s_name_table[i]))
			return i + 1;
	}
	return EC_AFTER_G3_STATE_UNKNOWN;
}

static inline enum ec_after_g3_state ag3s_get_state(void)
{
	return ag3s_state;
}

static inline void ag3s_sync_reset_flags(void);

static int ag3s_set_state_ex(enum ec_after_g3_state state, int preserve_state,
		int sync_reset_flags)
{
	const char *state_name = ag3s_get_state_name(state);
	if (!state_name)
		return 0;

	CPRINTS("%s: Updating to '%s'", ag3s_title, state_name);

	ag3s_state = state;

	if (preserve_state) {
		/* TODO: Coreboot-independent preservation. */
	}

	if (sync_reset_flags)
		ag3s_sync_reset_flags();

	return 1;
}

static inline int ag3s_set_state(enum ec_after_g3_state state)
{
	return ag3s_set_state_ex(state, 1, 1);
}

/* Clearing operation always take precedence over saving for the same target. */
enum ag3s_reset_flags_op_flags {
	AG3S_RFO_SET_BBRM = (1 << 0),
	AG3S_RFO_SET_SYS  = (1 << 1),
	AG3S_RFO_SET      = (AG3S_RFO_SET_BBRM | AG3S_RFO_SET_SYS),
	AG3S_RFO_CLR_BBRM = (1 << 2),
	AG3S_RFO_CLR_SYS  = (1 << 3),
	AG3S_RFO_CLR      = (AG3S_RFO_CLR_BBRM | AG3S_RFO_CLR_SYS)
};

static void ag3s_update_reset_flags(const char *name, uint32_t flags,
		int rfo_flags)
{
	static const char *str_op_set = "+";
	static const char *str_op_clr = "-";
	static const char *str_op_eq  = "=";

	const char *op_bbram, *op_sys;

	if (!rfo_flags)
		return;

	if (rfo_flags & AG3S_RFO_CLR_BBRM) {
		chip_save_reset_flags(chip_read_reset_flags() & ~flags);
		op_bbram = str_op_clr;
	} else if (rfo_flags & AG3S_RFO_SET_BBRM) {
		chip_save_reset_flags(chip_read_reset_flags() | flags);
		op_bbram = str_op_set;
	} else {
		op_bbram = str_op_eq;
	}

	if (rfo_flags & AG3S_RFO_CLR_SYS) {
		system_clear_reset_flags(flags);
		op_sys = str_op_clr;
	} else if (rfo_flags & AG3S_RFO_SET_SYS) {
		system_set_reset_flags(flags);
		op_sys = str_op_set;
	} else {
		op_sys = str_op_eq;
	}

	CPRINTS("%s: %s flag updated (%sbbram, %ssys)", ag3s_title, name,
			op_bbram, op_sys);
}

/* Use AP_OFF as OFF flag if requested. */
#if defined(CONFIG_AFTER_G3_STATE_USE_AP_OFF_AS_OFF_FLAG)
  /*
   * In newer EC code RESET_FLAG_AP_OFF could have been migrated to
   * EC_RESET_FLAG_AP_OFF, AG3S_RESET_FLAG_AP_OFF macro is a shortcut to point
   * to the existing AP_OFF flag.
   */
# if defined(RESET_FLAG_AP_OFF)
#  define AG3S_RESET_FLAG_AP_OFF  (RESET_FLAG_AP_OFF)
# elif defined(EC_RESET_FLAG_AP_OFF)
#  define AG3S_RESET_FLAG_AP_OFF  (EC_RESET_FLAG_AP_OFF)
# else
#  error "CONFIG_AFTER_G3_STATE_USE_AP_OFF_AS_OFF_FLAG is defined, but" \
         " required RESET_FLAG_AP_OFF or EC_RESET_FLAG_AP_OFF is not available"
# endif
# define AG3S_RESET_FLAG_OFF       (AG3S_RESET_FLAG_AP_OFF)
# define AG3S_RESET_FLAG_OFF_NAME  "AP_OFF"
#else /* Fallback to AP_IDLE as OFF flag if AP_OFF is not requested. */
# if !defined(EC_RESET_FLAG_AP_IDLE)
#  error "Required EC_RESET_FLAG_AP_IDLE is not available"
# endif
# define AG3S_RESET_FLAG_OFF       (EC_RESET_FLAG_AP_IDLE)
# define AG3S_RESET_FLAG_OFF_NAME  "AP_IDLE"
#endif

static inline void ag3s_update_off_reset_flags(int rfo_flags)
{
	ag3s_update_reset_flags(AG3S_RESET_FLAG_OFF_NAME, AG3S_RESET_FLAG_OFF,
			rfo_flags);
}

static inline int ag3s_are_any_off_bbram_reset_flags_set(void)
{
	return !!(chip_read_reset_flags() & AG3S_RESET_FLAG_OFF);
}

#ifdef CONFIG_AFTER_G3_STATE_SYSJUMP_BBRAM_BUGFIX
static inline int ag3s_are_any_non_off_bbram_reset_flags_set(void)
{
	return !!(chip_read_reset_flags() & ~AG3S_RESET_FLAG_OFF);
}
#endif

/*
 * Hard/soft EC reboot is not hookable, but it will always unset OFF bbram reset
 * flags via system_reset(), unless specifically instructed not to do so. There
 * should be no risk of passing unwanted OFF bbram reset flags to RO, which
 * could cause issues.
 */
enum ag3s_sync_hook {
	/*
	 * HOOK_INIT with HOOK_PRIO_DEFAULT-1.
	 *
	 * Called before power button initial state is configured (on HOOK_INIT
	 * with HOOK_PRIO_DEFAULT). Changing OFF system reset flags here can
	 * influence power button initial state, either allowing device to auto
	 * power on, or prevent auto power on.
	 */
	AG3S_SYNC_HOOK_INIT_BEFORE_PB_ISTATE = 1,
	/*
	 * HOOK_INIT with HOOK_PRIO_DEFAULT+1.
	 *
	 * Called after power button initial state is configured.
	 *
	 * It should be safe now to properly sync reset flags, to match
	 * currently set after G3 behavior.
	 */
	AG3S_SYNC_HOOK_INIT_AFTER_PB_ISTATE,
	/*
	 * HOOK_CHIPSET_STARTUP with HOOK_PRIO_DEFAULT.
	 *
	 * Called on chipset startup.
	 *
	 * Checking for CHIPSET_STATE_ANY_OFF will evaluate to FALSE.
	 *
	 * OFF system reset flags should be unset here, while OFF bbram reset
	 * flags can be set based on after G3 behavior.
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
	 *
	 * OFF system reset flags should be set here, while bbram OFF reset
	 * flags can be set based on after G3 behavior.
	 */
	AG3S_SYNC_HOOK_CHIPSET_SHUTDOWN,
#ifdef CONFIG_AFTER_G3_STATE_SYSJUMP_BBRAM_BUGFIX
	/*
	 * HOOK_SYSJUMP with HOOK_PRIO_LAST.
	 *
	 * Called before making a jump to another image, as late as possible.
	 *
	 * Hard/soft EC reboot, which automatically reverts to RO, does not emit
	 * this hook.
	 *
	 * System reset flags are saved to jump data before HOOK_SYSJUMP is
	 * called. Any attempt to change these flags within a hook will not make
	 * any difference:
	 *   static void jump_to_image(uintptr_t init_addr):
	 *       [...]
	 *       // reset_flags in this context point to global variable storing
	 *       // active system reset flags
	 *       jdata->reset_flags = reset_flags;
	 *       [...]
	 *       hook_notify(HOOK_SYSJUMP);
	 *       [...]
	 *
	 * This hook requires explicit console flushing to avoid truncated
	 * output, since it is called right before jumping.
	 */
	AG3S_SYNC_HOOK_SYSJUMP,
#endif /* CONFIG_AFTER_G3_STATE_SYSJUMP_BBRAM_BUGFIX */

	/* Special Values */

	/* Reserved for non-hook related call. */
	AG3S_SYNC_HOOK_NONE = 0
};

static void ag3s_sync_on_hook(enum ag3s_sync_hook sync_hook);

#define AG3S_SYNC_ON_HOOK(d_sync_hook_enum, d_hook, d_hook_priority) \
	static void ag3s_sync_on_hook_##d_sync_hook_enum(void) \
	{ \
		ag3s_sync_on_hook(d_sync_hook_enum); \
	} \
	DECLARE_HOOK(d_hook, ag3s_sync_on_hook_##d_sync_hook_enum, \
			d_hook_priority)

/* Syncing on all non-special values in enum ag3s_sync_hook. */
AG3S_SYNC_ON_HOOK(AG3S_SYNC_HOOK_INIT_BEFORE_PB_ISTATE, HOOK_INIT,
		HOOK_PRIO_DEFAULT-1);
AG3S_SYNC_ON_HOOK(AG3S_SYNC_HOOK_INIT_AFTER_PB_ISTATE, HOOK_INIT,
		HOOK_PRIO_DEFAULT+1);
AG3S_SYNC_ON_HOOK(AG3S_SYNC_HOOK_CHIPSET_STARTUP, HOOK_CHIPSET_STARTUP,
		HOOK_PRIO_DEFAULT);
AG3S_SYNC_ON_HOOK(AG3S_SYNC_HOOK_CHIPSET_SHUTDOWN, HOOK_CHIPSET_SHUTDOWN,
		HOOK_PRIO_DEFAULT-1);
#ifdef CONFIG_AFTER_G3_STATE_SYSJUMP_BBRAM_BUGFIX
AG3S_SYNC_ON_HOOK(AG3S_SYNC_HOOK_SYSJUMP, HOOK_SYSJUMP, HOOK_PRIO_LAST);
#endif

/*
 * Checks for errors on chipset shutdown, allowing to determine if shutdown is
 * considered graceful.
 *
 * Returns 0 if shutdown is graceful, otherwise shutdown is not graceful.
 */
static inline int ag3s_on_chipset_shutdown_check_err(void)
{
	/* Shutting down due to power failure. */
	if (chipset_get_shutdown_reason() == CHIPSET_SHUTDOWN_POWERFAIL)
		return 1;

	return 0;
}

static void ag3s_sync_on_hook(enum ag3s_sync_hook sync_hook)
{
	enum ec_after_g3_state state;
	const char *state_name;
	/*
	 * CHIPSET_STATE_ANY_OFF will not evaluate to TRUE during chipset
	 * shutdown, so instead we always assume system is going off.
	 */
	int any_off = (sync_hook == AG3S_SYNC_HOOK_CHIPSET_SHUTDOWN ||
			chipset_in_state(CHIPSET_STATE_ANY_OFF));
	int rfo_flags = 0;

	state = ag3s_get_state();
	state_name = ag3s_get_state_name(state);
	if (!state_name) {
		state_name = "x";
		/* Fallback to default state if state is invalid. */
		state = AG3S_DEFAULT_STATE;
	}

	CPRINTS("%s: Sync on hook %d (%s)", ag3s_title, sync_hook, state_name);

	/*
	 * Some hooks may require special handling of reset flags. These hooks
	 * take precedence over proper reset flag syncing.
	 */
	switch(sync_hook) {
	case AG3S_SYNC_HOOK_INIT_BEFORE_PB_ISTATE:
		if (!any_off) {
			/*
			 * OFF system reset flags are always unset if device is
			 * running, otherwise power button initial state setup
			 * may trigger buggy behavior.
			 */
			rfo_flags = AG3S_RFO_CLR_SYS;
		} else if (ag3s_are_any_off_bbram_reset_flags_set()) {
			/*
			 * Presence of OFF bbram reset flags, while device is
			 * off, indicates that device should remain off. OFF
			 * system reset flags are explicitly set, to ensure
			 * power button initial state configuration will inhibit
			 * auto power on.
			 */
			rfo_flags = AG3S_RFO_SET_SYS;
		}

		/* Proper flag syncing will be done after power button init. */
		goto update_reset_flags;
	case AG3S_SYNC_HOOK_CHIPSET_SHUTDOWN:
		/*
		 * "Previous State" behavior should NOT save OFF bbram reset
		 * flags if there are errors on shutdown. This ensures that
		 * device will only stay off, when power is reapplied, if
		 * shutdown was graceful, otherwise it will auto power on.
		 */
		if (state == EC_AFTER_G3_STATE_PREVIOUS &&
		    ag3s_on_chipset_shutdown_check_err())
			return;
		break;
#ifdef CONFIG_AFTER_G3_STATE_SYSJUMP_BBRAM_BUGFIX
	case AG3S_SYNC_HOOK_SYSJUMP:
		/*
		 * Bugged EC will discard jump data if *any* bbram reset flags
		 * are present after a jump.
		 *
		 * We always unset OFF bbram reset flags, unless there are other
		 * reset flags in bbram, then jump will be bugged anyway, so we
		 * set OFF bbram reset flags based on current device state.
		 *
		 * Regardless of the pathway we have taken, device may not
		 * respect currently set after G3 behavior when power is
		 * lost/restored, but at least we can avoid issues with device
		 * abnormal states if running, or auto powering on if off.
		 */
		if (!ag3s_are_any_non_off_bbram_reset_flags_set()) {
			rfo_flags = AG3S_RFO_CLR_BBRM;
		} else {
			rfo_flags = (any_off ?
					AG3S_RFO_SET_BBRM : AG3S_RFO_CLR_BBRM);
		}

		/*
		 * This hook requires explicit console flushing to avoid
		 * truncated output, since it is called right before jumping.
		 */
		ag3s_update_off_reset_flags(rfo_flags);
		cflush();

		/* Proper flag syncing skipped due to pending image jump. */
		return;
#endif /* CONFIG_AFTER_G3_STATE_SYSJUMP_BBRAM_BUGFIX */
	case AG3S_SYNC_HOOK_INIT_AFTER_PB_ISTATE:
	case AG3S_SYNC_HOOK_CHIPSET_STARTUP:
	case AG3S_SYNC_HOOK_NONE:
	default:
		/* No special action needed, just do a proper flag syncing. */
		break;
	}

	/* Syncing OFF reset flags based on currently set after G3 behavior. */
	switch (state) {
	case EC_AFTER_G3_STATE_OFF:
		if (!any_off) {
			/*
			 * If device is running, then OFF system reset flags are
			 * always unset, otherwise these flags can get carried
			 * via jump data and trigger buggy behavior.
			 */
			rfo_flags = AG3S_RFO_CLR_SYS;

			/*
			 * OFF bbram reset flags are always set. After power
			 * failure device will stay off when power is reapplied.
			 */
			rfo_flags |= AG3S_RFO_SET_BBRM;
		} else {
			/*
			 * If device is off, then OFF reset flags are always
			 * set, after power failure device will stay off when
			 * power is reapplied.
			 */
			rfo_flags = AG3S_RFO_SET;
		}
		break;
	case EC_AFTER_G3_STATE_ON:
		if (!any_off) {
			/*
			 * If device is running, then OFF reset flags are unset.
			 * After power failure device will auto power on when
			 * power is reapplied.
			 */
			rfo_flags = AG3S_RFO_CLR;
		} else {
			/*
			 * If device is off, then OFF bbram reset flags are
			 * unset. After power failure device will auto power on
			 * when power is reapplied.
			 */
			rfo_flags = AG3S_RFO_CLR_BBRM;

			/*
			 * OFF system reset flags are set to prevent device from
			 * auto powering on after jumping (via jump data). These
			 * flags are lost on power loss, therefore should not
			 * interfere with auto powering on.
			 */
			rfo_flags |= AG3S_RFO_SET_SYS;
		}
		break;
	case EC_AFTER_G3_STATE_PREVIOUS:
	default: /* Fallback to "Previous State" if state is invalid. */
		/*
		 * If device is off, then OFF reset flags are set, otherwise
		 * these flags are unset. After power failure device will stay
		 * off when power is reapplied, but only if it was off before
		 * power was lost, otherwise it will auto power on.
		 */
		rfo_flags = (any_off ? AG3S_RFO_SET : AG3S_RFO_CLR);
		break;
	}

update_reset_flags:
	ag3s_update_off_reset_flags(rfo_flags);
}

static inline void ag3s_sync_reset_flags(void)
{
	ag3s_sync_on_hook(AG3S_SYNC_HOOK_NONE);
}

static int host_command_after_g3_state(struct host_cmd_handler_args *args)
{
	const struct ec_params_after_g3_state *p = args->params;
	struct ec_response_after_g3_state *r = args->response;
	enum ec_after_g3_state set_state = p->set_state;
	int rv = EC_RES_SUCCESS;

	if (set_state != EC_AFTER_G3_STATE_GET && !ag3s_set_state(set_state))
		rv = EC_RES_ERROR;

	r->cur_state = (uint8_t)(rv == EC_RES_SUCCESS ?
			ag3s_get_state() : EC_AFTER_G3_STATE_ERROR);
	args->response_size = sizeof(*r);

	return rv;
}
DECLARE_HOST_COMMAND(EC_CMD_AFTER_G3_STATE, host_command_after_g3_state,
		EC_VER_MASK(0));

static int command_after_g3_state(int argc, char **argv)
{
	const char *state_name;

	if (argc < 2) {
		state_name = ag3s_get_state_name(ag3s_get_state());
		if (!state_name)
			return EC_ERROR_UNKNOWN;
	} else if (argc == 2) {
		enum ec_after_g3_state state = ag3s_find_state_by_name(argv[1]);
		if (state == EC_AFTER_G3_STATE_UNKNOWN)
			return EC_ERROR_PARAM1;
		/* Normalize state name. */
		state_name = ag3s_get_state_name(state);
		if (!state_name || !ag3s_set_state(state))
			return EC_ERROR_UNKNOWN;
	} else {
		return EC_ERROR_PARAM_COUNT;
	}

	ccprintf("%s: %s\n", ag3s_title, state_name);

	return EC_SUCCESS;
}
DECLARE_CONSOLE_COMMAND(afterg3state, command_after_g3_state,
		"[off|on|previous]",
		"Set and/or get After G3 State value");

/*
 * Only use legacy code for handling OFF reset flags if CONFIG_AFTER_G3_STATE is
 * not defined. This way any upstream changes affecting legacy code could still
 * be merged, but will not be compiled, and will not affect After G3 State.
 */
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

