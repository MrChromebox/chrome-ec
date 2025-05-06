/* Copyright 2025 The ChromiumOS Authors
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 */

/* Physical fans. These are logically separate from pwm_channels. */

#include "chipset.h"
#include "common.h"
#include "compile_time_macros.h"
#include "console.h"
#include "fan.h"
#include "fan_chip.h"
#include "hooks.h"
#include "pwm.h"
#include "temp_sensor.h"

/* MFT channels. These are logically separate from pwm_channels. */
const struct mft_t mft_channels[] = {
	[MFT_CH_0] = {
		.module = NPCX_MFT_MODULE_2,
		.clk_src = TCKC_LFCLK,
		.pwm_id = PWM_CH_FAN,
	},
};
BUILD_ASSERT(ARRAY_SIZE(mft_channels) == MFT_CH_COUNT);

static const struct fan_conf fan_conf_0 = {
	.flags = FAN_USE_RPM_MODE,
	.ch = MFT_CH_0, /* Use MFT id to control fan */
	.pgood_gpio = -1,
	.enable_gpio = GPIO_EN_PP5000_FAN,
};

/*
 * TOOD(b/197478860): need to update for real fan
 *
 * Prototype fan spins at about 7200 RPM at 100% PWM.
 * Set minimum at around 30% PWM.
 */
static const struct fan_rpm fan_rpm_0 = {
	.rpm_min = 1800,
	.rpm_start = 2350,
	.rpm_max = 4300,
};

const struct fan_t fans[FAN_CH_COUNT] = {
	[FAN_CH_0] = {
		.conf = &fan_conf_0,
		.rpm = &fan_rpm_0,
	},
};

struct fan_step {
	/*
	 * Sensor 1~3 trigger point, set -1 if we're not using this
	 * sensor to determine fan speed.
	 */
	int8_t on[TEMP_SENSOR_COUNT];
	/*
	 * Sensor 1~3 trigger point, set -1 if we're not using this
	 * sensor to determine fan speed.
	 */
	int8_t off[TEMP_SENSOR_COUNT];
	/* Fan rpm */
	uint16_t rpm[FAN_CH_COUNT];
};

static const struct fan_step fan_table[] = {
	{
		/* level 0 */
		.on = { 25, -1, -1, -1 },
		.off = { 0, -1, -1, -1 },
		.rpm = { 0 },
	},
	{
		/* level 1 */
		.on = { 37, -1, -1, -1 },
		.off = { 33, -1, -1, -1 },
		.rpm = { 1950 },
	},
	{
		/* level 2 */
		.on = { 41, -1, -1, -1 },
		.off = { 36, -1, -1, -1 },
		.rpm = { 2350 },
	},
	{
		/* level 3 */
		.on = { 43, -1, -1, -1 },
		.off = { 40, -1, -1, -1 },
		.rpm = { 2700 },
	},
	{
		/* level 4 */
		.on = { 46, -1, -1, -1 },
		.off = { 42, -1, -1, -1 },
		.rpm = { 2950 },
	},
	{
		/* level 5 */
		.on = { 50, -1, -1, -1 },
		.off = { 45, -1, -1, -1 },
		.rpm = { 3250 },
	},
	{
		/* level 6 */
		.on = { 52, -1, -1, -1 },
		.off = { 49, -1, -1, -1 },
		.rpm = { 3620 },
	},
	{
		/* level 7 */
		.on = { 65, -1, -1, -1 },
		.off = { 59, -1, -1, -1 },
		.rpm = { 4050 },
	},
};

const int num_fan_levels = ARRAY_SIZE(fan_table);

#define NUM_FAN_LEVELS ARRAY_SIZE(fan_table)

int fan_table_to_rpm(int fan, int *temp, enum temp_sensor_id temp_sensor)
{
	const struct fan_step *fan_step_table;
	/* current fan level */
	static int current_level;
	/* previous sensor temperature */
	static int prev_tmp[TEMP_SENSOR_COUNT];

	int i;
	uint8_t fan_table_size;

	fan_step_table = fan_table;
	fan_table_size = ARRAY_SIZE(fan_table);

	/*
	 * Compare the current and previous temperature, we have
	 * the three paths :
	 *  1. decreasing path. (check the release point)
	 *  2. increasing path. (check the trigger point)
	 *  3. invariant path. (return the current RPM)
	 */

	if (temp[temp_sensor] < prev_tmp[temp_sensor]) {
		for (i = current_level; i > 0; i--) {
			if (temp[temp_sensor] <=
			    fan_step_table[i].off[temp_sensor])
				current_level = i - 1;
			else
				break;
		}
	} else if (temp[temp_sensor] > prev_tmp[temp_sensor]) {
		for (i = current_level; i < fan_table_size; i++) {
			if (temp[temp_sensor] >=
			    fan_step_table[i].on[temp_sensor])
				current_level = i;
			else
				break;
		}
	}

	if (current_level < 0)
		current_level = 0;

	if (current_level >= fan_table_size)
		current_level = fan_table_size - 1;

	prev_tmp[temp_sensor] = temp[temp_sensor];

	return fan_step_table[current_level].rpm[fan];
}

void board_override_fan_control(int fan, int *tmp)
{
	if (chipset_in_state(CHIPSET_STATE_ON | CHIPSET_STATE_ANY_SUSPEND)) {
		int new_rpm = fan_table_to_rpm(fan, tmp, TEMP_SENSOR_1_CPU);

		if (new_rpm != fan_get_rpm_target(FAN_CH(fan))) {
			ccprints("Setting fan RPM to %d", new_rpm);
			fan_set_rpm_mode(FAN_CH(fan), 1);
			fan_set_rpm_target(FAN_CH(fan), new_rpm);
		}
	}
}
