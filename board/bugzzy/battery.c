/* Copyright 2021 The ChromiumOS Authors
 * Use of this source code is governed by a BSD-style license that can be
 * found in the LICENSE file.
 *
 * Battery pack vendor provided charging profile
 */

#include "battery_fuel_gauge.h"
#include "charge_manager.h"
#include "charge_state.h"
#include "common.h"
#include "gpio.h"
#include "hooks.h"
#include "math_util.h"
#include "usb_pd.h"
#include "util.h"

#define CPRINTS(format, args...) cprints(CC_CHARGER, format, ##args)

/*
 * Battery info for all bugzzy battery types. Note that the fields
 * start_charging_min/max and charging_min/max are not used for the charger.
 * The effective temperature limits are given by discharging_min/max_c.
 *
 * Fuel Gauge (FG) parameters which are used for determining if the battery
 * is connected, the appropriate ship mode (battery cutoff) command, and the
 * charge/discharge FETs status.
 *
 * Ship mode (battery cutoff) requires 2 writes to the appropriate smart battery
 * register. For some batteries, the charge/discharge FET bits are set when
 * charging/discharging is active, in other types, these bits set mean that
 * charging/discharging is disabled. Therefore, in addition to the mask for
 * these bits, a disconnect value must be specified. Note that for TI fuel
 * gauge, the charge/discharge FET status is found in Operation Status (0x54),
 * but a read of Manufacturer Access (0x00) will return the lower 16 bits of
 * Operation status which contains the FET status bits.
 *
 * The assumption for battery types supported is that the charge/discharge FET
 * status can be read with a sb_read() command and therefore, only the register
 * address, mask, and disconnect value need to be provided.
 *
 * Battery FET Status in Manufacture Access : bit15 & bit14
 *  b'00 - dfet : on / cfet : on
 *  b'01 - dfet : on / cfet : off
 *  b'10 - dfet : off / cfet : off
 *  b'11 - dfet : off / cfet : on
 *  The value b'10 is disconnect_val, so we can use b'01 for cfet_off_val
 */

/* charging current is limited to 0.45C */
#define CHARGING_CURRENT_45C 2601

/* charging data */
#define DEFAULT_DESIGN_CAPACITY 5723
#define CHARGING_VOLTAGE_SDI_4404D57 8600
#define CHARGING_VOLTAGE_SDI_4404D57M 8700
#define BAT_SERIES 2
#define BAT_MAX_SERIES 4
#define TC_CHARGING_VOLTAGE 8300
#define CRATE_100 80
#define CFACT_10 9
#define BAT_CELL_VOLT_SPEC 4400
#define BAT_CELL_OVERVOLTAGE (BAT_CELL_VOLT_SPEC - 50)
#define BAT_CELL_MARGIN (BAT_CELL_VOLT_SPEC - 32)
#define BAT_CELL_READY_OVER_VOLT 4150

/* parameter for battery lifetime extension */
#define RATE_FCC_DC_85 85
#define RATE_FCC_DC_99 99

#define DROP_CELL_VOLT_MV 16 // 2S battery
#define DROP_VOLT_MV (DROP_CELL_VOLT_MV * BAT_SERIES)

struct therm_item {
	int low;
	int high;
};

static enum {
	STOP_LOW_TEMP = 0,
	LOW_TEMP3,
	LOW_TEMP2,
	LOW_TEMP1,
	NORMAL_TEMP,
	HIGH_TEMP,
	STOP_HIGH_TEMP,
	TEMP_TYPE_COUNT,
} temp_zone = NORMAL_TEMP;

static const struct therm_item bat_temp_table[] = {
	{ .low = -100, .high = 2 }, { .low = 0, .high = 10 },
	{ .low = 8, .high = 17 },   { .low = 15, .high = 20 },
	{ .low = 18, .high = 42 },  { .low = 40, .high = 51 },
	{ .low = 46, .high = 500 },
};
BUILD_ASSERT(ARRAY_SIZE(bat_temp_table) == TEMP_TYPE_COUNT);

static struct charge_state_data *charging_data;
static int design_capacity = 0;
static uint16_t bat_cell_volt[BAT_MAX_SERIES];
static uint8_t bat_cell_over_volt_flag;
static int bat_cell_ovp_volt;
static int bat_drop_voltage = 0;

static enum battery_type board_battery_type = BATTERY_TYPE_COUNT;
static enum battery_present batt_pres_prev = BP_NOT_SURE;
const struct batt_conf_embed board_battery_info[] = {
	/* SDI Battery Information */
	[BATTERY_SDI_4404D57M] = {
		.manuf_name = "SDI",
		.device_name = "4404D57M",
		.config = {
			.fuel_gauge = {
				.ship_mode = {
					.reg_addr = 0x00,
					.reg_data = { 0x0010, 0x0010 },
				},
				.fet = {
					.reg_addr = 0x00,
					.reg_mask = 0xc000,
					.disconnect_val = 0x8000,
					.cfet_mask = 0xc000,
					.cfet_off_val = 0x4000,
				},
			},
			.batt_info = {
				.voltage_max            = 8700,
				.voltage_normal         = 7700, /* mV */
				.voltage_min            = 6000, /* mV */
				.precharge_current      = 200,  /* mA */
				.start_charging_min_c   = 0,
				.start_charging_max_c   = 45,
				.charging_min_c         = 0,
				.charging_max_c         = 55,
				.discharging_min_c      = -20,
				.discharging_max_c      = 70,
			},
		},
	},
	/* SDI Battery Information */
	[BATTERY_SDI_4404D57] = {
		.manuf_name = "SDI",
		.device_name = "4404D57",
		.config = {
			.fuel_gauge = {
				.ship_mode = {
					.reg_addr = 0x00,
					.reg_data = { 0x0010, 0x0010 },
				},
				.fet = {
					.reg_addr = 0x00,
					.reg_mask = 0xc000,
					.disconnect_val = 0x8000,
					.cfet_mask = 0xc000,
					.cfet_off_val = 0x4000,
				},
			},
			.batt_info = {
				.voltage_max            = 8600,
				.voltage_normal         = 7700, /* mV */
				.voltage_min            = 6000, /* mV */
				.precharge_current      = 200,  /* mA */
				.start_charging_min_c   = 0,
				.start_charging_max_c   = 45,
				.charging_min_c         = 0,
				.charging_max_c         = 55,
				.discharging_min_c      = -20,
				.discharging_max_c      = 70,
			},
		},
	},
};
BUILD_ASSERT(ARRAY_SIZE(board_battery_info) == BATTERY_TYPE_COUNT);

const enum battery_type DEFAULT_BATTERY_TYPE = BATTERY_SDI_4404D57M;

void find_battery_thermal_zone(int bat_temp)
{
	static int prev_temp;
	int i;

	if (bat_temp < prev_temp) {
		for (i = temp_zone; i > 0; i--) {
			if (bat_temp < bat_temp_table[i].low)
				temp_zone = i - 1;
			else
				break;
		}
	} else if (bat_temp > prev_temp) {
		for (i = temp_zone; i < ARRAY_SIZE(bat_temp_table); i++) {
			if (bat_temp >= bat_temp_table[i].high)
				temp_zone = i + 1;
			else
				break;
		}
	}

	if (temp_zone < 0)
		temp_zone = 0;

	if (temp_zone >= ARRAY_SIZE(bat_temp_table))
		temp_zone = ARRAY_SIZE(bat_temp_table) - 1;

	prev_temp = bat_temp;
}

void check_battery_cell_voltage(void)
{
	int rv;
	static int cell_check_flag = 0;
	static uint8_t idx = 0;
	int data;
	uint16_t max_voltage, min_voltage, delta_voltage;
	static uint8_t over_volt_count[BAT_MAX_SERIES] = {
		0,
	};

	if (charging_data->state == ST_CHARGE) {
		cell_check_flag = 1;
		rv = sb_read(SB_OPTIONAL_MFG_FUNC1 + idx, &data);
		if (rv)
			return;
		bat_cell_volt[idx] = data;

		if (bat_cell_volt[idx] >= BAT_CELL_OVERVOLTAGE &&
		    bat_cell_over_volt_flag == 0) {
			over_volt_count[idx]++;
			if (over_volt_count[idx] >= 4) {
				max_voltage = min_voltage = bat_cell_volt[idx];
				for (int i = 0; i < BAT_MAX_SERIES; i++) {
					if (bat_cell_volt[i] > max_voltage)
						max_voltage = bat_cell_volt[i];
					if (bat_cell_volt[i] < min_voltage &&
					    bat_cell_volt[i] != 0)
						min_voltage = bat_cell_volt[i];
				}
				delta_voltage = max_voltage - min_voltage;
				if ((delta_voltage < 600) &&
				    (delta_voltage > 10)) {
					bat_cell_over_volt_flag = 1;
					bat_cell_ovp_volt =
						BAT_CELL_MARGIN * BAT_SERIES -
						delta_voltage *
							(BAT_SERIES - 1);
				}
			}
		} else {
			over_volt_count[idx] = 0;
		}

		idx++;
		if (idx >= BAT_MAX_SERIES)
			idx = 0;
	} else {
		if (cell_check_flag != 0) {
			cell_check_flag = 0;
			for (int i = 0; i < BAT_MAX_SERIES; i++) {
				over_volt_count[i] = 0;
			}
			bat_cell_over_volt_flag = 0;
			bat_cell_ovp_volt = 0;
		}
	}
}
DECLARE_HOOK(HOOK_TICK, check_battery_cell_voltage, HOOK_PRIO_DEFAULT);

int check_ready_for_high_temperature(void)
{
	for (int i = 0; i < BAT_MAX_SERIES; i++) {
		if (bat_cell_volt[i] >= BAT_CELL_READY_OVER_VOLT) {
			return 0;
		}
	}

	return 1;
}

void set_current_volatage_by_capacity(int *current, int *voltage)
{
	int rateFCDC = 0;
	uint32_t cal_current = 0;

	*current = 0;
	if (board_battery_type == BATTERY_SDI_4404D57M)
		*voltage = CHARGING_VOLTAGE_SDI_4404D57M;
	else
		*voltage = CHARGING_VOLTAGE_SDI_4404D57;

	cal_current = charging_data->batt.full_capacity * 100;
	cal_current += (design_capacity / 2);
	cal_current /= design_capacity;
	rateFCDC = (int)cal_current;

	/* calculate current & voltage */
	if (rateFCDC <= RATE_FCC_DC_85) {
		cal_current = charging_data->batt.full_capacity;

		/* ChargingVoltage - (170mV * series) */
		*voltage -= (170 * BAT_SERIES);
	} else if (rateFCDC <= RATE_FCC_DC_99) {
		cal_current = charging_data->batt.full_capacity;

		/* ChargingVoltage - ((1-FCC/DC)*100*series) -
		 * (25*series) */
		*voltage -= (((100 - rateFCDC) * 10 * BAT_SERIES) +
			     (25 * BAT_SERIES));
	} else {
		cal_current = design_capacity;
	}

	/* FCC or DC * 0.45C */
	cal_current *= 9;
	cal_current /= 20;

	*current = (int)cal_current;
}

void set_current_voltage_by_temperature(int *current, int *voltage)
{
	switch (temp_zone) {
	/* low temp step 3 */
	case LOW_TEMP3:
		/* DC * 8% */
		*current = design_capacity;
		*current *= 2;
		*current /= 25;
		break;
	/* low temp step 2 */
	case LOW_TEMP2:
		/* DC * 24% */
		*current = design_capacity;
		*current *= 6;
		*current /= 25;
		break;
	/* low temp step 1 */
	case LOW_TEMP1:
		*current = charging_data->batt.full_capacity;
		/* FCC * 0.45C */
		*current *= 9;
		*current /= 20;
		break;
	/* high temp */
	case HIGH_TEMP:
		if (check_ready_for_high_temperature()) {
			/* DC * 0.45C */
			*current = design_capacity;
			*current *= 9;
			*current /= 20;
			*voltage = TC_CHARGING_VOLTAGE;
		} else {
			temp_zone = NORMAL_TEMP;
		}
		break;
	default:
		break;
	}
}

void check_battery_life_time(void)
{
	int rv;
	int data;
	uint8_t drop_step;
	uint16_t bat_health_cycle;

	bat_drop_voltage = 0;
	rv = sb_read(0x25, &data);

	if (!rv) {
		bat_health_cycle = data / 6;

		if (bat_health_cycle <= 50)
			drop_step = 0;
		else if (bat_health_cycle <= 160)
			drop_step = 1;
		else if (bat_health_cycle <= 300)
			drop_step = 2;
		else if (bat_health_cycle <= 420)
			drop_step = 3;
		else if (bat_health_cycle <= 520)
			drop_step = 4;
		else if (bat_health_cycle <= 650)
			drop_step = 5;
		else
			drop_step = 6;

		bat_drop_voltage = CHARGING_VOLTAGE_SDI_4404D57M -
				   (DROP_VOLT_MV * drop_step);
	}
}

int charger_profile_override(struct charge_state_data *curr)
{
	int data_c;
	int data_v;

	enum charge_state state = ST_IDLE;

	charging_data = curr;

	if (curr->batt.is_present != BP_YES) {
		design_capacity = 0;
		temp_zone = NORMAL_TEMP;
		bat_drop_voltage = 0;
		board_battery_type = BATTERY_TYPE_COUNT;
		return 0;
	}

	/*
	 * Precharge must be executed when communication is failed on
	 * dead battery.
	 */
	if (!(curr->batt.flags & BATT_FLAG_RESPONSIVE))
		return 0;

	if (!(curr->batt.flags & BATT_FLAG_BAD_TEMPERATURE)) {
		int bat_temp = DECI_KELVIN_TO_CELSIUS(curr->batt.temperature);
		find_battery_thermal_zone(bat_temp);
	}

	/* charge stop */
	if (temp_zone == STOP_LOW_TEMP || temp_zone == STOP_HIGH_TEMP) {
		curr->requested_current = curr->requested_voltage = 0;
		curr->batt.flags &= ~BATT_FLAG_WANT_CHARGE;
		curr->state = ST_IDLE;

		return 0;
	}

	state = curr->state;
	if (state == ST_CHARGE) {
		int port = charge_manager_get_active_charge_port();

		if (design_capacity == 0) {
			if (battery_design_capacity(&design_capacity)) {
				design_capacity = DEFAULT_DESIGN_CAPACITY;
			}
		}

		if (board_battery_type == BATTERY_SDI_4404D57M) {
			check_battery_life_time();
		}
		set_current_volatage_by_capacity(&data_c, &data_v);
		set_current_voltage_by_temperature(&data_c, &data_v);

		if (bat_drop_voltage != 0 &&
		    board_battery_type == BATTERY_SDI_4404D57M) {
			if (data_v > bat_drop_voltage)
				data_v = bat_drop_voltage;
		}

		if (bat_cell_over_volt_flag) {
			if (data_v > bat_cell_ovp_volt)
				data_v = bat_cell_ovp_volt;
		}

		if (port == CHARGER_SECONDARY) {
			data_v -= 300;
		}

		if (curr->requested_current != data_c &&
		    /* If charging current of battery is 0(fully
		     * charged), then EC shouldn't change it for AC
		     * standby power */
		    curr->requested_current != 0) {
			curr->requested_current = data_c;
		}
		curr->requested_voltage = data_v;
	} else {
		temp_zone = NORMAL_TEMP;
	}

	return 0;
}

/* Customs options controllable by host command. */
#define PARAM_FASTCHARGE (CS_PARAM_CUSTOM_PROFILE_MIN + 0)

enum ec_status charger_profile_override_get_param(uint32_t param,
						  uint32_t *value)
{
	return EC_RES_INVALID_PARAM;
}

enum ec_status charger_profile_override_set_param(uint32_t param,
						  uint32_t value)
{
	return EC_RES_INVALID_PARAM;
}

/* Lower our input voltage to 5V in S0iX when battery is full. */
#define PD_VOLTAGE_WHEN_FULL 5000
static void reduce_input_voltage_when_full(void)
{
	static int saved_input_voltage = -1;
	int max_pd_voltage_mv = pd_get_max_voltage();
	int port;

	if (charge_get_percent() == 100 &&
	    chipset_in_state(CHIPSET_STATE_ANY_SUSPEND)) {
		if (max_pd_voltage_mv != PD_VOLTAGE_WHEN_FULL) {
			saved_input_voltage = max_pd_voltage_mv;
			max_pd_voltage_mv = PD_VOLTAGE_WHEN_FULL;
		}
	} else if (saved_input_voltage != -1) {
		if (max_pd_voltage_mv == PD_VOLTAGE_WHEN_FULL)
			max_pd_voltage_mv = saved_input_voltage;
		saved_input_voltage = -1;
	}

	if (pd_get_max_voltage() != max_pd_voltage_mv) {
		for (port = 0; port < CONFIG_USB_PD_PORT_MAX_COUNT; port++)
			pd_set_external_voltage_limit(port, max_pd_voltage_mv);
	}
}
DECLARE_HOOK(HOOK_SECOND, reduce_input_voltage_when_full, HOOK_PRIO_DEFAULT);

/* Get type of the battery connected on the board */
static int board_get_battery_type(void)
{
	char device_name[32];
	int i;

	for (i = 0; i < BATTERY_TYPE_COUNT; i++) {
		if (!battery_device_name(device_name, sizeof(device_name))) {
			if (!strcasecmp(device_name,
					board_battery_info[i].device_name)) {
				board_battery_type = i;
				break;
			}
		}
	}

	return board_battery_type;
}

/*
 * Initialize the battery type for the board.
 *
 * Very first battery info is called by the charger driver to initialize
 * the charger parameters hence initialize the battery type for the board
 * as soon as the I2C is initialized.
 */
static void board_init_battery_type(void)
{
	if (board_get_battery_type() != BATTERY_TYPE_COUNT)
		CPRINTS("found batt:%s",
			board_battery_info[board_battery_type].device_name);
	else
		CPRINTS("battery not found");
}
DECLARE_HOOK(HOOK_INIT, board_init_battery_type, HOOK_PRIO_POST_BATTERY_INIT);

enum battery_present battery_is_present(void)
{
	enum battery_present batt_pres;

	/* The GPIO is low when the battery is present */
	batt_pres = gpio_get_level(GPIO_EC_BATTERY_PRES_ODL) ? BP_NO : BP_YES;

	if (batt_pres_prev != BP_YES && batt_pres == BP_YES)
		board_init_battery_type();

	batt_pres_prev = batt_pres;

	return batt_pres;
}
