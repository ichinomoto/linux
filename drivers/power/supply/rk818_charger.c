// SPDX-License-Identifier: GPL-2.0-only
/*
 * Battery and charger monitor for the Rockchip RK818 PMIC
 *
 * The gauge conversion constants and register layout are derived from the
 * Rockchip BSP driver.  Charger programming follows the board battery data:
 * the charge-current selector is taken from the monitored-battery node.
 */

#include <linux/bitfield.h>
#include <linux/devm-helpers.h>
#include <linux/extcon.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/mfd/rk808.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <linux/power_supply.h>
#include <linux/reboot.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>

#define RK818_MONITOR_INTERVAL_MS	5000
#define RK818_GASCNT_PER_MAH		2390
#define RK818_MIN_FCC_MAH			500
#define RK818_USB_INPUT_CURRENT_UA	450000
#define RK818_THERMAL_FEEDBACK_MASK	GENMASK(3, 2)
#define RK818_GAUGE_REST_MINUTES		30
#define RK818_GAUGE_CORRECTION_MS		60000
#define RK818_LOW_VOLTAGE_MARGIN_UV	50000
#define RK818_POWEROFF_TIMEOUT_MS		10000

static const int rk818_input_currents[] = {
	450000, 800000, 850000, 1000000, 1250000, 1500000,
	1750000, 2000000, 2250000, 2500000, 2750000, 3000000,
};

enum rk818_charge_status {
	RK818_CHARGE_OFF,
	RK818_CHARGE_DEAD,
	RK818_CHARGE_TRICKLE,
	RK818_CHARGE_CC_CV,
	RK818_CHARGE_FINISH,
	RK818_CHARGE_USB_OVERVOLTAGE,
	RK818_CHARGE_TEMPERATURE_ERROR,
	RK818_CHARGE_TIMER_ERROR,
};

struct rk818_charger {
	struct device *dev;
	struct rk808 *rk808;
	struct power_supply *battery;
	struct power_supply *ac;
	struct power_supply *usb;
	struct extcon_dev *edev;
	struct notifier_block cable_nb;
	struct delayed_work work;
	struct power_supply_battery_ocv_table *ocv_table;
	int ocv_table_len;
	int voltage_k;
	int voltage_b;
	int charge_full_design_uah;
	int charge_full_max_uah;
	int voltage_min_design_uv;
	int voltage_max_design_uv;
	int internal_resistance_uohm;
	int constant_charge_current_max_ua;
	int constant_charge_voltage_max_uv;
	int input_current_max_ua;
	u32 feedback_temp_mc;
	/* Serialize live counter reads with gauge calibration writes. */
	struct mutex gauge_lock;
	int saved_charge_mah;
	int saved_soc;
	unsigned long negative_since;
	unsigned long low_voltage_since;
	bool negative_pending;
	bool low_voltage_pending;
	bool shutdown_pending;
};

static int rk818_read_be16(struct rk818_charger *charger, unsigned int reg,
			   int *value)
{
	u8 data[2];
	int ret;

	ret = regmap_bulk_read(charger->rk808->regmap, reg, data, sizeof(data));
	if (ret)
		return ret;

	*value = get_unaligned_be16(data);
	return 0;
}

static int rk818_read_be32(struct rk818_charger *charger, unsigned int reg,
			   u32 *value)
{
	u8 data[4];
	int ret;

	ret = regmap_bulk_read(charger->rk808->regmap, reg, data, sizeof(data));
	if (ret)
		return ret;

	*value = get_unaligned_be32(data);
	return 0;
}

static int rk818_write_be32(struct rk818_charger *charger, unsigned int reg,
			    u32 value)
{
	u8 data[4];

	put_unaligned_be32(value, data);
	return regmap_bulk_write(charger->rk808->regmap, reg, data, sizeof(data));
}

static int rk818_calibrate_voltage(struct rk818_charger *charger)
{
	int vcalib0, vcalib1;
	int ret;

	ret = rk818_read_be16(charger, RK818_VCALIB0_REGH, &vcalib0);
	if (ret)
		return ret;

	ret = rk818_read_be16(charger, RK818_VCALIB1_REGH, &vcalib1);
	if (ret)
		return ret;

	if (vcalib1 <= vcalib0)
		return -EINVAL;

	/* The BSP calibrates the two ADC points against 3000 mV and 4200 mV. */
	charger->voltage_k = (4200 - 3000) * 1000 / (vcalib1 - vcalib0);
	charger->voltage_b = 4200 - charger->voltage_k * vcalib1 / 1000;

	return 0;
}

static int rk818_charge_current_sel(int current_ua)
{
	switch (current_ua) {
	case 1000000:
		return RK818_CHRG_CTRL1_CUR_1000MA;
	case 1200000:
		return RK818_CHRG_CTRL1_CUR_1200MA;
	case 1400000:
		return RK818_CHRG_CTRL1_CUR_1400MA;
	case 1600000:
		return RK818_CHRG_CTRL1_CUR_1600MA;
	case 1800000:
		return RK818_CHRG_CTRL1_CUR_1800MA;
	case 2000000:
		return RK818_CHRG_CTRL1_CUR_2000MA;
	case 2250000:
		return RK818_CHRG_CTRL1_CUR_2250MA;
	case 2400000:
		return RK818_CHRG_CTRL1_CUR_2400MA;
	case 2600000:
		return RK818_CHRG_CTRL1_CUR_2600MA;
	case 2800000:
		return RK818_CHRG_CTRL1_CUR_2800MA;
	case 3000000:
		return RK818_CHRG_CTRL1_CUR_3000MA;
	default:
		return -EINVAL;
	}
}

static int rk818_input_current_sel(int current_ua)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(rk818_input_currents); i++)
		if (current_ua == rk818_input_currents[i])
			return i;

	return -EINVAL;
}

static int rk818_charger_init(struct rk818_charger *charger)
{
	int charge_current_sel;
	int ret;

	/* Stay at the BSP's USB limit until the source has been classified. */
	charge_current_sel =
		rk818_charge_current_sel(charger->constant_charge_current_max_ua);
	if (charge_current_sel < 0)
		return charge_current_sel;

	ret = regmap_update_bits(charger->rk808->regmap, RK818_USB_CTRL_REG,
				 RK818_USB_ILIM_SEL_MASK |
				 RK818_USB_CHRG_CT_EN,
				 rk818_input_current_sel(RK818_USB_INPUT_CURRENT_UA) |
				 RK818_USB_CHRG_CT_EN);
	if (ret)
		return ret;

	/* Only change the feedback field; preserve shutdown/other thermal bits. */
	ret = regmap_update_bits(charger->rk808->regmap, RK818_THERMAL_REG,
				 RK818_THERMAL_FEEDBACK_MASK,
				 FIELD_PREP(RK818_THERMAL_FEEDBACK_MASK,
					    (charger->feedback_temp_mc - 85000) / 10000));
	if (ret)
		return ret;

	ret = regmap_update_bits(charger->rk808->regmap,
				 RK818_CHRG_CTRL_REG1,
				 RK818_CHRG_CTRL1_EN |
				 RK818_CHRG_CTRL1_VOL_MASK |
				 RK818_CHRG_CTRL1_CUR_MASK,
				 RK818_CHRG_CTRL1_EN |
				 RK818_CHRG_CTRL1_VOL_4200 |
				 charge_current_sel);
	if (ret)
		return ret;

	/* These termination/timer values are shared by the DM200/DM250 BSP. */
	ret = regmap_write(charger->rk808->regmap, RK818_CHRG_CTRL_REG2,
			   0x0d);
	if (ret)
		return ret;

	ret = regmap_write(charger->rk808->regmap, RK818_CHRG_CTRL_REG3,
			   0x26);
	if (ret)
		return ret;

	return regmap_update_bits(charger->rk808->regmap, RK818_SUP_STS_REG,
				  RK818_SUP_STS_DC_CHG_MASK,
				  RK818_SUP_STS_DC_CHG_ENABLE);
}

static int rk818_read_voltage(struct rk818_charger *charger, int *voltage_uv)
{
	int raw;
	int ret;

	ret = rk818_read_be16(charger, RK818_BAT_VOL_REGH, &raw);
	if (ret)
		return ret;

	*voltage_uv = (charger->voltage_k * raw / 1000 +
			 charger->voltage_b) * 1000;
	return 0;
}

static int rk818_read_current(struct rk818_charger *charger, int *current_ua)
{
	unsigned int high;
	unsigned int low;
	int sample[3];
	int raw;
	int ret;
	int i;

	/*
	 * Read low then high, as the RK818 BSP does.  The two registers are
	 * updated by the gauge while they are being read, so use three samples
	 * and prefer the first two when they agree.
	 */
	for (i = 0; i < ARRAY_SIZE(sample); i++) {
		ret = regmap_read(charger->rk808->regmap,
				  RK818_BAT_CUR_AVG_REGL, &low);
		if (ret)
			return ret;

		ret = regmap_read(charger->rk808->regmap,
				  RK818_BAT_CUR_AVG_REGH, &high);
		if (ret)
			return ret;

		sample[i] = (high << 8) | low;
	}

	raw = sample[0] == sample[1] ? sample[0] : sample[2];

	raw &= GENMASK(11, 0);
	if (raw & BIT(11))
		raw -= BIT(12);

	/* One ADC count is 1.506 mA according to the BSP conversion. */
	*current_ua = raw * 1506;
	return 0;
}

static int rk818_read_charge_counter(struct rk818_charger *charger, s32 *counter)
{
	u32 first, raw;
	int ret;

	/* The gauge updates GASCNT while the I2C transaction is in progress. */
	ret = rk818_read_be32(charger, RK818_GASCNT3_REG, &first);
	if (ret)
		return ret;
	ret = rk818_read_be32(charger, RK818_GASCNT3_REG, &raw);
	if (ret)
		return ret;
	if (first != raw) {
		ret = rk818_read_be32(charger, RK818_GASCNT3_REG, &raw);
		if (ret)
			return ret;
	}

	*counter = (s32)raw;
	return 0;
}

static int rk818_read_charge_now(struct rk818_charger *charger, int *charge_uah)
{
	s32 counter;
	int ret;

	mutex_lock(&charger->gauge_lock);
	if (charger->shutdown_pending) {
		*charge_uah = 0;
		ret = 0;
	} else {
		ret = rk818_read_charge_counter(charger, &counter);
		if (!ret)
			*charge_uah = max(counter, 0) / RK818_GASCNT_PER_MAH * 1000;
	}
	mutex_unlock(&charger->gauge_lock);

	return ret;
}

static int rk818_read_charge_full(struct rk818_charger *charger, int *charge_uah)
{
	u32 raw;
	int ret;

	ret = rk818_read_be32(charger, RK818_NEW_FCC_REG3, &raw);
	if (ret)
		return ret;

	/* Reject implausible saved FCC before converting to uAh. */
	if (raw > 1 && raw - 1 >= RK818_MIN_FCC_MAH &&
	    raw - 1 <= charger->charge_full_max_uah / 1000) {
		*charge_uah = (raw - 1) * 1000;
	} else {
		dev_warn_once(charger->dev,
			      "invalid saved FCC %u, using design capacity %d uAh\n",
			      raw, charger->charge_full_design_uah);
		*charge_uah = charger->charge_full_design_uah;
	}

	return 0;
}

static int rk818_read_supply_status(struct rk818_charger *charger,
				    unsigned int *status)
{
	return regmap_read(charger->rk808->regmap, RK818_SUP_STS_REG, status);
}

static int rk818_read_plugged(struct rk818_charger *charger, bool *plugged)
{
	unsigned int value;
	int ret;

	ret = regmap_read(charger->rk808->regmap, RK808_VB_MON_REG, &value);
	if (ret)
		return ret;

	*plugged = value & RK818_VB_MON_PLUG_IN_STS;
	return 0;
}

static int rk818_battery_status(struct rk818_charger *charger, int *status)
{
	unsigned int supply_status;
	unsigned int charge_status;
	bool plugged;
	int ret;

	if (READ_ONCE(charger->shutdown_pending)) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	ret = rk818_read_plugged(charger, &plugged);
	if (ret)
		return ret;

	if (!plugged) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	ret = rk818_read_supply_status(charger, &supply_status);
	if (ret)
		return ret;

	charge_status = FIELD_GET(RK818_SUP_STS_CHRG_MASK, supply_status);
	switch (charge_status) {
	case RK818_CHARGE_DEAD:
	case RK818_CHARGE_TRICKLE:
	case RK818_CHARGE_CC_CV:
		*status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case RK818_CHARGE_FINISH:
		*status = POWER_SUPPLY_STATUS_FULL;
		break;
	case RK818_CHARGE_OFF:
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		*status = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return 0;
}

static int rk818_battery_health(struct rk818_charger *charger, int *health)
{
	unsigned int supply_status;
	unsigned int charge_status;
	int ret;

	ret = rk818_read_supply_status(charger, &supply_status);
	if (ret)
		return ret;

	if (!(supply_status & RK818_SUP_STS_BAT_EXS)) {
		*health = POWER_SUPPLY_HEALTH_NO_BATTERY;
		return 0;
	}

	charge_status = FIELD_GET(RK818_SUP_STS_CHRG_MASK, supply_status);
	switch (charge_status) {
	case RK818_CHARGE_USB_OVERVOLTAGE:
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		break;
	case RK818_CHARGE_TEMPERATURE_ERROR:
		*health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		break;
	case RK818_CHARGE_TIMER_ERROR:
		*health = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
		break;
	default:
		*health = POWER_SUPPLY_HEALTH_GOOD;
		break;
	}

	return 0;
}

static int rk818_voltage_capacity(struct rk818_charger *charger, int voltage_uv)
{
	int capacity;

	if (charger->ocv_table)
		capacity = power_supply_ocv2cap_simple(charger->ocv_table,
						       charger->ocv_table_len,
						       voltage_uv);
	else
		capacity = DIV_ROUND_CLOSEST((voltage_uv -
					     charger->voltage_min_design_uv) * 100,
					    charger->voltage_max_design_uv -
					     charger->voltage_min_design_uv);

	return clamp(capacity, 0, 100);
}

/* gauge_lock must be held across calibration and the subsequent saved state. */
static int rk818_calibrate_charge(struct rk818_charger *charger, int charge_uah)
{
	int charge_mah = charge_uah / 1000;

	if (charge_uah < 0 || charge_mah > INT_MAX / RK818_GASCNT_PER_MAH)
		return -ERANGE;

	return rk818_write_be32(charger, RK818_GASCNT_CAL_REG3,
				charge_mah * RK818_GASCNT_PER_MAH);
}

static int rk818_save_gauge(struct rk818_charger *charger, int charge_uah,
			    int full_uah)
{
	int charge_mah = clamp(charge_uah, 0, full_uah) / 1000;
	int soc = DIV_ROUND_CLOSEST_ULL((u64)charge_mah * 100000, full_uah);
	int ret;

	/* Match the BSP's plain-mAh REMAIN_CAP and percentage SOC layout. */
	if (charge_mah != charger->saved_charge_mah) {
		ret = rk818_write_be32(charger, RK818_REMAIN_CAP_REG3, charge_mah);
		if (ret)
			return ret;
		charger->saved_charge_mah = charge_mah;
	}
	if (soc != charger->saved_soc) {
		ret = regmap_write(charger->rk808->regmap, RK818_SOC_REG, soc);
		if (ret)
			return ret;
		charger->saved_soc = soc;
	}

	return 0;
}

static int rk818_gauge_init(struct rk818_charger *charger)
{
	unsigned int gauge_status, mark, off_minutes, saved_minutes;
	unsigned int reset_mask = RK818_MISC_FG_RESET_NOW;
	bool relaxed, use_ocv;
	int charge_uah, full_uah, ocv_raw, ocv_uv;
	s32 counter;
	int ret;

	mutex_lock(&charger->gauge_lock);
	ret = rk818_read_charge_full(charger, &full_uah);
	if (ret)
		goto out;
	ret = rk818_read_charge_counter(charger, &counter);
	if (ret)
		goto out;
	ret = regmap_read(charger->rk808->regmap, RK818_GGSTS_REG, &gauge_status);
	if (ret)
		goto out;
	ret = regmap_read(charger->rk808->regmap, RK818_MISC_MARK_REG, &mark);
	if (ret)
		goto out;
	ret = regmap_read(charger->rk808->regmap, RK818_NON_ACT_TIMER_CNT_REG,
			  &off_minutes);
	if (ret)
		goto out;
	ret = regmap_read(charger->rk808->regmap, RK818_NON_ACT_TIMER_CNT_SAVE_REG,
			  &saved_minutes);
	if (ret)
		goto out;

	relaxed = off_minutes != saved_minutes &&
		  off_minutes >= RK818_GAUGE_REST_MINUTES;
	if (relaxed)
		reset_mask |= RK818_MISC_FG_RESET_LATE;
	use_ocv = (gauge_status & RK818_GGSTS_BAT_CON) || (mark & reset_mask) ||
		  (relaxed && !(mark & RK818_MISC_FG_INIT));
	charge_uah = max(counter, 0) / RK818_GASCNT_PER_MAH * 1000;
	if (use_ocv) {
		ret = rk818_read_be16(charger, RK818_BAT_OCV_REGH, &ocv_raw);
		if (ret)
			goto out;
		ocv_uv = (charger->voltage_k * ocv_raw / 1000 +
			  charger->voltage_b) * 1000;
		charge_uah = div_u64((u64)full_uah *
				    rk818_voltage_capacity(charger, ocv_uv), 100);
	}

	if (use_ocv || counter < 0 || charge_uah > full_uah) {
		charge_uah = clamp(charge_uah, 0, full_uah);
		ret = rk818_calibrate_charge(charger, charge_uah);
		if (ret)
			goto out;
	}
	ret = rk818_save_gauge(charger, charge_uah, full_uah);
	if (ret)
		goto out;

	/* Do not consume a calibration request until its writes have succeeded. */
	ret = regmap_update_bits(charger->rk808->regmap, RK818_GGSTS_REG,
				 RK818_GGSTS_BAT_CON, 0);
	if (ret)
		goto out;
	ret = regmap_update_bits(charger->rk808->regmap, RK818_MISC_MARK_REG,
				 RK818_MISC_FG_INIT | (mark & reset_mask), 0);
	if (ret)
		goto out;
	ret = regmap_write(charger->rk808->regmap,
			   RK818_NON_ACT_TIMER_CNT_SAVE_REG, off_minutes);
out:
	mutex_unlock(&charger->gauge_lock);
	return ret;
}

static bool rk818_check_low_voltage(struct rk818_charger *charger,
				    int voltage_uv, int current_ua, bool plugged)
{
	bool discharging = current_ua < 0 || (!plugged && !current_ua);

	if (!discharging || voltage_uv >= charger->voltage_min_design_uv) {
		charger->low_voltage_pending = false;
		return false;
	}
	if (!charger->low_voltage_pending) {
		charger->low_voltage_since = jiffies;
		charger->low_voltage_pending = true;
	}

	return voltage_uv <= charger->voltage_min_design_uv -
			     RK818_LOW_VOLTAGE_MARGIN_UV ||
	       time_after_eq(jiffies, charger->low_voltage_since +
			     msecs_to_jiffies(RK818_GAUGE_CORRECTION_MS));
}

static int rk818_gauge_update(struct rk818_charger *charger)
{
	unsigned int supply_status, charge_status;
	int voltage_uv, current_ua, charge_uah, full_uah;
	s64 ocv_uv;
	s32 counter;
	bool plugged, poweroff = false;
	int ret;

	mutex_lock(&charger->gauge_lock);
	if (charger->shutdown_pending) {
		ret = 0;
		goto out;
	}
	ret = rk818_read_supply_status(charger, &supply_status);
	if (ret)
		goto sample_error;
	if (!(supply_status & RK818_SUP_STS_BAT_EXS)) {
		ret = -ENODATA;
		goto sample_error;
	}
	ret = rk818_read_voltage(charger, &voltage_uv);
	if (ret)
		goto sample_error;
	ret = rk818_read_current(charger, &current_ua);
	if (ret)
		goto sample_error;
	ret = rk818_read_plugged(charger, &plugged);
	if (ret)
		goto sample_error;

	/* An attached charger is not sufficient if the battery still discharges. */
	if (rk818_check_low_voltage(charger, voltage_uv, current_ua, plugged)) {
		WRITE_ONCE(charger->shutdown_pending, true);
		charger->negative_pending = false;
		poweroff = true;
		/* Do not resurrect stale charge on a short-off reboot. */
		ret = rk818_calibrate_charge(charger, 0);
		if (!ret)
			ret = rk818_save_gauge(charger, 0,
					       charger->charge_full_design_uah);
		goto out;
	}

	ret = rk818_read_charge_full(charger, &full_uah);
	if (ret)
		goto gauge_error;
	ret = rk818_read_charge_counter(charger, &counter);
	if (ret)
		goto gauge_error;
	charge_uah = max(counter, 0) / RK818_GASCNT_PER_MAH * 1000;
	charge_status = FIELD_GET(RK818_SUP_STS_CHRG_MASK, supply_status);
	if ((plugged && current_ua >= 0 && charge_status == RK818_CHARGE_FINISH &&
	     voltage_uv >= charger->constant_charge_voltage_max_uv - 150000) ||
	    charge_uah > full_uah) {
		if (charge_uah != full_uah) {
			ret = rk818_calibrate_charge(charger, full_uah);
			if (ret)
				goto gauge_error;
		}
		charge_uah = full_uah;
		charger->negative_pending = false;
	} else if (counter < 0) {
		if (!charger->negative_pending) {
			charger->negative_since = jiffies;
			charger->negative_pending = true;
		}
		if (time_after_eq(jiffies, charger->negative_since +
				  msecs_to_jiffies(RK818_GAUGE_CORRECTION_MS))) {
			/* BSP load compensation, converted from mV/mA to uV/uA. */
			ocv_uv = voltage_uv -
				 div_s64((s64)current_ua *
					 charger->internal_resistance_uohm, 1000000);
			ocv_uv = clamp_t(s64, ocv_uv, charger->voltage_min_design_uv,
					 charger->voltage_max_design_uv);
			charge_uah = div_u64((u64)full_uah *
					    rk818_voltage_capacity(charger, ocv_uv), 100);
			ret = rk818_calibrate_charge(charger, charge_uah);
			if (ret)
				goto gauge_error;
			charger->negative_pending = false;
		}
	} else {
		charger->negative_pending = false;
	}

	ret = rk818_save_gauge(charger, charge_uah, full_uah);
	goto out;
sample_error:
	charger->low_voltage_pending = false;
gauge_error:
	charger->negative_pending = false;
out:
	mutex_unlock(&charger->gauge_lock);
	if (poweroff)
		__hw_protection_trigger("RK818 battery voltage too low",
					RK818_POWEROFF_TIMEOUT_MS, HWPROT_ACT_SHUTDOWN);
	return ret;
}

static int rk818_battery_capacity(struct rk818_charger *charger, int *capacity)
{
	int charge_now, charge_full;
	int voltage_uv;
	u64 percent;
	int ret;

	if (READ_ONCE(charger->shutdown_pending)) {
		*capacity = 0;
		return 0;
	}

	/*
	 * SOC_REG is a software-saved display value, not a live gauge result.
	 * Derive the percentage from the same coulomb counter and FCC as the
	 * charge_now/charge_full properties, including on each monitor event.
	 */
	ret = rk818_read_charge_now(charger, &charge_now);
	if (!ret) {
		ret = rk818_read_charge_full(charger, &charge_full);
		if (ret)
			return ret;
		if (charge_full <= 0)
			return -ENODATA;

		percent = DIV_ROUND_CLOSEST_ULL((u64)charge_now * 100, charge_full);
		*capacity = min_t(u64, percent, 100);
		return 0;
	}

	ret = rk818_read_voltage(charger, &voltage_uv);
	if (ret)
		return ret;

	*capacity = rk818_voltage_capacity(charger, voltage_uv);
	return 0;
}

static int rk818_battery_get_property(struct power_supply *psy,
				      enum power_supply_property property,
				      union power_supply_propval *value)
{
	struct rk818_charger *charger = power_supply_get_drvdata(psy);
	unsigned int supply_status;
	int ret;

	switch (property) {
	case POWER_SUPPLY_PROP_PRESENT:
		ret = rk818_read_supply_status(charger, &supply_status);
		if (!ret)
			value->intval = !!(supply_status & RK818_SUP_STS_BAT_EXS);
		return ret;
	case POWER_SUPPLY_PROP_STATUS:
		return rk818_battery_status(charger, &value->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		return rk818_battery_health(charger, &value->intval);
	case POWER_SUPPLY_PROP_CAPACITY:
		return rk818_battery_capacity(charger, &value->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return rk818_read_voltage(charger, &value->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return rk818_read_current(charger, &value->intval);
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		return rk818_read_charge_now(charger, &value->intval);
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		return rk818_read_charge_full(charger, &value->intval);
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		value->intval = charger->charge_full_design_uah;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		value->intval = charger->voltage_min_design_uv;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		value->intval = charger->voltage_max_design_uv;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		value->intval = charger->constant_charge_current_max_ua;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		value->intval = charger->constant_charge_voltage_max_uv;
		return 0;
	default:
		return -EINVAL;
	}
}

static const enum power_supply_property rk818_battery_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
};

static const struct power_supply_desc rk818_battery_desc = {
	.name = "BATTERY",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = rk818_battery_properties,
	.num_properties = ARRAY_SIZE(rk818_battery_properties),
	.get_property = rk818_battery_get_property,
};

static enum power_supply_usb_type rk818_source_type(struct rk818_charger *charger)
{
	int sdp, cdp, dcp;

	if (!charger->edev)
		return POWER_SUPPLY_USB_TYPE_UNKNOWN;

	sdp = extcon_get_state(charger->edev, EXTCON_CHG_USB_SDP);
	cdp = extcon_get_state(charger->edev, EXTCON_CHG_USB_CDP);
	dcp = extcon_get_state(charger->edev, EXTCON_CHG_USB_DCP);
	if (sdp < 0 || cdp < 0 || dcp < 0 || sdp + cdp + dcp != 1)
		return POWER_SUPPLY_USB_TYPE_UNKNOWN;
	if (dcp)
		return POWER_SUPPLY_USB_TYPE_DCP;
	if (cdp)
		return POWER_SUPPLY_USB_TYPE_CDP;

	return POWER_SUPPLY_USB_TYPE_SDP;
}

static int rk818_update_input_limit(struct rk818_charger *charger)
{
	int current_ua = RK818_USB_INPUT_CURRENT_UA;
	enum power_supply_usb_type type;
	bool plugged;
	int ret, read_ret;

	read_ret = rk818_read_plugged(charger, &plugged);
	if (!read_ret && plugged) {
		type = rk818_source_type(charger);
		if (type == POWER_SUPPLY_USB_TYPE_DCP)
			current_ua = charger->input_current_max_ua;
		else if (type == POWER_SUPPLY_USB_TYPE_CDP)
			/* CDP is a USB data port; never exceed its 1.5 A limit. */
			current_ua = min(charger->input_current_max_ua, 1500000);
	}

	ret = regmap_update_bits(charger->rk808->regmap, RK818_USB_CTRL_REG,
				 RK818_USB_ILIM_SEL_MASK,
				 rk818_input_current_sel(current_ua));
	return ret ?: read_ret;
}

static int rk818_cable_changed(struct notifier_block *nb,
			       unsigned long event, void *data)
{
	struct rk818_charger *charger =
		container_of(nb, struct rk818_charger, cable_nb);

	/* Extcon uses an atomic notifier: defer sleeping I2C operations. */
	mod_delayed_work(system_freezable_wq, &charger->work, 0);
	return NOTIFY_OK;
}

static int rk818_external_get_property(struct power_supply *psy,
				       enum power_supply_property property,
				       union power_supply_propval *value)
{
	struct rk818_charger *charger = power_supply_get_drvdata(psy);
	enum power_supply_usb_type type;
	unsigned int reg;
	bool plugged;
	int ret;

	if (property == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT) {
		ret = regmap_read(charger->rk808->regmap, RK818_USB_CTRL_REG, &reg);
		if (ret)
			return ret;
		reg &= RK818_USB_ILIM_SEL_MASK;
		if (reg >= ARRAY_SIZE(rk818_input_currents))
			return -EINVAL;
		value->intval = rk818_input_currents[reg];
		return 0;
	}

	if (property != POWER_SUPPLY_PROP_ONLINE &&
	    property != POWER_SUPPLY_PROP_USB_TYPE)
		return -EINVAL;

	ret = rk818_read_plugged(charger, &plugged);
	if (ret)
		return ret;

	type = plugged ? rk818_source_type(charger) : POWER_SUPPLY_USB_TYPE_UNKNOWN;
	if (property == POWER_SUPPLY_PROP_USB_TYPE)
		value->intval = type;
	else if (psy == charger->usb)
		value->intval = plugged && type != POWER_SUPPLY_USB_TYPE_DCP;
	else
		value->intval = plugged && type == POWER_SUPPLY_USB_TYPE_DCP;

	return 0;
}

static const enum power_supply_property rk818_external_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static const enum power_supply_property rk818_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc rk818_ac_desc = {
	.name = "AC",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = rk818_external_properties,
	.num_properties = ARRAY_SIZE(rk818_external_properties),
	.get_property = rk818_external_get_property,
};

static const struct power_supply_desc rk818_usb_desc = {
	.name = "USB",
	.type = POWER_SUPPLY_TYPE_USB,
	.properties = rk818_usb_properties,
	.num_properties = ARRAY_SIZE(rk818_usb_properties),
	.usb_types = BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
		     BIT(POWER_SUPPLY_USB_TYPE_SDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_CDP) |
		     BIT(POWER_SUPPLY_USB_TYPE_DCP),
	.get_property = rk818_external_get_property,
};

static void rk818_monitor_work(struct work_struct *work)
{
	struct rk818_charger *charger =
		container_of(work, struct rk818_charger, work.work);
	int ret;

	ret = rk818_update_input_limit(charger);
	if (ret)
		dev_err_ratelimited(charger->dev, "input current update failed: %d\n", ret);

	ret = rk818_gauge_update(charger);
	if (ret)
		dev_err_ratelimited(charger->dev, "fuel gauge update failed: %d\n", ret);

	power_supply_changed(charger->battery);
	power_supply_changed(charger->ac);
	power_supply_changed(charger->usb);
	queue_delayed_work(system_freezable_wq, &charger->work,
			   msecs_to_jiffies(RK818_MONITOR_INTERVAL_MS));
}

static irqreturn_t rk818_plug_irq(int irq, void *data)
{
	struct rk818_charger *charger = data;
	int ret;

	/* Drop a previous DCP limit before a new cable can be classified. */
	ret = regmap_update_bits(charger->rk808->regmap, RK818_USB_CTRL_REG,
				 RK818_USB_ILIM_SEL_MASK,
				 rk818_input_current_sel(RK818_USB_INPUT_CURRENT_UA));
	if (ret)
		dev_err_ratelimited(charger->dev, "USB input limit reset failed: %d\n", ret);

	mod_delayed_work(system_freezable_wq, &charger->work, 0);
	return IRQ_HANDLED;
}

static irqreturn_t rk818_vb_low_irq(int irq, void *data)
{
	struct rk818_charger *charger = data;

	/* Recheck voltage/current; a stale threshold IRQ is not a shutdown request. */
	mod_delayed_work(system_freezable_wq, &charger->work, 0);
	return IRQ_HANDLED;
}

static void rk818_disable_wakeup(void *data)
{
	struct device *dev = data;

	dev_pm_clear_wake_irq(dev);
	device_init_wakeup(dev, false);
}

static void rk818_put_of_node(void *data)
{
	of_node_put(data);
}

static int rk818_read_fcc_limit(struct rk818_charger *charger,
				struct device_node *node)
{
	u32 max_uah;
	int ret;

	charger->charge_full_max_uah = charger->charge_full_design_uah;
	if (!of_find_property(node, "rockchip,charge-full-max-microamp-hours", NULL))
		return 0;

	ret = of_property_read_u32(node, "rockchip,charge-full-max-microamp-hours",
				   &max_uah);
	if (ret)
		return ret;

	if (max_uah < charger->charge_full_design_uah || max_uah > INT_MAX)
		return -EINVAL;

	charger->charge_full_max_uah = max_uah;
	return 0;
}

static int rk818_read_battery_info(struct rk818_charger *charger)
{
	const struct power_supply_battery_ocv_table *table;
	struct power_supply_battery_info *info;
	size_t table_size;
	int ret;

	ret = power_supply_get_battery_info(charger->battery, &info);
	if (ret)
		return ret;

	if (info->charge_full_design_uah <= 0 ||
	    info->voltage_min_design_uv <= 0 ||
	    info->voltage_max_design_uv <= info->voltage_min_design_uv ||
	    info->constant_charge_current_max_ua <= 0 ||
	    info->constant_charge_voltage_max_uv <= 0) {
		ret = -EINVAL;
		goto out;
	}

	charger->charge_full_design_uah = info->charge_full_design_uah;
	charger->voltage_min_design_uv = info->voltage_min_design_uv;
	charger->voltage_max_design_uv = info->voltage_max_design_uv;
	charger->internal_resistance_uohm = max(info->factory_internal_resistance_uohm, 0);
	charger->constant_charge_current_max_ua =
		info->constant_charge_current_max_ua;
	charger->constant_charge_voltage_max_uv =
		info->constant_charge_voltage_max_uv;

	table = power_supply_find_ocv2cap_table(info, 20,
						&charger->ocv_table_len);
	if (table) {
		table_size = sizeof(*table) * charger->ocv_table_len;
		charger->ocv_table =
			devm_kmemdup(charger->dev, table, table_size, GFP_KERNEL);
		if (!charger->ocv_table)
			ret = -ENOMEM;
	}

out:
	power_supply_put_battery_info(charger->battery, info);
	return ret;
}

static int rk818_read_charger_info(struct rk818_charger *charger,
				   struct device_node *node)
{
	struct device_node *extcon_node;
	u32 current_ua;
	int ret;

	/* Old DTs remain usable, but never assume an unknown source is 3 A. */
	charger->input_current_max_ua = RK818_USB_INPUT_CURRENT_UA;
	charger->feedback_temp_mc = 85000;
	if (of_property_present(node, "input-current-limit-microamp")) {
		ret = of_property_read_u32(node, "input-current-limit-microamp",
					   &current_ua);
		if (ret)
			return ret;
		if (current_ua > INT_MAX || rk818_input_current_sel(current_ua) < 0)
			return -EINVAL;
		charger->input_current_max_ua = current_ua;
	}

	if (of_property_present(node, "rockchip,charge-feedback-temperature-millicelsius")) {
		ret = of_property_read_u32(node,
				"rockchip,charge-feedback-temperature-millicelsius",
				&charger->feedback_temp_mc);
		if (ret)
			return ret;
		if (charger->feedback_temp_mc < 85000 || charger->feedback_temp_mc > 115000 ||
		    (charger->feedback_temp_mc - 85000) % 10000)
			return -EINVAL;
	}

	if (of_property_present(node, "extcon")) {
		extcon_node = of_parse_phandle(node, "extcon", 0);
		if (!extcon_node)
			return -EINVAL;
		charger->edev = extcon_find_edev_by_node(extcon_node);
		of_node_put(extcon_node);
		if (IS_ERR(charger->edev))
			return PTR_ERR(charger->edev);
	}

	return 0;
}

static int rk818_charger_probe(struct platform_device *pdev)
{
	struct rk808 *rk808 = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config config = {};
	struct device_node *node;
	struct rk818_charger *charger;
	int plug_in_irq, plug_out_irq, vb_low_irq;
	int ret;

	node = of_get_child_by_name(pdev->dev.parent->of_node, "charger");
	if (!node)
		return -ENODEV;

	ret = devm_add_action_or_reset(&pdev->dev, rk818_put_of_node, node);
	if (ret)
		return ret;

	charger = devm_kzalloc(&pdev->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	charger->dev = &pdev->dev;
	charger->rk808 = rk808;
	mutex_init(&charger->gauge_lock);
	charger->saved_charge_mah = -1;
	charger->saved_soc = -1;
	platform_set_drvdata(pdev, charger);

	ret = regmap_update_bits(rk808->regmap, RK818_TS_CTRL_REG,
				 RK818_TS_CTRL_GG_EN, RK818_TS_CTRL_GG_EN);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to enable fuel gauge\n");

	ret = rk818_calibrate_voltage(charger);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "invalid voltage calibration data\n");

	config.drv_data = charger;
	config.fwnode = &node->fwnode;

	charger->battery =
		devm_power_supply_register(&pdev->dev, &rk818_battery_desc,
					   &config);
	if (IS_ERR(charger->battery))
		return dev_err_probe(&pdev->dev, PTR_ERR(charger->battery),
				     "failed to register battery\n");

	ret = rk818_read_battery_info(charger);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "invalid monitored-battery data\n");

	ret = rk818_read_fcc_limit(charger, node);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "invalid maximum full-charge capacity\n");

	ret = rk818_read_charger_info(charger, node);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "invalid charger limits or extcon\n");

	ret = rk818_charger_init(charger);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to initialize charger\n");

	/* Match the BSP's eight-minute OCV and ADC-offset sampling periods. */
	ret = regmap_update_bits(rk808->regmap, RK818_GGCON_REG,
				 RK818_GGCON_OCV_SAMPLE_MASK | RK818_GGCON_ADC_CAL_MASK, 0);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to set gauge sampling\n");
	ret = rk818_gauge_init(charger);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to initialize fuel gauge\n");

	charger->ac = devm_power_supply_register(&pdev->dev, &rk818_ac_desc,
						 &config);
	if (IS_ERR(charger->ac))
		return dev_err_probe(&pdev->dev, PTR_ERR(charger->ac),
				     "failed to register AC supply\n");

	charger->usb = devm_power_supply_register(&pdev->dev, &rk818_usb_desc,
						  &config);
	if (IS_ERR(charger->usb))
		return dev_err_probe(&pdev->dev, PTR_ERR(charger->usb),
				     "failed to register USB supply\n");

	/* Initialize work before any IRQ/notifier can schedule it. */
	ret = devm_delayed_work_autocancel(&pdev->dev, &charger->work,
					   rk818_monitor_work);
	if (ret)
		return ret;

	if (charger->edev) {
		charger->cable_nb.notifier_call = rk818_cable_changed;
		ret = devm_extcon_register_notifier_all(&pdev->dev, charger->edev,
						      &charger->cable_nb);
		if (ret)
			return dev_err_probe(&pdev->dev, ret,
					     "failed to register charger notifier\n");
	}

	plug_in_irq = platform_get_irq(pdev, 0);
	if (plug_in_irq < 0)
		return plug_in_irq;

	plug_out_irq = platform_get_irq(pdev, 1);
	if (plug_out_irq < 0)
		return plug_out_irq;

	vb_low_irq = platform_get_irq(pdev, 2);
	if (vb_low_irq < 0)
		return vb_low_irq;

	ret = devm_request_threaded_irq(&pdev->dev, plug_in_irq, NULL,
					rk818_plug_irq, IRQF_ONESHOT,
					"rk818-plug-in", charger);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to request plug-in IRQ\n");

	ret = devm_request_threaded_irq(&pdev->dev, plug_out_irq, NULL,
					rk818_plug_irq, IRQF_ONESHOT,
					"rk818-plug-out", charger);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to request plug-out IRQ\n");

	ret = devm_request_threaded_irq(&pdev->dev, vb_low_irq, NULL,
					rk818_vb_low_irq, IRQF_ONESHOT,
					"rk818-vb-low", charger);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to request low-voltage IRQ\n");
	ret = device_init_wakeup(&pdev->dev, true);
	if (ret)
		return ret;
	ret = dev_pm_set_wake_irq(&pdev->dev, vb_low_irq);
	if (ret) {
		device_init_wakeup(&pdev->dev, false);
		return dev_err_probe(&pdev->dev, ret, "failed to set low-voltage wake IRQ\n");
	}
	ret = devm_add_action_or_reset(&pdev->dev, rk818_disable_wakeup, &pdev->dev);
	if (ret)
		return ret;

	mod_delayed_work(system_freezable_wq, &charger->work, 0);
	dev_info(&pdev->dev, "RK818 battery monitor initialized\n");

	return 0;
}

static int rk818_charger_suspend(struct device *dev)
{
	struct rk818_charger *charger = dev_get_drvdata(dev);

	if (READ_ONCE(charger->shutdown_pending))
		return -EBUSY;

	mutex_lock(&charger->gauge_lock);
	charger->negative_pending = false;
	charger->low_voltage_pending = false;
	mutex_unlock(&charger->gauge_lock);

	/* A new cable cannot be classified while the system is asleep. */
	return regmap_update_bits(charger->rk808->regmap, RK818_USB_CTRL_REG,
				  RK818_USB_ILIM_SEL_MASK,
				  rk818_input_current_sel(RK818_USB_INPUT_CURRENT_UA));
}

static int rk818_charger_resume(struct device *dev)
{
	struct rk818_charger *charger = dev_get_drvdata(dev);
	int ret;

	ret = rk818_charger_init(charger);
	if (ret)
		return ret;
	mod_delayed_work(system_freezable_wq, &charger->work, 0);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(rk818_charger_pm_ops,
			      rk818_charger_suspend, rk818_charger_resume);

static struct platform_driver rk818_charger_driver = {
	.probe = rk818_charger_probe,
	.driver = {
		.name = "rk818-charger",
		.pm = pm_sleep_ptr(&rk818_charger_pm_ops),
	},
};
module_platform_driver(rk818_charger_driver);

MODULE_DESCRIPTION("Battery and charger monitor for RK818 PMIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rk818-charger");
