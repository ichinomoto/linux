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
#include <linux/interrupt.h>
#include <linux/mfd/rk808.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>

#define RK818_MONITOR_INTERVAL_MS	5000
#define RK818_GASCNT_PER_MAH		2390

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
	struct delayed_work work;
	struct power_supply_battery_ocv_table *ocv_table;
	int ocv_table_len;
	int voltage_k;
	int voltage_b;
	int charge_full_design_uah;
	int voltage_min_design_uv;
	int voltage_max_design_uv;
	int constant_charge_current_max_ua;
	int constant_charge_voltage_max_uv;
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

static int rk818_charger_init(struct rk818_charger *charger)
{
	int charge_current_sel;
	int ret;

	/*
	 * The input limit is set to 3 A.  The battery charge current is selected
	 * from the board-specific simple-battery description.
	 */
	charge_current_sel =
		rk818_charge_current_sel(charger->constant_charge_current_max_ua);
	if (charge_current_sel < 0)
		return charge_current_sel;

	ret = regmap_update_bits(charger->rk808->regmap, RK818_USB_CTRL_REG,
				 RK818_USB_ILIM_SEL_MASK |
				 RK818_USB_CHRG_CT_EN,
				 RK818_USB_ILIM_3000MA |
				 RK818_USB_CHRG_CT_EN);
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

static int rk818_read_charge_now(struct rk818_charger *charger, int *charge_uah)
{
	u32 raw;
	u32 charge_mah;
	int ret;

	ret = rk818_read_be32(charger, RK818_GASCNT3_REG, &raw);
	if (ret)
		return ret;

	charge_mah = raw / RK818_GASCNT_PER_MAH;
	if (charge_mah > INT_MAX / 1000)
		return -ERANGE;

	*charge_uah = charge_mah * 1000;
	return 0;
}

static int rk818_read_charge_full(struct rk818_charger *charger, int *charge_uah)
{
	u32 raw;
	int ret;

	ret = rk818_read_be32(charger, RK818_NEW_FCC_REG3, &raw);
	if (ret)
		return ret;

	/* The BSP stores FCC in mAh with a one-unit validity marker. */
	if (raw > 1 && raw - 1 <= INT_MAX / 1000)
		*charge_uah = (raw - 1) * 1000;
	else
		*charge_uah = charger->charge_full_design_uah;

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

static int rk818_battery_capacity(struct rk818_charger *charger, int *capacity)
{
	int charge_now, charge_full;
	int voltage_uv;
	u64 percent;
	int ret;

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

	if (charger->ocv_table)
		*capacity = power_supply_ocv2cap_simple(charger->ocv_table,
							charger->ocv_table_len,
							voltage_uv);
	else
		*capacity = DIV_ROUND_CLOSEST((voltage_uv -
					charger->voltage_min_design_uv) * 100,
				       charger->voltage_max_design_uv -
					charger->voltage_min_design_uv);

	*capacity = clamp(*capacity, 0, 100);
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

static int rk818_external_get_property(struct power_supply *psy,
				       enum power_supply_property property,
				       union power_supply_propval *value)
{
	struct rk818_charger *charger = power_supply_get_drvdata(psy);
	bool plugged;
	int ret;

	if (property != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	ret = rk818_read_plugged(charger, &plugged);
	if (ret)
		return ret;

	/*
	 * RK818 cannot classify the source without the USB controller's DP/DM
	 * result.  Preserve the DM200 BSP behaviour: report any external source
	 * as AC and leave USB offline until that integration is implemented.
	 */
	if (psy == charger->usb)
		value->intval = 0;
	else
		value->intval = plugged;

	return 0;
}

static const enum power_supply_property rk818_external_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
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
	.properties = rk818_external_properties,
	.num_properties = ARRAY_SIZE(rk818_external_properties),
	.get_property = rk818_external_get_property,
};

static void rk818_monitor_work(struct work_struct *work)
{
	struct rk818_charger *charger =
		container_of(work, struct rk818_charger, work.work);

	power_supply_changed(charger->battery);
	power_supply_changed(charger->ac);
	power_supply_changed(charger->usb);
	queue_delayed_work(system_percpu_wq, &charger->work,
			   msecs_to_jiffies(RK818_MONITOR_INTERVAL_MS));
}

static irqreturn_t rk818_plug_irq(int irq, void *data)
{
	struct rk818_charger *charger = data;

	mod_delayed_work(system_percpu_wq, &charger->work, 0);
	return IRQ_HANDLED;
}

static void rk818_put_of_node(void *data)
{
	of_node_put(data);
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

static int rk818_charger_probe(struct platform_device *pdev)
{
	struct rk808 *rk808 = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config config = {};
	struct device_node *node;
	struct rk818_charger *charger;
	int plug_in_irq, plug_out_irq;
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

	ret = rk818_charger_init(charger);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to initialize charger\n");

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

	plug_in_irq = platform_get_irq(pdev, 0);
	if (plug_in_irq < 0)
		return plug_in_irq;

	plug_out_irq = platform_get_irq(pdev, 1);
	if (plug_out_irq < 0)
		return plug_out_irq;

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

	ret = devm_delayed_work_autocancel(&pdev->dev, &charger->work,
					   rk818_monitor_work);
	if (ret)
		return ret;

	mod_delayed_work(system_percpu_wq, &charger->work, 0);
	dev_info(&pdev->dev, "RK818 battery monitor initialized\n");

	return 0;
}

static struct platform_driver rk818_charger_driver = {
	.probe = rk818_charger_probe,
	.driver = {
		.name = "rk818-charger",
	},
};
module_platform_driver(rk818_charger_driver);

MODULE_DESCRIPTION("Battery and charger monitor for RK818 PMIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rk818-charger");
