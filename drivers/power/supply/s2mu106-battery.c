// SPDX-License-Identifier: GPL-2.0
/*
 * Battery fuel gauge driver for the Samsung S2MU106 PMIC
 *
 * Based on the Samsung S2MU106 fuel gauge driver:
 * Copyright (C) 2018 Samsung Electronics
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device-id/of.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/units.h>

#define S2MU106_FG_REG_STATUS		0x00
#define S2MU106_FG_REG_IRQ		0x02
#define S2MU106_FG_REG_IRQ_MASK		0x03
#define S2MU106_FG_REG_RVBAT		0x04
#define S2MU106_FG_REG_RCURCC		0x06
#define S2MU106_FG_REG_RSOC		0x08
#define S2MU106_FG_REG_MONOUT		0x0a
#define S2MU106_FG_REG_MONOUTSEL	0x0c
#define S2MU106_FG_REG_BATCAP		0x0e	/* 4 bytes, 0x0e - 0x11 */
#define S2MU106_FG_REG_CTRL14		0x14
#define S2MU106_FG_REG_IRQLVL		0x1a
#define S2MU106_FG_REG_IRQLVL_H		0x1b
#define S2MU106_FG_REG_START		0x1e
#define S2MU106_FG_REG_POR_STATE	0x1f
#define S2MU106_FG_REG_AVG_CFG0		0x40
#define S2MU106_FG_REG_AVG_CFG1		0x41
#define S2MU106_FG_REG_ACCUM_L		0x44
#define S2MU106_FG_REG_ACCUM_H		0x45
#define S2MU106_FG_REG_MODE		0x4a
#define S2MU106_FG_REG_MODE_EN		0x4b
#define S2MU106_FG_REG_CSLOPE_L		0x58	/* 4 bytes, 0x58 - 0x5b */
#define S2MU106_FG_REG_CSLOPE_H		0x59
#define S2MU106_FG_REG_COFFSET_L	0x5a
#define S2MU106_FG_REG_TRIM_H		0x5b
#define S2MU106_FG_REG_CTRL5C		0x5c
#define S2MU106_FG_REG_VM		0x67
#define S2MU106_FG_REG_MODEL		0x92	/* 110 bytes, 0x92 - 0xff */

#define S2MU106_FG_MONOUTSEL_AVGVOLTAGE		0x16
#define S2MU106_FG_MONOUTSEL_AVGCURRENT		0x17
#define S2MU106_FG_MONOUTSEL_CYCLE		0x27
#define S2MU106_FG_MONOUTSEL_IDLE		0x10
#define S2MU106_FG_MONOUTSEL_BATCAP_OCV_EN	BIT(6)

#define S2MU106_FG_IRQ_MASK_SOC		BIT(0)
#define S2MU106_FG_IRQ_MASK_VBAT	BIT(1)
#define S2MU106_FG_IRQ_MASK_IDLE	BIT(2)
#define S2MU106_FG_IRQ_MASK_INIT	BIT(3)

#define S2MU106_FG_IRQLVL_SOC		GENMASK(7, 4)
#define S2MU106_FG_IRQLVL_VBAT		GENMASK(3, 0)

#define S2MU106_FG_START_RESTART	0x0f
#define S2MU106_FG_START_READY		0x03

#define S2MU106_FG_MODE_CURRENT		0x10
#define S2MU106_FG_MODE_EN_VOLTAGE	GENMASK(6, 4)
#define S2MU106_FG_VM_VM3_FLAG_EN	BIT(2)

#define S2MU106_FG_ACCUM_H_MASK		GENMASK(3, 0)
#define S2MU106_FG_ACCUM_RATE		0x800

#define S2MU106_FG_TRIM_H_CSLOPE	GENMASK(7, 4)
#define S2MU106_FG_TRIM_H_COFFSET	GENMASK(3, 0)

#define S2MU106_FG_BATCAP_LEN		4
#define S2MU106_FG_MODEL_LEN		110

/* SOC alert at 1 %, voltage alert at the lowest (3.55 V) threshold */
#define S2MU106_FG_ALERT_SOC		1
#define S2MU106_FG_ALERT_VBAT		0

#define S2MU106_FG_SHUNT_DEFAULT_UOHM	10000

struct s2mu106_fg {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	/* Serializes MONOUTSEL selection and MONOUT readback */
	struct mutex monout_lock;
};

static const struct regmap_config s2mu106_fg_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xff,
};

/* 16-bit values are split over two little-endian registers */
static int s2mu106_fg_read16(struct s2mu106_fg *fg, unsigned int reg, u16 *val)
{
	__le16 raw;
	int ret;

	ret = regmap_bulk_read(fg->regmap, reg, &raw, sizeof(raw));
	if (ret) {
		dev_err(fg->dev, "failed to read register 0x%02x (%d)\n", reg, ret);
		return ret;
	}

	*val = le16_to_cpu(raw);

	return 0;
}

static int s2mu106_fg_read_monout(struct s2mu106_fg *fg, u8 sel, u16 *val)
{
	int ret;

	guard(mutex)(&fg->monout_lock);

	ret = regmap_write(fg->regmap, S2MU106_FG_REG_MONOUTSEL, sel);
	if (ret)
		return ret;

	/* Give the gauge time to latch the selected value */
	fsleep(1000);

	ret = s2mu106_fg_read16(fg, S2MU106_FG_REG_MONOUT, val);

	regmap_write(fg->regmap, S2MU106_FG_REG_MONOUTSEL,
		     S2MU106_FG_MONOUTSEL_IDLE);

	return ret;
}

/* Voltages are unsigned, in 1/8192 V (instant) or 1/4096 V (average) */
static int s2mu106_fg_get_voltage_now(struct s2mu106_fg *fg, int *uv)
{
	u16 val;
	int ret;

	ret = s2mu106_fg_read16(fg, S2MU106_FG_REG_RVBAT, &val);
	if (ret)
		return ret;

	*uv = ((u64)val * MICRO) >> 13;

	return 0;
}

static int s2mu106_fg_get_voltage_avg(struct s2mu106_fg *fg, int *uv)
{
	u16 val;
	int ret;

	ret = s2mu106_fg_read_monout(fg, S2MU106_FG_MONOUTSEL_AVGVOLTAGE, &val);
	if (ret)
		return ret;

	*uv = ((u64)val * MICRO) >> 12;

	return 0;
}

/* Currents are signed, in 1/4096 A, negative while charging */
static int s2mu106_fg_raw_to_ua(u16 val)
{
	return -(((s64)(s16)val * MICRO) >> 12);
}

static int s2mu106_fg_get_current_now(struct s2mu106_fg *fg, int *ua)
{
	u16 val;
	int ret;

	ret = s2mu106_fg_read16(fg, S2MU106_FG_REG_RCURCC, &val);
	if (ret)
		return ret;

	*ua = s2mu106_fg_raw_to_ua(val);

	return 0;
}

static int s2mu106_fg_get_current_avg(struct s2mu106_fg *fg, int *ua)
{
	u16 val;
	int ret;

	ret = s2mu106_fg_read_monout(fg, S2MU106_FG_MONOUTSEL_AVGCURRENT, &val);
	if (ret)
		return ret;

	*ua = s2mu106_fg_raw_to_ua(val);

	return 0;
}

/* The SOC is signed, in 1/16384 of the full charge */
static int s2mu106_fg_get_soc(struct s2mu106_fg *fg, int *soc)
{
	u16 val;
	int ret;

	ret = s2mu106_fg_read16(fg, S2MU106_FG_REG_RSOC, &val);
	if (ret)
		return ret;

	*soc = clamp_t(s16, val, 0, 1 << 14);

	return 0;
}

static int s2mu106_fg_get_capacity(struct s2mu106_fg *fg, int *percent)
{
	int soc, ret;

	ret = s2mu106_fg_get_soc(fg, &soc);
	if (ret)
		return ret;

	*percent = (soc * 100) >> 14;

	return 0;
}

/* Learned battery capacity, in 1/4 mAh */
static int s2mu106_fg_get_charge_full(struct s2mu106_fg *fg, int *uah)
{
	u16 val;
	int ret;

	ret = s2mu106_fg_read16(fg, S2MU106_FG_REG_BATCAP, &val);
	if (ret)
		return ret;

	*uah = (val >> 2) * 1000;

	return 0;
}

static int s2mu106_fg_get_charge_counter(struct s2mu106_fg *fg, int *uah)
{
	int full, soc, ret;

	ret = s2mu106_fg_get_charge_full(fg, &full);
	if (ret)
		return ret;

	ret = s2mu106_fg_get_soc(fg, &soc);
	if (ret)
		return ret;

	*uah = ((s64)full * soc) >> 14;

	return 0;
}

static int s2mu106_fg_get_cycle_count(struct s2mu106_fg *fg, int *cycles)
{
	u16 val;
	int ret;

	ret = s2mu106_fg_read_monout(fg, S2MU106_FG_MONOUTSEL_CYCLE, &val);
	if (ret)
		return ret;

	*cycles = val;

	return 0;
}

static int s2mu106_fg_get_status(struct s2mu106_fg *fg, int *status)
{
	int current_now, current_avg, capacity;
	int ret;

	ret = s2mu106_fg_get_current_now(fg, &current_now);
	if (ret)
		return ret;

	ret = s2mu106_fg_get_current_avg(fg, &current_avg);
	if (ret)
		return ret;

	/* Check both values, the instant current is noisy around zero */
	if (current_now < 0 && current_avg < 0) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else if (current_now == 0) {
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		ret = s2mu106_fg_get_capacity(fg, &capacity);
		if (ret)
			return ret;

		*status = capacity > 98 ? POWER_SUPPLY_STATUS_FULL :
					  POWER_SUPPLY_STATUS_CHARGING;
	}

	return 0;
}

static const enum power_supply_property s2mu106_fg_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
};

static int s2mu106_fg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct s2mu106_fg *fg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return s2mu106_fg_get_status(fg, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return s2mu106_fg_get_voltage_now(fg, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		return s2mu106_fg_get_voltage_avg(fg, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return s2mu106_fg_get_current_now(fg, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		return s2mu106_fg_get_current_avg(fg, &val->intval);
	case POWER_SUPPLY_PROP_CAPACITY:
		return s2mu106_fg_get_capacity(fg, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		return s2mu106_fg_get_charge_full(fg, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		return s2mu106_fg_get_charge_counter(fg, &val->intval);
	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		return s2mu106_fg_get_cycle_count(fg, &val->intval);
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc s2mu106_fg_desc = {
	.name = "s2mu106-fuel-gauge",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = s2mu106_fg_properties,
	.num_properties = ARRAY_SIZE(s2mu106_fg_properties),
	.get_property = s2mu106_fg_get_property,
};

static irqreturn_t s2mu106_fg_irq(int irq, void *data)
{
	struct s2mu106_fg *fg = data;
	unsigned int val;

	/* Reading the interrupt register acknowledges the alert */
	regmap_read(fg->regmap, S2MU106_FG_REG_IRQ, &val);

	power_supply_changed(fg->psy);

	return IRQ_HANDLED;
}

/*
 * The factory trim assumes a 10 mOhm sense resistor. With 5 mOhm, double the
 * 20-bit current slope and the 12-bit current offset (both two's complement)
 * so that the current registers keep their scale.
 */
static int s2mu106_fg_trim_shunt(struct s2mu106_fg *fg)
{
	struct regmap *regmap = fg->regmap;
	u32 cslope, coffset;
	u8 trim[4];
	int ret;

	ret = regmap_bulk_read(regmap, S2MU106_FG_REG_CSLOPE_L, trim,
			       sizeof(trim));
	if (ret)
		return ret;

	cslope = FIELD_GET(S2MU106_FG_TRIM_H_CSLOPE, trim[3]) << 16 |
		 trim[1] << 8 | trim[0];
	coffset = FIELD_GET(S2MU106_FG_TRIM_H_COFFSET, trim[3]) << 8 | trim[2];

	cslope = (cslope << 1) & GENMASK(19, 0);
	coffset = (coffset << 1) & GENMASK(11, 0);

	dev_dbg(fg->dev, "5 mOhm current trim: slope 0x%05x offset 0x%03x\n",
		cslope, coffset);

	ret = regmap_write(regmap, S2MU106_FG_REG_CSLOPE_L, cslope & 0xff);
	if (ret)
		return ret;

	ret = regmap_write(regmap, S2MU106_FG_REG_CSLOPE_H, (cslope >> 8) & 0xff);
	if (ret)
		return ret;

	ret = regmap_write(regmap, S2MU106_FG_REG_TRIM_H,
			   FIELD_PREP(S2MU106_FG_TRIM_H_CSLOPE, cslope >> 16) |
			   FIELD_PREP(S2MU106_FG_TRIM_H_COFFSET, coffset >> 8));
	if (ret)
		return ret;

	return regmap_write(regmap, S2MU106_FG_REG_COFFSET_L, coffset & 0xff);
}

static int s2mu106_fg_load_model(struct s2mu106_fg *fg, const u8 *batcap,
				 const u8 *model)
{
	struct regmap *regmap = fg->regmap;
	unsigned int i;
	int ret;

	for (i = 0; i < S2MU106_FG_BATCAP_LEN; i++) {
		ret = regmap_write(regmap, S2MU106_FG_REG_BATCAP + i, batcap[i]);
		if (ret)
			return ret;
	}

	/* Latch the new battery capacity */
	scoped_guard(mutex, &fg->monout_lock) {
		ret = regmap_set_bits(regmap, S2MU106_FG_REG_MONOUTSEL,
				      S2MU106_FG_MONOUTSEL_BATCAP_OCV_EN);
		if (ret)
			return ret;
	}

	for (i = 0; i < S2MU106_FG_MODEL_LEN; i++) {
		ret = regmap_write(regmap, S2MU106_FG_REG_MODEL + i, model[i]);
		if (ret)
			return ret;
	}

	ret = regmap_write(regmap, S2MU106_FG_REG_CTRL14, 0x67);
	if (ret)
		return ret;

	ret = regmap_write(regmap, S2MU106_FG_REG_ACCUM_L,
			   S2MU106_FG_ACCUM_RATE & 0xff);
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, S2MU106_FG_REG_ACCUM_H,
				 S2MU106_FG_ACCUM_H_MASK,
				 S2MU106_FG_ACCUM_RATE >> 8);
	if (ret)
		return ret;

	ret = regmap_write(regmap, S2MU106_FG_REG_AVG_CFG1, 0x04);
	if (ret)
		return ret;

	return regmap_write(regmap, S2MU106_FG_REG_CTRL5C, 0x1a);
}

/* Reprogram the gauge after it lost its state */
static int s2mu106_fg_reinit(struct s2mu106_fg *fg, u32 shunt_uohm)
{
	struct device *dev = fg->dev;
	u8 batcap[S2MU106_FG_BATCAP_LEN];
	u8 model[S2MU106_FG_MODEL_LEN];
	int ret;

	ret = device_property_read_u8_array(dev, "samsung,battery-capacity",
					    batcap, sizeof(batcap));
	if (!ret)
		ret = device_property_read_u8_array(dev, "samsung,battery-model",
						    model, sizeof(model));
	if (!ret) {
		ret = s2mu106_fg_load_model(fg, batcap, model);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to load the battery model\n");
	} else {
		dev_warn(dev, "no battery model, using the factory defaults\n");
	}

	/* Restart gauging with the new parameters */
	ret = regmap_write(fg->regmap, S2MU106_FG_REG_START,
			   S2MU106_FG_START_RESTART);
	if (ret)
		return ret;

	msleep(300);

	ret = regmap_write(fg->regmap, S2MU106_FG_REG_POR_STATE, 0);
	if (ret)
		return ret;

	if (shunt_uohm == 5000) {
		ret = s2mu106_fg_trim_shunt(fg);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to trim the current sense\n");
	}

	return 0;
}

static int s2mu106_fg_init(struct s2mu106_fg *fg)
{
	struct regmap *regmap = fg->regmap;
	struct device *dev = fg->dev;
	u32 shunt_uohm = S2MU106_FG_SHUNT_DEFAULT_UOHM;
	unsigned int por_state, start;
	int ret;

	device_property_read_u32(dev, "shunt-resistor-micro-ohms", &shunt_uohm);
	if (shunt_uohm != 5000 && shunt_uohm != 10000)
		return dev_err_probe(dev, -EINVAL,
				     "unsupported shunt resistor %u uOhm\n",
				     shunt_uohm);

	/* Average voltage sampling, used by the MONOUT readings */
	ret = regmap_write(regmap, S2MU106_FG_REG_AVG_CFG0, 0x08);
	if (ret)
		return ret;

	/* Track the SOC with the coulomb counter, also across a restart */
	ret = regmap_write(regmap, S2MU106_FG_REG_MODE, S2MU106_FG_MODE_CURRENT);
	if (ret)
		return ret;

	ret = regmap_clear_bits(regmap, S2MU106_FG_REG_MODE_EN,
				S2MU106_FG_MODE_EN_VOLTAGE);
	if (ret)
		return ret;

	ret = regmap_read(regmap, S2MU106_FG_REG_POR_STATE, &por_state);
	if (ret)
		return ret;

	ret = regmap_read(regmap, S2MU106_FG_REG_START, &start);
	if (ret)
		return ret;

	/*
	 * The gauge is powered by the battery and keeps its configuration
	 * across reboots, only reprogram it after it was reset.
	 */
	if (por_state || start != S2MU106_FG_START_READY) {
		dev_info(dev, "gauge was reset (por 0x%02x, start 0x%02x), reinitializing\n",
			 por_state, start);
		ret = s2mu106_fg_reinit(fg, shunt_uohm);
		if (ret)
			return ret;
	}

	ret = regmap_clear_bits(regmap, S2MU106_FG_REG_VM,
				S2MU106_FG_VM_VM3_FLAG_EN);
	if (ret)
		return ret;

	/* Only raise the alert interrupt on low SOC and low voltage */
	ret = regmap_write(regmap, S2MU106_FG_REG_IRQLVL,
			   FIELD_PREP(S2MU106_FG_IRQLVL_SOC, S2MU106_FG_ALERT_SOC) |
			   FIELD_PREP(S2MU106_FG_IRQLVL_VBAT, S2MU106_FG_ALERT_VBAT));
	if (ret)
		return ret;

	ret = regmap_write(regmap, S2MU106_FG_REG_IRQLVL_H, 0);
	if (ret)
		return ret;

	return regmap_update_bits(regmap, S2MU106_FG_REG_IRQ_MASK,
				  S2MU106_FG_IRQ_MASK_SOC |
				  S2MU106_FG_IRQ_MASK_VBAT |
				  S2MU106_FG_IRQ_MASK_IDLE |
				  S2MU106_FG_IRQ_MASK_INIT,
				  S2MU106_FG_IRQ_MASK_IDLE |
				  S2MU106_FG_IRQ_MASK_INIT);
}

static int s2mu106_fg_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config psy_cfg = {};
	struct s2mu106_fg *fg;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->dev = dev;

	ret = devm_mutex_init(dev, &fg->monout_lock);
	if (ret)
		return ret;

	fg->regmap = devm_regmap_init_i2c(client, &s2mu106_fg_regmap_config);
	if (IS_ERR(fg->regmap))
		return dev_err_probe(dev, PTR_ERR(fg->regmap),
				     "failed to initialize regmap\n");

	ret = s2mu106_fg_init(fg);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize the gauge\n");

	psy_cfg.drv_data = fg;
	psy_cfg.fwnode = dev_fwnode(dev);
	fg->psy = devm_power_supply_register(dev, &s2mu106_fg_desc, &psy_cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(dev, PTR_ERR(fg->psy),
				     "failed to register power supply\n");

	if (client->irq) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						s2mu106_fg_irq, IRQF_ONESHOT,
						"s2mu106-fuel-gauge", fg);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request IRQ\n");
	}

	return 0;
}

static const struct of_device_id s2mu106_fg_of_match[] = {
	{ .compatible = "samsung,s2mu106-fuel-gauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, s2mu106_fg_of_match);

static struct i2c_driver s2mu106_fg_driver = {
	.driver = {
		.name = "s2mu106-fuel-gauge",
		.of_match_table = s2mu106_fg_of_match,
	},
	.probe = s2mu106_fg_probe,
};
module_i2c_driver(s2mu106_fg_driver);

MODULE_DESCRIPTION("Samsung S2MU106 PMIC battery fuel gauge driver");
MODULE_LICENSE("GPL");
