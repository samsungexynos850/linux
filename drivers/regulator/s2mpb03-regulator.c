// SPDX-License-Identifier: GPL-2.0+
//
// Regulator driver for the Samsung S2MPB03
//
// Copyright (C) 2016 Samsung Electronics
// Copyright (C) 2026 schoosh212 <superaviation001@gmail.com>

#include <linux/err.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/s2mpb03.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define S2MPB03_LDO_ENABLE_TIME		150	/* us */
#define S2MPB03_LDO_RAMP_DELAY		12000	/* uV/us */

static const struct regulator_ops s2mpb03_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.set_voltage_time_sel	= regulator_set_voltage_time_sel,
};

#define S2MPB03_LDO(_num, _min_uV, _step_uV) {				\
	.name		= "ldo" #_num,					\
	.of_match	= "ldo" #_num,					\
	.regulators_node = "regulators",				\
	.id		= S2MPB03_LDO##_num,				\
	.ops		= &s2mpb03_ldo_ops,				\
	.type		= REGULATOR_VOLTAGE,				\
	.owner		= THIS_MODULE,					\
	.min_uV		= _min_uV,					\
	.uV_step	= _step_uV,					\
	.n_voltages	= S2MPB03_LDO_N_VOLTAGES,			\
	.vsel_reg	= S2MPB03_REG_LDO##_num##_CTRL,			\
	.vsel_mask	= S2MPB03_LDO_VSEL_MASK,			\
	.enable_reg	= S2MPB03_REG_LDO##_num##_CTRL,			\
	.enable_mask	= S2MPB03_LDO_ENABLE_MASK,			\
	.enable_time	= S2MPB03_LDO_ENABLE_TIME,			\
	.ramp_delay	= S2MPB03_LDO_RAMP_DELAY,			\
}

static const struct regulator_desc s2mpb03_regulators[] = {
	S2MPB03_LDO(1, 700000, 12500),
	S2MPB03_LDO(2, 700000, 12500),
	S2MPB03_LDO(3, 700000, 25000),
	S2MPB03_LDO(4, 700000, 12500),
	S2MPB03_LDO(5, 1800000, 25000),
	S2MPB03_LDO(6, 1800000, 25000),
	S2MPB03_LDO(7, 1800000, 25000),
};

static int s2mpb03_regulator_probe(struct platform_device *pdev)
{
	struct sec_pmic_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct regulator_config config = { };
	unsigned int i;

	/* The regulators node is a child of the parent PMIC node */
	config.dev = pdev->dev.parent;
	config.regmap = iodev->regmap_pmic;

	for (i = 0; i < ARRAY_SIZE(s2mpb03_regulators); i++) {
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(&pdev->dev,
					       &s2mpb03_regulators[i], &config);
		if (IS_ERR(rdev))
			return dev_err_probe(&pdev->dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     s2mpb03_regulators[i].name);
	}

	return 0;
}

static const struct platform_device_id s2mpb03_regulator_id[] = {
	{ .name = "s2mpb03-regulator" },
	{ }
};
MODULE_DEVICE_TABLE(platform, s2mpb03_regulator_id);

static struct platform_driver s2mpb03_regulator_driver = {
	.driver = {
		.name = "s2mpb03-regulator",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = s2mpb03_regulator_probe,
	.id_table = s2mpb03_regulator_id,
};
module_platform_driver(s2mpb03_regulator_driver);

MODULE_DESCRIPTION("Samsung S2MPB03 regulator driver");
MODULE_LICENSE("GPL");
