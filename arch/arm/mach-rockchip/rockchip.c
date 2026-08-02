// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Device Tree support for Rockchip SoCs
 *
 * Copyright (c) 2013 MundoReader S.L.
 * Author: Heiko Stuebner <heiko@sntech.de>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_clk.h>
#include <linux/clocksource.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include "core.h"
#include "pm.h"

#define RK3128_TIMER5_PHYS      0x200440a0
#define RK3128_CRU_PHYS         0x20000000
#define RK3128_CRU_MISC_CON     0x134
#define RK3288_TIMER6_7_PHYS    0xff810000

static void __init rk3128_stimer_init(void)
{
	void __iomem *reg_base;

	/*
	 * The architected timer needs timer5 to run, but the stock DM200
	 * U-Boot does not enable it.  Mirror the vendor RK312x kernel setup.
	 */
	reg_base = ioremap(RK3128_TIMER5_PHYS, SZ_4K);
	if (!reg_base) {
		pr_err("rockchip: could not map timer5 registers\n");
		return;
	}

	writel_relaxed(0, reg_base + 0x10);
	dsb();
	writel_relaxed(0xffffffff, reg_base + 0x00);
	writel_relaxed(0xffffffff, reg_base + 0x04);
	dsb();
	writel_relaxed(1, reg_base + 0x10);
	dsb();
	iounmap(reg_base);

	reg_base = ioremap(RK3128_CRU_PHYS, SZ_4K);
	if (!reg_base) {
		pr_err("rockchip: could not map CRU registers\n");
		return;
	}

	writel_relaxed(0x80000000, reg_base + RK3128_CRU_MISC_CON);
	dsb();
	iounmap(reg_base);
}

static void __init rockchip_timer_init(void)
{
	if (of_machine_is_compatible("rockchip,rk3128"))
		rk3128_stimer_init();

	if (of_machine_is_compatible("rockchip,rk3288")) {
		void __iomem *reg_base;

		/*
		 * Most/all uboot versions for rk3288 don't enable timer7
		 * which is needed for the architected timer to work.
		 * So make sure it is running during early boot.
		 */
		reg_base = ioremap(RK3288_TIMER6_7_PHYS, SZ_16K);
		if (reg_base) {
			writel(0, reg_base + 0x30);
			writel(0xffffffff, reg_base + 0x20);
			writel(0xffffffff, reg_base + 0x24);
			writel(1, reg_base + 0x30);
			dsb();
			iounmap(reg_base);
		} else {
			pr_err("rockchip: could not map timer7 registers\n");
		}
	}

	of_clk_init(NULL);
	timer_probe();
}

static void __init rockchip_dt_init(void)
{
	rockchip_suspend_init();
}

static const char * const rockchip_board_dt_compat[] = {
	"rockchip,rk2928",
	"rockchip,rk3066a",
	"rockchip,rk3066b",
	"rockchip,rk3188",
	"rockchip,rk3128",
	"rockchip,rk3228",
	"rockchip,rk3288",
	"rockchip,rv1108",
	NULL,
};

DT_MACHINE_START(ROCKCHIP_DT, "Rockchip (Device Tree)")
	.l2c_aux_val	= 0,
	.l2c_aux_mask	= ~0,
	.init_time	= rockchip_timer_init,
	.dt_compat	= rockchip_board_dt_compat,
	.init_machine	= rockchip_dt_init,
MACHINE_END
