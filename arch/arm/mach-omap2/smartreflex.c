/*
 * linux/arch/arm/mach-omap3/smartreflex.c
 *
 * OMAP34XX SmartReflex Voltage Control
 *
 * Copyright (C) 2008 Nokia Corporation
 * Kalle Jokiniemi
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Lesly A M <x0080970@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>

#include <mach/prcm_34xx.h>
#ifdef CONFIG_TWL4030_CORE
#include <mach/power_companion.h>
#else
/* Add appropriate power IC header file */
#endif
#include <asm/io.h>

#include "prcm-regs.h"
#include "smartreflex.h"
#include "ti-compat.h"


/*#define DEBUG_SR 1*/
#ifdef DEBUG_SR
#define DPRINTK(fmt, args...) printk(KERN_ERR "%s: " fmt, __func__ ,\
									## args)
#else
#define DPRINTK(fmt, args...)
#endif

static u32 sr_nvalues = 0x0;

struct omap_sr{
	int srid;
	int is_sr_reset;
	int is_autocomp_active;
	struct clk *fck;
	u32 req_opp_no;
	u32 opp1_nvalue, opp2_nvalue, opp3_nvalue, opp4_nvalue, opp5_nvalue;
	u32 opp6_nvalue, opp7_nvalue;
	u32 senp_mod, senn_mod;
	u32 srbase_addr;
	u32 vpbase_addr;
};

static struct omap_sr sr1 = {
	.srid = SR1,
	.is_sr_reset = 1,
	.is_autocomp_active = 0,
	.srbase_addr = OMAP34XX_SR1_BASE,
};

static struct omap_sr sr2 = {
	.srid = SR2,
	.is_sr_reset = 1,
	.is_autocomp_active = 0,
	.srbase_addr = OMAP34XX_SR2_BASE,
};

static inline void sr_write_reg(struct omap_sr *sr, int offset, u32 value)
{
	omap_writel(value, sr->srbase_addr + offset);
}

static inline void sr_modify_reg(struct omap_sr *sr, int offset, u32 mask,
								u32 value)
{
	u32 reg_val;

	reg_val = omap_readl(sr->srbase_addr + offset);
	reg_val &= ~mask;
	reg_val |= value;

	omap_writel(reg_val, sr->srbase_addr + offset);
}

static inline u32 sr_read_reg(struct omap_sr *sr, int offset)
{
	return omap_readl(sr->srbase_addr + offset);
}

static void cal_reciprocal(u32 sensor, u32 *sengain, u32 *rnsen)
{
	u32 gn, rn, mul;

	for (gn = 0; gn < GAIN_MAXLIMIT; gn++) {
		mul = 1 << (gn + 8);
		rn = mul / sensor;
		if (rn < R_MAXLIMIT) {
			*sengain = gn;
			*rnsen = rn;
		}
	}
}

static int sr_clk_enable(struct omap_sr *sr)
{
	if (clk_enable(sr->fck) != 0) {
		printk(KERN_ERR "Could not enable sr%d_fck\n", sr->srid);
		goto clk_enable_err;
	}

	/* set fclk- active , iclk- idle */
	sr_modify_reg(sr, ERRCONFIG, SR_CLKACTIVITY_MASK,
						SR_CLKACTIVITY_IOFF_FON);

	return 0;

clk_enable_err:
	return -1;
}

static int sr_clk_disable(struct omap_sr *sr)
{
	/* set fclk, iclk- idle */
	sr_modify_reg(sr, ERRCONFIG, SR_CLKACTIVITY_MASK,
						SR_CLKACTIVITY_IOFF_FOFF);

	clk_disable(sr->fck);

	return 0;
}


static u32 cal_test_nvalue(u32 sennval, u32 senpval)
{
	u32 senpgain, senngain;
	u32 rnsenp, rnsenn;

	/* Calculating the gain and reciprocal of
	* the SenN and SenP values */
	cal_reciprocal(senpval, &senpgain, &rnsenp);
	cal_reciprocal(sennval, &senngain, &rnsenn);

	return (senpgain << NVALUERECIPROCAL_SENPGAIN_SHIFT) |
		(senngain << NVALUERECIPROCAL_SENNGAIN_SHIFT) |
		(rnsenp << NVALUERECIPROCAL_RNSENP_SHIFT) |
		(rnsenn << NVALUERECIPROCAL_RNSENN_SHIFT);
}


void sr_calculate_rg(u32 rfuse, u32 gain_fuse, u32 delta_nt,
				       u32 *rnsen, u32 *sengain)
{
       u32 nadj;
       nadj = ((1 << (gain_fuse + 8)) / rfuse) + delta_nt;
       cal_reciprocal(nadj, sengain, rnsen);
}

static u32 calculate_opp_nvalue(u32 opp5_nvalue, u32 delta_p, u32 delta_n) {
       u32 sen_pgain_fuse, sen_ngain_fuse, sen_prn_fuse, sen_nrn_fuse;
       u32 sen_nrn, sen_ngain, sen_prn, sen_pgain;
       
       sen_pgain_fuse = (opp5_nvalue & 0x00F0000) >> 0x14;
       sen_ngain_fuse = (opp5_nvalue & 0x000F0000) >> 0x10;
       sen_prn_fuse = (opp5_nvalue & 0x0000FF00) >> 0x08;
       sen_nrn_fuse = (opp5_nvalue & 0x000000FF);
       sr_calculate_rg(sen_nrn_fuse, sen_ngain_fuse, delta_n, &sen_nrn,
				       &sen_ngain);
       sr_calculate_rg(sen_prn_fuse, sen_pgain_fuse, delta_p, &sen_prn,
				       &sen_pgain);
       return  (sen_pgain << 0x14) | (sen_ngain << 0x10) | (sen_prn << 0x08) | (sen_nrn);
}


static int sr_set_efuse_nvalues(void)
{
	u32 n;
	u32 fuse_sr;
	int ret;

	fuse_sr = omap_readl(CONTROL_FUSE_SR);

	n = (fuse_sr & 0x0F);
	sr1.senp_mod = (n & 0x03);
	sr1.senn_mod = (n & 0x0C) >> 2;
	/* Read values for VDD1 from EFUSE */
	sr1.opp1_nvalue = omap_readl(CONTROL_FUSE_OPP1_VDD1) & 0xFFFFFF;
	sr1.opp2_nvalue = omap_readl(CONTROL_FUSE_OPP2_VDD1) & 0xFFFFFF;
	sr1.opp3_nvalue = omap_readl(CONTROL_FUSE_OPP3_VDD1) & 0xFFFFFF;
	sr1.opp4_nvalue = omap_readl(CONTROL_FUSE_OPP4_VDD1) & 0xFFFFFF;
	sr1.opp5_nvalue = omap_readl(CONTROL_FUSE_OPP5_VDD1) & 0xFFFFFF;
	if (sr1.opp5_nvalue) {
		sr1.opp6_nvalue = calculate_opp_nvalue(sr1.opp5_nvalue,
			227, 379);
		sr1.opp7_nvalue = calculate_opp_nvalue(sr1.opp5_nvalue,
			434, 730);
	}
	n = (fuse_sr & 0x0F00) >> 8;
	sr2.senp_mod = (n & 0x03);
	sr2.senn_mod = (n & 0x0C) >> 2;
	/* Read values for VDD2 from EFUSE */
	sr2.opp1_nvalue = omap_readl(CONTROL_FUSE_OPP1_VDD2) & 0xFFFFFF;
	sr2.opp2_nvalue = omap_readl(CONTROL_FUSE_OPP2_VDD2) & 0xFFFFFF;
	sr2.opp3_nvalue = omap_readl(CONTROL_FUSE_OPP3_VDD2) & 0xFFFFFF;

	if (fuse_sr  == 0x0)
		ret = 0;
	else
		ret = USE_EFUSE_NVALUES;

	return ret;
}

static int sr_set_test_nvalues(void)
{

	sr1.senp_mod = 0x03;    /* SenN-M5 enabled */
	sr1.senn_mod = 0x03;
	/* calculate nvalues for VDD1 OPP */
	sr1.opp1_nvalue = cal_test_nvalue(0x373 + 0x100, 0x28c + 0x100);
	sr1.opp2_nvalue = cal_test_nvalue(0x506 + 0x1a0, 0x3be + 0x1a0);
	sr1.opp3_nvalue = cal_test_nvalue(0x85b + 0x200, 0x655 + 0x200);
	sr1.opp4_nvalue = cal_test_nvalue(0x964 + 0x2a0, 0x727 + 0x2a0);
	sr1.opp5_nvalue = cal_test_nvalue(0xacd + 0x330, 0x848 + 0x330);
	if (sr1.opp5_nvalue) {
		sr1.opp6_nvalue = calculate_opp_nvalue(sr1.opp5_nvalue,
			227, 379);
		sr1.opp7_nvalue = calculate_opp_nvalue(sr1.opp5_nvalue,
			434, 730);
		}
	sr2.senp_mod = 0x03;
	sr2.senn_mod = 0x03;
	/* calculate nvalues for VDD2 OPP */
	sr2.opp1_nvalue = cal_test_nvalue(0x359, 0x25d);
	sr2.opp2_nvalue = cal_test_nvalue(0x4f5 + 0x1c0, 0x390 + 0x1c0);
	sr2.opp3_nvalue = cal_test_nvalue(0x76f + 0x200, 0x579 + 0x200);

	return USE_TEST_NVALUES;
}

static void sr_configure_vp(int srid)
{
	u32 vpconfig;

	if (srid == SR1) {
		vpconfig = PRM_VP1_CONFIG_ERROROFFSET | PRM_VP1_CONFIG_ERRORGAIN
			| PRM_VP1_CONFIG_INITVOLTAGE | PRM_VP1_CONFIG_TIMEOUTEN;

		PRM_VP1_CONFIG = vpconfig;
		PRM_VP1_VSTEPMIN = PRM_VP1_VSTEPMIN_SMPSWAITTIMEMIN |
						PRM_VP1_VSTEPMIN_VSTEPMIN;

		PRM_VP1_VSTEPMAX = PRM_VP1_VSTEPMAX_SMPSWAITTIMEMAX |
						PRM_VP1_VSTEPMAX_VSTEPMAX;

		PRM_VP1_VLIMITTO = PRM_VP1_VLIMITTO_VDDMAX |
			PRM_VP1_VLIMITTO_VDDMIN | PRM_VP1_VLIMITTO_TIMEOUT;

		PRM_VP1_CONFIG |= PRM_VP1_CONFIG_INITVDD;
		PRM_VP1_CONFIG &= ~PRM_VP1_CONFIG_INITVDD;

	} else if (srid == SR2) {
		vpconfig = PRM_VP2_CONFIG_ERROROFFSET | PRM_VP2_CONFIG_ERRORGAIN
			| PRM_VP2_CONFIG_INITVOLTAGE | PRM_VP2_CONFIG_TIMEOUTEN;

		PRM_VP2_CONFIG = vpconfig;
		PRM_VP2_VSTEPMIN = PRM_VP2_VSTEPMIN_SMPSWAITTIMEMIN |
						PRM_VP2_VSTEPMIN_VSTEPMIN;

		PRM_VP2_VSTEPMAX = PRM_VP2_VSTEPMAX_SMPSWAITTIMEMAX |
						PRM_VP2_VSTEPMAX_VSTEPMAX;

		PRM_VP2_VLIMITTO = PRM_VP2_VLIMITTO_VDDMAX |
			PRM_VP2_VLIMITTO_VDDMIN | PRM_VP2_VLIMITTO_TIMEOUT;

		PRM_VP2_CONFIG |= PRM_VP2_CONFIG_INITVDD;
		PRM_VP2_CONFIG &= ~PRM_VP2_CONFIG_INITVDD;

	}
}

static void sr_configure_vc(void)
{
	PRM_VC_SMPS_SA =
		(R_SRI2C_SLAVE_ADDR << PRM_VC_SMPS_SA1_SHIFT) |
		(R_SRI2C_SLAVE_ADDR << PRM_VC_SMPS_SA0_SHIFT);

	PRM_VC_SMPS_VOL_RA = (R_VDD2_SR_CONTROL << PRM_VC_SMPS_VOLRA1_SHIFT) |
				(R_VDD1_SR_CONTROL << PRM_VC_SMPS_VOLRA0_SHIFT);

	PRM_VC_CMD_VAL_0 = (PRM_VC_CMD_VAL0_ON << PRM_VC_CMD_ON_SHIFT) |
			(PRM_VC_CMD_VAL0_ONLP << PRM_VC_CMD_ONLP_SHIFT) |
			(PRM_VC_CMD_VAL0_RET << PRM_VC_CMD_RET_SHIFT) |
			(PRM_VC_CMD_VAL0_OFF << PRM_VC_CMD_OFF_SHIFT);
#ifdef CONFIG_TWL4030_CORE
	if (is_twl5030()) {
		PRM_VC_CMD_VAL_1 = (PRM_VC_CMD_VAL1_ON << PRM_VC_CMD_ON_SHIFT) |
			(PRM_VC_CMD_VAL1_ON << PRM_VC_CMD_ONLP_SHIFT) |
			(PRM_VC_CMD_VAL1_ON << PRM_VC_CMD_RET_SHIFT) |
			(PRM_VC_CMD_VAL1_ON << PRM_VC_CMD_OFF_SHIFT);
	} else {
		PRM_VC_CMD_VAL_1 = (PRM_VC_CMD_VAL1_ON << PRM_VC_CMD_ON_SHIFT) |
			(PRM_VC_CMD_VAL1_ONLP << PRM_VC_CMD_ONLP_SHIFT) |
			(PRM_VC_CMD_VAL1_RET << PRM_VC_CMD_RET_SHIFT) |
			(PRM_VC_CMD_VAL1_OFF << PRM_VC_CMD_OFF_SHIFT);
	}
#else
/* Add appropriate power IC configuration */
#endif

		PRM_VC_CH_CONF = PRM_VC_CH_CONF_CMD1 | PRM_VC_CH_CONF_RAV1;

		PRM_VC_I2C_CFG = PRM_VC_I2C_CFG_MCODE | PRM_VC_I2C_CFG_HSEN
							| PRM_VC_I2C_CFG_SREN;

	/* Setup voltctrl and other setup times */
#ifdef CONFIG_SYSOFFMODE
	PRM_VOLTCTRL = PRM_VOLTCTRL_AUTO_OFF | PRM_VOLTCTRL_AUTO_RET;
	PRM_CLKSETUP = PRM_CLKSETUP_DURATION;
	PRM_VOLTSETUP1 = (PRM_VOLTSETUP_TIME2 << PRM_VOLTSETUP_TIME2_OFFSET) |
			(PRM_VOLTSETUP_TIME1 << PRM_VOLTSETUP_TIME1_OFFSET);
	PRM_VOLTOFFSET = PRM_VOLTOFFSET_DURATION;
	PRM_VOLTSETUP2 = PRM_VOLTSETUP2_DURATION;
#else
	PRM_VOLTCTRL |= PRM_VOLTCTRL_AUTO_RET;
#endif

}

static void sr_configure(struct omap_sr *sr)
{
	u32 sys_clk, sr_clk_length = 0;
	u32 sr_config;
	u32 senp_en , senn_en;

	senp_en = sr->senp_mod;
	senn_en = sr->senn_mod;

	sys_clk = prcm_get_system_clock_speed();

	switch (sys_clk) {
	case 12000:
		sr_clk_length = SRCLKLENGTH_12MHZ_SYSCLK;
		break;
	case 13000:
		sr_clk_length = SRCLKLENGTH_13MHZ_SYSCLK;
		break;
	case 19200:
		sr_clk_length = SRCLKLENGTH_19MHZ_SYSCLK;
		break;
	case 26000:
		sr_clk_length = SRCLKLENGTH_26MHZ_SYSCLK;
		break;
	case 38400:
		sr_clk_length = SRCLKLENGTH_38MHZ_SYSCLK;
		break;
	default :
		printk(KERN_ERR "Invalid sysclk value\n");
		break;
	}

	DPRINTK("SR : sys clk %d\n", sys_clk);
	if (sr->srid == SR1) {
		sr_config = SR1_SRCONFIG_ACCUMDATA |
			(sr_clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			SRCONFIG_MINMAXAVG_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;

		sr_write_reg(sr, SRCONFIG, sr_config);

		sr_write_reg(sr, AVGWEIGHT, SR1_AVGWEIGHT_SENPAVGWEIGHT |
					SR1_AVGWEIGHT_SENNAVGWEIGHT);

		sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
			SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR1_ERRWEIGHT | SR1_ERRMAXLIMIT | SR1_ERRMINLIMIT));

	} else if (sr->srid == SR2) {
		sr_config = SR2_SRCONFIG_ACCUMDATA |
			(sr_clk_length << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_SENENABLE | SRCONFIG_ERRGEN_EN |
			SRCONFIG_MINMAXAVG_EN |
			(senn_en << SRCONFIG_SENNENABLE_SHIFT) |
			(senp_en << SRCONFIG_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;

		sr_write_reg(sr, SRCONFIG, sr_config);

		sr_write_reg(sr, AVGWEIGHT, SR2_AVGWEIGHT_SENPAVGWEIGHT |
					SR2_AVGWEIGHT_SENNAVGWEIGHT);

		sr_modify_reg(sr, ERRCONFIG, (SR_ERRWEIGHT_MASK |
			SR_ERRMAXLIMIT_MASK | SR_ERRMINLIMIT_MASK),
			(SR2_ERRWEIGHT | SR2_ERRMAXLIMIT | SR2_ERRMINLIMIT));

	}
	sr->is_sr_reset = 0;
}

static int sr_reset_voltage(int srid)
{
	int ret;
	u32 vsel = 0, target_opp_no;
	u32 reg_addr = 0;
	u32 vc_bypass_value;
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 t2_smps_steps = 0;
	u32 t2_smps_delay = 0;

	if (srid == SR1) {
		target_opp_no = get_opp_no(current_vdd1_opp);
		vsel = mpu_iva2_vdd1_volts[target_opp_no - 1];
		reg_addr = R_VDD1_SR_CONTROL;
		t2_smps_steps = abs(vsel - PRM_VP1_VOLTAGE);

	} else if (srid == SR2) {
		target_opp_no = get_opp_no(current_vdd2_opp);
		vsel = core_l3_vdd2_volts[target_opp_no - 1];
		reg_addr = R_VDD2_SR_CONTROL;
		t2_smps_steps = abs(vsel - PRM_VP2_VOLTAGE);
	}

	vc_bypass_value = (vsel << PRM_VC_BYPASS_DATA_SHIFT) |
			(reg_addr << PRM_VC_BYPASS_REGADDR_SHIFT) |
			(R_SRI2C_SLAVE_ADDR << PRM_VC_BYPASS_SLAVEADDR_SHIFT);

	PRM_VC_BYPASS_VAL = vc_bypass_value;

	PRM_VC_BYPASS_VAL |= PRM_VC_BYPASS_VALID;

	DPRINTK("%s : PRM_VC_BYPASS_VAL %X\n", __func__, PRM_VC_BYPASS_VAL);
	DPRINTK("PRM_IRQST_MPU %X\n", PRM_IRQSTATUS_MPU);

	while ((PRM_VC_BYPASS_VAL & PRM_VC_BYPASS_VALID) != 0x0) {
		ret = loop_wait(&loop_cnt, &retries_cnt, 10);
		if (ret != PRCM_PASS) {
			printk(KERN_INFO "Loop count exceeded in check SR I2C"
								"write\n");
			return ret;
		}
	}

	t2_smps_delay = 2 + ((t2_smps_steps * 125) / 40);
	udelay(t2_smps_delay);

	return SR_PASS;
}

static void sr_enable(struct omap_sr *sr, u32 target_opp_no)
{
	u32 nvalue_reciprocal, current_nvalue, vc;

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRDISABLE);

	sr->req_opp_no = target_opp_no;

	if (sr->srid == SR1) {
		switch (target_opp_no) {
		case 7:
			nvalue_reciprocal = sr->opp7_nvalue;
			break;
		case 6:
			nvalue_reciprocal = sr->opp6_nvalue;
			break;
		case 5:
			nvalue_reciprocal = sr->opp5_nvalue;
			break;
		case 4:
			nvalue_reciprocal = sr->opp4_nvalue;
			break;
		case 3:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		case 2:
			nvalue_reciprocal = sr->opp2_nvalue;
			break;
		case 1:
			nvalue_reciprocal = sr->opp1_nvalue;
			break;
		default:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		}
	} else {
		switch (target_opp_no) {
		case 3:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		case 2:
			nvalue_reciprocal = sr->opp2_nvalue;
			break;
		case 1:
			nvalue_reciprocal = sr->opp1_nvalue;
			break;
		default:
			nvalue_reciprocal = sr->opp3_nvalue;
			break;
		}
	}

	current_nvalue = sr_read_reg(sr, NVALUERECIPROCAL);

	if (nvalue_reciprocal == 0) {
		printk(KERN_ERR "SmartReflex nvalues not configured.\n");
		return;
	}

	sr_write_reg(sr, NVALUERECIPROCAL, nvalue_reciprocal);

	if (sr->srid == SR1) {
		/* Enable the interrupt */
		sr_modify_reg(sr, ERRCONFIG,
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST |
			ERRCONFIG_MCUBOUNDINTEN | ERRCONFIG_MCUBOUNDINTST),
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST |
			ERRCONFIG_MCUBOUNDINTEN | ERRCONFIG_MCUBOUNDINTST));

		/* set/latch init voltage */
		vc = (PRM_VP1_CONFIG & ~(PRM_VPX_CONFIG_INITVOLTAGE_MASK
				| PRM_VP1_CONFIG_INITVDD))
				| mpu_iva2_vdd1_volts[target_opp_no - 1] << 8;
		PRM_VP1_CONFIG = vc; /* set initvoltage & clear InitVdd */
		PRM_VP1_CONFIG |= PRM_VP1_CONFIG_INITVDD; /* write1 to latch */
		PRM_VP1_CONFIG &= ~PRM_VP1_CONFIG_INITVDD;/* write2 clear */
		PRM_VP1_CONFIG |= PRM_VP1_CONFIG_VPENABLE;/* write3 enable */
	} else if (sr->srid == SR2) {
		/* Enable the interrupt */
		sr_modify_reg(sr, ERRCONFIG,
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST),
			(ERRCONFIG_VPBOUNDINTEN | ERRCONFIG_VPBOUNDINTST));

		/* set/latch init voltage */
		vc = (PRM_VP2_CONFIG & ~(PRM_VPX_CONFIG_INITVOLTAGE_MASK
				| PRM_VP2_CONFIG_INITVDD))
				| core_l3_vdd2_volts[target_opp_no - 1] << 8;
		PRM_VP2_CONFIG = vc; /* set initvoltage & clear InitVdd */
		PRM_VP2_CONFIG |= PRM_VP2_CONFIG_INITVDD; /* write1 to latch */
		PRM_VP2_CONFIG &= ~PRM_VP2_CONFIG_INITVDD;/* write2 clear */
		PRM_VP2_CONFIG |= PRM_VP2_CONFIG_VPENABLE;/* write3 enale */
	}

	/* SRCONFIG - enable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRENABLE);

}

static void sr_disable(struct omap_sr *sr)
{

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRDISABLE);

	sr->is_sr_reset = 1;

	if (sr->srid == SR1) {
		/* Disable VP1 */
		PRM_VP1_CONFIG &= ~PRM_VP1_CONFIG_VPENABLE;
	} else if (sr->srid == SR2) {
		/* Disable VP2 */
		PRM_VP2_CONFIG &= ~PRM_VP2_CONFIG_VPENABLE;
	}
}


void sr_start_vddautocomap(int srid, u32 target_opp_no)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	}

	if (sr->is_sr_reset == 1) {
		sr_clk_enable(sr);
		sr_configure(sr);
	}

	if (sr->is_autocomp_active == 1)
		DPRINTK(KERN_WARNING "SR%d: VDD autocomp is already active\n",
									srid);

	sr->is_autocomp_active = 1;
	sr_enable(sr, target_opp_no);
}
EXPORT_SYMBOL(sr_start_vddautocomap);

int sr_stop_vddautocomap(int srid)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	}

	if (sr->is_autocomp_active == 1) {
		sr_disable(sr);
		sr_clk_disable(sr);
		sr->is_autocomp_active = 0;
		return SR_TRUE;
	} else {
		DPRINTK(KERN_WARNING "SR%d: VDD autocomp \
						is not active\n", srid);
		return SR_FALSE;
	}

}
EXPORT_SYMBOL(sr_stop_vddautocomap);

void enable_smartreflex(int srid)
{
	u32 target_opp_no = 0;
	struct omap_sr *sr = NULL;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	}

	if (sr->is_autocomp_active == 1) {
		if (sr->is_sr_reset == 1) {
			/* Enable SR clks */
			sr_clk_enable(sr);

			if (srid == SR1) {
				target_opp_no = get_opp_no(current_vdd1_opp);

			} else if (srid == SR2) {
				target_opp_no = get_opp_no(current_vdd2_opp);
			}

			sr_configure(sr);

			sr_enable(sr, target_opp_no);
		}
	}
}

void disable_smartreflex(int srid)
{
	struct omap_sr *sr = NULL;

	if (srid == SR1) {
		sr = &sr1;
	} else if (srid == SR2) {
		sr = &sr2;
	}

	if (sr->is_autocomp_active == 1) {

		if (sr->is_sr_reset == 0) {

			/* SRCONFIG - disable SR */
			sr_disable(sr);

			/* Disable SR clk */
			sr_clk_disable(sr);
			if (srid == SR1) {
				/* Reset the VDD1 voltage for current OPP */
				sr_reset_voltage(SR1);
			} else if (srid == SR2) {
				/* Reset the VDD2 voltage for current OPP */
				sr_reset_voltage(SR2);
			}
		}
	}
}


/* Voltage Scaling using SR VCBYPASS */
int sr_voltagescale_vcbypass(u32 target_opp, u8 vsel)
{
	int ret;
	int sr_status = 0;
	u32 vdd;
	u32 target_opp_no, cur_opp_no;
	u32 vc_bypass_value;
	u32 reg_addr = 0;
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 t2_smps_steps = 0;
	u32 t2_smps_delay = 0;

	vdd = get_vdd(target_opp);
	target_opp_no = get_opp_no(target_opp);

	if (vdd == PRCM_VDD1) {
		sr_status = sr_stop_vddautocomap(SR1);
		if (sr_status) {
			t2_smps_steps = abs(vsel - PRM_VP1_VOLTAGE);
		} else {
			cur_opp_no = get_opp_no(current_vdd1_opp);
			t2_smps_steps = abs(vsel -
					mpu_iva2_vdd1_volts[cur_opp_no - 1]);
		}

		PRM_VC_CMD_VAL_0 = (PRM_VC_CMD_VAL_0 & ~PRM_VC_CMD_ON_MASK) |
						(vsel << PRM_VC_CMD_ON_SHIFT);
		reg_addr = R_VDD1_SR_CONTROL;

	} else if (vdd == PRCM_VDD2) {
		sr_status = sr_stop_vddautocomap(SR2);
		if (sr_status) {
			t2_smps_steps = abs(vsel - PRM_VP1_VOLTAGE);
		} else {
			cur_opp_no = get_opp_no(current_vdd2_opp);
			t2_smps_steps = abs(vsel -
					mpu_iva2_vdd1_volts[cur_opp_no - 1]);
		}

		PRM_VC_CMD_VAL_1 = (PRM_VC_CMD_VAL_1 & ~PRM_VC_CMD_ON_MASK) |
						(vsel << PRM_VC_CMD_ON_SHIFT);
		reg_addr = R_VDD2_SR_CONTROL;
	}

	vc_bypass_value = (vsel << PRM_VC_BYPASS_DATA_SHIFT) |
			(reg_addr << PRM_VC_BYPASS_REGADDR_SHIFT) |
			(R_SRI2C_SLAVE_ADDR << PRM_VC_BYPASS_SLAVEADDR_SHIFT);

	PRM_VC_BYPASS_VAL = vc_bypass_value;

	PRM_VC_BYPASS_VAL |= PRM_VC_BYPASS_VALID;

	DPRINTK("%s : PRM_VC_BYPASS_VAL %X\n", __FUNCTION__, PRM_VC_BYPASS_VAL);
	DPRINTK("PRM_IRQST_MPU %X\n", PRM_IRQSTATUS_MPU);

	while ((PRM_VC_BYPASS_VAL & PRM_VC_BYPASS_VALID) != 0x0) {
		ret = loop_wait(&loop_cnt, &retries_cnt, 10);
		if (ret != PRCM_PASS) {
			printk(KERN_INFO "Loop count exceeded in check SR I2C"
								"write\n");
			return ret;
		}
	}

	/* T2 SMPS slew rate (min) 4mV/uS */
	t2_smps_delay = 2 + ((t2_smps_steps * 125) / 40);
	udelay(t2_smps_delay);

	if (sr_status) {
		if (vdd == PRCM_VDD1)
			sr_start_vddautocomap(SR1, target_opp_no);
		else if (vdd == PRCM_VDD2)
			sr_start_vddautocomap(SR2, target_opp_no);
	}

	return SR_PASS;
}


/* Sysfs interface to select SR VDD1 & VDD2 auto compensation */
static ssize_t omap_sr_autocomp_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf);
static ssize_t omap_sr_autocomp_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t n);

static struct kobj_attribute sr1_autocomp_attr = __ATTR(sr_vdd1_autocomp, 0644,
				omap_sr_autocomp_show, omap_sr_autocomp_store);
static struct kobj_attribute sr2_autocomp_attr = __ATTR(sr_vdd2_autocomp, 0644,
				omap_sr_autocomp_show, omap_sr_autocomp_store);

static ssize_t omap_sr_autocomp_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int ret;

	if (attr == &sr1_autocomp_attr)
		ret = sprintf(buf, "%d\n", sr1.is_autocomp_active);
	else if (attr == &sr2_autocomp_attr)
		ret = sprintf(buf, "%d\n", sr2.is_autocomp_active);
	else
		return -EINVAL;

	return ret;
}

static ssize_t omap_sr_autocomp_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t n)
{
	u32 current_opp_no;
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		printk(KERN_ERR "sr_autocomp: Invalid value\n");
		return -EINVAL;
	}

	if (sr_nvalues == 0) {
		printk(KERN_ERR "\nnvalues not configured, use test nvalues\n");
		return -EINVAL;
	}

	if (attr == &sr1_autocomp_attr) {
		current_opp_no = get_opp_no(current_vdd1_opp);

		if (value == 0) {
			sr_stop_vddautocomap(SR1);
			/* Reset the VDD1 voltage for current OPP */
			sr_reset_voltage(SR1);
			PRM_VC_CMD_VAL_0 =
				(PRM_VC_CMD_VAL_0 & ~PRM_VC_CMD_RET_MASK) |
				(PRM_VC_CMD_VAL0_RET << PRM_VC_CMD_RET_SHIFT);
		} else
			sr_start_vddautocomap(SR1, current_opp_no);

	} else if (attr == &sr2_autocomp_attr) {
		current_opp_no = get_opp_no(current_vdd2_opp);

		if (value == 0) {
			sr_stop_vddautocomap(SR2);
			/* Reset the VDD2 voltage for current OPP */
			sr_reset_voltage(SR2);
		} else
			sr_start_vddautocomap(SR2, current_opp_no);

	} else
		return -EINVAL;

	return n;
}

/* Sysfs interface to set TEST NVALUES */
static ssize_t  omap_sr_setnvalues_test_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", (sr_nvalues - 1));
}

static ssize_t  omap_sr_setnvalues_test_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t n)
{
	unsigned short value;

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		printk(KERN_ERR "sr_setnvalues: Invalid value\n");
		return -EINVAL;
	}

	if ((sr1.is_autocomp_active == 1) || (sr2.is_autocomp_active == 1)) {
		printk(KERN_ERR "\nDisable SR autocomp before "
						"changing nvalues\n");
		return -EINVAL;
	}

	/* Disable SmartReflex before changing the nvalues */
	if (value == 0)
		sr_nvalues = sr_set_efuse_nvalues();
	else
		sr_nvalues = sr_set_test_nvalues();

	return n;
}

static struct kobj_attribute sr_setnvalues_test = {
	.attr = {
	.name = __stringify(sr_setnvalues_test),
	.mode = 0644,
	},
	.show = omap_sr_setnvalues_test_show,
	.store = omap_sr_setnvalues_test_store,
};

static irqreturn_t sr1_omap_irq(int irq, void *dev_id)
{
	static u32 vp_voltge_value = 0x0;

	if (vp_voltge_value != PRM_VP1_VOLTAGE) {

		if (get_opp_no(current_vdd1_opp) == 1)
			PRM_VC_CMD_VAL_0 =
				(PRM_VC_CMD_VAL_0 & ~PRM_VC_CMD_RET_MASK) |
				(PRM_VP1_VOLTAGE << PRM_VC_CMD_RET_SHIFT);

		vp_voltge_value = PRM_VP1_VOLTAGE;
	}
	sr_modify_reg(&sr1, ERRCONFIG, ERRCONFIG_MCUBOUNDINTST,
					ERRCONFIG_MCUBOUNDINTST);

	return IRQ_HANDLED;
}

static int __init omap3_sr_init(void)
{
	int ret = 0;
#ifdef CONFIG_TWL4030_CORE
	u8 RdReg;
#else
#endif /* #ifdef  CONFIG_TWL4030_CORE */

#ifdef CONFIG_ARCH_OMAP34XX
	sr1.fck = clk_get(NULL, "sr1_fck");
	if (IS_ERR(sr1.fck))
		printk(KERN_ERR "Could not get sr1_fck\n");

	sr2.fck = clk_get(NULL, "sr2_fck");
	if (IS_ERR(sr2.fck))
		printk(KERN_ERR "Could not get sr2_fck\n");
#endif /* #ifdef CONFIG_ARCH_OMAP34XX */

	/* set nvalues */
	sr_nvalues = sr_set_efuse_nvalues();
	if (sr_nvalues == 0x0)
		DPRINTK(KERN_INFO "Use TEST nvalues to enable SmartReflex\n");

	/* Call the VPConfig, VCConfig */
	sr_configure_vp(SR1);
	sr_configure_vp(SR2);

	sr_configure_vc();

#ifdef	CONFIG_TWL4030_CORE
	/* Enable SR on T2 */
	ret = t2_in(PM_RECEIVER, &RdReg, R_DCDC_GLOBAL_CFG);
	RdReg |= DCDC_GLOBAL_CFG_ENABLE_SRFLX;
	ret |= t2_out(PM_RECEIVER, RdReg, R_DCDC_GLOBAL_CFG);
	if (ret)
		printk(KERN_ERR "Error: Triton-SR mode not set : %d\n", ret);
#else
/* Add appropriate power IC function call to Enable SR on T2 */
#endif /* CONFIG_TWL4030_CORE */
	DPRINTK(KERN_INFO "SmartReflex driver initialized\n");

	/* Request IRQ for SR1 */
	ret = request_irq(SR1_IRQ, sr1_omap_irq, IRQF_DISABLED,
					"SmartReflex_1", &sr1);

	ret = sysfs_create_file(power_kobj, &sr1_autocomp_attr.attr);
	if (ret)
		printk(KERN_ERR "sysfs_create_file failed: %d\n", ret);

	ret = sysfs_create_file(power_kobj, &sr2_autocomp_attr.attr);
	if (ret)
		printk(KERN_ERR "sysfs_create_file failed: %d\n", ret);

	ret = sysfs_create_file(power_kobj, &sr_setnvalues_test.attr);
	if (ret)
		printk(KERN_ERR "sysfs_create_file failed: %d\n", ret);

	return 0;
}

late_initcall(omap3_sr_init);
