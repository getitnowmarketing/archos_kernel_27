/*
 * venc.c
 *
 *  Created on: Jan 28, 2009
 *      Author: Matthias Welwarsky <welwarsky@archos.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/types.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>

#include <mach/venc.h>

#include "dispc.h"

//#ifndef CONFIG_ARCH_OMAP3410

#define VENC_REG_BASE		0x48050C00

/* VENC register offsets */
#define VENC_F_CONTROL				0x0008
#define VENC_VIDOUT_CTRL			0x0010
#define VENC_SYNC_CONTROL			0x0014
#define VENC_LLEN				0x001C
#define VENC_FLENS				0x0020
#define VENC_HFLTR_CTRL			0x0024
#define VENC_CC_CARR_WSS_CARR			0x0028
#define VENC_C_PHASE				0x002C
#define VENC_GAIN_U				0x0030
#define VENC_GAIN_V				0x0034
#define VENC_GAIN_Y				0x0038
#define VENC_BLACK_LEVEL			0x003C
#define VENC_BLANK_LEVEL			0x0040
#define VENC_X_COLOR				0x0044
#define VENC_M_CONTROL				0x0048
#define VENC_BSTAMP_WSS_DATA			0x004C
#define VENC_S_CARR				0x0050
#define VENC_LINE21				0x0054
#define VENC_LN_SEL				0x0058
#define VENC_L21_WC_CTL			0x005C
#define VENC_HTRIGGER_VTRIGGER			0x0060
#define VENC_SAVID_EAVID			0x0064
#define VENC_FLEN_FAL				0x0068
#define VENC_LAL_PHASE_RESET			0x006C
#define VENC_HS_INT_START_STOP_X		0x0070
#define VENC_HS_EXT_START_STOP_X		0x0074
#define VENC_VS_INT_START_X			0x0078
#define VENC_VS_INT_STOP_X_VS_INT_START_Y	0x007C
#define VENC_VS_INT_STOP_Y_VS_EXT_START_X	0x0080
#define VENC_VS_EXT_STOP_X_VS_EXT_START_Y	0x0084
#define VENC_VS_EXT_STOP_Y			0x0088
#define VENC_AVID_START_STOP_X			0x0090
#define VENC_AVID_START_STOP_Y			0x0094
#define VENC_FID_INT_START_X_FID_INT_START_Y	0x00A0
#define VENC_FID_INT_OFFSET_Y_FID_EXT_START_X	0x00A4
#define VENC_FID_EXT_START_Y_FID_EXT_OFFSET_Y	0x00A8
#define VENC_TVDETGP_INT_START_STOP_X		0x00B0
#define VENC_TVDETGP_INT_START_STOP_Y		0x00B4
#define VENC_GEN_CTRL				0x00B8
#define VENC_DAC_TST				0x00C4
#define VENC_DAC				0x00C8

/* VENC bit fields */
#define VENC_FCONTROL_RESET			(1<<8)

/*
 * VENC register I/O Routines
 */
static inline u32 venc_reg_in(u32 offset)
{
	return omap_readl(VENC_REG_BASE + offset);
}

static inline u32 venc_reg_out(u32 offset, u32 val)
{
	omap_writel(val, VENC_REG_BASE + offset);
	return val;
}

static inline u32 venc_reg_merge(u32 offset, u32 val, u32 mask)
{
	u32 addr = VENC_REG_BASE + offset;
	u32 new_val = (omap_readl(addr) & ~mask) | (val & mask);

	omap_writel(new_val, addr);
	return new_val;
}

/*
 *  Sets VENC registers for TV operation.
 */
static void config_venc(struct tv_standard_config *tvstd)
{
	venc_reg_out(VENC_F_CONTROL, F_CONTROL_GEN);
	venc_reg_out(VENC_SYNC_CONTROL,SYNC_CONTROL_GEN);
	venc_reg_out(VENC_LLEN, tvstd->venc_llen);
	venc_reg_out(VENC_FLENS, tvstd->venc_flens);
	venc_reg_out(VENC_HFLTR_CTRL, tvstd->venc_hfltr_ctrl);
	venc_reg_out(VENC_CC_CARR_WSS_CARR, tvstd->venc_cc_carr_wss_carr);
	venc_reg_out(VENC_C_PHASE, tvstd->venc_c_phase);
	venc_reg_out(VENC_GAIN_U, tvstd->venc_gain_u);
	venc_reg_out(VENC_GAIN_V, tvstd->venc_gain_v);
	venc_reg_out(VENC_GAIN_Y, tvstd->venc_gain_y);
	venc_reg_out(VENC_BLACK_LEVEL, tvstd->venc_black_level);
	venc_reg_out(VENC_BLANK_LEVEL, tvstd->venc_blank_level);
	venc_reg_out(VENC_X_COLOR, tvstd->venc_x_color);
	venc_reg_out(VENC_M_CONTROL, tvstd->venc_m_control);
	venc_reg_out(VENC_BSTAMP_WSS_DATA, tvstd->venc_bstamp_wss_data);
	venc_reg_out(VENC_S_CARR, tvstd->venc_s_carr);
	venc_reg_out(VENC_LINE21, tvstd->venc_line21);
	venc_reg_out(VENC_LN_SEL, tvstd->venc_ln_sel);
	venc_reg_out(VENC_L21_WC_CTL, tvstd->venc_l21_wc_ctl);
	venc_reg_out(VENC_HTRIGGER_VTRIGGER, tvstd->venc_htrigger_vtrigger);
	venc_reg_out(VENC_SAVID_EAVID, tvstd->venc_savid_eavid);
	venc_reg_out(VENC_FLEN_FAL, tvstd->venc_flen_fal);
	venc_reg_out(VENC_LAL_PHASE_RESET, tvstd->venc_lal_phase_reset);
	venc_reg_out(VENC_HS_INT_START_STOP_X,
			tvstd->venc_hs_int_start_stop_x);
	venc_reg_out(VENC_HS_EXT_START_STOP_X,
			tvstd->venc_hs_ext_start_stop_x);
	venc_reg_out(VENC_VS_INT_START_X, tvstd->venc_vs_int_start_x);
	venc_reg_out(VENC_VS_INT_STOP_X_VS_INT_START_Y,
			tvstd-> venc_vs_int_stop_x_vs_int_start_y);
	venc_reg_out(VENC_VS_INT_STOP_Y_VS_EXT_START_X,
			tvstd->venc_vs_int_stop_y_vs_ext_start_x);
	venc_reg_out(VENC_VS_EXT_STOP_X_VS_EXT_START_Y,
			tvstd->venc_vs_ext_stop_x_vs_ext_start_y);
	venc_reg_out(VENC_VS_EXT_STOP_Y, tvstd->venc_vs_ext_stop_y);
	venc_reg_out(VENC_AVID_START_STOP_X, tvstd->venc_avid_start_stop_x);
	venc_reg_out(VENC_AVID_START_STOP_Y, tvstd->venc_avid_start_stop_y);
	venc_reg_out(VENC_FID_INT_START_X_FID_INT_START_Y,
			tvstd-> venc_fid_int_start_x_fid_int_start_y);
	venc_reg_out(VENC_FID_INT_OFFSET_Y_FID_EXT_START_X,
			tvstd->venc_fid_int_offset_y_fid_ext_start_x);
	venc_reg_out(VENC_FID_EXT_START_Y_FID_EXT_OFFSET_Y,
			tvstd->venc_fid_ext_start_y_fid_ext_offset_y);
	venc_reg_out(VENC_TVDETGP_INT_START_STOP_X,
			tvstd->venc_tvdetgp_int_start_stop_x);
	venc_reg_out(VENC_TVDETGP_INT_START_STOP_Y,
			tvstd->venc_tvdetgp_int_start_stop_y);
	venc_reg_out(VENC_GEN_CTRL, tvstd->venc_gen_ctrl);
	venc_reg_out(VENC_DAC_TST, tvstd->venc_dac_tst);
	venc_reg_out(VENC_DAC, venc_reg_in(VENC_DAC));
}

static struct tv_standard_config *tv_standards[] = {
	[TVSTD_PAL_BDGHI] = &pal_bdghi_cfg,
	[TVSTD_PAL_N] = &pal_n_cfg,
	[TVSTD_PAL_NC] = &pal_nc_cfg,
	[TVSTD_PAL_M] = &pal_m_cfg,
	[TVSTD_PAL_60] = &pal_60_cfg,
	[TVSTD_NTSC_M] = &ntsc_m_cfg,
	[TVSTD_NTSC_443] = &ntsc_443_cfg,
	[TVSTD_NTSC_J] = &ntsc_j_cfg,
};

struct clk *dss_54m_fck;

int venc_enable(void)
{
	if (dss_54m_fck == NULL)
		return -EIO;
	return clk_enable(dss_54m_fck);
}

int venc_disable(void)
{
	if (dss_54m_fck == NULL)
		return -EIO;
	
	clk_disable(dss_54m_fck);
	return 0;
}

int venc_set_tvstandard(enum tv_standards tvstd)
{
	if ( (tvstd < TVSTD_PAL_BDGHI) || (tvstd > TVSTD_NTSC_J))
		return -EINVAL;
	
	config_venc(tv_standards[tvstd]);
	return 0;
}

static int venc_init(void)
{
	struct clk *c;
	
	c = clk_get(NULL, "dss_54m_fck");
	if (IS_ERR(c)) {
		pr_err("venc_init: cannot get dss_54m_fck\n");
		return PTR_ERR(c);
	}
	
	dss_54m_fck = c;
	return 0;
}

EXPORT_SYMBOL(venc_set_tvstandard);
EXPORT_SYMBOL(venc_enable);
EXPORT_SYMBOL(venc_disable);

arch_initcall(venc_init);
