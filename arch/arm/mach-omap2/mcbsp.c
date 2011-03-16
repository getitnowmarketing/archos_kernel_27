/*
 * linux/arch/arm/mach-omap2/mcbsp.c
 *
 * Copyright (C) 2008 Instituto Nokia de Tecnologia
 * Contact: Eduardo Valentin <eduardo.valentin@indt.org.br>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Multichannel mode not supported.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>

#include <mach/dma.h>
#include <mach/mux.h>
#include <mach/cpu.h>
#include <mach/mcbsp.h>

struct omap_mcbsp_reg_cfg mcbsp_cfg = {0};
struct mcbsp_internal_clk {
	struct clk clk;
	struct clk **childs;
	int n_childs;
};

#ifdef CONFIG_TRACK_RESOURCES
struct device_driver mcbsp_clk_layer_drv = {
	.name =  "mcbsp_clock_layer",
};
#endif

#if defined(CONFIG_ARCH_OMAP24XX) || defined(CONFIG_ARCH_OMAP34XX)
static void omap_mcbsp_clk_init(struct mcbsp_internal_clk *mclk)
{
	const char *clk_names[] = { "mcbsp_ick", "mcbsp_fck" };
	int i;
#ifdef CONFIG_TRACK_RESOURCES
	static struct platform_device pdev;

	pdev.dev.bus = &platform_bus_type;
	pdev.dev.driver = &mcbsp_clk_layer_drv;
#endif

	mclk->n_childs = ARRAY_SIZE(clk_names);
	mclk->childs = kzalloc(mclk->n_childs * sizeof(struct clk *),
				GFP_KERNEL);

	for (i = 0; i < mclk->n_childs; i++) {
#ifndef CONFIG_TRACK_RESOURCES
		/* We fake a platform device to get correct device id */
		struct platform_device pdev;

		pdev.dev.bus = &platform_bus_type;
#endif
		pdev.id = mclk->clk.id;
		mclk->childs[i] = clk_get(&pdev.dev, clk_names[i]);
		if (IS_ERR(mclk->childs[i]))
			printk(KERN_ERR "Could not get clock %s (%d).\n",
				clk_names[i], mclk->clk.id);
	}
}

#ifdef CONFIG_TRACK_RESOURCES
#define R(clk) (res->clk)
#else
#define R(clk) (clk)
#endif

static int omap_mcbsp_clk_enable(struct clk *clk)
{
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res = (struct resource_handle *) clk;
#endif
	struct mcbsp_internal_clk *mclk = container_of(R(clk),
					struct mcbsp_internal_clk, clk);
	int i;

	for (i = 0; i < mclk->n_childs; i++)
		clk_enable(mclk->childs[i]);
	return 0;
}

static void omap_mcbsp_clk_disable(struct clk *clk)
{
#ifdef CONFIG_TRACK_RESOURCES
	struct resource_handle *res = (struct resource_handle *) clk;
#endif
	struct mcbsp_internal_clk *mclk = container_of(R(clk),
					struct mcbsp_internal_clk, clk);
	int i;

	for (i = 0; i < mclk->n_childs; i++)
		clk_disable(mclk->childs[i]);
}

static struct mcbsp_internal_clk omap_mcbsp_clks[] = {
	{
		.clk = {
			.name 		= "mcbsp_clk",
			.id		= 1,
			.enable		= omap_mcbsp_clk_enable,
			.disable	= omap_mcbsp_clk_disable,
		},
	},
	{
		.clk = {
			.name 		= "mcbsp_clk",
			.id		= 2,
			.enable		= omap_mcbsp_clk_enable,
			.disable	= omap_mcbsp_clk_disable,
		},
	},
	{
		.clk = {
			.name 		= "mcbsp_clk",
			.id		= 3,
			.enable		= omap_mcbsp_clk_enable,
			.disable	= omap_mcbsp_clk_disable,
		},
	},
	{
		.clk = {
			.name 		= "mcbsp_clk",
			.id		= 4,
			.enable		= omap_mcbsp_clk_enable,
			.disable	= omap_mcbsp_clk_disable,
		},
	},
	{
		.clk = {
			.name 		= "mcbsp_clk",
			.id		= 5,
			.enable		= omap_mcbsp_clk_enable,
			.disable	= omap_mcbsp_clk_disable,
		},
	},
};

#define omap_mcbsp_clks_size	ARRAY_SIZE(omap_mcbsp_clks)
#else
#define omap_mcbsp_clks_size	0
static struct mcbsp_internal_clk __initdata *omap_mcbsp_clks;
static inline void omap_mcbsp_clk_init(struct clk *clk)
{ }
#endif

static void omap2_mcbsp2_mux_setup(void)
{
	omap_cfg_reg(Y15_24XX_MCBSP2_CLKX);
	omap_cfg_reg(R14_24XX_MCBSP2_FSX);
	omap_cfg_reg(W15_24XX_MCBSP2_DR);
	omap_cfg_reg(V15_24XX_MCBSP2_DX);
	omap_cfg_reg(V14_24XX_GPIO117);
	/*
	 * TODO: Need to add MUX settings for OMAP 2430 SDP
	 */
}

static void omap2_mcbsp_request(unsigned int id)
{
	struct omap_mcbsp *mcbsp = mcbsp_ptr[id];

	omap_mcbsp_clk_enable(mcbsp->clk);

	if (cpu_is_omap2420() && (id == OMAP_MCBSP2))
		omap2_mcbsp2_mux_setup();
}

/*
 * mcbsp power settings
 * id		: McBSP interface number
 * level	: power settings level
 */
static void  mcbsp_power_settings(unsigned int id, int level)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;

	if (level == MCBSP2_SYSCONFIG_LVL1)
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SYSCON,
		CLOCKACTIVITY(MCBSP_SYSC_IOFF_FON) | SIDLEMODE(SMART_IDLE) |
								ENAWAKEUP);

	if (level == MCBSP2_SYSCONFIG_LVL2)
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SYSCON,
		CLOCKACTIVITY(MCBSP_SYSC_IOFF_FOFF) | SIDLEMODE(FORCE_IDLE));
}

static void omap2_mcbsp_free(unsigned int id)
{
	struct omap_mcbsp *mcbsp;
	mcbsp = id_to_mcbsp_ptr(id);

	if (!cpu_is_omap2420()) {
		if (mcbsp->dma_rx_lch != -1) {
			omap_free_dma_chain(mcbsp->dma_rx_lch);
			 mcbsp->dma_rx_lch = -1;
		}

		if (mcbsp->dma_tx_lch != -1) {
			omap_free_dma_chain(mcbsp->dma_tx_lch);
			mcbsp->dma_tx_lch = -1;
		}
	mcbsp_power_settings(id, MCBSP2_SYSCONFIG_LVL2);
	}

	omap_mcbsp_clk_disable(mcbsp->clk);

	return;
}
void omap2_mcbsp_config(unsigned int id,
			 const struct omap_mcbsp_reg_cfg *config)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_XCCR, config->xccr);
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_RCCR, config->rccr);
}
static struct omap_mcbsp_ops omap2_mcbsp_ops = {
	.request	= omap2_mcbsp_request,
	.free 		= omap2_mcbsp_free,
	.config		= omap2_mcbsp_config,
};
static void omap2_mcbsp_rx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct omap_mcbsp *mcbsp_dma_rx = data;
	void __iomem *io_base;

	io_base = mcbsp_dma_rx->io_base;

	/* If we are at the last transfer, Shut down the reciever */
	if ((mcbsp_dma_rx->auto_reset & OMAP_MCBSP_AUTO_RRST)
		&& (omap_dma_chain_status(mcbsp_dma_rx->dma_rx_lch) ==
						 OMAP_DMA_CHAIN_INACTIVE))
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1,
			omap_mcbsp_read(io_base,
					 OMAP_MCBSP_REG_SPCR1) & (~RRST));

	if (mcbsp_dma_rx->rx_callback != NULL)
		mcbsp_dma_rx->rx_callback(ch_status, mcbsp_dma_rx->rx_cb_arg);

}

static void omap2_mcbsp_tx_dma_callback(int lch, u16 ch_status, void *data)
{
	struct omap_mcbsp *mcbsp_dma_tx = data;
	void __iomem *io_base;

	io_base = mcbsp_dma_tx->io_base;

	/* If we are at the last transfer, Shut down the Transmitter */
	if ((mcbsp_dma_tx->auto_reset & OMAP_MCBSP_AUTO_XRST)
		&& (omap_dma_chain_status(mcbsp_dma_tx->dma_tx_lch) ==
						 OMAP_DMA_CHAIN_INACTIVE))
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
			omap_mcbsp_read(io_base,
					 OMAP_MCBSP_REG_SPCR2) & (~XRST));

	if (mcbsp_dma_tx->tx_callback != NULL)
		mcbsp_dma_tx->tx_callback(ch_status, mcbsp_dma_tx->tx_cb_arg);
}

/*
 * Set McBSP recv parameters
 * id           : McBSP interface ID
 * mcbsp_cfg    : McBSP register configuration
 * rp           : McBSP recv parameters
 */
void omap2_mcbsp_set_recv_param(unsigned int id,
				struct omap_mcbsp_reg_cfg *mcbsp_cfg,
				struct omap_mcbsp_cfg_param *rp)
{
	mcbsp_cfg->spcr1 = RJUST(rp->justification);
	mcbsp_cfg->rcr2 = RCOMPAND(rp->reverse_compand) |
				RDATDLY(rp->data_delay);
	if (rp->phase == OMAP_MCBSP_FRAME_SINGLEPHASE)
		mcbsp_cfg->rcr2 = mcbsp_cfg->rcr2 & ~(RPHASE);
	else
		mcbsp_cfg->rcr2 = mcbsp_cfg->rcr2  | (RPHASE) |
			RWDLEN2(rp->word_length2) | RFRLEN2(rp->frame_length2);
	mcbsp_cfg->rcr1 = RWDLEN1(rp->word_length1) |
				RFRLEN1(rp->frame_length1);
	if (rp->fsync_src == OMAP_MCBSP_RXFSYNC_INTERNAL)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | FSRM;
	if (rp->clk_mode == OMAP_MCBSP_CLKRXSRC_INTERNAL)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | CLKRM;
	if (rp->clk_polarity == OMAP_MCBSP_CLKR_POLARITY_RISING)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | CLKRP;
	if (rp->fs_polarity == OMAP_MCBSP_FS_ACTIVE_LOW)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | FSRP;
	return;
}

/*
 * Set McBSP transmit parameters
 * id		: McBSP interface ID
 * mcbsp_cfg	: McBSP register configuration
 * tp		: McBSP transmit parameters
 */

void omap2_mcbsp_set_trans_param(unsigned int id,
				struct omap_mcbsp_reg_cfg *mcbsp_cfg,
				struct omap_mcbsp_cfg_param *tp)
{
	mcbsp_cfg->xcr2 = XCOMPAND(tp->reverse_compand) |
					XDATDLY(tp->data_delay);
	if (tp->phase == OMAP_MCBSP_FRAME_SINGLEPHASE)
		mcbsp_cfg->xcr2 = mcbsp_cfg->xcr2 & ~(XPHASE);
	else
		mcbsp_cfg->xcr2 = mcbsp_cfg->xcr2 | (XPHASE) |
			RWDLEN2(tp->word_length2) | RFRLEN2(tp->frame_length2);
	mcbsp_cfg->xcr1 = XWDLEN1(tp->word_length1) |
				XFRLEN1(tp->frame_length1);
	if (tp->fs_polarity == OMAP_MCBSP_FS_ACTIVE_LOW)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | FSXP;
	if (tp->fsync_src == OMAP_MCBSP_TXFSYNC_INTERNAL)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | FSXM;
	if (tp->clk_mode == OMAP_MCBSP_CLKTXSRC_INTERNAL)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | CLKXM;
	if (tp->clk_polarity == OMAP_MCBSP_CLKX_POLARITY_FALLING)
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 | CLKXP;
	return;
}

 /*
  * Set McBSP SRG configuration
  * id			: McBSP interface ID
  * mcbsp_cfg		: McBSP register configuration
  * interface_mode	: Master/Slave
  * param		: McBSP SRG and FSG configuration
  */

void omap2_mcbsp_set_srg_cfg_param(unsigned int id, int interface_mode,
					struct omap_mcbsp_reg_cfg *mcbsp_cfg,
					struct omap_mcbsp_srg_fsg_cfg *param)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;
	u32 clk_rate, clkgdv;

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;

	mcbsp->interface_mode = interface_mode;
	mcbsp_cfg->srgr1 = FWID(param->pulse_width);

	if (interface_mode == OMAP_MCBSP_MASTER) {
		/* clk_rate = clk_get_rate(omap_mcbsp_clk[id].fck); */
		clk_rate = 96000000;
		clkgdv = clk_rate / (param->sample_rate *
				(param->bits_per_sample - 1));
		mcbsp_cfg->srgr1 = mcbsp_cfg->srgr1 | CLKGDV(clkgdv);
	}
	if (param->dlb)
		mcbsp_cfg->spcr1 = mcbsp_cfg->spcr1 & ~(ALB);

	if (param->sync_mode == OMAP_MCBSP_SRG_FREERUNNING)
		mcbsp_cfg->spcr2 = mcbsp_cfg->spcr2 | FREE;
	mcbsp_cfg->srgr2 = FPER(param->period)|(param->fsgm? FSGM : 0);

	switch (param->srg_src) {

	case OMAP_MCBSP_SRGCLKSRC_CLKS:
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 & ~(SCLKME);
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 & ~(CLKSM);
		/*
		 * McBSP master operation at low voltage is only possible if
		 * CLKSP=0 In Master mode, if client driver tries to configiure
		 * input clock polarity as falling edge, we force it to Rising
		 */

		if ((param->polarity == OMAP_MCBSP_CLKS_POLARITY_RISING) ||
					(interface_mode == OMAP_MCBSP_MASTER))
			mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2  & ~(CLKSP);
		else
			mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2  |  (CLKSP);
		break;


	case OMAP_MCBSP_SRGCLKSRC_FCLK:
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0 & ~(SCLKME);
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 | (CLKSM);

		break;

	case OMAP_MCBSP_SRGCLKSRC_CLKR:
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0   | (SCLKME);
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 & ~(CLKSM);
		if (param->polarity == OMAP_MCBSP_CLKR_POLARITY_FALLING)
			mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0  & ~(CLKRP);
		else
			mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0  | (CLKRP);

		break;

	case OMAP_MCBSP_SRGCLKSRC_CLKX:
		mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0   | (SCLKME);
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 | (CLKSM);

		if (param->polarity == OMAP_MCBSP_CLKX_POLARITY_RISING)
			mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0  & ~(CLKXP);
		else
			mcbsp_cfg->pcr0 = mcbsp_cfg->pcr0  | (CLKXP);
		break;

	}
	if (param->sync_mode == OMAP_MCBSP_SRG_FREERUNNING)
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 & ~(GSYNC);
	else if (param->sync_mode == OMAP_MCBSP_SRG_RUNNING)
		mcbsp_cfg->srgr2 = mcbsp_cfg->srgr2 | (GSYNC);

	mcbsp_cfg->xccr = omap_mcbsp_read(io_base, OMAP_MCBSP_REG_XCCR);
	if (param->dlb)
		mcbsp_cfg->xccr = mcbsp_cfg->xccr | (DILB);
	mcbsp_cfg->rccr = omap_mcbsp_read(io_base, OMAP_MCBSP_REG_RCCR);

	return;
}


/*
 * configure the McBSP registers
 * id			: McBSP interface ID
 * interface_mode	: Master/Slave
 * rp			: McBSP recv parameters
 * tp			: McBSP transmit parameters
 * param		: McBSP SRG and FSG configuration
 */
int omap2_mcbsp_params_cfg(unsigned int id, int interface_mode,
				struct omap_mcbsp_cfg_param *rp,
				struct omap_mcbsp_cfg_param *tp,
				struct omap_mcbsp_srg_fsg_cfg *param)
 {
	if (rp)
		omap2_mcbsp_set_recv_param(id, &mcbsp_cfg, rp);
	if (tp)
		omap2_mcbsp_set_trans_param(id, &mcbsp_cfg, tp);
	if (param)
		omap2_mcbsp_set_srg_cfg_param(id,
					interface_mode, &mcbsp_cfg, param);
	omap_mcbsp_config(id, &mcbsp_cfg);

	return 0;
 }
EXPORT_SYMBOL(omap2_mcbsp_params_cfg);

/*
 * Enable/Disable the sample rate generator
 * id		: McBSP interface ID
 * state	: Enable/Disable
 */
void omap2_mcbsp_set_srg_fsg(unsigned int id, u8 state)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;

	if (state == OMAP_MCBSP_DISABLE_FSG_SRG) {
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
			omap_mcbsp_read(io_base,
					 OMAP_MCBSP_REG_SPCR2) & (~GRST));
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
			omap_mcbsp_read(io_base,
					 OMAP_MCBSP_REG_SPCR2) & (~FRST));
	} else {
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
			omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR2) | GRST);
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
			omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR2) | FRST);
	}
	return;
}

/*
 * Stop transmitting data on a McBSP interface
 * id		: McBSP interface ID
 */
int omap2_mcbsp_stop_datatx(unsigned int id)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;

	if (mcbsp->dma_tx_lch != -1) {
		if (omap_stop_dma_chain_transfers(mcbsp->dma_tx_lch) != 0)
			return -EINVAL;
	}
	mcbsp->tx_dma_chain_state = 0;
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
		omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR2) & (~XRST));

	if (!mcbsp->rx_dma_chain_state)
		omap2_mcbsp_set_srg_fsg(id, OMAP_MCBSP_DISABLE_FSG_SRG);

	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_stop_datatx);

/*
 * Stop receving data on a McBSP interface
 * id		: McBSP interface ID
 */
int omap2_mcbsp_stop_datarx(u32 id)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;

	if (mcbsp->dma_rx_lch != -1) {
		if (omap_stop_dma_chain_transfers(mcbsp->dma_rx_lch) != 0)
			return -EINVAL;
	}
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1,
		omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR1) & (~RRST));

	mcbsp->rx_dma_chain_state = 0;
	if (!mcbsp->tx_dma_chain_state)
		omap2_mcbsp_set_srg_fsg(id, OMAP_MCBSP_DISABLE_FSG_SRG);

	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_stop_datarx);

/*
 * Interface Reset
 * id	: McBSP interface ID
 * Resets the McBSP interface
 */
int omap2_mcbsp_reset(unsigned int id)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;
	int counter = 0;
	int wait_for_reset = 10000;

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;

	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SYSCON,
		omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SYSCON) | (SOFTRST));

	while (omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SYSCON) & SOFTRST) {
		if (!in_interrupt()) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(10);
		}
		if (counter++ > wait_for_reset) {
			printk(KERN_ERR "mcbsp[%d] Reset timeout\n", id);
			return -ETIMEDOUT;
		}
	}
	mcbsp_power_settings(id, MCBSP2_SYSCONFIG_LVL1);
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_WKUPEN, 0xFFFF);
	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_reset);

/*
 * Get the element index and frame index of transmitter
 * id		: McBSP interface ID
 * ei		: element index
 * fi		: frame index
 */
int omap2_mcbsp_transmitter_index(int id, int *ei, int *fi)
{
	struct omap_mcbsp *mcbsp;
	int eix = 0, fix = 0;

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);

	if ((!ei) || (!fi)) {
		printk(KERN_ERR	"OMAP_McBSP: Invalid ei and fi params \n");
		goto txinx_err;
	}

	if (mcbsp->dma_tx_lch == -1) {
		printk(KERN_ERR "OMAP_McBSP: Transmitter not started\n");
		goto txinx_err;
	}

	if (omap_get_dma_chain_index
		(mcbsp->dma_tx_lch, &eix, &fix) != 0) {
		printk(KERN_ERR "OMAP_McBSP: Getting chain index failed\n");
		goto txinx_err;
	}

	*ei = eix;
	*fi = fix;

	return 0;

txinx_err:
	return -EINVAL;
}
EXPORT_SYMBOL(omap2_mcbsp_transmitter_index);

/*
 * Get the element index and frame index of receiver
 * id	: McBSP interface ID
 * ei		: element index
 * fi		: frame index
 */
int omap2_mcbsp_receiver_index(int id, int *ei, int *fi)
{
	struct omap_mcbsp *mcbsp;
	int eix = 0, fix = 0;

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);

	if ((!ei) || (!fi)) {
		printk(KERN_ERR	"OMAP_McBSP: Invalid ei and fi params x\n");
		goto rxinx_err;
	}

	/* Check if chain exists */
	if (mcbsp->dma_rx_lch == -1) {
		printk(KERN_ERR "OMAP_McBSP: Receiver not started\n");
		goto rxinx_err;
	}

	/* Get dma_chain_index */
	if (omap_get_dma_chain_index
		(mcbsp->dma_rx_lch, &eix, &fix) != 0) {
		printk(KERN_ERR "OMAP_McBSP: Getting chain index failed\n");
		goto rxinx_err;
	}

	*ei = eix;
	*fi = fix;
	return 0;

rxinx_err:
	return -EINVAL;
}
EXPORT_SYMBOL(omap2_mcbsp_receiver_index);

/*
 * Basic Reset Transmitter
 * id		: McBSP interface number
 * state	: Disable (0)/ Enable (1) the transmitter
 */
int omap2_mcbsp_set_xrst(unsigned int id, u8 state)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;

	if (state == OMAP_MCBSP_XRST_DISABLE)
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
		      omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR2) & (~XRST));
	else
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
			omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR2) | XRST);
	udelay(10);

	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_set_xrst);

/*
 * Reset Receiver
 * id		: McBSP interface number
 * state	: Disable (0)/ Enable (1) the receiver
 */
int omap2_mcbsp_set_rrst(unsigned int id, u8 state)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;

	if (state == OMAP_MCBSP_RRST_DISABLE)
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1,
			omap_mcbsp_read(io_base,
				 OMAP_MCBSP_REG_SPCR1) & (~RRST));
	else
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1,
		      omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR1) | RRST);
	udelay(10);
	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_set_rrst);

/*
 * Receive multichannel selection
 * id		: McBSP Interface ID.
 * state	: Enable/Disable multichannel for reception
 */
int omap2_mcbsp_rxmultich_enable(unsigned int id, u8 state)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;

	if (state == OMAP_MCBSP_RXMUTICH_ENABLE) {
		/* All the 128 channels are enabled */
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_MCR1,
			omap_mcbsp_read(io_base,
					 OMAP_MCBSP_REG_MCR1) & ~(RMCM));
	} else if (state == OMAP_MCBSP_RXMUTICH_DISABLE) {
		/*
		 * All channels are disabled by default.
		 * Required channels are selected by enabling
		 * RP(A/B)BLK and RCER(A/B) appropriately.
		 */
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_MCR1,
			omap_mcbsp_read(io_base, OMAP_MCBSP_REG_MCR1) | (RMCM));
	} else {
		printk(KERN_ERR "OMAP_McBSP: Invalid rxmultichannel state \n");
		return -EINVAL;
	}

	return 0;

}
EXPORT_SYMBOL(omap2_mcbsp_rxmultich_enable);

/*
 * Transmit multichannel selection
 * id		: McBSP interface ID
 * state	: Enable/Disable multichannel mode for transmission
 */
int omap2_mcbsp_txmultich_enable(unsigned int id, u32 state)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;

	if (state == OMAP_MCBSP_TXMUTICH_ENABLE) {
		/* All the 128 channels are enabled */
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_MCR2,
				  omap_mcbsp_read(io_base,
					 OMAP_MCBSP_REG_MCR2) | (XMCM(0)));
	} else if (state == OMAP_MCBSP_TXMUTICH_DISABLE) {
		/*
		 * All channels are disabled by default.
		 * Required channels are selected by enabling
		 * RP(A/B)BLK and RCER(A/B) appropriately.
		 */
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_MCR2,
			omap_mcbsp_read(io_base,
					 OMAP_MCBSP_REG_MCR2) | (XMCM(1)));
	} else {
		printk(KERN_ERR "OMAP_McBSP: Invalid rxmultichannel state \n");
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_txmultich_enable);

/*
 * Transmit multichannel configuration
 * id		: McBSP interface No.
 * part_mode	: Partition mode
 * parta_enable	: channel to enable in partition A
 * partb_enable	: channel to enable in partition B
 * ch_enable	: channel enable
 */
int omap2_mcbsp_txmultich_cfg(unsigned int id, u8 part_mode, u8 parta_enable,
					 u8 partb_enable, u32 ch_enable)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;

	/* check for valid partition mode */
	if ((part_mode != OMAP_MCBSP_TWOPARTITION_MODE
	     && part_mode != OMAP_MCBSP_EIGHTPARTITION_MODE))

		return -EINVAL;

	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_MCR2,
		(omap_mcbsp_read(io_base, OMAP_MCBSP_REG_MCR2) | part_mode |
						parta_enable | partb_enable));
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_XCERA,
		(omap_mcbsp_read(io_base, OMAP_MCBSP_REG_XCERA) | ch_enable));

	return 0;


}
EXPORT_SYMBOL(omap2_mcbsp_txmultich_cfg);

/*
 * Receive multichannel configuration
 * id	: McBSP interface No.
 * part_mode	: Partition mode
 * parta_enable	: channel to enable in partition A
 * partb_enable	: channel to enable in partition B
 * ch_enable	: channel enable
 */
int omap2_mcbsp_rxmultich_cfg(unsigned int id, u8 part_mode,
			       u8 parta_enable, u8 partb_enable, u32 ch_enable)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;

	/* check for valid partition mode */
	if ((part_mode != OMAP_MCBSP_TWOPARTITION_MODE
	     && part_mode != OMAP_MCBSP_EIGHTPARTITION_MODE))
		return -EINVAL;

	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_MCR1,
		(omap_mcbsp_read(io_base, OMAP_MCBSP_REG_MCR1) | part_mode |
						parta_enable | partb_enable));
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_RCERA,
		(omap_mcbsp_read(io_base, OMAP_MCBSP_REG_RCERA) | ch_enable));

	return 0;

}
EXPORT_SYMBOL(omap2_mcbsp_rxmultich_cfg);

/*
 * Configure the receiver parameters
 * id		: McBSP Interface ID
 * rp		: DMA Receive parameters
 */
int omap2_mcbsp_dma_recv_params(unsigned int id,
				omap_mcbsp_dma_transfer_params *rp)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;
	int err, chain_id = -1;
	struct omap_dma_channel_params rx_params;
	u32  dt = 0;
#ifdef CONFIG_USE_MCBSP_FIFO
	u32 mcbsp_fifo_size;
#endif

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}
#ifdef CONFIG_USE_MCBSP_FIFO
	if (id == OMAP_MCBSP2)
		mcbsp_fifo_size = MCBSP2_FIFO_SIZE;
	else
		mcbsp_fifo_size = MCBSP_FIFO_SIZE;
#endif

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;
	dt = rp->word_length1;

	if (dt == OMAP_MCBSP_WORD_8)
		rx_params.data_type = OMAP_DMA_DATA_TYPE_S8;
	else if (dt == OMAP_MCBSP_WORD_16)
		rx_params.data_type = OMAP_DMA_DATA_TYPE_S16;
	else if (dt == OMAP_MCBSP_WORD_32)
		rx_params.data_type = OMAP_DMA_DATA_TYPE_S32;
	else
		return -EINVAL;

	rx_params.read_prio = DMA_CH_PRIO_HIGH;
	rx_params.write_prio = DMA_CH_PRIO_HIGH;
/* If McBSP FIFO is used, do a packet sync DMA */
#ifdef CONFIG_USE_MCBSP_FIFO
	mcbsp->rx_config_done = 0;
	rx_params.sync_mode = OMAP_DMA_SYNC_PACKET;
	rx_params.src_fi = mcbsp_fifo_size;
#else
	rx_params.sync_mode = OMAP_DMA_SYNC_ELEMENT;
	rx_params.src_fi = 0;
#endif
	rx_params.trigger = mcbsp->dma_rx_sync;
	rx_params.src_or_dst_synch = 0x01;
	rx_params.src_amode = OMAP_DMA_AMODE_CONSTANT;
	rx_params.src_ei = 0x0;
	/* Indexing is always in bytes - so multiply with dt */

	dt = (rx_params.data_type == OMAP_DMA_DATA_TYPE_S8) ? 1 :
		(rx_params.data_type == OMAP_DMA_DATA_TYPE_S16) ? 2 : 4;

	/* SKIP_FIRST and sKIP_SECOND- 24 bit data in stereo mode*/
	if (rp->skip_alt == OMAP_MCBSP_SKIP_SECOND) {
		rx_params.dst_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		rx_params.dst_ei = (1);
		rx_params.dst_fi = (1) + ((-1) * dt);
	} else if (rp->skip_alt == OMAP_MCBSP_SKIP_FIRST) {
		rx_params.dst_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		rx_params.dst_ei = 1 + (-2) * dt;
		rx_params.dst_fi = 1 + (2) * dt;
	} else {
		rx_params.dst_amode = OMAP_DMA_AMODE_POST_INC;
		rx_params.dst_ei = 0;
		rx_params.dst_fi = 0;
	}

	mcbsp->rxskip_alt = rp->skip_alt;
	mcbsp->auto_reset &= ~OMAP_MCBSP_AUTO_RRST;
	mcbsp->auto_reset |=	(rp->auto_reset & OMAP_MCBSP_AUTO_RRST);

	mcbsp->rx_word_length = rx_params.data_type << 0x1;
	if (rx_params.data_type == 0)
		mcbsp->rx_word_length = 1;

	mcbsp->rx_callback = rp->callback;
	mcbsp->rx_params = rx_params;
	/* request for a chain of dma channels for data reception */
	if (mcbsp->dma_rx_lch == -1) {
		err = omap_request_dma_chain(id, "McBSP RX",
					 omap2_mcbsp_rx_dma_callback, &chain_id,
					 2, OMAP_DMA_DYNAMIC_CHAIN, rx_params);
		if (err < 0) {
			printk(KERN_ERR "Receive path configuration failed \n");
			return -EINVAL;
		}
		mcbsp->dma_rx_lch = chain_id;
		mcbsp->rx_dma_chain_state = 0;
	} else {
		/* DMA params already set, modify the same!! */
		err = omap_modify_dma_chain_params(mcbsp->dma_rx_lch,
								 rx_params);
		if (err < 0)
			return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_dma_recv_params);

/*
 * Configure the transmitter parameters
 * id		: McBSP Interface ID
 * tp		: DMA Transfer parameters
 */

int omap2_mcbsp_dma_trans_params(unsigned int id,
				omap_mcbsp_dma_transfer_params *tp)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;
	struct omap_dma_channel_params tx_params;
	int err = 0, chain_id = -1;
	u32 dt = 0;
#ifdef CONFIG_USE_MCBSP_FIFO
	u32 mcbsp_fifo_size;
#endif

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;
#ifdef CONFIG_USE_MCBSP_FIFO
	if (id == OMAP_MCBSP2)
		mcbsp_fifo_size = MCBSP2_FIFO_SIZE;
	else
		mcbsp_fifo_size = MCBSP_FIFO_SIZE;
#endif

	dt = tp->word_length1;
	if ((dt != OMAP_MCBSP_WORD_8) && (dt != OMAP_MCBSP_WORD_16)
						 && (dt != OMAP_MCBSP_WORD_32))
		return -EINVAL;
	if (dt == OMAP_MCBSP_WORD_8)
		tx_params.data_type = OMAP_DMA_DATA_TYPE_S8;
	else if (dt == OMAP_MCBSP_WORD_16)
		tx_params.data_type = OMAP_DMA_DATA_TYPE_S16;
	else if (dt == OMAP_MCBSP_WORD_32)
		tx_params.data_type = OMAP_DMA_DATA_TYPE_S32;
	else
		return -EINVAL;

	tx_params.read_prio = DMA_CH_PRIO_HIGH;
	tx_params.write_prio = DMA_CH_PRIO_HIGH;
/* IF McBSP FIFO is used, use packet sync DMA*/
#ifdef CONFIG_USE_MCBSP_FIFO
	tx_params.sync_mode = OMAP_DMA_SYNC_PACKET;
	tx_params.dst_fi = mcbsp_fifo_size;
#else
	tx_params.sync_mode = OMAP_DMA_SYNC_ELEMENT;
	tx_params.dst_fi = 0;
#endif
	tx_params.trigger = mcbsp->dma_tx_sync;
	tx_params.src_or_dst_synch = 0;
	/* Indexing is always in bytes - so multiply with dt */
	mcbsp->tx_word_length = tx_params.data_type << 0x1;

	if (tx_params.data_type == 0)
		mcbsp->tx_word_length = 1;
	dt = mcbsp->tx_word_length;

	/* SKIP_FIRST and sKIP_SECOND- 24 bit data in stereo mode*/
	if (tp->skip_alt == OMAP_MCBSP_SKIP_SECOND) {
		tx_params.src_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		tx_params.src_ei = (1);
		tx_params.src_fi = (1) + ((-1) * dt);
	} else if (tp->skip_alt == OMAP_MCBSP_SKIP_FIRST) {
		tx_params.src_amode = OMAP_DMA_AMODE_DOUBLE_IDX;
		tx_params.src_ei = 1 + (-2) * dt;
		tx_params.src_fi = 1 + (2) * dt;
	} else {
		tx_params.src_amode = OMAP_DMA_AMODE_POST_INC;
		tx_params.src_ei = 0;
		tx_params.src_fi = 0;
	}

	tx_params.dst_amode = OMAP_DMA_AMODE_CONSTANT;
	tx_params.dst_ei = 0;
	mcbsp->txskip_alt = tp->skip_alt;
	mcbsp->auto_reset &= ~OMAP_MCBSP_AUTO_XRST;
	mcbsp->auto_reset |=
		(tp->auto_reset & OMAP_MCBSP_AUTO_XRST);
	mcbsp->tx_callback = tp->callback;

	/* Based on Rjust we can do double indexing DMA params configuration */
	if (mcbsp->dma_tx_lch == -1) {
		err = omap_request_dma_chain(id, "McBSP TX",
					 omap2_mcbsp_tx_dma_callback, &chain_id,
					 2, OMAP_DMA_DYNAMIC_CHAIN, tx_params);
		if (err < 0) {
			printk(KERN_ERR
				"Transmit path configuration failed \n");
			return -EINVAL;
		}
		mcbsp->tx_dma_chain_state = 0;
	mcbsp->dma_tx_lch = chain_id;
	} else {
		/* DMA params already set, modify the same!! */
		err = omap_modify_dma_chain_params(mcbsp->dma_tx_lch,
								 tx_params);
		if (err < 0)
			return -EINVAL;
	}
#ifdef CONFIG_USE_MCBSP_FIFO
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_THRSH2, (mcbsp_fifo_size - 1));
#endif

	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_dma_trans_params);

/*
 * Start receving data on a McBSP interface
 * id			: McBSP interface ID
 * cbdata		: User data to be returned with callback
 * buf_start_addr	: The destination address [physical address]
 * buf_size		: Buffer size
 */

int omap2_mcbsp_receive_data(unsigned int id, void *cbdata,
			     dma_addr_t buf_start_addr, u32 buf_size)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;
	int enable_rx = 0;
	int e_count = 0;
	int f_count = 0;
	int ret = 0;
#ifdef CONFIG_USE_MCBSP_FIFO
	u32 thrsh1 = 256;	/* lowest value for McBSP threshold*/
	u32 mcbsp_fifo_size;
	int err;
#endif

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;
	mcbsp->rx_cb_arg = cbdata;

	/* Auto RRST handling logic - disable the Reciever before 1st dma */
	if ((mcbsp->auto_reset & OMAP_MCBSP_AUTO_RRST) &&
		(omap_dma_chain_status(mcbsp->dma_rx_lch)
				== OMAP_DMA_CHAIN_INACTIVE)) {
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1,
			omap_mcbsp_read(io_base,
				 OMAP_MCBSP_REG_SPCR1) & (~RRST));
		enable_rx = 1;
	}

	/*
	 * for skip_first and second, we need to set e_count =2,
	 * and f_count = number of frames = number of elements/e_count
	 */
	e_count = (buf_size / mcbsp->rx_word_length);

	/* IF McBSP FIFO is used, change receive side configuration */
#ifdef CONFIG_USE_MCBSP_FIFO
	if (mcbsp->rx_config_done == 0) {
		mcbsp->rx_config_done = 1;
		if (id == OMAP_MCBSP2)
			mcbsp_fifo_size = MCBSP2_FIFO_SIZE;
		else
			mcbsp_fifo_size = MCBSP_FIFO_SIZE;

		if (e_count < mcbsp_fifo_size) {
			thrsh1 = e_count;
		} else {
			/* Find the optimum threshold value for MCBSP
					to transfer complete data*/
			if ((e_count % mcbsp_fifo_size) == 0)
				thrsh1 = mcbsp_fifo_size;
			else if ((e_count % ((mcbsp_fifo_size * 3)/4)) == 0)
				thrsh1 = (mcbsp_fifo_size * 3)/4;
			else if ((e_count % ((mcbsp_fifo_size * 1)/2)) == 0)
				thrsh1 = (mcbsp_fifo_size * 1)/2;
			else if ((e_count % ((mcbsp_fifo_size * 1)/4)) == 0)
				thrsh1 = (mcbsp_fifo_size * 1)/4;
			else
				thrsh1 = 1;
		}

		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_THRSH1, (thrsh1-1));

		if (thrsh1 != mcbsp_fifo_size) {
			mcbsp->rx_params.src_fi = thrsh1;
		/* if threshold =1, use element sync DMA */
			if (thrsh1 == 1) {
				mcbsp->rx_params.sync_mode =
							 OMAP_DMA_SYNC_ELEMENT;
				mcbsp->rx_params.src_fi = 0;
			}
			err = omap_modify_dma_chain_params(
				mcbsp->dma_rx_lch, mcbsp->rx_params);
			if (err < 0) {
				printk(KERN_ERR "DMA reconfiguration failed\n");
				return -EINVAL;
			}
		}
	}
#endif


	if (mcbsp->rxskip_alt != OMAP_MCBSP_SKIP_NONE) {
		/*
		 * since the number of frames = total number of elements/element
		 * count, However, with double indexing for data transfers,
		 * double the number of elements need to be transmitted
		 */
		f_count = e_count;
		e_count = 2;
	} else {
		f_count = 1;
	}
	/*
	 * If the DMA is to be configured to skip the first byte, we need
	 * to jump backwards, so we need to move one chunk forward and
	 * ask dma if we dont want the client driver knowing abt this.
	 */
	if (mcbsp->rxskip_alt == OMAP_MCBSP_SKIP_FIRST)
		buf_start_addr += mcbsp->rx_word_length;

	ret = omap_dma_chain_a_transfer(mcbsp->dma_rx_lch,
		mcbsp->phys_base + OMAP_MCBSP_REG_DRR, buf_start_addr,
		e_count, f_count, mcbsp);
	if (ret < 0)
		return ret;

	if (mcbsp->rx_dma_chain_state == 0) {
		if (mcbsp->interface_mode == OMAP_MCBSP_MASTER)
			omap2_mcbsp_set_srg_fsg(id, OMAP_MCBSP_ENABLE_FSG_SRG);

		ret = omap_start_dma_chain_transfers(mcbsp->dma_rx_lch);
		if (ret < 0)
			return ret;

		mcbsp->rx_dma_chain_state = 1;
	}
	/* Auto RRST handling logic - Enable the Reciever after 1st dma */
	if (enable_rx &&
		(omap_dma_chain_status(mcbsp->dma_rx_lch)
				== OMAP_DMA_CHAIN_ACTIVE))
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1,
			omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR1) | RRST);

	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_receive_data);

/*
 * Start transmitting data through a McBSP interface
 * id			: McBSP interface ID
 * cbdata		: User data to be returned with callback
 * buf_start_addr	: The source address [This should be physical address]
 * buf_size		: Buffer size
 */
int omap2_mcbsp_send_data(unsigned int id, void *cbdata,
			  dma_addr_t buf_start_addr, u32 buf_size)
{
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;
	u8 enable_tx = 0;
	int e_count = 0;
	int f_count = 0;
	int ret = 0;

	if (!omap_mcbsp_check_valid_id(id)) {
		printk(KERN_ERR "%s: Invalid id (%d)\n", __func__, id + 1);
		return -ENODEV;
	}

	mcbsp = id_to_mcbsp_ptr(id);
	io_base = mcbsp->io_base;
	mcbsp->tx_cb_arg = cbdata;

	/* Auto RRST handling logic - disable the Reciever before 1st dma */
	if ((mcbsp->auto_reset & OMAP_MCBSP_AUTO_XRST) &&
			(omap_dma_chain_status(mcbsp->dma_tx_lch)
				== OMAP_DMA_CHAIN_INACTIVE)) {
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
			omap_mcbsp_read(io_base,
				OMAP_MCBSP_REG_SPCR2) & (~XRST));
		enable_tx = 1;
	}
	/*
	 * for skip_first and second, we need to set e_count =2, and
	 * f_count = number of frames = number of elements/e_count
	 */
	e_count = (buf_size / mcbsp->tx_word_length);
	if (mcbsp->txskip_alt != OMAP_MCBSP_SKIP_NONE) {
		/*
		 * number of frames = total number of elements/element count,
		 * However, with double indexing for data transfers, double I
		 * the number of elements need to be transmitted
		 */
		f_count = e_count;
		e_count = 2;
	} else {
		f_count = 1;
	}

	/*
	 * If the DMA is to be configured to skip the first byte, we need
	 * to jump backwards, so we need to move one chunk forward and ask
	 * dma if we dont want the client driver knowing abt this.
	 */
	if (mcbsp->txskip_alt == OMAP_MCBSP_SKIP_FIRST)
		buf_start_addr += mcbsp->tx_word_length;

	ret = omap_dma_chain_a_transfer(mcbsp->dma_tx_lch,
		buf_start_addr, mcbsp->phys_base + OMAP_MCBSP_REG_DXR,
		e_count, f_count, mcbsp);
	if (ret < 0)
		return ret;

	if (mcbsp->tx_dma_chain_state == 0) {
		if (mcbsp->interface_mode == OMAP_MCBSP_MASTER)
			omap2_mcbsp_set_srg_fsg(id, OMAP_MCBSP_ENABLE_FSG_SRG);

		ret = omap_start_dma_chain_transfers(mcbsp->dma_tx_lch);
		if (ret < 0)
			return ret;
		mcbsp->tx_dma_chain_state = 1;
	}

	/* Auto XRST handling logic - Enable the Reciever after 1st dma */
	if (enable_tx &&
		(omap_dma_chain_status(mcbsp->dma_tx_lch)
		== OMAP_DMA_CHAIN_ACTIVE))
		omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2,
			omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR2) | XRST);

	return 0;
}
EXPORT_SYMBOL(omap2_mcbsp_send_data);

#ifdef CONFIG_ARCH_OMAP2420
static struct omap_mcbsp_platform_data omap2420_mcbsp_pdata[] = {
	{
		.phys_base	= OMAP24XX_MCBSP1_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP1_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP1_TX,
		.rx_irq		= INT_24XX_MCBSP1_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP1_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.phys_base	= OMAP24XX_MCBSP2_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP2_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP2_TX,
		.rx_irq		= INT_24XX_MCBSP2_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP2_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
};
#define OMAP2420_MCBSP_PDATA_SZ		ARRAY_SIZE(omap2420_mcbsp_pdata)
#else
#define omap2420_mcbsp_pdata		NULL
#define OMAP2420_MCBSP_PDATA_SZ		0
#endif

#ifdef CONFIG_ARCH_OMAP2430
static struct omap_mcbsp_platform_data omap2430_mcbsp_pdata[] = {
	{
		.phys_base	= OMAP24XX_MCBSP1_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP1_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP1_TX,
		.rx_irq		= INT_24XX_MCBSP1_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP1_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.phys_base	= OMAP24XX_MCBSP2_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP2_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP2_TX,
		.rx_irq		= INT_24XX_MCBSP2_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP2_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.phys_base	= OMAP2430_MCBSP3_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP3_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP3_TX,
		.rx_irq		= INT_24XX_MCBSP3_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP3_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.phys_base	= OMAP2430_MCBSP4_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP4_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP4_TX,
		.rx_irq		= INT_24XX_MCBSP4_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP4_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.phys_base	= OMAP2430_MCBSP5_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP5_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP5_TX,
		.rx_irq		= INT_24XX_MCBSP5_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP5_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
};
#define OMAP2430_MCBSP_PDATA_SZ		ARRAY_SIZE(omap2430_mcbsp_pdata)
#else
#define omap2430_mcbsp_pdata		NULL
#define OMAP2430_MCBSP_PDATA_SZ		0
#endif

#ifdef CONFIG_ARCH_OMAP34XX
static struct omap_mcbsp_platform_data omap34xx_mcbsp_pdata[] = {
	{
		.phys_base	= OMAP34XX_MCBSP1_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP1_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP1_TX,
		.rx_irq		= INT_24XX_MCBSP1_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP1_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.phys_base	= OMAP34XX_MCBSP2_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP2_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP2_TX,
		.rx_irq		= INT_24XX_MCBSP2_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP2_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.phys_base	= OMAP34XX_MCBSP3_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP3_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP3_TX,
		.rx_irq		= INT_24XX_MCBSP3_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP3_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.phys_base	= OMAP34XX_MCBSP4_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP4_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP4_TX,
		.rx_irq		= INT_24XX_MCBSP4_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP4_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
	{
		.phys_base	= OMAP34XX_MCBSP5_BASE,
		.dma_rx_sync	= OMAP24XX_DMA_MCBSP5_RX,
		.dma_tx_sync	= OMAP24XX_DMA_MCBSP5_TX,
		.rx_irq		= INT_24XX_MCBSP5_IRQ_RX,
		.tx_irq		= INT_24XX_MCBSP5_IRQ_TX,
		.ops		= &omap2_mcbsp_ops,
		.clk_name	= "mcbsp_clk",
	},
};
#define OMAP34XX_MCBSP_PDATA_SZ		ARRAY_SIZE(omap34xx_mcbsp_pdata)
#else
#define omap34xx_mcbsp_pdata		NULL
#define OMAP34XX_MCBSP_PDATA_SZ		0
#endif

static int __init omap2_mcbsp_init(void)
{
	int i;

	for (i = 0; i < omap_mcbsp_clks_size; i++) {
		/* Once we call clk_get inside init, we do not register it */
		omap_mcbsp_clk_init(&omap_mcbsp_clks[i]);
		clk_register(&omap_mcbsp_clks[i].clk);
	}

	if (cpu_is_omap2420())
		omap_mcbsp_count = OMAP2420_MCBSP_PDATA_SZ;
	if (cpu_is_omap2430())
		omap_mcbsp_count = OMAP2430_MCBSP_PDATA_SZ;
	if (cpu_is_omap34xx())
		omap_mcbsp_count = OMAP34XX_MCBSP_PDATA_SZ;

	mcbsp_ptr = kzalloc(omap_mcbsp_count * sizeof(struct omap_mcbsp *),
								GFP_KERNEL);
	if (!mcbsp_ptr)
		return -ENOMEM;

	if (cpu_is_omap2420())
		omap_mcbsp_register_board_cfg(omap2420_mcbsp_pdata,
						OMAP2420_MCBSP_PDATA_SZ);
	if (cpu_is_omap2430())
		omap_mcbsp_register_board_cfg(omap2430_mcbsp_pdata,
						OMAP2430_MCBSP_PDATA_SZ);
	if (cpu_is_omap34xx())
		omap_mcbsp_register_board_cfg(omap34xx_mcbsp_pdata,
						OMAP34XX_MCBSP_PDATA_SZ);

	return omap_mcbsp_init();
}
arch_initcall(omap2_mcbsp_init);
