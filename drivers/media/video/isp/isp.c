/*
 * drivers/media/video/isp/isp.c
 *
 * Driver Library for ISP Control module in TI's OMAP3430 Camera ISP
 * ISP interface and IRQ related APIs are defined here.
 *
 * Copyright (C) 2008 Texas Instruments.
 * Copyright (C) 2008 Nokia.
 *
 * Contributors:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Mohit Jalori <mjalori@ti.com>
 * 	Sakari Ailus <sakari.ailus@nokia.com>
 * 	Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
 *	Toni Leinonen <toni.leinonen@nokia.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <asm/irq.h>
#include <linux/bitops.h>
#include <linux/scatterlist.h>
#include <asm/mach-types.h>
#include <linux/device.h>
#include <linux/videodev2.h>
#include <linux/vmalloc.h>

#include <mach/control.h>

#include "isp.h"
#include "ispmmu.h"
#include "ispreg.h"
#include "ispccdc.h"
#include "isph3a.h"
#include "isphist.h"
#include "isp_af.h"
#include "isppreview.h"
#include "ispresizer.h"
#include "ispcsi2.h"

#define DEBUG_RECORD

static DECLARE_MUTEX(isp_mutex);

#if ISP_WORKAROUND
void *buff_addr;
dma_addr_t buff_addr_mapped;
struct scatterlist *sglist_alloc;
static int alloc_done, num_sc;
unsigned long offset_value;
#endif

/* List of image formats supported via OMAP ISP */
const static struct v4l2_fmtdesc isp_formats[] = {
	{
		.description = "UYVY, packed",
		.pixelformat = V4L2_PIX_FMT_UYVY,
	},
	{
		.description = "YUYV (YUV 4:2:2), packed",
		.pixelformat = V4L2_PIX_FMT_YUYV,
	},
	{
		.description = "Bayer10 (GrR/BGb)",
		.pixelformat = V4L2_PIX_FMT_SGRBG10,
	},
	{
		.description = "Bayer10 (pattern)",
		.pixelformat = V4L2_PIX_FMT_PATT,
	}
};

/* ISP Crop capabilities */
static struct v4l2_rect ispcroprect;
static struct v4l2_rect cur_rect;

/* helper variable for interrupt state machine */
#define VD1_ON_FRAME_BEGINNING 0
#define VD1_ON_FRAME_ENDING 1
static u8 vd1_position = VD1_ON_FRAME_BEGINNING;

/**
 * struct vcontrol - Video control structure.
 * @qc: V4L2 Query control structure.
 * @current_value: Current value of the control.
 */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {
	{
		{
			.id = V4L2_CID_BRIGHTNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Brightness",
			.minimum = ISPPRV_BRIGHT_LOW,
			.maximum = ISPPRV_BRIGHT_HIGH,
			.step = ISPPRV_BRIGHT_STEP,
			.default_value = ISPPRV_BRIGHT_DEF,
		},
		.current_value = ISPPRV_BRIGHT_DEF,
	},
	{
		{
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = ISPPRV_CONTRAST_LOW,
			.maximum = ISPPRV_CONTRAST_HIGH,
			.step = ISPPRV_CONTRAST_STEP,
			.default_value = ISPPRV_CONTRAST_DEF,
		},
		.current_value = ISPPRV_CONTRAST_DEF,
	},
	{
		{
			.id = V4L2_CID_PRIVATE_ISP_COLOR_FX,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Color Effects",
			.minimum = PREV_DEFAULT_COLOR,
			.maximum = PREV_BW_COLOR,
			.step = 1,
			.default_value = PREV_DEFAULT_COLOR,
		},
		.current_value = PREV_DEFAULT_COLOR,
	}
};


/**
 * struct ispirq - Structure for containing callbacks to be called in ISP ISR.
 * @isp_callbk: Array which stores callback functions, indexed by the type of
 *              callback (8 possible types).
 * @isp_callbk_arg1: Pointer to array containing pointers to the first argument
 *                   to be passed to the requested callback function.
 * @isp_callbk_arg2: Pointer to array containing pointers to the second
 *                   argument to be passed to the requested callback function.
 *
 * This structure is used to contain all the callback functions related for
 * each callback type (CBK_CCDC_VD0, CBK_CCDC_VD1, CBK_PREV_DONE,
 * CBK_RESZ_DONE, CBK_MMU_ERR, CBK_H3A_AWB_DONE, CBK_HIST_DONE, CBK_HS_VS,
 * CBK_LSC_ISR).
 */
static struct ispirq {
	isp_callback_t isp_callbk[CBK_END];
	isp_vbq_callback_ptr isp_callbk_arg1[CBK_END];
	void *isp_callbk_arg2[CBK_END];
} ispirq_obj;

/**
 * struct isp - Structure for storing ISP Control module information
 * @lock: Spinlock to sync between isr and processes.
 * @isp_temp_buf_lock: Temporary spinlock for buffer control.
 * @isp_mutex: Semaphore used to get access to the ISP.
 * @if_status: Type of interface used in ISP.
 * @interfacetype: (Not used).
 * @ref_count: Reference counter.
 * @cam_ick: Pointer to ISP Interface clock.
 * @cam_fck: Pointer to ISP Functional clock.
 *
 * This structure is used to store the OMAP ISP Control Information.
 */
static struct isp {
	spinlock_t lock;	/* For handling registered ISP callbacks */
	spinlock_t isp_temp_buf_lock;	/* For handling isp buffers state */
	struct mutex isp_mutex;	/* For handling ref_count field */
	u8 if_status;
	u8 interfacetype;
	int ref_count;
	struct clk *cam_ick;
	struct clk *cam_mclk;
	struct clk *csi2_fck;
} isp_obj;

struct isp_sgdma ispsg;

/**
 * struct ispmodule - Structure for storing ISP sub-module information.
 * @isp_pipeline: Bit mask for submodules enabled within the ISP.
 * @isp_temp_state: State of current buffers.
 * @applyCrop: Flag to do a crop operation when video buffer queue ISR is done
 * @pix: Structure containing the format and layout of the output image.
 * @ccdc_input_width: ISP CCDC module input image width.
 * @ccdc_input_height: ISP CCDC module input image height.
 * @ccdc_output_width: ISP CCDC module output image width.
 * @ccdc_output_height: ISP CCDC module output image height.
 * @preview_input_width: ISP Preview module input image width.
 * @preview_input_height: ISP Preview module input image height.
 * @preview_output_width: ISP Preview module output image width.
 * @preview_output_height: ISP Preview module output image height.
 * @resizer_input_width: ISP Resizer module input image width.
 * @resizer_input_height: ISP Resizer module input image height.
 * @resizer_output_width: ISP Resizer module output image width.
 * @resizer_output_height: ISP Resizer module output image height.
 */
struct ispmodule {
	unsigned int isp_pipeline;
	int isp_temp_state;
	int applyCrop;
	struct v4l2_pix_format pix;
	u8 isp_skip_frms;
	unsigned int ccdc_input_width;
	unsigned int ccdc_input_height;
	unsigned int ccdc_output_width;
	unsigned int ccdc_output_height;
	unsigned int preview_input_width;
	unsigned int preview_input_height;
	unsigned int preview_output_width;
	unsigned int preview_output_height;
	unsigned int resizer_input_width;
	unsigned int resizer_input_height;
	unsigned int resizer_output_width;
	unsigned int resizer_output_height;
};

static struct ispmodule ispmodule_obj = {
	.isp_pipeline = OMAP_ISP_CCDC,
	.isp_temp_state = ISP_BUF_INIT,
	.applyCrop = 0,
	.pix = {
		.width = ISP_OUTPUT_WIDTH_DEFAULT,
		.height = ISP_OUTPUT_HEIGHT_DEFAULT,
		.pixelformat = V4L2_PIX_FMT_UYVY,
		.field = V4L2_FIELD_NONE,
		.bytesperline = ISP_OUTPUT_WIDTH_DEFAULT * ISP_BYTES_PER_PIXEL,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.priv = 0,
	},
};

/* Structure for saving/restoring ISP module registers */
static struct isp_reg isp_reg_list[] = {
	{ISP_SYSCONFIG, 0},
	{ISP_TCTRL_GRESET_LENGTH, 0},
	{ISP_TCTRL_PSTRB_REPLAY, 0},
	{ISP_CTRL, 0},
	{ISP_TCTRL_CTRL, 0},
	{ISP_TCTRL_FRAME, 0},
	{ISP_TCTRL_PSTRB_DELAY, 0},
	{ISP_TCTRL_STRB_DELAY, 0},
	{ISP_TCTRL_SHUT_DELAY, 0},
	{ISP_TCTRL_PSTRB_LENGTH, 0},
	{ISP_TCTRL_STRB_LENGTH, 0},
	{ISP_TCTRL_SHUT_LENGTH, 0},
	{ISP_CBUFF_SYSCONFIG, 0},
	{ISP_CBUFF_IRQENABLE, 0},
	{ISP_CBUFF0_CTRL, 0},
	{ISP_CBUFF1_CTRL, 0},
	{ISP_CBUFF0_START, 0},
	{ISP_CBUFF1_START, 0},
	{ISP_CBUFF0_END, 0},
	{ISP_CBUFF1_END, 0},
	{ISP_CBUFF0_WINDOWSIZE, 0},
	{ISP_CBUFF1_WINDOWSIZE, 0},
	{ISP_CBUFF0_THRESHOLD, 0},
	{ISP_CBUFF1_THRESHOLD, 0},
	{ISP_TOK_TERM, 0}
};

/*
 *
 * V4L2 Handling
 *
 */

/**
 * find_vctrl - Returns the index of the ctrl array of the requested ctrl ID.
 * @id: Requested control ID.
 *
 * Returns 0 if successful, -EINVAL if not found, or -EDOM if its out of
 * domain.
 **/
static int find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;

	if (i < 0)
		i = -EINVAL;

	return i;
}

static int find_next_vctrl(int id)
{
	int i;
	u32 best = (u32)-1;

	for (i = 0; i < ARRAY_SIZE(video_control); i++) {
		if (video_control[i].qc.id > id
		    && (best == (u32)-1
			|| video_control[i].qc.id
			< video_control[best].qc.id)) {
			best = i;
		}
	}

	if (best == (u32)-1)
		return -EINVAL;

	return best;
}

/**
 * isp_release_resources - Free ISP submodules
 **/
void isp_release_resources(void)
{
	if (ispmodule_obj.isp_pipeline & OMAP_ISP_CCDC)
		ispccdc_free();

	if (ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW)
		isppreview_free();

	if (ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER)
		ispresizer_free();
	return;
}
EXPORT_SYMBOL(omapisp_unset_callback);

/* Flag to check first time of isp_get */
static int off_mode;

/**
 * isp_set_sgdma_callback - Set Scatter-Gather DMA Callback.
 * @sgdma_state: Pointer to structure with the SGDMA state for each videobuffer
 * @func_ptr: Callback function pointer for SG-DMA management
 **/
static int isp_set_sgdma_callback(struct isp_sgdma_state *sgdma_state,
						isp_vbq_callback_ptr func_ptr)
{
	if ((ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER) &&
						is_ispresizer_enabled()) {
		isp_set_callback(CBK_RESZ_DONE, sgdma_state->callback,
						func_ptr, sgdma_state->arg);
	}

	if ((ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW) &&
						is_isppreview_enabled()) {
			isp_set_callback(CBK_PREV_DONE, sgdma_state->callback,
						func_ptr, sgdma_state->arg);
	}

	if (ispmodule_obj.isp_pipeline & OMAP_ISP_CCDC) {
		/* If We are using the a BT656 sensor, we have to configure
			VD0, and if we are using a RAW sensor, we will use VD1 */
		if (isp_obj.if_status & ISP_PARLL_YUV_BT)
			isp_set_callback(CBK_CCDC_VD0, sgdma_state->callback, func_ptr,
								sgdma_state->arg);
		else
			isp_set_callback(CBK_CCDC_VD1, sgdma_state->callback, func_ptr,
								sgdma_state->arg);
		isp_set_callback(CBK_LSC_ISR, NULL, NULL, NULL);
	}

	isp_set_callback(CBK_HS_VS, sgdma_state->callback, func_ptr,
							sgdma_state->arg);
	return 0;
}

/**
 * isp_set_callback - Sets the callback for the ISP module done events.
 * @type: Type of the event for which callback is requested.
 * @callback: Method to be called as callback in the ISR context.
 * @arg1: First argument to be passed when callback is called in ISR.
 * @arg2: Second argument to be passed when callback is called in ISR.
 *
 * This function sets a callback function for a done event in the ISP
 * module, and enables the corresponding interrupt.
 **/
int isp_set_callback(enum isp_callback_type type, isp_callback_t callback,
						isp_vbq_callback_ptr arg1,
						void *arg2)
{
	unsigned long irqflags = 0;

	if (callback == NULL) {
		DPRINTK_ISPCTRL("ISP_ERR : Null Callback\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&isp_obj.lock, irqflags);
	ispirq_obj.isp_callbk[type] = callback;
	ispirq_obj.isp_callbk_arg1[type] = arg1;
	ispirq_obj.isp_callbk_arg2[type] = arg2;
	spin_unlock_irqrestore(&isp_obj.lock, irqflags);

	switch (type) {
	case CBK_CCDC_VD0:
		omap_writel(IRQ0ENABLE_CCDC_VD0_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE)|IRQ0ENABLE_CCDC_VD0_IRQ,
				ISP_IRQ0ENABLE);
		break;
	case CBK_HS_VS:
/*		omap_writel(IRQ0ENABLE_HS_VS_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE) | IRQ0ENABLE_HS_VS_IRQ,
							ISP_IRQ0ENABLE);*/
		break;
	case CBK_PREV_DONE:
		omap_writel(IRQ0ENABLE_PRV_DONE_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE) |
					IRQ0ENABLE_PRV_DONE_IRQ,
					ISP_IRQ0ENABLE);
		break;
	case CBK_RESZ_DONE:
		omap_writel(IRQ0ENABLE_RSZ_DONE_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE) |
					IRQ0ENABLE_RSZ_DONE_IRQ,
					ISP_IRQ0ENABLE);
		break;
	case CBK_MMU_ERR:
		omap_writel(omap_readl(ISP_IRQ0ENABLE) |
					IRQ0ENABLE_MMU_ERR_IRQ,
					ISP_IRQ0ENABLE);

		omap_writel(omap_readl(ISPMMU_IRQENABLE) |
					IRQENABLE_MULTIHITFAULT |
					IRQENABLE_TWFAULT |
					IRQENABLE_EMUMISS |
					IRQENABLE_TRANSLNFAULT |
					IRQENABLE_TLBMISS,
					ISPMMU_IRQENABLE);
		break;
	case CBK_H3A_AWB_DONE:
		omap_writel(IRQ0ENABLE_H3A_AWB_DONE_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE) |
					IRQ0ENABLE_H3A_AWB_DONE_IRQ,
					ISP_IRQ0ENABLE);
		break;
	case CBK_H3A_AF_DONE:
		omap_writel(IRQ0ENABLE_H3A_AF_DONE_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE)|
				IRQ0ENABLE_H3A_AF_DONE_IRQ,
				ISP_IRQ0ENABLE);
		break;
	case CBK_HIST_DONE:
		omap_writel(IRQ0ENABLE_HIST_DONE_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE) |
					IRQ0ENABLE_HIST_DONE_IRQ,
					ISP_IRQ0ENABLE);
		break;
	case CBK_LSC_ISR:
		omap_writel(IRQ0ENABLE_CCDC_LSC_DONE_IRQ |
					IRQ0ENABLE_CCDC_LSC_PREF_COMP_IRQ |
					IRQ0ENABLE_CCDC_LSC_PREF_ERR_IRQ,
					ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE) |
					IRQ0ENABLE_CCDC_LSC_DONE_IRQ |
					IRQ0ENABLE_CCDC_LSC_PREF_COMP_IRQ |
					IRQ0ENABLE_CCDC_LSC_PREF_ERR_IRQ,
					ISP_IRQ0ENABLE);
		break;
	default:
		break;
	}
	return 0;
}
EXPORT_SYMBOL(isp_set_callback);

/**
 * isp_unset_callback - Clears the callback for the ISP module done events.
 * @type: Type of the event for which callback to be cleared.
 *
 * This function clears a callback function for a done event in the ISP
 * module, and disables the corresponding interrupt.
 **/
int isp_unset_callback(enum isp_callback_type type)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&isp_obj.lock, irqflags);
	ispirq_obj.isp_callbk[type] = NULL;
	ispirq_obj.isp_callbk_arg1[type] = NULL;
	ispirq_obj.isp_callbk_arg2[type] = NULL;

	switch (type) {
	case CBK_CCDC_VD0:
		omap_writel((omap_readl(ISP_IRQ0ENABLE)) &
						~IRQ0ENABLE_CCDC_VD0_IRQ,
						ISP_IRQ0ENABLE);
		break;
	case CBK_CCDC_VD1:
		omap_writel((omap_readl(ISP_IRQ0ENABLE)) &
						~IRQ0ENABLE_CCDC_VD1_IRQ,
						ISP_IRQ0ENABLE);
		break;
	case CBK_PREV_DONE:
		omap_writel((omap_readl(ISP_IRQ0ENABLE)) &
						~IRQ0ENABLE_PRV_DONE_IRQ,
						ISP_IRQ0ENABLE);
		break;
	case CBK_RESZ_DONE:
		omap_writel((omap_readl(ISP_IRQ0ENABLE)) &
						~IRQ0ENABLE_RSZ_DONE_IRQ,
						ISP_IRQ0ENABLE);
		break;
	case CBK_MMU_ERR:
		omap_writel(omap_readl(ISPMMU_IRQENABLE) &
						~(IRQENABLE_MULTIHITFAULT |
						IRQENABLE_TWFAULT |
						IRQENABLE_EMUMISS |
						IRQENABLE_TRANSLNFAULT |
						IRQENABLE_TLBMISS),
						ISPMMU_IRQENABLE);
		break;
	case CBK_H3A_AWB_DONE:
		omap_writel((omap_readl(ISP_IRQ0ENABLE)) &
						~IRQ0ENABLE_H3A_AWB_DONE_IRQ,
						ISP_IRQ0ENABLE);
		break;
	case CBK_H3A_AF_DONE:
		omap_writel((omap_readl(ISP_IRQ0ENABLE))&
				(~IRQ0ENABLE_H3A_AF_DONE_IRQ), ISP_IRQ0ENABLE);
		break;
	case CBK_HIST_DONE:
		omap_writel((omap_readl(ISP_IRQ0ENABLE)) &
						~IRQ0ENABLE_HIST_DONE_IRQ,
						ISP_IRQ0ENABLE);
		break;
	case CBK_HS_VS:
		omap_writel((omap_readl(ISP_IRQ0ENABLE)) &
						~IRQ0ENABLE_HS_VS_IRQ,
						ISP_IRQ0ENABLE);
		break;
	case CBK_LSC_ISR:
		omap_writel(omap_readl(ISP_IRQ0ENABLE) &
					~(IRQ0ENABLE_CCDC_LSC_DONE_IRQ |
					IRQ0ENABLE_CCDC_LSC_PREF_COMP_IRQ |
					IRQ0ENABLE_CCDC_LSC_PREF_ERR_IRQ),
					ISP_IRQ0ENABLE);
		break;
	case CBK_CSIA:
		isp_csi2_irq_set(0);
		break;
	case CBK_CSIB:
		omap_writel(IRQ0ENABLE_CSIB_IRQ, ISP_IRQ0STATUS);
		omap_writel(omap_readl(ISP_IRQ0ENABLE)|IRQ0ENABLE_CSIB_IRQ,
					ISP_IRQ0ENABLE);
		break;
	default:
		break;
	}
	spin_unlock_irqrestore(&isp_obj.lock, irqflags);
	return 0;
}
EXPORT_SYMBOL(isp_unset_callback);

/**
 * isp_request_interface - Requests an ISP interface type (parallel or serial).
 * @if_t: Type of requested ISP interface (parallel or serial).
 *
 * This function requests for allocation of an ISP interface type.
 **/
int isp_request_interface(enum isp_interface_type if_t)
{
	enum isp_interface_type temp_if_t = if_t;

	if (if_t == ISP_PARLL_YUV_BT)
		if_t = ISP_PARLL;

	if (isp_obj.if_status & if_t) {
		DPRINTK_ISPCTRL("ISP_ERR : Requested Interface already \
			allocated\n");
		goto err_ebusy;
	}
	if ((isp_obj.if_status == (ISP_PARLL | ISP_CSIA))
			|| isp_obj.if_status == (ISP_CSIA | ISP_CSIB)) {
		DPRINTK_ISPCTRL("ISP_ERR : No Free interface now\n");
		goto err_ebusy;
	}

	if (((isp_obj.if_status == ISP_PARLL) && (if_t == ISP_CSIA)) ||
				((isp_obj.if_status == ISP_CSIA) &&
				(if_t == ISP_PARLL)) ||
				((isp_obj.if_status == ISP_CSIA) &&
				(if_t == ISP_CSIB)) ||
				((isp_obj.if_status == ISP_CSIB) &&
				(if_t == ISP_CSIA)) ||
				(isp_obj.if_status == 0)) {
		isp_obj.if_status |= (if_t | temp_if_t);
		return 0;
	} else {
		DPRINTK_ISPCTRL("ISP_ERR : Invalid Combination Serial- \
			Parallel interface\n");
		return -EINVAL;
	}

err_ebusy:
	return -EBUSY;
}
EXPORT_SYMBOL(isp_request_interface);

/**
 * isp_free_interface - Frees an ISP interface type (parallel or serial).
 * @if_t: Type of ISP interface to be freed (parallel or serial).
 *
 * This function frees the allocation of an ISP interface type.
 **/
int isp_free_interface(enum isp_interface_type if_t)
{
	if ((if_t == ISP_PARLL) || (if_t == ISP_PARLL_YUV_BT))
		if_t |= (ISP_PARLL | ISP_PARLL_YUV_BT);

	isp_obj.if_status &= ~if_t;
	return 0;
}
EXPORT_SYMBOL(isp_free_interface);

/**
 * isp_set_xclk - Configures the specified cam_xclk to the desired frequency.
 * @xclk: Desired frequency of the clock in Hz.
 * @xclksel: XCLK to configure (0 = A, 1 = B).
 *
 * Configures the specified MCLK divisor in the ISP timing control register
 * (TCTRL_CTRL) to generate the desired xclk clock value.
 *
 * Divisor = CM_CAM_MCLK_HZ / xclk
 *
 * Returns the final frequency that is actually being generated
 **/
u32 isp_set_xclk(u32 xclk, u8 xclksel)
{
	u32 divisor;
	u32 currentxclk;

//	printk(KERN_ERR "+isp_set_xclk, xclk =%d, xclksel =%d\n", xclk, xclksel);
	if (xclk >= CM_CAM_MCLK_HZ) {
		divisor = ISPTCTRL_CTRL_DIV_BYPASS;
		currentxclk = CM_CAM_MCLK_HZ;
	} else if (xclk >= 2) {
		divisor = CM_CAM_MCLK_HZ / xclk;
		if (divisor >= ISPTCTRL_CTRL_DIV_BYPASS)
			divisor = ISPTCTRL_CTRL_DIV_BYPASS - 1;
		currentxclk = CM_CAM_MCLK_HZ / divisor;
	} else {
		divisor = xclk;
		currentxclk = 0;
	}

	switch (xclksel) {
	case 0:
		omap_writel((omap_readl(ISP_TCTRL_CTRL) &
				~ISPTCTRL_CTRL_DIVA_MASK) |
				(divisor << ISPTCTRL_CTRL_DIVA_SHIFT),
				ISP_TCTRL_CTRL);
		DPRINTK_ISPCTRL("isp_set_xclk(): cam_xclka set to %d Hz\n",
								currentxclk);
		break;
	case 1:
		omap_writel((omap_readl(ISP_TCTRL_CTRL) &
				~ISPTCTRL_CTRL_DIVB_MASK) |
				(divisor << ISPTCTRL_CTRL_DIVB_SHIFT),
				ISP_TCTRL_CTRL);
		DPRINTK_ISPCTRL("isp_set_xclk(): cam_xclkb set to %d Hz\n",
								currentxclk);
		break;
	default:
		DPRINTK_ISPCTRL("ISP_ERR: isp_set_xclk(): Invalid requested "
						"xclk. Must be 0 (A) or 1 (B)."
						"\n");
		return -EINVAL;
	}

	return currentxclk;
}
EXPORT_SYMBOL(isp_set_xclk);

/**
 * isp_get_xclk - Returns the frequency in Hz of the desired cam_xclk.
 * @xclksel: XCLK to retrieve (0 = A, 1 = B).
 *
 * This function returns the External Clock (XCLKA or XCLKB) value generated
 * by the ISP.
 **/
u32 isp_get_xclk(u8 xclksel)
{
	u32 xclkdiv;
	u32 xclk;

	switch (xclksel) {
	case 0:
		xclkdiv = omap_readl(ISP_TCTRL_CTRL) & ISPTCTRL_CTRL_DIVA_MASK;
		xclkdiv = xclkdiv >> ISPTCTRL_CTRL_DIVA_SHIFT;
		break;
	case 1:
		xclkdiv = omap_readl(ISP_TCTRL_CTRL) & ISPTCTRL_CTRL_DIVB_MASK;
		xclkdiv = xclkdiv >> ISPTCTRL_CTRL_DIVB_SHIFT;
		break;
	default:
		DPRINTK_ISPCTRL("ISP_ERR: isp_get_xclk(): Invalid requested "
						"xclk. Must be 0 (A) or 1 (B)."
						"\n");
		return -EINVAL;
	}

	switch (xclkdiv) {
	case 0:
	case 1:
		xclk = 0;
		break;
	case 0x1f:
		xclk = CM_CAM_MCLK_HZ;
		break;
	default:
		xclk = CM_CAM_MCLK_HZ / xclkdiv;
		break;
	}

	return xclk;
}
EXPORT_SYMBOL(isp_get_xclk);

/**
 * isp_power_settings - Sysconfig settings, for Power Management.
 * @isp_sysconfig: Structure containing the power settings for ISP to configure
 *
 * Sets the power settings for the ISP, and SBL bus.
 **/
void isp_power_settings(struct isp_sysc isp_sysconfig)
{
	if (isp_sysconfig.idle_mode) {
		omap_writel(ISP_SYSCONFIG_AUTOIDLE |
				(ISP_SYSCONFIG_MIDLEMODE_SMARTSTANDBY <<
				ISP_SYSCONFIG_MIDLEMODE_SHIFT),
				ISP_SYSCONFIG);

		omap_writel(ISPMMU_AUTOIDLE | (ISPMMU_SIDLEMODE_SMARTIDLE <<
						ISPMMU_SIDLEMODE_SHIFT),
						ISPMMU_SYSCONFIG);
		if (system_rev == OMAP3430_REV_ES1_0) {
			omap_writel(ISPCSI1_AUTOIDLE |
					(ISPCSI1_MIDLEMODE_SMARTSTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
					ISP_CSIA_SYSCONFIG);
			omap_writel(ISPCSI1_AUTOIDLE |
					(ISPCSI1_MIDLEMODE_SMARTSTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
					ISP_CSIB_SYSCONFIG);
		}
		omap_writel(ISPCTRL_SBL_AUTOIDLE, ISP_CTRL);

	} else {
		omap_writel(ISP_SYSCONFIG_AUTOIDLE |
				(ISP_SYSCONFIG_MIDLEMODE_FORCESTANDBY <<
				ISP_SYSCONFIG_MIDLEMODE_SHIFT),
				ISP_SYSCONFIG);

		omap_writel(ISPMMU_AUTOIDLE |
			(ISPMMU_SIDLEMODE_NOIDLE << ISPMMU_SIDLEMODE_SHIFT),
							ISPMMU_SYSCONFIG);
		if (system_rev == OMAP3430_REV_ES1_0) {
			omap_writel(ISPCSI1_AUTOIDLE |
					(ISPCSI1_MIDLEMODE_FORCESTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
					ISP_CSIA_SYSCONFIG);

			omap_writel(ISPCSI1_AUTOIDLE |
					(ISPCSI1_MIDLEMODE_FORCESTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
					ISP_CSIB_SYSCONFIG);
		}

		omap_writel(ISPCTRL_SBL_AUTOIDLE, ISP_CTRL);
	}
}
EXPORT_SYMBOL(isp_power_settings);

#define BIT_SET(var, shift, mask, val)		\
	do {					\
		var = (var & ~(mask << shift))	\
			| (val << shift);	\
	} while (0)

static int isp_init_csi(struct isp_interface_config *config)
{
	u32 i = 0, val, reg;
	int format;

	switch (config->u.csi.format) {
	case V4L2_PIX_FMT_SGRBG10:
		format = 0x16;		/* RAW10+VP */
		break;
	case V4L2_PIX_FMT_SGRBG10DPCM8:
		format = 0x12;		/* RAW8+DPCM10+VP */
		break;
	default:
		printk(KERN_ERR "isp_init_csi: bad csi format\n");
		return -EINVAL;
	}

	/* Reset the CSI and wait for reset to complete */
	omap_writel(omap_readl(ISPCSI1_SYSCONFIG) | BIT(1), ISPCSI1_SYSCONFIG);
	while (!(omap_readl(ISPCSI1_SYSSTATUS) & BIT(0))) {
		udelay(10);
		if (i++ > 10)
			break;
	}
	if (!(omap_readl(ISPCSI1_SYSSTATUS) & BIT(0))) {
		printk(KERN_WARNING
			"omap3_isp: timeout waiting for csi reset\n");
	}

	/* CONTROL_CSIRXFE */
	omap_writel(
		/* CSIb receiver data/clock or data/strobe mode */
		(config->u.csi.signalling << 10)
		| BIT(12)	/* Enable differential transceiver */
		| BIT(13)	/* Disable reset */
#ifdef TERM_RESISTOR
		| BIT(8)	/* Enable internal CSIb resistor (no effect) */
#endif
/*		| BIT(7) */	/* Strobe/clock inversion (no effect) */
	, OMAP3_CONTROL_CSIRXFE);

#ifdef TERM_RESISTOR
	/* Set CONTROL_CSI */
	val = omap_readl(OMAP3_CONTROL_CSI);
	val &= ~(0x1F<<16);
	val |= BIT(31) | (TERM_RESISTOR<<16);
	omap_writel(val, OMAP3_CONTROL_CSI);
#endif

	/* ISPCSI1_CTRL */
	val = omap_readl(ISPCSI1_CTRL);
	val &= ~BIT(11);	/* Enable VP only off ->
				extract embedded data to interconnect */
	BIT_SET(val, 8, 0x3, config->u.csi.vpclk);	/* Video port clock */
/*	val |= BIT(3);	*/	/* Wait for FEC before disabling interface */
	val |= BIT(2);		/* I/O cell output is parallel
				(no effect, but errata says should be enabled
				for class 1/2) */
	val |= BIT(12);		/* VP clock polarity to falling edge
				(needed or bad picture!) */

	/* Data/strobe physical layer */
	BIT_SET(val, 1, 1, config->u.csi.signalling);
	BIT_SET(val, 10, 1, config->u.csi.strobe_clock_inv);
	val |= BIT(4);		/* Magic bit to enable CSI1 and strobe mode */
	omap_writel(val, ISPCSI1_CTRL);

	/* ISPCSI1_LCx_CTRL logical channel #0 */
	reg = ISPCSI1_LCx_CTRL(0);	/* reg = ISPCSI1_CTRL1; */
	val = omap_readl(reg);
	/* Format = RAW10+VP or RAW8+DPCM10+VP*/
	BIT_SET(val, 3, 0x1f, format);
	/* Enable setting of frame regions of interest */
	BIT_SET(val, 1, 1, 1);
	BIT_SET(val, 2, 1, config->u.csi.crc);
	omap_writel(val, reg);

	/* ISPCSI1_DAT_START for logical channel #0 */
	reg = ISPCSI1_LCx_DAT_START(0);		/* reg = ISPCSI1_DAT_START; */
	val = omap_readl(reg);
	BIT_SET(val, 16, 0xfff, config->u.csi.data_start);
	omap_writel(val, reg);

	/* ISPCSI1_DAT_SIZE for logical channel #0 */
	reg = ISPCSI1_LCx_DAT_SIZE(0);		/* reg = ISPCSI1_DAT_SIZE; */
	val = omap_readl(reg);
	BIT_SET(val, 16, 0xfff, config->u.csi.data_size);
	omap_writel(val, reg);

	/* Clear status bits for logical channel #0 */
	omap_writel(0xFFF & ~BIT(6), ISPCSI1_LC01_IRQSTATUS);

	/* Enable CSI1 */
	val = omap_readl(ISPCSI1_CTRL);
	val |=  BIT(0) | BIT(4);
	omap_writel(val, ISPCSI1_CTRL);

	if (!(omap_readl(ISPCSI1_CTRL) & BIT(4))) {
		printk(KERN_WARNING "OMAP3 CSI1 bus not available\n");
		if (config->u.csi.signalling)	/* Strobe mode requires CSI1 */
			return -EIO;
	}

	return 0;
}

/**
 * isp_set_skipfrms - Store the component order as component offset.
 * @frm_skip: Input data component order.
 *
 * Turns the component order into a horizontal & vertical offset and store
 * offsets to be used later.
 **/
void isp_set_skipfrms(u8 frm_skip)
{
	ispmodule_obj.isp_skip_frms = frm_skip;
}


/**
 * isp_configure_interface - Configures ISP Control I/F related parameters.
 * @config: Pointer to structure containing the desired configuration for the
 * 	ISP.
 *
 * Configures ISP control register (ISP_CTRL) with the values specified inside
 * the config structure. Controls:
 * - Selection of parallel or serial input to the preview hardware.
 * - Data lane shifter.
 * - Pixel clock polarity.
 * - 8 to 16-bit bridge at the input of CCDC module.
 * - HS or VS synchronization signal detection
 **/
int isp_configure_interface(struct isp_interface_config *config)
{
	u32 ispctrl_val = omap_readl(ISP_CTRL);
	u32 ispccdc_vdint_val;
	int r;

	ispctrl_val &= ISPCTRL_SHIFT_MASK;
	ispctrl_val |= (config->dataline_shift << ISPCTRL_SHIFT_SHIFT);
	ispctrl_val &= ~ISPCTRL_PAR_CLK_POL_INV;

	ispctrl_val &= (ISPCTRL_PAR_SER_CLK_SEL_MASK);
	switch (config->ccdc_par_ser) {
	case ISP_PARLL:
	case ISP_PARLL_YUV_BT:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_PARALLEL;
		ispctrl_val |= (config->u.par.par_clk_pol
						<< ISPCTRL_PAR_CLK_POL_SHIFT);
		ispctrl_val &= ~ISPCTRL_PAR_BRIDGE_BENDIAN;
		ispctrl_val |= (config->u.par.par_bridge
						<< ISPCTRL_PAR_BRIDGE_SHIFT);
		break;
	case ISP_CSIA:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_CSIA;
		ispctrl_val &= ~ISPCTRL_PAR_BRIDGE_BENDIAN;
		ispctrl_val |= (0x03 << ISPCTRL_PAR_BRIDGE_SHIFT);

//		isp_csi2_ctx_config_format(0, config->u.csi.format);
//		isp_csi2_ctx_update(0, false);

		if (config->u.csi.crc)
			isp_csi2_ctrl_config_ecc_enable(true);

		isp_csi2_ctrl_config_vp_out_ctrl(config->u.csi.vpclk);
		isp_csi2_ctrl_config_vp_only_enable(true);
		isp_csi2_ctrl_config_vp_clk_enable(true);
		isp_csi2_ctrl_update(false);
		isp_csi2_ctx_config_format(0, config->u.csi.format);
		isp_csi2_ctx_update(0, false);

		isp_csi2_irq_complexio1_set(1);
		isp_csi2_irq_status_set(1);
		isp_csi2_irq_set(1);

		isp_csi2_enable(1);
		mdelay(3);
		break;
	case ISP_CSIB:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_CSIB;
		r = isp_init_csi(config);
		if (r)
			return r;
		break;
	default:
		return -EINVAL;
	}

	ispctrl_val &= ~(ISPCTRL_SYNC_DETECT_VSRISE);
	ispctrl_val |= (config->hsvs_syncdetect);
	spin_lock(&isp_obj.isp_temp_buf_lock);
	omap_writel(ispctrl_val, ISP_CTRL);
	spin_unlock(&isp_obj.isp_temp_buf_lock);
	ispccdc_vdint_val = omap_readl(ISPCCDC_VDINT);
	ispccdc_vdint_val &= ~(ISPCCDC_VDINT_0_MASK << ISPCCDC_VDINT_0_SHIFT);
	ispccdc_vdint_val &= ~(ISPCCDC_VDINT_1_MASK << ISPCCDC_VDINT_1_SHIFT);
	omap_writel((config->vdint0_timing << ISPCCDC_VDINT_0_SHIFT) |
						(config->vdint1_timing <<
						ISPCCDC_VDINT_1_SHIFT),
						ISPCCDC_VDINT);

	/* Set sensor specific fields in CCDC and Previewer module.*/
	ispccdc_set_wenlog(config->wenlog);
	ispccdc_set_dcsub(config->dcsub);
	ispccdc_set_crop_offset(config->raw_fmt_in);
/*	isppreview_set_whitebalance(&config->wbal); */ //MMS-L25.6E
	isp_set_skipfrms(config->frm_skip);

	return 0;
}
EXPORT_SYMBOL(isp_configure_interface);

/**
 * isp_configure_interface_bridge - Configure CCDC i/f bridge.
 *
 * Sets the bit field that controls the 8 to 16-bit bridge at
 * the input to CCDC.
 **/
int isp_configure_interface_bridge(u32 par_bridge)
{
	u32 ispctrl_val = omap_readl(ISP_CTRL);

	ispctrl_val &= ~ISPCTRL_PAR_BRIDGE_BENDIAN;
	ispctrl_val |= (par_bridge << ISPCTRL_PAR_BRIDGE_SHIFT);
	omap_writel(ispctrl_val, ISP_CTRL);
	return 0;
}
EXPORT_SYMBOL(isp_configure_interface_bridge);

/**
 * isp_CCDC_VD01_enable - Enables VD0 and VD1 IRQs.
 *
 * Sets VD0 and VD1 bits in IRQ0STATUS to reset the flag, and sets them in
 * IRQ0ENABLE to enable the corresponding IRQs.
 **/
void isp_CCDC_VD01_enable(void)
{
	u32 ispirq_val = omap_readl(ISP_IRQ0ENABLE) & ~(IRQ0ENABLE_CCDC_VD0_IRQ | IRQ0ENABLE_CCDC_VD1_IRQ);

	omap_writel(IRQ0STATUS_CCDC_VD0_IRQ | IRQ0STATUS_CCDC_VD1_IRQ,
				ISP_IRQ0STATUS);

	/* If We are using the a BT656 sensor, we have to configure
		VD0, and if we are using a RAW sensor, we will use VD1 */
	if (isp_obj.if_status & ISP_PARLL_YUV_BT)
		ispirq_val |= IRQ0ENABLE_CCDC_VD0_IRQ;
	else
		ispirq_val |= IRQ0ENABLE_CCDC_VD1_IRQ;

	omap_writel(ispirq_val, ISP_IRQ0ENABLE);
}

/**
 * isp_CCDC_VD01_disable - Disables VD0 and VD1 IRQs.
 *
 * Clears VD0 and VD1 bits in IRQ0ENABLE register.
 **/
void isp_CCDC_VD01_disable(void)
{
	omap_writel(omap_readl(ISP_IRQ0ENABLE) & ~(IRQ0ENABLE_CCDC_VD0_IRQ |
						IRQ0ENABLE_CCDC_VD1_IRQ),
						ISP_IRQ0ENABLE);
}

/**
 * omap34xx_isp_isr - Interrupt Service Routine for Camera ISP module.
 * @irq: Not used currently.
 * @ispirq_disp: Pointer to the object that is passed while request_irq is
 *               called. This is the ispirq_obj object containing info on the
 *               callback.
 *
 * Handles the corresponding callback if plugged in.
 *
 * Returns IRQ_HANDLED when IRQ was correctly handled, or IRQ_NONE when the
 * IRQ wasn't handled.
 **/
static irqreturn_t omap34xx_isp_isr(int irq, void *ispirq_disp)
{
	struct ispirq *irqdis = (struct ispirq *)ispirq_disp;
	u32 irqstatus = 0;
	unsigned long irqflags = 0;
	u8 is_irqhandled = 0;
	u32 irqenabled = 0;

	irqstatus = omap_readl(ISP_IRQ0STATUS);
	irqenabled = omap_readl(ISP_IRQ0ENABLE);
	ISP_REG_WR(irqstatus, ISP_IRQ0STATUS);

	spin_lock_irqsave(&isp_obj.lock, irqflags);
	irqstatus &= irqenabled;

	if (irqdis->isp_callbk[CBK_CATCHALL])
		irqdis->isp_callbk[CBK_CATCHALL](
			irqstatus,
			irqdis->isp_callbk_arg1[CBK_CATCHALL],
			irqdis->isp_callbk_arg2[CBK_CATCHALL]);

	if (irqstatus & MMU_ERR) {
		if (irqdis->isp_callbk[CBK_MMU_ERR])
			irqdis->isp_callbk[CBK_MMU_ERR](irqstatus,
				irqdis->isp_callbk_arg1[CBK_MMU_ERR],
				irqdis->isp_callbk_arg2[CBK_MMU_ERR]);
		is_irqhandled = 1;
		goto out;
	}

	if (irqstatus & HS_VS) {
		if (irqdis->isp_callbk[CBK_HS_VS])
			irqdis->isp_callbk[CBK_HS_VS](HS_VS,
				irqdis->isp_callbk_arg1[CBK_HS_VS],
				irqdis->isp_callbk_arg2[CBK_HS_VS]);
		is_irqhandled = 1;
	}

	if (irqstatus & CCDC_VD1) {
		if (irqdis->isp_callbk[CBK_CCDC_VD1])
				irqdis->isp_callbk[CBK_CCDC_VD1](CCDC_VD1,
				irqdis->isp_callbk_arg1[CBK_CCDC_VD1],
				irqdis->isp_callbk_arg2[CBK_CCDC_VD1]);
		is_irqhandled = 1;
	}

	if (irqstatus & CCDC_VD0) {
		if (irqdis->isp_callbk[CBK_CCDC_VD0])
			irqdis->isp_callbk[CBK_CCDC_VD0](CCDC_VD0,
				irqdis->isp_callbk_arg1[CBK_CCDC_VD0],
				irqdis->isp_callbk_arg2[CBK_CCDC_VD0]);
		is_irqhandled = 1;
	}

	if (irqstatus & PREV_DONE) {
		if (irqdis->isp_callbk[CBK_PREV_DONE])
			irqdis->isp_callbk[CBK_PREV_DONE](PREV_DONE,
				irqdis->isp_callbk_arg1[CBK_PREV_DONE],
				irqdis->isp_callbk_arg2[CBK_PREV_DONE]);
		is_irqhandled = 1;
	}

	if (irqstatus & RESZ_DONE) {
		if (irqdis->isp_callbk[CBK_RESZ_DONE])
			irqdis->isp_callbk[CBK_RESZ_DONE](RESZ_DONE,
				irqdis->isp_callbk_arg1[CBK_RESZ_DONE],
				irqdis->isp_callbk_arg2[CBK_RESZ_DONE]);
		is_irqhandled = 1;
	}

	if (irqstatus & H3A_AWB_DONE) {
		if (irqdis->isp_callbk[CBK_H3A_AWB_DONE])
			irqdis->isp_callbk[CBK_H3A_AWB_DONE](H3A_AWB_DONE,
				irqdis->isp_callbk_arg1[CBK_H3A_AWB_DONE],
				irqdis->isp_callbk_arg2[CBK_H3A_AWB_DONE]);
		is_irqhandled = 1;
	}

	if (irqstatus & HIST_DONE) {
		if (irqdis->isp_callbk[CBK_HIST_DONE])
			irqdis->isp_callbk[CBK_HIST_DONE](HIST_DONE,
				irqdis->isp_callbk_arg1[CBK_HIST_DONE],
				irqdis->isp_callbk_arg2[CBK_HIST_DONE]);
		is_irqhandled = 1;
	}

	if (irqstatus & H3A_AF_DONE) {
		if (irqdis->isp_callbk[CBK_H3A_AF_DONE])
			irqdis->isp_callbk[CBK_H3A_AF_DONE](H3A_AF_DONE,
				irqdis->isp_callbk_arg1[CBK_H3A_AF_DONE],
				irqdis->isp_callbk_arg2[CBK_H3A_AF_DONE]);
		is_irqhandled = 1;
	}

	if (irqstatus & CSIA) {
		isp_csi2_isr();
		is_irqhandled = 1;
	}

	if (irqstatus & LSC_PRE_ERR) {
		printk(KERN_ERR "isp_isr: LSC_PRE_ERR\n");
		omap_writel(LSC_PRE_ERR, ISP_IRQ0STATUS);
		ispccdc_enable_lsc(0);
		ispccdc_enable_lsc(1);
		spin_unlock_irqrestore(&isp_obj.lock, irqflags);
		return IRQ_HANDLED;
	}

	if (irqstatus & IRQ0STATUS_CSIB_IRQ) {
		u32 ispcsi1_irqstatus;

		ispcsi1_irqstatus = omap_readl(ISPCSI1_LC01_IRQSTATUS);
		DPRINTK_ISPCTRL("%x\n", ispcsi1_irqstatus);
	}

out:
	if (isp_obj.if_status & ISP_PARLL_YUV_BT)
		omap_writel(irqstatus, ISP_IRQ0STATUS);
	spin_unlock_irqrestore(&isp_obj.lock, irqflags);

	if (is_irqhandled)
		return IRQ_HANDLED;
	else
		return IRQ_NONE;
}
/* Device name, needed for resource tracking layer */
struct device_driver camera_drv = {
	.name = "camera"
};

struct device camera_dev = {
	.driver = &camera_drv,
};

/**
 * omapisp_unset_callback - Unsets all the callbacks associated with ISP module
 **/
void omapisp_unset_callback()
{
	isp_unset_callback(CBK_HS_VS);

	if ((ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER) &&
						is_ispresizer_enabled())
		isp_unset_callback(CBK_RESZ_DONE);

	if ((ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW) &&
						is_isppreview_enabled())
		isp_unset_callback(CBK_PREV_DONE);

	if (ispmodule_obj.isp_pipeline & OMAP_ISP_CCDC) {
		isp_unset_callback(CBK_CCDC_VD0);
		isp_unset_callback(CBK_CCDC_VD1);
		isp_unset_callback(CBK_LSC_ISR);
	}
	omap_writel(omap_readl(ISP_IRQ0STATUS) | ISP_INT_CLR, ISP_IRQ0STATUS);
}

#if ISP_WORKAROUND
/**
 *  isp_buf_allocation - To allocate a 10MB memory
 *
 **/
u32 isp_buf_allocation(void)
{
	buff_addr = (void *) vmalloc(ISP_BUFFER_MAX_SIZE);

	if (!buff_addr) {
		printk(KERN_ERR "ISP_ERR: Cannot allocate "
			"ISP_WORKAROUND memory\n");
		return -ENOMEM;
	}

	sglist_alloc = videobuf_vmalloc_to_sg(buff_addr, ISP_BUFFER_MAX_PAGES);
	if (!sglist_alloc) {
		printk(KERN_ERR "ISP_ERR: videobuf_vmalloc_to_sg failed\n");
		return -ENOMEM;
	}
	num_sc = dma_map_sg(NULL, sglist_alloc, ISP_BUFFER_MAX_PAGES, 1);
	return 0;
}

/**
 *  isp_buf_mmap - Create MMU scatter-gather list.
 *
 **/
u32 isp_buf_mmap(void)
{
	buff_addr_mapped = ispmmu_map_sg(sglist_alloc, ISP_BUFFER_MAX_PAGES);
	if (!buff_addr_mapped) {
		printk(KERN_ERR "ISP_ERR: ispmmu_map_sg mapping failed\n");
		return -ENOMEM;
	}
	isppreview_set_outaddr(buff_addr_mapped);
	alloc_done = 1;
	return 0;
}

/**
 *  isp_buf_get - Get the buffer pointer address
 **/
dma_addr_t isp_buf_get(void)
{
	dma_addr_t retaddr;

	if (alloc_done == 1)
		retaddr = buff_addr_mapped + offset_value;
	else
		retaddr = 0;
	return retaddr;
}

/**
 *  isp_buf_free - To free allocated 10MB memory
 *
 **/
void isp_buf_free(void)
{
	if (alloc_done == 1) {
		ispmmu_unmap(buff_addr_mapped);
		dma_unmap_sg(NULL, sglist_alloc, ISP_BUFFER_MAX_PAGES, 1);
		kfree(sglist_alloc);
		vfree(buff_addr);
		alloc_done = 0;
	}
}
#endif

/**
 * isp_start - Starts ISP submodule
 *
 * Start the needed isp components assuming these components
 * are configured correctly.
 **/
void isp_start(void)
{
	ISP_REG_WR(0xffffffff, ISP_IRQ0STATUS);

	if ((ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW) &&
						is_isppreview_enabled())
		isppreview_enable(1);

//HTC_CSP_START
	/* YUV sensor - resizer started in continues mode */
	if ( (ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER) &&
			is_ispresizer_enabled() &&
			(ispmodule_obj.isp_pipeline & OMAP_ISP_CCDC) &&
			(!(ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW)) ) {
		ispresizer_enable(1, 0);
	}
//HTC_CSP_END

	/* clear any pending IRQs */
	if (isp_obj.if_status & ISP_PARLL_YUV_BT) {
		spin_lock(&isp_obj.isp_temp_buf_lock);
		omap_writel(0xFFFFFFFF, ISP_IRQ0STATUS);
		spin_unlock(&isp_obj.isp_temp_buf_lock);
	}
	return;
}
EXPORT_SYMBOL(isp_start);

/**
 * isp_stop - Stops isp submodules
 **/
void isp_stop()
{
	int timeout = 0;

	spin_lock(&isp_obj.isp_temp_buf_lock);
	ispmodule_obj.isp_temp_state = ISP_FREE_RUNNING;
	spin_unlock(&isp_obj.isp_temp_buf_lock);

	omapisp_unset_callback();
	ispccdc_enable_lsc(0);
	ispccdc_enable(0);
	while (ispccdc_busy() && (timeout < 100)) {
		timeout++;
		mdelay(10);
	}

	timeout = 0;
	isppreview_enable(0);
	while (isppreview_busy() && (timeout < 100)) {
		timeout++;
		mdelay(10);
	}

	timeout = 0;

//HTC_CSP_START
	if (ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW) {
		ispresizer_enable(0, 1);
	} else {
		ispresizer_enable(0, 0);
	}
//HTC_CSP_END
	while (ispresizer_busy() && (timeout < 100)) {
		timeout++;
		mdelay(10);
	}

	timeout = 0;
	isp_save_ctx();
	spin_lock(&isp_obj.isp_temp_buf_lock);
	omap_writel(omap_readl(ISP_SYSCONFIG) |
		ISP_SYSCONFIG_SOFTRESET, ISP_SYSCONFIG);
	spin_unlock(&isp_obj.isp_temp_buf_lock);
	while (!(omap_readl(ISP_SYSSTATUS) & 0x1)) {
		timeout++;
		if (timeout >= 10) {
			printk(KERN_ALERT "isp.c: cannot reset ISP\n");
			return;
		}
		msleep(1);
	}
	isp_restore_ctx();
}

/**
 * isp_set_buf - Sets output address for submodules.
 * @sgdma_state: Pointer to structure with the SGDMA state for each videobuffer
 **/
void isp_set_buf(struct isp_sgdma_state *sgdma_state)
{
	if ((ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER) &&
						is_ispresizer_enabled())
		ispresizer_set_outaddr(sgdma_state->isp_addr);
#if (ISP_WORKAROUND == 0)
	else if ((ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW) &&
						is_isppreview_enabled())
		isppreview_set_outaddr(sgdma_state->isp_addr);
#endif
	else if (ispmodule_obj.isp_pipeline & OMAP_ISP_CCDC)
		ispccdc_set_outaddr(sgdma_state->isp_addr);

}

/**
 * isp_calc_pipeline - Sets pipeline depending of input and output pixel format
 * @pix_input: Pointer to V4L2 pixel format structure for input image.
 * @pix_output: Pointer to V4L2 pixel format structure for output image.
 **/
u32 isp_calc_pipeline(struct v4l2_pix_format *pix_input,
					struct v4l2_pix_format *pix_output)
{
#if ISP_WORKAROUND
	int rval;
#endif

	isp_release_resources();
	if ((pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10) &&
		((pix_output->pixelformat == V4L2_PIX_FMT_YUYV) ||
		(pix_output->pixelformat == V4L2_PIX_FMT_UYVY))) {
			ispmodule_obj.isp_pipeline = OMAP_ISP_CCDC |
					OMAP_ISP_PREVIEW | OMAP_ISP_RESIZER;
			ispccdc_request();
			isppreview_request();
			ispresizer_request();
		ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_VP);
#if ISP_WORKAROUND
		isppreview_config_datapath(PRV_RAW_CCDC, PREVIEW_MEM);
		ispresizer_config_datapath(RSZ_MEM_YUV);
		if (alloc_done == 0) {
#if !defined(CONFIG_VIDEO_OMAP3_BUFFALLOC)
			rval = isp_buf_allocation();
			if (rval)
				return -EINVAL;
#endif
			rval = isp_buf_mmap();
			if (rval)
				return -EINVAL;
		}
#else
		isppreview_config_datapath(PRV_RAW_CCDC, PREVIEW_RSZ);
		ispresizer_config_datapath(RSZ_OTFLY_YUV);
#endif
	} else if ((pix_input->pixelformat == pix_output->pixelformat)) {
		ispmodule_obj.isp_pipeline = OMAP_ISP_CCDC;
		ispccdc_request();
		if (pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10)
			ispccdc_config_datapath(CCDC_RAW, CCDC_OTHERS_VP_MEM);
		else if (pix_input->pixelformat == V4L2_PIX_FMT_PATT) {
			/* MMS */
			ispccdc_config_datapath(CCDC_RAW_PATTERN,
							CCDC_OTHERS_LSC_MEM);
		} else if ((pix_input->pixelformat == V4L2_PIX_FMT_YUYV) ||
				(pix_input->pixelformat == V4L2_PIX_FMT_UYVY)) {
			if (isp_obj.if_status & ISP_PARLL_YUV_BT)
				ispccdc_config_datapath(CCDC_YUV_BT,
						CCDC_OTHERS_MEM);
			else {
				//HTC_CSP_START
//				ispccdc_config_datapath(CCDC_YUV_SYNC, CCDC_OTHERS_MEM);
				ispmodule_obj.isp_pipeline |= OMAP_ISP_RESIZER;
				ispresizer_request();
				ispresizer_config_datapath(RSZ_OTFLY_YUV);
				ispccdc_config_datapath(CCDC_YUV_SYNC, CCDC_YUV_RSZ);
				//HTC_CSP_END
			}
		} else
			return -EINVAL;
	} else
		return -EINVAL;
	return 0;
}

/**
 * isp_config_pipeline - Configures the image size and ycpos for ISP submodules
 * @pix_input: Pointer to V4L2 pixel format structure for input image.
 * @pix_output: Pointer to V4L2 pixel format structure for output image.
 *
 * The configuration of ycpos depends on the output pixel format for both the
 * Preview and Resizer submodules.
 **/
void isp_config_pipeline(struct v4l2_pix_format *pix_input,
					struct v4l2_pix_format *pix_output)
{
	ispccdc_config_size(ispmodule_obj.ccdc_input_width,
			ispmodule_obj.ccdc_input_height,
			ispmodule_obj.ccdc_output_width,
			ispmodule_obj.ccdc_output_height);

	if (ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW)
		isppreview_config_size(ispmodule_obj.preview_input_width,
			ispmodule_obj.preview_input_height,
			ispmodule_obj.preview_output_width,
			ispmodule_obj.preview_output_height);

	if (ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER)
		ispresizer_config_size(ispmodule_obj.resizer_input_width,
			ispmodule_obj.resizer_input_height,
			ispmodule_obj.resizer_output_width,
			ispmodule_obj.resizer_output_height);

	if (pix_input->pixelformat == V4L2_PIX_FMT_UYVY)
		ispccdc_config_y8pos(Y8POS_ODD);
	else if (pix_input->pixelformat == V4L2_PIX_FMT_YUYV)
		ispccdc_config_y8pos(Y8POS_EVEN);

	if (((pix_input->pixelformat == V4L2_PIX_FMT_UYVY) &&
			(pix_output->pixelformat == V4L2_PIX_FMT_UYVY)) ||
			((pix_input->pixelformat == V4L2_PIX_FMT_YUYV) &&
			(pix_output->pixelformat == V4L2_PIX_FMT_YUYV)))
		/* input and output formats are in same order */
		ispccdc_config_byteswap(0);
	else if (((pix_input->pixelformat == V4L2_PIX_FMT_YUYV) &&
			(pix_output->pixelformat == V4L2_PIX_FMT_UYVY)) ||
			((pix_input->pixelformat == V4L2_PIX_FMT_UYVY) &&
			(pix_output->pixelformat == V4L2_PIX_FMT_YUYV)))
		/* input and output formats are in reverse order */
		ispccdc_config_byteswap(1);

	/*
	 * Configure Pitch - This enables application to use a different pitch
	 * other than active pixels per line.
	 */
	if (isp_obj.if_status & ISP_PARLL_YUV_BT)
		ispccdc_config_outlineoffset(ispmodule_obj.pix.bytesperline,
						0, 0);

	if (pix_output->pixelformat == V4L2_PIX_FMT_UYVY) {
		isppreview_config_ycpos(YCPOS_YCrYCb);
		if (is_ispresizer_enabled())
			ispresizer_config_ycpos(0);
	} else {
		isppreview_config_ycpos(YCPOS_CrYCbY);
		if (is_ispresizer_enabled())
			ispresizer_config_ycpos(1);
	}

	return;
}

/**
 * isp_vbq_done - Callback for interrupt completion
 * @status: IRQ0STATUS register value. Passed by the ISR, or the caller.
 * @arg1: Pointer to callback function for SG-DMA management.
 * @arg2: Pointer to videobuffer structure managed by ISP.
 **/
void isp_vbq_done(unsigned long status, isp_vbq_callback_ptr arg1, void *arg2)
{
	struct videobuf_buffer *vb = (struct videobuf_buffer *) arg2;
	int notify = 0;
	int rval = 0;
	unsigned long flags;
	int field = ((omap_readl(ISPCCDC_SYN_MODE) & ISPCCDC_SYN_MODE_FLDSTAT) == ISPCCDC_SYN_MODE_FLDSTAT) ? V4L2_FIELD_TOP : V4L2_FIELD_BOTTOM;
	int field_mode = ((omap_readl(ISPCCDC_SYN_MODE) & ISPCCDC_SYN_MODE_FLDMODE) == ISPCCDC_SYN_MODE_FLDMODE) ? 1 : 0;
	switch (status) {
	case CCDC_VD0:
		ispccdc_config_shadow_registers();
		if ((ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER) ||
			(ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW))
			return;
		else {
			spin_lock(&isp_obj.isp_temp_buf_lock);
#if !defined DEBUG_RECORD
			if (ispmodule_obj.isp_temp_state != ISP_BUF_INIT) {
				spin_unlock(&isp_obj.isp_temp_buf_lock);
				return;

			} else {
				spin_unlock(&isp_obj.isp_temp_buf_lock);
				break;
			}
#else
			/* For debub purpose only */
			/* re-inint buf state at the end of dma */
			ispmodule_obj.isp_temp_state = ISP_BUF_INIT;

			spin_unlock(&isp_obj.isp_temp_buf_lock);
			break;
#endif
		}
		break;
	case CCDC_VD1:
		if ((ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER) ||
			(ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW))
			return;

		if (ispmodule_obj.isp_skip_frms) {
			ispmodule_obj.isp_skip_frms--;
			return;
		}

		if (vd1_position == VD1_ON_FRAME_ENDING) {
			vd1_position = VD1_ON_FRAME_BEGINNING;
		} else {
			omap_writel((ispmodule_obj.ccdc_output_height - 2) <<
				ISPCCDC_VDINT_1_SHIFT,
				ISPCCDC_VDINT);
			spin_lock(&isp_obj.isp_temp_buf_lock);
			if ((ispmodule_obj.isp_temp_state == ISP_BUF_TRAN) ||
				(ispmodule_obj.isp_temp_state == ISP_FREE_RUNNING)) {
				ispmodule_obj.isp_temp_state = ISP_BUF_INIT;
			}
			if (ispmodule_obj.isp_temp_state == ISP_BUF_INIT) {
				spin_unlock(&isp_obj.isp_temp_buf_lock);
				ispccdc_enable(0);
				vd1_position = VD1_ON_FRAME_ENDING;
				return;
			}
			spin_unlock(&isp_obj.isp_temp_buf_lock);
		}
		break;
	case PREV_DONE:
		if (is_isppreview_enabled() == 0)
			break;
		if (ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER) {
			spin_lock(&isp_obj.isp_temp_buf_lock);
			if (!ispmodule_obj.applyCrop &&
						(ispmodule_obj.isp_temp_state == ISP_BUF_INIT))
                        //HTC_CSP_START
					ispresizer_enable(1, 1);

			spin_unlock(&isp_obj.isp_temp_buf_lock);
				if (ispmodule_obj.applyCrop && !ispresizer_busy()) {
					ispresizer_enable(0, 1);
                       //HTC_CSP_END
				ispresizer_applycrop();
				ispmodule_obj.applyCrop = 0;
			}
			if (!isppreview_busy())
			isppreview_config_shadow_registers();

			if (!isppreview_busy())
			isph3a_update_wb();
		}
		if (ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER)
			return;
		break;
	case RESZ_DONE:
		if (is_ispresizer_enabled() == 0)
			break;
		ispresizer_config_shadow_registers();
		spin_lock(&isp_obj.isp_temp_buf_lock);
		if (ispmodule_obj.isp_temp_state != ISP_BUF_INIT) {
			spin_unlock(&isp_obj.isp_temp_buf_lock);
			return;
		}
		spin_unlock(&isp_obj.isp_temp_buf_lock);
		break;
	case HS_VS:
		spin_lock(&isp_obj.isp_temp_buf_lock);

		/*
		 *  If we make RAW capture we must skip 1st frame. In almost sensors
		 *  the 1st frame used for shot is bad. Also we need syncronize the
		 *  ISP hw.
		 */
		if (ispmodule_obj.isp_skip_frms &&
				(ispmodule_obj.isp_pipeline == OMAP_ISP_CCDC)) {
			ispmodule_obj.isp_skip_frms--;
			spin_unlock(&isp_obj.isp_temp_buf_lock);
			return;
		}

		if (ispmodule_obj.isp_temp_state == ISP_BUF_TRAN) {
			isp_CCDC_VD01_enable();
			ispmodule_obj.isp_temp_state = ISP_BUF_INIT;
		}
		spin_unlock(&isp_obj.isp_temp_buf_lock);
		return;
	default:
		return;
	}

	spin_lock_irqsave(&ispsg.lock, flags);
	/* Skip even fields only in interlaced mode (TVP5146) */
	if( field_mode && field == V4L2_FIELD_TOP ) {
		spin_unlock_irqrestore(&ispsg.lock, flags);
		/* We should check if vdev->streaming is true, but we cannot call arg1(vb) since we don't want the state to be "done" */
		/*if(1)*/ isp_sgdma_process(&ispsg, 1, &notify, arg1);
	}
	else {
		ispsg.free_sgdma++;
		if (ispsg.free_sgdma > NUM_SG_DMA)
			ispsg.free_sgdma = NUM_SG_DMA;
		spin_unlock_irqrestore(&ispsg.lock, flags);
		rval = arg1(vb);

		if (rval)
			isp_sgdma_process(&ispsg, 1, &notify, arg1);		
	}

	return;
}

/**
 * isp_sgdma_init - Initializes Scatter Gather DMA status and operations.
 **/
void isp_sgdma_init()
{
	int sg;

	ispsg.free_sgdma = NUM_SG_DMA;
	ispsg.next_sgdma = 0;
	for (sg = 0; sg < NUM_SG_DMA; sg++) {
		ispsg.sg_state[sg].status = 0;
		ispsg.sg_state[sg].callback = NULL;
		ispsg.sg_state[sg].arg = NULL;
	}
}
EXPORT_SYMBOL(isp_stop);

/**
 * isp_vbq_sync - Walks the pages table and flushes the cache for
 *                each page.
 **/
int isp_vbq_sync(struct videobuf_buffer *vb)
{
	struct videobuf_dmabuf *vdma;
	u32 sg_element_addr;
	int i;

	vdma = videobuf_to_dma(vb);

	if (vdma->bus_addr)
		return 0;

	for (i = 0; i < vdma->sglen; i++) {
		sg_element_addr = sg_dma_address(vdma->sglist + i);
		/* Page align address */
		sg_element_addr &= ~(PAGE_SIZE-1);

		dma_sync_single_for_cpu(NULL, sg_element_addr,
				PAGE_SIZE, DMA_FROM_DEVICE);
	}
	return 0;
}

/**
 * isp_sgdma_process - Sets operations and config for specified SG DMA
 * @sgdma: SG-DMA function to work on.
 * @irq: Flag to specify if an IRQ is associated with the DMA completion.
 * @dma_notify: Pointer to flag that says when the ISP has to be started.
 * @func_ptr: Callback function pointer for SG-DMA setup.
 **/
void isp_sgdma_process(struct isp_sgdma *sgdma, int irq, int *dma_notify,
						isp_vbq_callback_ptr func_ptr)
{
	struct isp_sgdma_state *sgdma_state;
	unsigned long flags;
	spin_lock_irqsave(&sgdma->lock, flags);

	if (NUM_SG_DMA > sgdma->free_sgdma) {
		sgdma_state = sgdma->sg_state +
			(sgdma->next_sgdma + sgdma->free_sgdma) % NUM_SG_DMA;
		if (!irq) {
			if (*dma_notify) {
				isp_set_sgdma_callback(sgdma_state, func_ptr);
				isp_set_buf(sgdma_state);
				isp_start();
				isp_CCDC_VD01_enable();
				ispccdc_enable(1);
				*dma_notify = 0;
				spin_lock(&isp_obj.isp_temp_buf_lock);
				if (ispmodule_obj.isp_pipeline
					& OMAP_ISP_RESIZER) {
					ispmodule_obj.isp_temp_state =
						ISP_BUF_INIT;
				} else
					ispmodule_obj.isp_temp_state =
						ISP_BUF_TRAN;
				spin_unlock(&isp_obj.isp_temp_buf_lock);
			} else {
				spin_lock(&isp_obj.isp_temp_buf_lock);
				if (ispmodule_obj.isp_temp_state ==
							ISP_FREE_RUNNING) {
					isp_set_sgdma_callback(sgdma_state,
								func_ptr);
					isp_set_buf(sgdma_state);
					/* Non startup case */
					if (ispmodule_obj.isp_pipeline
					& OMAP_ISP_RESIZER) {
						ispmodule_obj.isp_temp_state =
							ISP_BUF_INIT;
					} else {
						ispmodule_obj.isp_temp_state =
							ISP_BUF_TRAN;
						isp_CCDC_VD01_enable();
						ispccdc_enable(1);
					}
				}
				spin_unlock(&isp_obj.isp_temp_buf_lock);
			}
		} else {
			isp_set_sgdma_callback(sgdma_state, func_ptr);
			isp_set_buf(sgdma_state);
			/* Non startup case */
			if (!(ispmodule_obj.isp_pipeline
				& OMAP_ISP_RESIZER)) {
				isp_CCDC_VD01_enable();
				ispccdc_enable(1);
			}
			if (*dma_notify) {
				isp_start();
				*dma_notify = 0;
			}
		}
	} else {
		spin_lock(&isp_obj.isp_temp_buf_lock);
		isp_CCDC_VD01_disable();
		if (!(ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER))
			ispccdc_enable(0);
		else {
                //HTC_CSP_START
			if (ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW)
				ispresizer_enable(0, 1);
		else
				{
				ispresizer_enable(0, 0);
				}
                //HTC_CSP_END
		}
		ispmodule_obj.isp_temp_state = ISP_FREE_RUNNING;
		spin_unlock(&isp_obj.isp_temp_buf_lock);
	}
	spin_unlock_irqrestore(&sgdma->lock, flags);
	return;
}

/**
 * isp_sgdma_queue - Queues a Scatter-Gather DMA videobuffer.
 * @vdma: Pointer to structure containing the desired DMA video buffer
 *        transfer parameters.
 * @vb: Pointer to structure containing the target videobuffer.
 * @irq: Flag to specify if an IRQ is associated with the DMA completion.
 * @dma_notify: Pointer to flag that says when the ISP has to be started.
 * @func_ptr: Callback function pointer for SG-DMA setup.
 *
 * Returns 0 if successful, -EINVAL if invalid SG linked list setup, or -EBUSY
 * if the ISP SG-DMA is not free.
 **/
int isp_sgdma_queue(struct videobuf_dmabuf *vdma, struct videobuf_buffer *vb,
						int irq, int *dma_notify,
						isp_vbq_callback_ptr func_ptr)
{
	unsigned long flags;
	struct isp_sgdma_state *sg_state;
	const struct scatterlist *sglist = vdma->sglist;
	int sglen = vdma->sglen;

	if ((sglen < 0) || ((sglen > 0) & !sglist))
		return -EINVAL;
	isp_vbq_sync(vb);

	spin_lock_irqsave(&ispsg.lock, flags);

	if (!ispsg.free_sgdma) {
		spin_unlock_irqrestore(&ispsg.lock, flags);
		return -EBUSY;
	}

	sg_state = ispsg.sg_state + ispsg.next_sgdma;
	sg_state->isp_addr = ispsg.isp_addr_capture[vb->i];
	sg_state->status = 0;
	sg_state->callback = isp_vbq_done;
	sg_state->arg = vb;

	ispsg.next_sgdma = (ispsg.next_sgdma + 1) % NUM_SG_DMA;
	ispsg.free_sgdma--;

	spin_unlock_irqrestore(&ispsg.lock, flags);

	isp_sgdma_process(&ispsg, irq, dma_notify, func_ptr);

	return 0;
}
EXPORT_SYMBOL(isp_sgdma_queue);

/**
 * isp_vbq_prepare - Videobuffer queue prepare.
 * @vbq: Pointer to videobuf_queue structure.
 * @vb: Pointer to videobuf_buffer structure.
 * @field: Requested Field order for the videobuffer.
 *
 * Returns 0 if successful, or -EIO if the ispmmu was unable to map a
 * scatter-gather linked list data space.
 **/
int isp_vbq_prepare(struct videobuf_queue *vbq, struct videobuf_buffer *vb,
							enum v4l2_field field)
{
	unsigned int isp_addr;
	struct videobuf_dmabuf	*vdma;
	struct vm_area_struct *vma;
	unsigned long first, last;

	int err = 0;

	vdma = videobuf_to_dma(vb);

	/* For kernel direct-mapped memory, take the easy way */
	if (vb->baddr >= PAGE_OFFSET) {
		vdma->bus_addr = virt_to_phys((void *)vb->baddr);
	} else if ((vma = find_vma(current->mm, vb->baddr)) && (vma->vm_flags & VM_IO) && (vma->vm_pgoff)) {
		/* this will catch, kernel-allocated,
			mmaped-to-usermode addresses */
		vdma->bus_addr = (vma->vm_pgoff << PAGE_SHIFT) + (vb->baddr - vma->vm_start);
	}

	if (vdma->bus_addr) {
		first = (vdma->bus_addr & PAGE_MASK)              >> PAGE_SHIFT;
		last  = ((vdma->bus_addr+vb->size-1) & PAGE_MASK) >> PAGE_SHIFT;
		vdma->direction = DMA_FROM_DEVICE;
		vdma->offset    = vdma->bus_addr & ~PAGE_MASK;
		vdma->nr_pages  = last-first+1;
		err = videobuf_dma_map(vbq, vdma);
		if (err)
			return err;
		isp_addr = ispmmu_map(vdma->bus_addr, vdma->nr_pages << PAGE_SHIFT);
	} else {
		err = videobuf_iolock(vbq, vb, NULL);
		if (err)
			return err;
		isp_addr = ispmmu_map_sg(vdma->sglist, vdma->sglen);
	}

	if (!isp_addr)
		err = -EIO;
	else
		ispsg.isp_addr_capture[vb->i] = isp_addr;

	return err;
}
EXPORT_SYMBOL(isp_vbq_prepare);

/**
 * isp_vbq_release - Videobuffer queue release.
 * @vbq: Pointer to videobuf_queue structure.
 * @vb: Pointer to videobuf_buffer structure.
 **/
void isp_vbq_release(struct videobuf_queue *vbq, struct videobuf_buffer *vb)
{
	if (ispsg.isp_addr_capture[vb->i]) {
		ispmmu_unmap(ispsg.isp_addr_capture[vb->i]);
		ispsg.isp_addr_capture[vb->i] = (dma_addr_t) NULL;
	}
	return;
}
EXPORT_SYMBOL(isp_vbq_release);

/**
 * isp_queryctrl - Query V4L2 control from existing controls in ISP.
 * @a: Pointer to v4l2_queryctrl structure. It only needs the id field filled.
 *
 * Returns 0 if successful, or -EINVAL if not found in ISP.
 **/
int isp_queryctrl(struct v4l2_queryctrl *a)
{
	int i;

	if (a->id & V4L2_CTRL_FLAG_NEXT_CTRL) {
		a->id &= ~V4L2_CTRL_FLAG_NEXT_CTRL;
		i = find_next_vctrl(a->id);
	} else {
		i = find_vctrl(a->id);
	}

	if (i < 0)
		return -EINVAL;

	*a = video_control[i].qc;
	return 0;
}
EXPORT_SYMBOL(isp_queryctrl);

/**
 * isp_g_ctrl - Gets value of the desired V4L2 control.
 * @a: V4L2 control to read actual value from.
 *
 * Return 0 if successful, or -EINVAL if chosen control is not found.
 **/
int isp_g_ctrl(struct v4l2_control *a)
{
	u8 current_value;
	int rval = 0;

	switch (a->id) {
	case V4L2_CID_BRIGHTNESS:
		isppreview_query_brightness(&current_value);
		a->value = current_value / ISPPRV_BRIGHT_UNITS;
		break;
	case V4L2_CID_CONTRAST:
		isppreview_query_contrast(&current_value);
		a->value = current_value / ISPPRV_CONTRAST_UNITS;
		break;
	case V4L2_CID_PRIVATE_ISP_COLOR_FX:
		isppreview_get_color(&current_value);
		a->value = current_value;
		break;
	default:
		rval = -EINVAL;
		break;
	}

	return rval;
}
EXPORT_SYMBOL(isp_g_ctrl);

/**
 * isp_s_ctrl - Sets value of the desired V4L2 control.
 * @a: V4L2 control to read actual value from.
 *
 * Return 0 if successful, -EINVAL if chosen control is not found or value
 * is out of bounds, -EFAULT if copy_from_user or copy_to_user operation fails
 * from camera abstraction layer related controls or the transfered user space
 * pointer via the value field is not set properly.
 **/
int isp_s_ctrl(struct v4l2_control *a)
{
	int rval = 0;
	u8 new_value = a->value;

	switch (a->id) {
	case V4L2_CID_BRIGHTNESS:
		if (new_value > ISPPRV_BRIGHT_HIGH)
			rval = -EINVAL;
		else
			isppreview_update_brightness(&new_value);
		break;
	case V4L2_CID_CONTRAST:
		if (new_value > ISPPRV_CONTRAST_HIGH)
			rval = -EINVAL;
		else
			isppreview_update_contrast(&new_value);
		break;
	case V4L2_CID_PRIVATE_ISP_COLOR_FX:
		if (new_value > PREV_BW_COLOR)
			rval = -EINVAL;
		else
			isppreview_set_color(&new_value);
		break;
	default:
		rval = -EINVAL;
		break;
	}

	return rval;
}
EXPORT_SYMBOL(isp_s_ctrl);

/**
 * isp_handle_private - Handle all private ioctls for isp module.
 * @cmd: ioctl cmd value
 * @arg: ioctl arg value
 *
 * Return 0 if successful, -EINVAL if chosen cmd value is not handled or value
 * is out of bounds, -EFAULT if ioctl arg value is not valid.
 * Function simply routes the input ioctl cmd id to the appropriate handler in
 * the isp module.
 **/
int isp_handle_private(int cmd, void *arg)
{
	int rval = 0;

	switch (cmd) {
	case VIDIOC_PRIVATE_ISP_CCDC_CFG:
		rval = omap34xx_isp_ccdc_config(arg);
		break;
	case VIDIOC_PRIVATE_ISP_PRV_CFG:
		rval = omap34xx_isp_preview_config(arg);
		break;
	case VIDIOC_PRIVATE_ISP_AEWB_CFG: {
		struct isph3a_aewb_config *params;
		params = (struct isph3a_aewb_config *) arg;
		rval = isph3a_aewb_configure(params);
		}
		break;
	case VIDIOC_PRIVATE_ISP_AEWB_REQ: {
		struct isph3a_aewb_data *data;
		data = (struct isph3a_aewb_data *) arg;
		rval = isph3a_aewb_request_statistics(data);
		}
		break;
	case VIDIOC_PRIVATE_ISP_HIST_CFG: {
		struct isp_hist_config *params;
		params = (struct isp_hist_config *) arg;
		rval = isp_hist_configure(params);
		}
		break;
	case VIDIOC_PRIVATE_ISP_HIST_REQ: {
		struct isp_hist_data *data;
		data = (struct isp_hist_data *) arg;
		rval = isp_hist_request_statistics(data);
		}
		break;
	case VIDIOC_PRIVATE_ISP_AF_CFG: {
		struct af_configuration *params;
		params = (struct af_configuration *) arg;
		rval = isp_af_configure(params);
		}
	break;
	case VIDIOC_PRIVATE_ISP_AF_REQ: {
		struct isp_af_data *data;
		data = (struct isp_af_data *) arg;
		rval = isp_af_request_statistics(data);
		}
	break;
	default:
		rval = -EINVAL;
		break;
	}
	return rval;
}
EXPORT_SYMBOL(isp_handle_private);

/**
 * isp_enum_fmt_cap - Gets more information of chosen format index and type
 * @f: Pointer to structure containing index and type of format to read from.
 *
 * Returns 0 if successful, or -EINVAL if format index or format type is
 * invalid.
 **/
int isp_enum_fmt_cap(struct v4l2_fmtdesc *f)
{
	int index = f->index;
	enum v4l2_buf_type type = f->type;
	int rval = -EINVAL;

	if (index >= NUM_ISP_CAPTURE_FORMATS)
		goto err;

	memset(f, 0, sizeof(*f));
	f->index = index;
	f->type = type;

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		rval = 0;
		break;
	default:
		goto err;
	}

	f->flags = isp_formats[index].flags;
	strncpy(f->description, isp_formats[index].description,
						sizeof(f->description));
	f->pixelformat = isp_formats[index].pixelformat;
err:
	return rval;
}
EXPORT_SYMBOL(isp_enum_fmt_cap);

/**
 * isp_g_fmt_cap - Gets current output image format.
 * @f: Pointer to V4L2 format structure to be filled with current output format
 **/
void isp_g_fmt_cap(struct v4l2_pix_format *pix)
{
	*pix = ispmodule_obj.pix;
	return;
}
EXPORT_SYMBOL(isp_g_fmt_cap);

/**
 * isp_s_fmt_cap - Sets I/O formats and crop and configures pipeline in ISP
 * @f: Pointer to V4L2 format structure to be filled with current output format
 *
 * Returns 0 if successful, or return value of either isp_try_size or
 * isp_try_fmt if there is an error.
 **/
int isp_s_fmt_cap(struct v4l2_pix_format *pix_input,
					struct v4l2_pix_format *pix_output)
{
	int crop_scaling_w = 0, crop_scaling_h = 0;
	int rval = 0;

	isp_calc_pipeline(pix_input, pix_output);
	rval = isp_try_size(pix_input, pix_output);

	if (rval)
		goto out;

	rval = isp_try_fmt(pix_input, pix_output);
	if (rval)
		goto out;

	if (ispcroprect.width != pix_output->width) {
		crop_scaling_w = 1;
		ispcroprect.left = 0;
		ispcroprect.width = pix_output->width;
	}

	if (ispcroprect.height != pix_output->height) {
		crop_scaling_h = 1;
		ispcroprect.top = 0;
		ispcroprect.height = pix_output->height;
	}

	isp_config_pipeline(pix_input, pix_output);

	if ((ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER) &&
		(crop_scaling_h || crop_scaling_w))
		isp_config_crop(pix_output);

out:
	return rval;
}
EXPORT_SYMBOL(isp_s_fmt_cap);

/**
 * isp_config_crop - Configures crop parameters in isp resizer.
 * @croppix: Pointer to V4L2 pixel format structure containing crop parameters
 **/
void isp_config_crop(struct v4l2_pix_format *croppix)
{
	u8 crop_scaling_w;
	u8 crop_scaling_h;
#if ISP_WORKAROUND
	unsigned long org_left, num_pix, new_top;
#endif

	struct v4l2_pix_format *pix = croppix;

	crop_scaling_w = (ispmodule_obj.preview_output_width * 10) /
								pix->width;
	crop_scaling_h = (ispmodule_obj.preview_output_height * 10) /
								pix->height;

	cur_rect.left = (ispcroprect.left * crop_scaling_w) / 10;
	cur_rect.top = (ispcroprect.top * crop_scaling_h) / 10;
	cur_rect.width = (ispcroprect.width * crop_scaling_w) / 10;
	cur_rect.height = (ispcroprect.height * crop_scaling_h) / 10;

#if ISP_WORKAROUND
	org_left = cur_rect.left;
	while (((int)cur_rect.left & 0xFFFFFFF0) != (int)cur_rect.left)
		(int)cur_rect.left--;

	num_pix = org_left - cur_rect.left;
	new_top = (int)(num_pix * 3) / 4;
	cur_rect.top = cur_rect.top - new_top;
	cur_rect.height = (2 * new_top) + cur_rect.height;

	cur_rect.width = cur_rect.width + (2 * num_pix);
	while (((int)cur_rect.width & 0xFFFFFFF0) != (int)cur_rect.width)
		(int)cur_rect.width--;

	offset_value = ((cur_rect.left * 2) + \
		((ispmodule_obj.preview_output_width) * 2 * cur_rect.top));
#endif

	ispresizer_trycrop(cur_rect.left, cur_rect.top, cur_rect.width,
					cur_rect.height,
					ispmodule_obj.resizer_output_width,
					ispmodule_obj.resizer_output_height);

	return;
}
EXPORT_SYMBOL(isp_config_crop);

/**
 * isp_g_crop - Gets crop rectangle size and position.
 * @a: Pointer to V4L2 crop structure to be filled.
 *
 * Always returns 0.
 **/
int isp_g_crop(struct v4l2_crop *a)
{
	struct v4l2_crop *crop = a;

	crop->c = ispcroprect;
	return 0;
}
EXPORT_SYMBOL(isp_g_crop);

/**
 * isp_s_crop - Sets crop rectangle size and position and queues crop operation
 * @a: Pointer to V4L2 crop structure with desired parameters.
 * @pix: Pointer to V4L2 pixel format structure with desired parameters.
 *
 * Returns 0 if successful, or -EINVAL if crop parameters are out of bounds.
 **/
int isp_s_crop(struct v4l2_crop *a, struct v4l2_pix_format *pix)
{
	struct v4l2_crop *crop = a;
	int rval = 0;

	if ((crop->c.left + crop->c.width) > pix->width) {
		rval = -EINVAL;
		goto out;
	}

	if ((crop->c.top + crop->c.height) > pix->height) {
		rval = -EINVAL;
		goto out;
	}

	ispcroprect.left = crop->c.left;
	ispcroprect.top = crop->c.top;
	ispcroprect.width = crop->c.width;
	ispcroprect.height = crop->c.height;

	isp_config_crop(pix);

	ispmodule_obj.applyCrop = 1;
out:
	return rval;
}
EXPORT_SYMBOL(isp_s_crop);

/**
 * isp_try_fmt_cap - Tries desired input/output image formats
 * @pix_input: Pointer to V4L2 pixel format structure for input image.
 * @pix_output: Pointer to V4L2 pixel format structure for output image.
 *
 * Returns 0 if successful, or return value of either isp_try_size or
 * isp_try_fmt if there is an error.
 **/
int isp_try_fmt_cap(struct v4l2_pix_format *pix_input,
					struct v4l2_pix_format *pix_output)
{
	int rval = 0;

	isp_calc_pipeline(pix_input, pix_output);
	rval = isp_try_size(pix_input, pix_output);

	if (rval)
		goto out;

	rval = isp_try_fmt(pix_input, pix_output);

	if (rval)
		goto out;

out:
	return rval;
}
EXPORT_SYMBOL(isp_try_fmt_cap);

/**
 * isp_try_size - Tries size configuration for I/O images of each ISP submodule
 * @pix_input: Pointer to V4L2 pixel format structure for input image.
 * @pix_output: Pointer to V4L2 pixel format structure for output image.
 *
 * Returns 0 if successful, or return value of ispccdc_try_size,
 * isppreview_try_size, or ispresizer_try_size (depending on the pipeline
 * configuration) if there is an error.
 **/
int isp_try_size(struct v4l2_pix_format *pix_input,
					struct v4l2_pix_format *pix_output)
{
	int rval = 0;

	if (pix_output->width <= ISPRSZ_MIN_OUTPUT ||
		pix_output->height <= ISPRSZ_MIN_OUTPUT)
		return -EINVAL;

	if (pix_output->width >= ISPRSZ_MAX_OUTPUT ||
		pix_output->height > ISPRSZ_MAX_OUTPUT)
		return -EINVAL;

	ispmodule_obj.ccdc_input_width = pix_input->width;
	ispmodule_obj.ccdc_input_height = pix_input->height;
	ispmodule_obj.resizer_output_width = pix_output->width;
	ispmodule_obj.resizer_output_height = pix_output->height;

	if (ispmodule_obj.isp_pipeline & OMAP_ISP_CCDC) {
		rval = ispccdc_try_size(ispmodule_obj.ccdc_input_width,
					ispmodule_obj.ccdc_input_height,
					&ispmodule_obj.ccdc_output_width,
					&ispmodule_obj.ccdc_output_height);
		if (rval) {
			printk(KERN_ERR "ISP_ERR: The dimensions %dx%d are not"
					" supported\n", pix_input->width,
					pix_input->height);
			return rval;
		}
		pix_output->width = ispmodule_obj.ccdc_output_width;
		pix_output->height = ispmodule_obj.ccdc_output_height;
	}

	if (ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW) {
		ispmodule_obj.preview_input_width =
					ispmodule_obj.ccdc_output_width;
		ispmodule_obj.preview_input_height =
					ispmodule_obj.ccdc_output_height;
		rval = isppreview_try_size(ispmodule_obj.preview_input_width,
					ispmodule_obj.preview_input_height,
					&ispmodule_obj.preview_output_width,
					&ispmodule_obj.preview_output_height);
		if (rval) {
			printk(KERN_ERR "ISP_ERR: The dimensions %dx%d are not"
					" supported\n", pix_input->width,
					pix_input->height);
			return rval;
		}
		pix_output->width = ispmodule_obj.preview_output_width;
		pix_output->height = ispmodule_obj.preview_output_height;
	}

	if (ispmodule_obj.isp_pipeline & OMAP_ISP_RESIZER) {
                //HTC_CSP_START
		if (ispmodule_obj.isp_pipeline & OMAP_ISP_PREVIEW) {
		ispmodule_obj.resizer_input_width =
					ispmodule_obj.preview_output_width;
		ispmodule_obj.resizer_input_height =
					ispmodule_obj.preview_output_height;
		} else {
			ispmodule_obj.resizer_input_width =
					ispmodule_obj.ccdc_output_width;
			ispmodule_obj.resizer_input_height =
					ispmodule_obj.ccdc_output_height;
		}
                //HTC_CSP_END
		rval = ispresizer_try_size(&ispmodule_obj.resizer_input_width,
					&ispmodule_obj.resizer_input_height,
					&ispmodule_obj.resizer_output_width,
					&ispmodule_obj.resizer_output_height);
		if (rval) {
			printk(KERN_ERR "ISP_ERR: The dimensions %dx%d are not"
					" supported\n", pix_input->width,
					pix_input->height);
			return rval;
		}
		pix_output->width = ispmodule_obj.resizer_output_width;
		pix_output->height = ispmodule_obj.resizer_output_height;
	}

	return rval;
}
EXPORT_SYMBOL(isp_try_size);

/**
 * isp_try_fmt - Validates input/output format parameters.
 * @pix_input: Pointer to V4L2 pixel format structure for input image.
 * @pix_output: Pointer to V4L2 pixel format structure for output image.
 *
 * Always returns 0.
 **/
int isp_try_fmt(struct v4l2_pix_format *pix_input,
					struct v4l2_pix_format *pix_output)
{
	int ifmt;

	for (ifmt = 0; ifmt < NUM_ISP_CAPTURE_FORMATS; ifmt++) {
		if (pix_output->pixelformat == isp_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_ISP_CAPTURE_FORMATS)
		ifmt = 1;
	pix_output->pixelformat = isp_formats[ifmt].pixelformat;

	if (isp_obj.if_status & ISP_PARLL_YUV_BT)
		pix_output->field = pix_input->field;
	else {
		pix_output->field = V4L2_FIELD_NONE;
		pix_output->bytesperline =
			pix_output->width * ISP_BYTES_PER_PIXEL;
	}

	pix_output->sizeimage =
		PAGE_ALIGN(pix_output->bytesperline * pix_output->height);
	pix_output->priv = 0;
	switch (pix_output->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		if (isp_obj.if_status & ISP_PARLL_YUV_BT)
			pix_output->colorspace = pix_input->colorspace;
		else
			pix_output->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	default:
		pix_output->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	}

	ispmodule_obj.pix.pixelformat = pix_output->pixelformat;
	ispmodule_obj.pix.width = pix_output->width;
	ispmodule_obj.pix.height = pix_output->height;
	ispmodule_obj.pix.field = pix_output->field;
	ispmodule_obj.pix.bytesperline = pix_output->bytesperline;
	ispmodule_obj.pix.sizeimage = pix_output->sizeimage;
	ispmodule_obj.pix.priv = pix_output->priv;
	ispmodule_obj.pix.colorspace = pix_output->colorspace;

	return 0;
}
EXPORT_SYMBOL(isp_try_fmt);

/**
 * isp_save_ctx - Saves ISP, CCDC, HIST, H3A, PREV, RESZ & MMU context.
 *
 * Routine for saving the context of each module in the ISP.
 * CCDC, HIST, H3A, PREV, RESZ and MMU.
 **/
void isp_save_ctx(void)
{
	isp_save_context(isp_reg_list);
	ispccdc_save_context();
	ispmmu_save_context();
	isphist_save_context();
	isph3a_save_context();
	isppreview_save_context();
	ispresizer_save_context();
}
EXPORT_SYMBOL(isp_save_ctx);

/**
 * isp_restore_ctx - Restores ISP, CCDC, HIST, H3A, PREV, RESZ & MMU context.
 *
 * Routine for restoring the context of each module in the ISP.
 * CCDC, HIST, H3A, PREV, RESZ and MMU.
 **/
void isp_restore_ctx(void)
{
	isp_restore_context(isp_reg_list);
	ispccdc_restore_context();
	ispmmu_restore_context();
	isphist_restore_context();
	isph3a_restore_context();
	isppreview_restore_context();
	ispresizer_restore_context();
}
EXPORT_SYMBOL(isp_restore_ctx);

/**
 * isp_get - Adquires the ISP resource.
 *
 * Initializes the clocks for the first acquire.
 **/
int isp_get(void)
{
	int ret_err = 0;
	DPRINTK_ISPCTRL("isp_get: old %d\n", isp_obj.ref_count);
	mutex_lock(&(isp_obj.isp_mutex));
	if (isp_obj.ref_count == 0) {
		isp_obj.cam_ick = clk_get(&camera_dev, "cam_ick");
		if (IS_ERR(isp_obj.cam_ick)) {
			DPRINTK_ISPCTRL("ISP_ERR: clk_get for "
							"cam_ick failed\n");
			ret_err = PTR_ERR(isp_obj.cam_ick);
			goto out_clk_get_ick;
		}
		isp_obj.cam_mclk = clk_get(&camera_dev, "cam_mclk");
		if (IS_ERR(isp_obj.cam_mclk)) {
			DPRINTK_ISPCTRL("ISP_ERR: clk_get for "
							"cam_mclk failed\n");
			ret_err = PTR_ERR(isp_obj.cam_mclk);
			goto out_clk_get_mclk;
		}
		isp_obj.csi2_fck = clk_get(&camera_dev, "csi2_96m_fck");
		if (IS_ERR(isp_obj.csi2_fck)) {
			DPRINTK_ISPCTRL("ISP_ERR: clk_get for csi2_fclk"
								" failed\n");
			ret_err = PTR_ERR(isp_obj.csi2_fck);
			goto out_clk_get_csi2_fclk;
		}
		ret_err = clk_enable(isp_obj.cam_ick);
		if (ret_err) {
			DPRINTK_ISPCTRL("ISP_ERR: clk_en for ick failed\n");
			goto out_clk_enable_ick;
		}
		ret_err = clk_enable(isp_obj.cam_mclk);
		if (ret_err) {
			DPRINTK_ISPCTRL("ISP_ERR: clk_en for mclk failed\n");
			goto out_clk_enable_mclk;
		}
		ret_err = clk_enable(isp_obj.csi2_fck);
		if (ret_err) {
			DPRINTK_ISPCTRL("ISP_ERR: clk_en for csi2_fclk"
								" failed\n");
			goto out_clk_enable_csi2_fclk;
		}
		if (off_mode == 1)
			isp_restore_ctx();
	}
	isp_obj.ref_count++;
	mutex_unlock(&(isp_obj.isp_mutex));


	DPRINTK_ISPCTRL("isp_get: new %d\n", isp_obj.ref_count);
	return isp_obj.ref_count;

out_clk_enable_csi2_fclk:
	clk_disable(isp_obj.cam_mclk);
out_clk_enable_mclk:
	clk_disable(isp_obj.cam_ick);
out_clk_enable_ick:
	clk_put(isp_obj.csi2_fck);
out_clk_get_csi2_fclk:
	clk_put(isp_obj.cam_mclk);
out_clk_get_mclk:
	clk_put(isp_obj.cam_ick);
out_clk_get_ick:

	mutex_unlock(&(isp_obj.isp_mutex));

	return ret_err;
}
EXPORT_SYMBOL(isp_get);

/**
 * isp_put - Releases the ISP resource.
 *
 * Releases the clocks also for the last release.
 **/
int isp_put(void)
{
	DPRINTK_ISPCTRL("isp_put: old %d\n", isp_obj.ref_count);
	mutex_lock(&(isp_obj.isp_mutex));
	if (isp_obj.ref_count)
		if (--isp_obj.ref_count == 0) {
			isp_save_ctx();
			off_mode = 1;
#if ISP_WORKAROUND && !defined(CONFIG_VIDEO_OMAP3_BUFFALLOC)
			isp_buf_free();
#endif
			isp_release_resources();
			ispmodule_obj.isp_pipeline = 0;
			clk_disable(isp_obj.cam_ick);
			clk_disable(isp_obj.cam_mclk);
			clk_disable(isp_obj.csi2_fck);
			clk_put(isp_obj.cam_ick);
			clk_put(isp_obj.cam_mclk);
			clk_put(isp_obj.csi2_fck);
			memset(&ispcroprect, 0, sizeof(ispcroprect));
			memset(&cur_rect, 0, sizeof(cur_rect));
		}
	mutex_unlock(&(isp_obj.isp_mutex));
	DPRINTK_ISPCTRL("isp_put: new %d\n", isp_obj.ref_count);
	return isp_obj.ref_count;
}
EXPORT_SYMBOL(isp_put);

/**
 * isp_save_context - Saves the values of the ISP module registers.
 * @reg_list: Structure containing pairs of register address and value to
 *            modify on OMAP.
 **/
void isp_save_context(struct isp_reg *reg_list)
{
	struct isp_reg *next = reg_list;

	for (; next->reg != ISP_TOK_TERM; next++)
		next->val = omap_readl(next->reg);
}
EXPORT_SYMBOL(isp_save_context);

/**
 * isp_restore_context - Restores the values of the ISP module registers.
 * @reg_list: Structure containing pairs of register address and value to
 *            modify on OMAP.
 **/
void isp_restore_context(struct isp_reg *reg_list)
{
	struct isp_reg *next = reg_list;

	for (; next->reg != ISP_TOK_TERM; next++)
		omap_writel(next->val, next->reg);
}
EXPORT_SYMBOL(isp_restore_context);

/**
 * isp_init - ISP module initialization.
 **/
static int __init isp_init(void)
{
#if ISP_WORKAROUND && defined(CONFIG_VIDEO_OMAP3_BUFFALLOC)
	int rval;
#endif
	DPRINTK_ISPCTRL("+isp_init for Omap 3430 Camera ISP\n");
	isp_obj.ref_count = 0;

	mutex_init(&(isp_obj.isp_mutex));
	spin_lock_init(&isp_obj.isp_temp_buf_lock);
	spin_lock_init(&isp_obj.lock);

#if ISP_WORKAROUND && defined(CONFIG_VIDEO_OMAP3_BUFFALLOC)
	if (alloc_done == 0) {
		rval = isp_buf_allocation();
		if (rval)
			return -EINVAL;
	}
#endif

	if (request_irq(INT_34XX_CAM_IRQ, omap34xx_isp_isr, IRQF_SHARED,
				"Omap 34xx Camera ISP", &ispirq_obj)) {
		DPRINTK_ISPCTRL("Could not install ISR\n");
		return -EINVAL;
	}

	isp_ccdc_init();
	isp_hist_init();
	isph3a_aewb_init();
	ispmmu_init();
	isp_preview_init();
	isp_resizer_init();
	isp_af_init();

	DPRINTK_ISPCTRL("-isp_init for Omap 3430 Camera ISP\n");
	return 0;
}
EXPORT_SYMBOL(isp_sgdma_init);

/**
 * isp_cleanup - ISP module cleanup.
 **/
static void __exit isp_cleanup(void)
{
	isp_af_exit();
	isp_resizer_cleanup();
	isp_preview_cleanup();
	ispmmu_cleanup();
	isph3a_aewb_cleanup();
	isp_hist_cleanup();
	isp_ccdc_cleanup();
	free_irq(INT_34XX_CAM_IRQ, &ispirq_obj);
#if ISP_WORKAROUND && defined(CONFIG_VIDEO_OMAP3_BUFFALLOC)
	isp_buf_free();
#endif
}

/**
 * isp_print_status - Prints the values of the ISP Control Module registers
 *
 * Also prints other debug information stored in the ISP module structure.
 **/
void isp_print_status(void)
{
	if (!is_ispctrl_debug_enabled())
		return;

	DPRINTK_ISPCTRL("###CM_FCLKEN_CAM=0x%x\n",
					omap_readl(OMAP3_CM_FCLKEN_CAM));
	DPRINTK_ISPCTRL("###CM_ICLKEN_CAM=0x%x\n",
					omap_readl(OMAP3_CM_ICLKEN_CAM));
	DPRINTK_ISPCTRL("###CM_CLKSEL_CAM=0x%x\n",
					omap_readl(OMAP3_CM_CLKSEL_CAM));
	DPRINTK_ISPCTRL("###CM_AUTOIDLE_CAM=0x%x\n",
					omap_readl(OMAP3_CM_AUTOIDLE_CAM));
	DPRINTK_ISPCTRL("###CM_CLKEN_PLL[18:16] should be 0x7, = 0x%x\n",
					omap_readl(OMAP3_CM_CLKEN_PLL));
	DPRINTK_ISPCTRL("###CM_CLKSEL2_PLL[18:8] should be 0x2D, [6:0] should "
			"be 1 = 0x%x\n", omap_readl(OMAP3_CM_CLKSEL2_PLL));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_HS=0x%x\n",
					omap_readl(CTRL_PADCONF_CAM_HS));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_XCLKA=0x%x\n",
					omap_readl(CTRL_PADCONF_CAM_XCLKA));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_D1=0x%x\n",
					omap_readl(CTRL_PADCONF_CAM_D1));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_D3=0x%x\n",
					omap_readl(CTRL_PADCONF_CAM_D3));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_D5=0x%x\n",
					omap_readl(CTRL_PADCONF_CAM_D5));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_D7=0x%x\n",
					omap_readl(CTRL_PADCONF_CAM_D7));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_D9=0x%x\n",
					omap_readl(CTRL_PADCONF_CAM_D9));
	DPRINTK_ISPCTRL("###CTRL_PADCONF_CAM_D11=0x%x\n",
					omap_readl(CTRL_PADCONF_CAM_D11));
}
EXPORT_SYMBOL(isp_print_status);

module_init(isp_init);
module_exit(isp_cleanup);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("ISP Control Module Library");
MODULE_LICENSE("GPL");
