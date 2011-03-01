/*
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * Implements lower edge channel class library functions.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

#include <dspbridge/cfg.h>
#include <dspbridge/drv.h>
#include <dspbridge/dev.h>

#include <dspbridge/dbg.h>

#include "_tiomap.h"
#include "_tiomap_pwr.h"

#ifndef CONFIG_DISABLE_BRIDGE_PM
#ifndef CONFIG_DISABLE_BRIDGE_DVFS
#ifndef CONFIG_OMAP3_PM
#include <mach/omap-pm.h>
#else
#include <mach/resource.h>
extern struct constraint_handle *dsp_constraint_handle;
#endif
#endif
#endif

extern struct MAILBOX_CONTEXT mboxsetting;

DSP_STATUS CHNLSM_EnableInterrupt(struct WMD_DEV_CONTEXT *pDevContext)
{
	DSP_STATUS status = DSP_SOK;
	u32 numMbxMsg;
	u32 mbxValue;
	struct CFG_HOSTRES resources;
	u32 devType;
	struct IO_MGR *hIOMgr;

	DBG_Trace(DBG_ENTER, "CHNLSM_EnableInterrupt(0x%x)\n", pDevContext);

	/* Read the messages in the mailbox until the message queue is empty */

	CFG_GetHostResources((struct CFG_DEVNODE *)DRV_GetFirstDevExtension(),
			     &resources);
	DEV_GetDevType(pDevContext->hDevObject, &devType);
	status = DEV_GetIOMgr(pDevContext->hDevObject, &hIOMgr);
	if (devType == DSP_UNIT) {
		HW_MBOX_NumMsgGet(resources.dwMboxBase,
				  MBOX_DSP2ARM, &numMbxMsg);
		while (numMbxMsg != 0) {
			HW_MBOX_MsgRead(resources.dwMboxBase,
					MBOX_DSP2ARM,
					&mbxValue);
			numMbxMsg--;
		}
		/* clear the DSP mailbox as well...*/
		HW_MBOX_NumMsgGet(resources.dwMboxBase,
				  MBOX_ARM2DSP, &numMbxMsg);
		while (numMbxMsg != 0) {
			HW_MBOX_MsgRead(resources.dwMboxBase,
					MBOX_ARM2DSP, &mbxValue);
			numMbxMsg--;
			udelay(10);

			HW_MBOX_EventAck(resources.dwMboxBase, MBOX_ARM2DSP,
					 HW_MBOX_U1_DSP1,
					 HW_MBOX_INT_NEW_MSG);
		}
		/* Enable the new message events on this IRQ line */
		HW_MBOX_EventEnable(resources.dwMboxBase,
				    MBOX_DSP2ARM,
				    MBOX_ARM,
				    HW_MBOX_INT_NEW_MSG);
	}

	return status;
}

DSP_STATUS CHNLSM_DisableInterrupt(struct WMD_DEV_CONTEXT *pDevContext)
{
	struct CFG_HOSTRES resources;

	DBG_Trace(DBG_ENTER, "CHNLSM_DisableInterrupt(0x%x)\n", pDevContext);

	CFG_GetHostResources((struct CFG_DEVNODE *)DRV_GetFirstDevExtension(),
			     &resources);
	HW_MBOX_EventDisable(resources.dwMboxBase, MBOX_DSP2ARM,
			     MBOX_ARM, HW_MBOX_INT_NEW_MSG);
	return DSP_SOK;
}

DSP_STATUS CHNLSM_InterruptDSP2(struct WMD_DEV_CONTEXT *pDevContext,
				u16 wMbVal)
{
#ifndef CONFIG_DISABLE_BRIDGE_PM
#ifndef CONFIG_DISABLE_BRIDGE_DVFS
	u32 opplevel;
#endif
#endif
	struct CFG_HOSTRES resources;
	unsigned long timeout;
	u32 temp;
	DSP_STATUS status = DSP_SOK;

	/* We are waiting indefinitely here. This needs to be fixed in the
	 * second phase */
	CFG_GetHostResources((struct CFG_DEVNODE *)DRV_GetFirstDevExtension(),
			&resources);

	if (pDevContext->dwBrdState == BRD_DSP_HIBERNATION ||
	    pDevContext->dwBrdState == BRD_HIBERNATION) {
#ifndef CONFIG_DISABLE_BRIDGE_PM
#ifndef CONFIG_DISABLE_BRIDGE_DVFS
#ifndef CONFIG_OMAP3_PM
		opplevel = omap_pm_dsp_get_opp();
		/* If OPP is at minimum level, increase it before waking up
		* the DSP */
		if (opplevel == 1) {
			omap_pm_dsp_set_min_opp(opplevel+1);
			DBG_Trace(DBG_LEVEL7, "CHNLSM_InterruptDSP:Setting "
			"the vdd1 constraint level to %d before "
			"waking DSP \n", (opplevel + 1));
		}

#else
		opplevel = constraint_get_level(dsp_constraint_handle);
		/* If OPP is at minimum level, increase it before waking up
		 * the DSP */
		if (opplevel == 1) {
			if (constraint_set(dsp_constraint_handle,
			   (opplevel+1)) != 0) {
				DBG_Trace(DBG_LEVEL7, "CHNLSM_InterruptDSP: "
					 "Constraint set failed\n");
				return DSP_EFAIL;
			}
			DBG_Trace(DBG_LEVEL7, "CHNLSM_InterruptDSP:Setting "
				 "the vdd1 constraint level to %d before "
				 "waking DSP \n", (opplevel + 1));
		}

#endif
#endif
#endif
		/* Restart the IVA clock that was disabled while
		 * the DSP initiated Hibernation. */
		if ((pDevContext->dwBrdState == BRD_DSP_HIBERNATION)
		   || (pDevContext->dwBrdState == BRD_HIBERNATION)) {
			status = CLK_Enable(SERVICESCLK_iva2_ck);
			if (DSP_FAILED(status))
				return status;
		}

		/* Read MMU register to invoke short wakeup of DSP */
		temp = (u32) *((REG_UWORD32 *) ((u32)
		       (resources.dwDmmuBase) + 0x10));

		/* Restore mailbox settings */
		HW_MBOX_restoreSettings(resources.dwMboxBase);
		DBG_Trace(DBG_LEVEL6, "MailBoxSettings: SYSCONFIG = 0x%x\n",
			  mboxsetting.sysconfig);
		DBG_Trace(DBG_LEVEL6, "MailBoxSettings: IRQENABLE0 = 0x%x\n",
			  mboxsetting.irqEnable0);
		DBG_Trace(DBG_LEVEL6, "MailBoxSettings: IRQENABLE1 = 0x%x\n",
			 mboxsetting.irqEnable1);
		/* Restart the peripheral clocks that were disabled only
		 * in DSP initiated Hibernation case.*/
		if (pDevContext->dwBrdState == BRD_DSP_HIBERNATION)
			DSP_PeripheralClocks_Enable(pDevContext, NULL);

		pDevContext->dwBrdState = BRD_RUNNING;
	}
	timeout = jiffies + msecs_to_jiffies(35);
	while (HW_MBOX_IsFull(resources.dwMboxBase, MBOX_ARM2DSP)) {
		if (time_after(jiffies, timeout)) {
			printk(KERN_ERR "dspbridge: "
				"timed out waiting for mailbox\n");
			return WMD_E_TIMEOUT;
		}
	}
	DBG_Trace(DBG_LEVEL3, "writing %x to Mailbox\n",
		  wMbVal);

	HW_MBOX_MsgWrite(resources.dwMboxBase, MBOX_ARM2DSP,
			 wMbVal);
	return DSP_SOK;
}

bool CHNLSM_ISR(struct WMD_DEV_CONTEXT *pDevContext, bool *pfSchedDPC,
		u16 *pwIntrVal)
{
	struct CFG_HOSTRES resources;
	u32 numMbxMsg;
	u32 mbxValue;

	DBG_Trace(DBG_ENTER, "CHNLSM_ISR(0x%x)\n", pDevContext);

	CFG_GetHostResources((struct CFG_DEVNODE *)DRV_GetFirstDevExtension(),
				&resources);

	HW_MBOX_NumMsgGet(resources.dwMboxBase, MBOX_DSP2ARM, &numMbxMsg);

	if (numMbxMsg > 0) {
		HW_MBOX_MsgRead(resources.dwMboxBase, MBOX_DSP2ARM, &mbxValue);

		HW_MBOX_EventAck(resources.dwMboxBase, MBOX_DSP2ARM,
				 HW_MBOX_U0_ARM, HW_MBOX_INT_NEW_MSG);

		DBG_Trace(DBG_LEVEL3, "Read %x from Mailbox\n", mbxValue);
		*pwIntrVal = (u16) mbxValue;
	}
	/* Set *pfSchedDPC to true; */
	*pfSchedDPC = true;
	return true;
}
