/*
 * dsipll.c
 *
 *  Created on: Jan 14, 2009
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

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/errno.h>

/* DSI module base addresses */
#define DSI_PROTO_ENG_BASE_ADDR	0x4804FC00
#define DSI_PLL_CONTROLLER_BASE_ADDR	0x4804FF00

/* DSI module registers */
#define DSI_REVISION		(DSI_PROTO_ENG_BASE_ADDR + 0x00)
#define DSI_SYSCONFIG		(DSI_PROTO_ENG_BASE_ADDR + 0x10)
#define DSI_SYSSTATUS		(DSI_PROTO_ENG_BASE_ADDR + 0x14)
#define DSI_IRQSTATUS		(DSI_PROTO_ENG_BASE_ADDR + 0x18)
#define DSI_IRQENABLE		(DSI_PROTO_ENG_BASE_ADDR + 0x1C)
#define DSI_CTRL		(DSI_PROTO_ENG_BASE_ADDR + 0x40)
#define DSI_COMPLEXIO_CFG1	(DSI_PROTO_ENG_BASE_ADDR + 0x48)
#define DSI_COMPLEXIO_IRQSTATUS	(DSI_PROTO_ENG_BASE_ADDR + 0x4C)
#define DSI_COMPLEXIO_IRQENABLE	(DSI_PROTO_ENG_BASE_ADDR + 0x50)
#define DSI_CLK_CTRL		(DSI_PROTO_ENG_BASE_ADDR + 0x54)
#define DSI_TIMING1		(DSI_PROTO_ENG_BASE_ADDR + 0x58)
#define DSI_TIMING2		(DSI_PROTO_ENG_BASE_ADDR + 0x5C)
#define DSI_VM_TIMING1		(DSI_PROTO_ENG_BASE_ADDR + 0x60)
#define DSI_VM_TIMING2		(DSI_PROTO_ENG_BASE_ADDR + 0x64)
#define DSI_VM_TIMING3		(DSI_PROTO_ENG_BASE_ADDR + 0x68)
#define DSI_CLK_TIMING		(DSI_PROTO_ENG_BASE_ADDR + 0x6C)
#define DSI_TX_FIFO_VC_SIZE	(DSI_PROTO_ENG_BASE_ADDR + 0x70)
#define DSI_RX_FIFO_VC_SIZE	(DSI_PROTO_ENG_BASE_ADDR + 0x74)
#define DSI_COMPLEXIO_CFG2	(DSI_PROTO_ENG_BASE_ADDR + 0x78)
#define DSI_RX_FIFO_VC_FULLNESS	(DSI_PROTO_ENG_BASE_ADDR + 0x7C)
#define DSI_VM_TIMING4		(DSI_PROTO_ENG_BASE_ADDR + 0x80)
#define DSI_TX_FIFO_VC_EMPTINESS	(DSI_PROTO_ENG_BASE_ADDR + 0x84)
#define DSI_VM_TIMING5		(DSI_PROTO_ENG_BASE_ADDR + 0x88)
#define DSI_VM_TIMING6		(DSI_PROTO_ENG_BASE_ADDR + 0x8C)
#define DSI_VM_TIMING7		(DSI_PROTO_ENG_BASE_ADDR + 0x90)
#define DSI_VC0_CTRL		(DSI_PROTO_ENG_BASE_ADDR + 0x100)
#define DSI_VC0_TE		(DSI_PROTO_ENG_BASE_ADDR + 0x104)
#define DSI_VC0_LONG_PACKET_HEADER	(DSI_PROTO_ENG_BASE_ADDR + 0x108)
#define DSI_VC0_LONG_PACKET_PAYLOAD	(DSI_PROTO_ENG_BASE_ADDR + 0x10C)
#define DSI_VC0_SHORT_PACKET_HEADER	(DSI_PROTO_ENG_BASE_ADDR + 0x110)
#define DSI_VC0_IRQSTATUS		(DSI_PROTO_ENG_BASE_ADDR + 0x118)
#define DSI_VC0_IRQENABLE		(DSI_PROTO_ENG_BASE_ADDR + 0x11C)

/* DSI PLL registers */
#define DSI_PLL_CONTROL	(DSI_PLL_CONTROLLER_BASE_ADDR + 0x00)
#define DSI_PLL_STATUS		(DSI_PLL_CONTROLLER_BASE_ADDR + 0x04)
#define DSI_PLL_GO		(DSI_PLL_CONTROLLER_BASE_ADDR + 0x08)
#define DSI_PLL_CONFIGURATION1	(DSI_PLL_CONTROLLER_BASE_ADDR + 0x0C)
#define DSI_PLL_CONFIGURATION2	(DSI_PLL_CONTROLLER_BASE_ADDR + 0x10)

static u32 dsi_clk_rate;

/*
 * DSI Proto Engine register I/O routines
 */
static inline u32 dsiproto_reg_in(u32 offset)
{
	return omap_readl(offset);
}

static inline u32 dsiproto_reg_out(u32 offset, u32 val)
{
	omap_writel(val, offset);
	return val;
}

/*
 * DSI PLL register I/O routines
 */
static inline u32 dsipll_reg_in(u32 offset)
{
	return omap_readl(offset);
}

static inline u32 dsipll_reg_out(u32 offset, u32 val)
{
	omap_writel(val, offset);
	return val;
}

static int lock_dsi_pll( u32 M, u32 N, u32 M3, u32 M4, u32 freqsel)
{
	u32 count = 1000, val1, val2;
	
	val1 =  ((M4<<23)|(M3<<19)|(M<<8)|(N<<1)|(1));
	
	val2 =  ((0<<20)|(0<<19)|(1<<18)|(0<<17)|(1<<16)|(0<<14)|
                        (1<<13)|(0<<12)|(0<<11)|(0<< 8)|(freqsel<<1));

	if (dsipll_reg_in( DSI_PLL_CONFIGURATION1 ) == val1 &&
	    dsipll_reg_in( DSI_PLL_CONFIGURATION2 ) == val2 &&
	    dsipll_reg_in(DSI_PLL_GO) == 0 &&
	    (dsipll_reg_in(DSI_PLL_STATUS) & 0x2) 
	) {
		printk("DSI PLL already programmed\n");
	}
 	
	dsipll_reg_out(DSI_PLL_CONFIGURATION1, val1);
	dsipll_reg_out(DSI_PLL_CONFIGURATION2, val2);
	
	dsipll_reg_out(DSI_PLL_GO, 1);

	while ((dsipll_reg_in(DSI_PLL_GO) != 0) && (--count)) 
		udelay(100);

	if (count == 0) { 
		printk ("GO bit not cleared\n");
		return 0;
	}	

	count = 1000;
	while (((dsipll_reg_in(DSI_PLL_STATUS) & 0x2 ) != 0x2) && (--count) )
		udelay(100);

	if (count == 0)  {
		printk ("DSI PLL lock request failed = %X\n", dsipll_reg_in(DSI_PLL_STATUS) );
		return 0;
	}

	return 1;	
}

static int power_dsi_pll (u32 cmd) 
{
	u32 val, count = 10000;
	
	pr_debug("power_dsi_pll %x \n", cmd);
	
	/* send the power command */
 	val = dsiproto_reg_in(DSI_CLK_CTRL);
	val = ((val & ~(3 << 30)) | (cmd << 30));	
        dsiproto_reg_out(DSI_CLK_CTRL, val);		

	/* Check whether the power status is changed */ 
	do {
		val = dsiproto_reg_in(DSI_CLK_CTRL);
		val = ((val & 0x30000000) >> 28);
	    	udelay(100); 			
	} while( (val != cmd) && (--count));

	return count;
} 

static int set_dsi_freq(u32 freq)
{
	int ret;

	pr_debug("set_dsi_freq %d \n", freq);
	
	/* PLL settings based on 12MHz SYS_CLK:
	 * Fint = 2MHz -> N+1 = 6, freqsel = 7
	 */
	const u32 N = 5;
	const u32 freqsel = 7;
	const u32 sys_clk = 12000; /* kHz */
	const u32 regm = 7;
	
	u32 M = ( (regm+1) * freq * (N+1) ) / (sys_clk * 2);

	ret = lock_dsi_pll(M, N, regm, regm, freqsel);

	if (1) {
		/* for debugging, compute the output frequency */
		u32 dsi_clk = (2 * M * sys_clk) / ((N+1) * (regm+1));
		pr_info("set_dsi_clkrate desired freq %lukHz, "
			  "M=%lu N+1=%lu -> dsi_clk=%lukHz, ret=%i\n",
			(unsigned long)freq, (unsigned long)M, 
			(unsigned long)N+1, (unsigned long)dsi_clk, ret);
	}
	
	return ret;
}

static int dsi_pll_pwr;

int omap_disp_start_dsi_pll(u32 clkrate)
{
	pr_debug("omap_disp_start_dsi_pll, clkrate = %d\n", clkrate);
	
	if (!dsi_pll_pwr) {
		//Command to change to ON state for both PLL and HSDIVISER
		//(no clock output to the DSI complex I/O)
		if (!power_dsi_pll(2)) {
			printk("Unable to power DSI PLL\n");
			return -EIO;
		}
		dsi_pll_pwr = 1;
	}

	if (!set_dsi_freq(clkrate)) {
		printk("FATAL ERROR: DSI PLL lock failed = %X\n",  dsipll_reg_in(DSI_PLL_STATUS));
		return -EIO;
	}
	
	return 0;
}

int omap_disp_stop_dsi_pll(void)
{
	if (dsi_pll_pwr) {
		if (!power_dsi_pll(0)) {
			pr_err("Unable to shut down DSI PLL\n");
			return -EIO;
		}
		dsi_pll_pwr = 0;
	}
	
	return 0;
}
