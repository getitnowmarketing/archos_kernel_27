/*
 * linux/sound/oss/davinci-audio-wm8985.h
 *
 * Hardware definitions for Wolfson WM8985 audio codec
 *
 * Copyright (C) 2005 Archos, S.A.
 * Author: Paul Ebeyan
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __WOLFSON_8985_H
#define __WOLFSON_8985_H

#define WM8985_RESET		0x00
#define WM8985_PWR_MGT1		0x01
#define WM8985_PWR_MGT2		0x02
#define WM8985_PWR_MGT3 	0x03
#define WM8985_IFACE		0x04
#define WM8985_COMPANDING	0x05
#define WM8985_CLK_GEN		0x06
#define WM8985_ADD_CTRL		0x07
#define WM8985_GPIO		0x08
#define WM8985_JACK_DETECT1	0x09
#define WM8985_DAC_CTRL		0x0a
#define WM8985_LEFT_DAC_DVOL	0x0b
#define WM8985_RIGHT_DAC_DVOL	0x0c
#define WM8985_JACK_DETECT2	0x0d
#define WM8985_ADC_CTRL		0x0e
#define WM8985_LINVOL		0x0f
#define WM8985_RINVOL		0x10
#define WM8985_EQ1		0x12
#define WM8985_EQ2		0x13
#define WM8985_EQ3		0x14
#define WM8985_EQ4		0x15
#define WM8985_EQ5		0x16
#define WM8985_CLASSD_CTRL	0x17
#define WM8985_DAC_LIM1		0x18
#define WM8985_DAC_LIM2		0x19
#define WM8985_NFILTER1		0x1b
#define WM8985_NFILTER2		0x1c
#define WM8985_NFILTER3		0x1d
#define WM8985_NFILTER4		0x1e
#define WM8985_ALC_CTRL1	0x20
#define WM8985_ALC_CTRL2	0x21
#define WM8985_ALC_CTRL3	0x22
#define WM8985_NOISE_GATE	0x23
#define WM8985_PLLN		0x24
#define WM8985_PLLK1		0x25
#define WM8985_PLLK2		0x26
#define WM8985_PLLK3		0x27
#define WM8985_3D_CTRL		0x29
#define WM8985_OUT42DAC		0x2a
#define WM8985_BEEP_CTRL	0x2b
#define WM8985_INPUT_CTRL	0x2c
#define WM8985_LEFT_PGA_GAIN	0x2d
#define WM8985_RIGHT_PGA_GAIN	0x2e
#define WM8985_LEFT_ADC_BOOST	0x2f
#define WM8985_RIGHT_ADC_BOOST	0x30
#define WM8985_OUTPUT_CTRL	0x31
#define WM8985_LEFT_MIXER_CTRL	0x32
#define WM8985_RIGHT_MIXER_CTRL	0x33
#define WM8985_LOUT1V		0x34
#define WM8985_ROUT1V		0x35
#define WM8985_LOUT2V		0x36
#define WM8985_ROUT2V		0x37
#define WM8985_OUT3_MIXER_CTRL	0x38
#define WM8985_OUT4_MIXER_CTRL	0x39
#define WM8985_BIAS_CTRL	0x3d


#define WM8985_CACHEREGNUM (WM8985_BIAS_CTRL+1)

/* R1: WM8985_PWR_MGT1 */
#define OUT3MIXEN		(1<<6)
#define OUT4MIXEN		(1<<7)
#define PLLEN		(1<<5)
#define PLLDIS		(0<<5)
#define PLLENM		PLLEN

#define MIC_BIASEN	(1<<4)
#define MIC_BIASDIS	(0<<4)
#define MIC_BIASEN_M	(MIC_BIASEN)


#define BIASEN		(1<<3)

#define BUFIOEN		(1<<2)

#define VMIDSEL_OFF	(0<<0)
#define VMIDSEL_75k	(1<<0)
#define VMIDSEL_300k	(2<<0)
#define VMIDSEL_5k	(3<<0)
#define VMIDSELM	VMIDSEL_5k

/* R2: WM8985_PWR_MGT2 */
#define BOOSTENR	(1<<5)
#define BOOSTDISR	(0<<5)
#define BOOSTENRM	BOOSTENR

#define BOOSTENL	(1<<4)
#define BOOSTDISL	(0<<4)
#define BOOSTENLM	BOOSTENL

#define INPPGAENR	(1<<3)
#define INPPGADISR	(0<<3)
#define INPPGAENRM	INPPGAENR

#define INPPGAENL	(1<<2)
#define INPPGADISL	(0<<2)
#define INPPGAENLM	INPPGAENL

#define ADCENR		(1<<1)
#define ADCDISR		(0<<1)
#define ADCENRM		ADCENR

#define ADCENL		(1<<0)
#define ADCDISL		(0<<0)
#define ADCENLM		ADCENL

#define ROUT1EN		(1<<8)
#define ROUT1ENM	ROUT1EN
#define LOUT1EN		(1<<7)
#define LOUT1ENM	LOUT1EN

/* R3: WM8985_PWR_MGT3 */
#define OUT3EN		(1<<7)
#define OUT3ENM		OUT3EN
#define OUT4EN		(1<<8)
#define OUT4ENM		OUT4EN
#define ROUT2EN		(1<<6)
#define ROUT2ENM	ROUT2EN
#define LOUT2EN		(1<<5)
#define LOUT2ENM	LOUT2EN
#define RMIXEN		(1<<3)
#define LMIXEN		(1<<2)
#define DACENR	(1<<1)
#define DACDISR	(0<<1)
#define DACENRM	(1<<1)
#define DACENL	(1<<0)
#define DACDISL	(0<<0)
#define DACENLM	(1<<0)


/* R4: WM8985_IFACE */
#define BCP		1
#define DLRSWAP		(1<<2)
#define DSP_MODE	(3<<3)
#define WORD_LENGTH16	(0<<5)
#define WORD_LENGTH32	(3<<5)

/* R5 WM8985_COMPANDING */
#define LOOP_ON 1
#define LOOP_OFF 0


/* R6: WM8985_CLK_GEN */
#define MS		(1<<0)
#define BCLKDIVB	2
#define BCLKDIVM	(0x7<<BCLKDIVB)
#define MCLKDIVB	5
#define MCLKDIVM	(0x7<<MCLKDIVB)
#define CLKSEL_PLL	(1<<8)

/* R7: WM8985_ADD_CTRL */
#define SLOWCLKEN	(1<<0)
#define SRB		1
#define SRM		(7<<SRB)

#define M128ENB_EN	(0<<8)
#define M128ENB_DIS	(1<<8)
#define M128ENM		(1<<8)

#define DCLKDIV8	(8<<4)

/* R10: WM8985_DAC_CTRL */
#define DAC_SMUTE	(1<<6)
#define DAC_SUNMUTE	(0<<6)
#define DAC_SMUTEM	DAC_SMUTE
#define DACOSR128	(1<<3)


/* R11: WM8985_LEFT_DAC_DVOL */
/* &				*/
/* R12: WM8985_RIGHT_DAC_DVOL	*/
#define DACVU		(1<<8)
#define DACVOLM		(0xff)

enum {
	DACVOL_00DB	= 0xFF,		// 0dB
	DACVOL_m01DB	= 0xFD,
	DACVOL_m02DB	= 0xFB,
	DACVOL_m03DB	= 0xF9,
	DACVOL_m04DB	= 0xF7,
	DACVOL_m05DB	= 0xF5,
	DACVOL_m06DB	= 0xF3,
	DACVOL_m07DB	= 0xF1,
	DACVOL_m08DB	= 0xEF,
	DACVOL_m09DB	= 0xED,
	DACVOL_m10DB	= 0xEB,
	DACVOL_m11DB	= 0xE9,
	DACVOL_m12DB	= 0xE7,		// -12dB	
	DACVOL_m13DB	= 0xE5,
	DACVOL_m14DB	= 0xE3,
	DACVOL_m15DB	= 0xE1,
	DACVOL_m16DB	= 0xDF,
	DACVOL_m17DB	= 0xDD,
	DACVOL_m18DB	= 0xDB,
	DACVOL_m19DB	= 0xD9,
	DACVOL_m20DB	= 0xD7,
};

/* R14: WM8985_ADC_CTRL */
#define HPFEN		(1<<8)
#define ADCOSR		(1<<3)

/* R15: WM8985_LINVOL */
/* ADC Vol */
#define ADCVU		(1<<8)
#define ADCVOL_MUTE	0
#define ADCVOLLM	(0xff<<0)

/* R16: WM8985_RINVOL */
/* ADC Vol */
#define ADCVOLRM	(0xff<<0)

/* R18: WM8985_EQ1 */
/*******************************************************
R18 to R22: Equalizer
*******************************************************/

#define EQ_GAINMASK	0x1F
#define EQ_BWMASK	(1<<8)
#define EQ_FREQMASK	(3<<5)

#define EQBW_NARROW	(0<<8)
#define EQBW_WIDE	(1<<8)

/* R18: WM8985_EQ1 */
#define EQ3DMODE_ADC	(0<<8)
#define EQ3DMODE_DAC	(1<<8)
#define EQ3DMODEM	EQ3DMODE_DAC

#define EQ1C_80HZ	(0<<5)
#define EQ1C_105HZ	(1<<5)
#define EQ1C_135HZ	(2<<5)
#define EQ1C_175HZ	(3<<5)

/* R19: WM8985_EQ2 */
#define EQ2C_230HZ	(0<<5)
#define EQ2C_300HZ	(1<<5)
#define EQ2C_385HZ	(2<<5)
#define EQ2C_500HZ	(3<<5)

/* R20: WM8985_EQ3 */
#define EQ3C_650HZ	(0<<5)
#define EQ3C_850HZ	(1<<5)
#define EQ3C_1100HZ	(2<<5)
#define EQ3C_1400HZ	(3<<5)

/* R21: WM8985_EQ4 */
#define EQ4C_1800HZ	(0<<5)
#define EQ4C_2400HZ	(1<<5)
#define EQ4C_3200HZ	(2<<5)
#define EQ4C_4100HZ	(3<<5)

/* R22: WM8985_EQ5 */
#define EQ5C_5300HZ	(0<<5)
#define EQ5C_6900HZ	(1<<5)
#define EQ5C_9000HZ	(2<<5)
#define EQ5C_11700HZ	(3<<5)

/*******************************************************
R18 to R22: Equalizer /END
*******************************************************/

/* R23: WM8985_CLASSD_CTRL */
#define CLASSDEN	(1<<8)


/* R24: WM8985_DAC_LIM1 */
#define LIMEN_EN	(0x1<<8)
#define LIMEN_DIS	(0x0<<8)
#define LIMATK_DEFAULT	(2<<0)
#define LIMDCY_DEFAULT	(3<<4)

#define LIMATKM		(0xf<<0)
#define LIMDCYM		(0xf<<4)
#define LIMENM		(0x1<<8)

/* R25: WM8985_DAC_LIM2 */
#define LIMLVL_DEFAULT	(1<<4)
#define LIMLVLM		(7<<4)
#define LIMLVL(val)	((val<<4)&LIMLVLM)



/* R32: WM8985_ALC_CTRL1 */
#define ALCSEL_DIS	(0<<7)
#define ALCSEL_ALL	(3<<7)

#define ALCSELM		(3<<7)
#define ALCMIN		(2<<0)
#define ALCMAX		(7<<3)


/* R34: WM8985_ALC_CTRL3 */
#define ALCMODE_LIM	(1<<8)

/* R36: WM8985_PLLN */
#define PLLNB		0
#define PLLNM		(0xf<<PLLNB)
#define PLLPRESCALEB	4
#define PLLPRESCALEM	(0x1<<PLLPRESCALEB)

/* R37: WM8985_PLLK1 */
#define PLLK1B		0
#define PLLK1M		(0x3f<<PLLK1B)

/* R38: WM8985_PLLK2 */
#define PLLK2B		0
#define PLLK2M		(0x1ff<<PLLK2B)

/* R39: WM8985_PLLK3 */
#define PLLK3B		0
#define PLLK3M		(0x1ff<<PLLK3B)

/* R42: WM8985_OUT42DAC */
#define DEPTH3D_MASK	0x0F

/* R42: WM8985_OUT42DAC */
#define POBCTRL		(1<<2)
#define POBCTRL_DIS	(0<<2)
#define POBCTRLM	POBCTRL

/* R44: WM8985_INPUT_CTRL */
#define L2_2INPPGA	(1<<2)
#define R2_2INPPGA	(1<<6)
#define LIN_2INPPGA	(1<<1)
#define RIN_2INPPGA	(1<<5)
#define LIP_2INPPGA	(1<<0)
#define RIP_2INPPGA	(1<<4)



/* R45: WM8985_LEFT_PGA_GAIN */
#define INPPGAVU	(1<<8)
#define INPPGAZCL	(1<<7)
#define INPPGAMUTELB	6
#define INPPGAMUTEL	(1<<INPPGAMUTELB)
#define INPPGAUNMUTEL	(0<<INPPGAMUTELB)
#define INPPGAMUTELM	INPPGAMUTEL
#define LINVOLM		(0x3F<<0)
#define INPPGAVOLL_MIN	0

/* R46: WM8985_RIGHT_PGA_GAIN */
#define INPPGAZCR	(1<<7)
#define INPPGAMUTERB	6
#define INPPGAMUTER	(1<<INPPGAMUTERB)
#define INPPGAUNMUTER	(0<<INPPGAMUTERB)
#define INPPGAMUTERM	INPPGAMUTER
#define RINVOLM (0x3F<<0)
#define INPPGAVOLR_MIN	0

/* R47: WM8985_LEFT_ADC_BOOST */
#define PGABOOSTL_0DB	(0<<8)
#define PGABOOSTL_20DB	(1<<8)
#define L2_2BOOSTVOLB	(4)
#define L2_2BOOSTVOLM	(0x7<<L2_2BOOSTVOLB)

/* R48: WM8985_RIGHT_ADC_BOOST */
#define PGABOOSTR_0DB	(0<<8)
#define R2_2BOOSTVOLB	(4)
#define R2_2BOOSTVOLM	(0x7<<L2_2BOOSTVOLB)

/* R49: WM8985_OUTPUT_CTRL */
#define TSDEN		(1<<1)
#define TSDDIS		(0<<1)
#define TSDENM		TSDEN
#define TSOPCTRL_OFF	(0<<2)
#define TSOPCTRL	(1<<2)
#define TSOPCTRLM	TSOPCTRL

/* R50: WM8985_LEFT_MIXER_CTRL */
#define DACL2LMIX	1
#define BYPL2LMIX	(1<<1)
#define BYPL2LMIX_OFF	(0<<1)
#define BYPL2LMIXM	BYPL2LMIX
#define AUXL2LMIX	(1<<5)
#define AUXL2LMIX_OFF	(0<<5)
#define AUXL2LMIXM	AUXL2LMIX
#define BYPLMIXVOL_0DB	(5<<2)
#define BYPLMIXVOL_MIN	(0<<2)
#define BYPLMIXVOLM	(7<<2)


/* R51: WM8985_RIGHT_MIXER_CTRL */
#define DACR2RMIX	1
#define BYPR2RMIX	(1<<1)
#define BYPR2RMIX_OFF	(0<<1)
#define BYPR2RMIXM	BYPL2LMIX
#define AUXR2RMIX	(1<<5)
#define AUXR2RMIX_OFF	(0<<5)
#define AUXR2RMIXM	AUXL2LMIX
#define BYPRMIXVOL_0DB	(5<<2)
#define BYPRMIXVOL_MIN	(0<<2)

#define BYPRMIXVOLM	(7<<2)


/* R52: WM8985_LOUT1V */
/* R54: WM8985_LOUT2V */
#define OUTVU		(1<<8)
#define ZCEN_B		7
#define	LHP_MUTE	(1<<6)
#define	LHP_UNMUTE	(0<<6)
#define	LHP_MUTEM	LHP_MUTE
#define LZCEN		(1<<ZCEN_B)
#define LZCDIS		(0<<ZCEN_B)
#define LZCENM		LZCEN
#define LHPVOLM		(0x3f<<0)

/* R53: WM8985_ROUT1V */
/* R55: WM8985_ROUT2V */
#define	RHP_MUTE	(1<<6)
#define	RHP_UNMUTE	(0<<6)
#define	RHP_MUTEM	RHP_MUTE
#define RZCEN		(1<<ZCEN_B)
#define RZCDIS		(0<<ZCEN_B)
#define RZCENM		RZCEN
#define RHPVOLM		(0x3f<<0)

/* R56: WM8985_OUT3_MIXER_CTRL */
#define OUT3MUTE	(1<<6)
#define LDAC2OUT3	(1<<0)
#define LMIX2OUT3	(1<<1)
#define LDAC2OUT3_DIS	(0<<0)

/* R57: WM8985_OUT4_MIXER_CTRL */
#define OUT4MUTE	(1<<6)
#define RMIX2OUT4	(1<<1)
#define RDAC2OUT4	(1<<0)
#define RDAC2OUT4_DIS	(0<<0)

/* R61: WM8985_BIAS_CTRL */
#define BIAS_NORMAL	(0<<8)
#define BIASCUT		(1<<8)


struct mcbsp_config {
	struct omap_mcbsp_cfg_param rx_param;
	struct omap_mcbsp_cfg_param tx_param;
	struct omap_mcbsp_srg_fsg_cfg srg_fsg_param;
	struct omap_mcbsp_dma_transfer_parameters dma_recv;
	struct omap_mcbsp_dma_transfer_parameters dma_trans;
};

#if 0 /* Old configuration way */
struct mcbsp_config {
	u8 mcbsp_clk_src;	/* source it from prcm? */
	u8 srg_clk_src;		/* clks/fclk/clkr/clkx */
	u8 srg_clk_sync;	/* free_running or just running */
	u8 srg_clk_pol;		/* polarity of srg clk */
	u8 tx_clk_pol;          /* TX clock polarity */
	u8 tx_clk_src;		/* internal/external (master based) */
	u8 rx_clk_pol;          /* TX clock polarity */
	u8 rx_clk_src;		/* internal/external (master based) */
	u8 fs_clk_pol;		/* Frame sync polarity */
	u8 tx_polarity;		/* TX polarity */
	u8 rx_polarity;		/* RX polarity */
	u8 rx_ip_clk;           /* Rx input clock */
	u8 tx_ip_clk;           /* Tx input clock */
	omap2_mcbsp_transfer_params tx_params; /* Transmit parameters */
	omap2_mcbsp_transfer_params rx_params; /* Recieve parameters */
};
#endif

/* Wrapper for McBSP callback */
struct stream_callback_struct {
	buf_irq_handler callback;
	audio_stream_t *s;
	audio_state_t *state;
};

/*******************************************************
  EQUALIZER DECLARATIONS SHARED WITH AVOS
*******************************************************/

// Also defined in AVOS (equalizer_wm8985.h)
typedef struct {
	// Equalizer bands
	int band1_freq;		// Center frequency
	int band1_gain;		// Peak gain
	int band2_freq;
	int band2_bw;		// Bandwidth
	int band2_gain;
	int band3_freq;
	int band3_bw;
	int band3_gain;
	int band4_freq;
	int band4_bw;
	int band4_gain;
	int band5_freq;
	int band5_gain;

	// 3D enhancement features
	int depth_3d;
	// dac vol attenuation to avoid equalizer saturation
	int dac_vol;
} wm8985_eqParams_t;

// EQ Gains (have to be in this order)
enum {
	EQ_WM8985_GAIN_12DB	= 0x00,		// +12dB
	EQ_WM8985_GAIN_11DB	= 0x01,
	EQ_WM8985_GAIN_10DB	= 0x02,
	EQ_WM8985_GAIN_09DB	= 0x03,
	EQ_WM8985_GAIN_08DB	= 0x04,
	EQ_WM8985_GAIN_07DB	= 0x05,
	EQ_WM8985_GAIN_06DB	= 0x06,
	EQ_WM8985_GAIN_05DB	= 0x07,
	EQ_WM8985_GAIN_04DB	= 0x08,
	EQ_WM8985_GAIN_03DB	= 0x09,
	EQ_WM8985_GAIN_02DB	= 0x0A,
	EQ_WM8985_GAIN_01DB	= 0x0B,
	EQ_WM8985_GAIN_00DB	= 0x0C,		// 0dB
	EQ_WM8985_GAIN_m01DB	= 0x0D,
	EQ_WM8985_GAIN_m02DB	= 0x0E,
	EQ_WM8985_GAIN_m03DB	= 0x0F,
	EQ_WM8985_GAIN_m04DB	= 0x10,
	EQ_WM8985_GAIN_m05DB	= 0x11,
	EQ_WM8985_GAIN_m06DB	= 0x12,
	EQ_WM8985_GAIN_m07DB	= 0x13,
	EQ_WM8985_GAIN_m08DB	= 0x14,
	EQ_WM8985_GAIN_m09DB	= 0x15,
	EQ_WM8985_GAIN_m10DB	= 0x16,
	EQ_WM8985_GAIN_m11DB	= 0x17,
	EQ_WM8985_GAIN_m12DB	= 0x18,		// -12dB
};

// EQ Bands
enum {
	EQ_WM8985_BAND1_80HZ = 0,
	EQ_WM8985_BAND1_105HZ,
	EQ_WM8985_BAND1_135HZ,
	EQ_WM8985_BAND1_175HZ,

	EQ_WM8985_BAND2_230HZ,
	EQ_WM8985_BAND2_300HZ,
	EQ_WM8985_BAND2_385HZ,
	EQ_WM8985_BAND2_500HZ,

	EQ_WM8985_BAND3_650HZ,
	EQ_WM8985_BAND3_850HZ,
	EQ_WM8985_BAND3_1100HZ,
	EQ_WM8985_BAND3_1400HZ,

	EQ_WM8985_BAND4_1800HZ,
	EQ_WM8985_BAND4_2400HZ,
	EQ_WM8985_BAND4_3200HZ,
	EQ_WM8985_BAND4_4100HZ,

	EQ_WM8985_BAND5_5300HZ,
	EQ_WM8985_BAND5_6900HZ,
	EQ_WM8985_BAND5_9000HZ,
	EQ_WM8985_BAND5_11700HZ,
};

// EQ Bandwidth  (have to be in this order)
enum {
	EQ_WM8985_BW_WIDE	= 1,
	EQ_WM8985_BW_NARROW	= 0,
};

// Default parameters
#define EQ_WM8985_DEFAULT_PARAMS					\
		.band1_freq	= EQ_WM8985_BAND1_80HZ,			\
		.band1_gain	= EQ_WM8985_GAIN_00DB,			\
		.band2_freq	= EQ_WM8985_BAND2_300HZ, 		\
		.band2_bw	= EQ_WM8985_BW_WIDE, 			\
		.band2_gain	= EQ_WM8985_GAIN_00DB, 			\
		.band3_freq	= EQ_WM8985_BAND3_1100HZ, 		\
		.band3_bw	= EQ_WM8985_BW_WIDE, 			\
		.band3_gain	= EQ_WM8985_GAIN_00DB, 			\
		.band4_freq	= EQ_WM8985_BAND4_4100HZ, 		\
		.band4_bw	= EQ_WM8985_BW_WIDE, 			\
		.band4_gain	= EQ_WM8985_GAIN_00DB, 			\
		.band5_freq	= EQ_WM8985_BAND5_11700HZ, 		\
		.band5_gain	= EQ_WM8985_GAIN_00DB, 			\
									\
		.depth_3d	= 0x00,					\
		.dac_vol	= 0x00,




#endif /* __WOLFSON_8985_H */
