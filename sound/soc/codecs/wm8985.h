/*
 * wm8985.h  --  WM8985 Soc Audio driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _WM8985_H
#define _WM8985_H

/* WM8985 register space */

#define WM8985_RESET		0x0
#define WM8985_POWER1		0x1
#define WM8985_POWER2		0x2
#define WM8985_POWER3		0x3
#define WM8985_IFACE		0x4
#define WM8985_COMP		0x5
#define WM8985_CLOCK		0x6
#define WM8985_ADD		0x7
#define WM8985_GPIO		0x8
#define WM8985_JACK1		0x9
#define WM8985_DAC		0xa
#define WM8985_DACVOLL		0xb
#define WM8985_DACVOLR		0xc
#define WM8985_JACK2		0xd
#define WM8985_ADC		0xe
#define WM8985_ADCVOLL		0xf
#define WM8985_ADCVOLR		0x10
#define WM8985_EQ1		0x12
#define WM8985_EQ2		0x13
#define WM8985_EQ3		0x14
#define WM8985_EQ4		0x15
#define WM8985_EQ5		0x16
#define WM8985_CLDCTRL		0x17
#define WM8985_DACLIM1		0x18
#define WM8985_DACLIM2		0x19
#define WM8985_NOTCH1		0x1b
#define WM8985_NOTCH2		0x1c
#define WM8985_NOTCH3		0x1d
#define WM8985_NOTCH4		0x1e
#define WM8985_ALC1		0x20
#define WM8985_ALC2		0x21
#define WM8985_ALC3		0x22
#define WM8985_NGATE		0x23
#define WM8985_PLLN		0x24
#define WM8985_PLLK1		0x25
#define WM8985_PLLK2		0x26
#define WM8985_PLLK3		0x27
#define WM8985_VIDEO		0x28
#define WM8985_3D		0x29
#define WM8985_OUT4ADC		0x2a
#define WM8985_BEEP		0x2b
#define WM8985_INPUT		0x2c
#define WM8985_INPPGAL  	0x2d
#define WM8985_INPPGAR		0x2e
#define WM8985_ADCBOOSTL	0x2f
#define WM8985_ADCBOOSTR	0x30
#define WM8985_OUTPUT		0x31
#define WM8985_MIXL		0x32
#define WM8985_MIXR		0x33
#define WM8985_HPVOLL		0x34
#define WM8985_HPVOLR		0x35
#define WM8985_SPKVOLL		0x36
#define WM8985_SPKVOLR		0x37
#define WM8985_OUT3MIX		0x38
#define WM8985_OUT4MIX		0x39
#define WM8985_BIASCTRL		0x3d

#define WM8985_CACHEREGNUM 	62

/*
 * WM8985 Clock dividers
 */
#define WM8985_MCLKDIV 		0
#define WM8985_BCLKDIV		1
#define WM8985_OPCLKDIV		2
#define WM8985_DACOSR		3
#define WM8985_ADCOSR		4
#define WM8985_MCLKSEL		5

#define WM8985_MCLK_MCLK		(0 << 8)
#define WM8985_MCLK_PLL			(1 << 8)

#define WM8985_MCLK_DIV_1		(0 << 5)
#define WM8985_MCLK_DIV_1_5		(1 << 5)
#define WM8985_MCLK_DIV_2		(2 << 5)
#define WM8985_MCLK_DIV_3		(3 << 5)
#define WM8985_MCLK_DIV_4		(4 << 5)
#define WM8985_MCLK_DIV_5_5		(5 << 5)
#define WM8985_MCLK_DIV_6		(6 << 5)

#define WM8985_BCLK_DIV_1		(0 << 2)
#define WM8985_BCLK_DIV_2		(1 << 2)
#define WM8985_BCLK_DIV_4		(2 << 2)
#define WM8985_BCLK_DIV_8		(3 << 2)
#define WM8985_BCLK_DIV_16		(4 << 2)
#define WM8985_BCLK_DIV_32		(5 << 2)

#define WM8985_DACOSR_64		(0 << 3)
#define WM8985_DACOSR_128		(1 << 3)

#define WM8985_ADCOSR_64		(0 << 3)
#define WM8985_ADCOSR_128		(1 << 3)

#define WM8985_OPCLK_DIV_1		(0 << 4)
#define WM8985_OPCLK_DIV_2		(1 << 4)
#define WM8985_OPCLK_DIV_3		(2 << 4)
#define WM8985_OPCLK_DIV_4		(3 << 4)

/*
 * WM8985 Clock ID
 */
#define WM8985_ID_MCLK	0


/*
 * Defines from the old OSS driver
 */

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
#define VROI		(1<<0)
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


/* Inputs */
#define INPUT_LIP	0x1
#define INPUT_RIP	0x2
#define INPUT_LIN	0x4
#define INPUT_RIN	0x8
#define INPUT_LINE	0x10

/* Input Blocs */
#define INP_BLOC_BOOST	0x1
#define INP_BLOC_PGA	0x2
#define INP_BLOC_MIC	0x4

/* Outputs */
#define OUTPUT_SPK	0x1
#define OUTPUT_HP	0x2
#define OUTPUT_OUT3	0x4
#define OUTPUT_OUT4	0x8

/* Mixers */
#define MIXER_OUT3	0x1
#define MIXER_OUT4	0x2

/* General */
#define ON		1
#define OFF		0


struct wm8985_setup_data {
	unsigned short i2c_address;
};

extern struct snd_soc_dai wm8985_dai;
extern struct snd_soc_codec_device soc_codec_dev_wm8985;

/* Needed in non-dynamic power mode */
extern void wm8985_power_up_inputs(struct snd_soc_codec *codec, int input_blocs, int on);
extern void wm8985_power_up_outn_mixers(struct snd_soc_codec *codec, int mixers, int on);
extern void wm8985_select_outputs(struct snd_soc_codec *codec, int outputs, int on);
extern void wm8985_connect_pga(struct snd_soc_codec *codec, int on);
extern void wm8985_select_pga_inputs(struct snd_soc_codec *codec, int inputs, int on);

#endif
