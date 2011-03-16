/*
 * linux/sound/oss/davinci-audio-wm8985.c
 *
 * Glue audio driver for Wolson WM8985 codec
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


#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/errno.h>
#include <linux/sound.h>
#include <linux/soundcard.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/poll.h>
#include <linux/freezer.h>
#include <linux/err.h>

#include <linux/semaphore.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <mach/dma.h>
#include <asm/io.h>

#include <mach/mux.h>
#include <mach/io.h>
#include <asm/mach-types.h>

#include "omap2-audio.h"

#include <mach/mcbsp.h>
#include <mach/clock.h>
#include <linux/clk.h>
#include <mach/archos-audio.h>


#include "omap2-audio-wm8985.h"


//#define DEBUG
#define DRIVER_VERSION "1.1"

#undef DPRINTK
//#define DEBUG
#ifdef DEBUG
#define DPRINTK(ARGS...)  printk(KERN_INFO "<%s>: ",__FUNCTION__);printk(ARGS)
#define FN_IN printk("[%s]: start\n", __FUNCTION__)
#define FN_OUT(n) printk(KERN_INFO "[%s]: end(%u)\n",__FUNCTION__, n)
#else
#define DPRINTK( x... )
#define FN_IN
#define FN_OUT(n)

#endif				/* DEBUG */


#undef DPRINTK_VOL
//#define DEBUG_VOL
#ifdef DEBUG_VOL
#define DPRINTK_VOL(ARGS...)  printk(KERN_INFO "<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DPRINTK_VOL( x... )
#endif				/* DEBUG */

#undef DPRINTK_SPDIF
//#define DEBUG_SPDIF
#ifdef DEBUG_SPDIF
#define DPRINTK_SPDIF(ARGS...)  printk(KERN_INFO "<%s>: ",__FUNCTION__);printk(ARGS)
#else
#define DPRINTK_SPDIF( x... )
#endif				/* DEBUG */

#define SIZEOF(table)	(sizeof(table)/sizeof(table[0]))


/* Define to set the WM8985_MASTER as the master w.r.t McBSP */
#define WM8985_MASTER

/* Define to set the codec of a source of sync for spdif */
#define WM8985_SPDIF_SRC_SYNC

#define CODEC_NAME		"WM8985"
#define AUDIO_CODEC		WM8985

/*
 * AUDIO related MACROS
 */
#define AUDIO_RATE_DEFAULT	      44100

#define REC_MASK 		      (SOUND_MASK_LINE | SOUND_MASK_MIC)
#define DEV_MASK 		      (REC_MASK | SOUND_MASK_VOLUME)

#define MAX_100DBA		      0

#define MAXIMUM_VOLUME 		      100L
#define DEFAULT_OUTPUT_VOLUME         50L
#define DEFAULT_INPUT_VOLUME          0	/* 0 ==> mute line in */

#define VOL_0dB				57

#define OUTPUT_VOLUME_MIN             0
/* maximum analog volume 0dB controlled by user */
#define OUTPUT_VOLUME_MAX             57

#define OUTPUT_VOLUME_RANGE           (OUTPUT_VOLUME_MAX - OUTPUT_VOLUME_MIN)

/* maximum analog gain 6dB */
#define OUTPUT_GAIN_MAX               63

/* maximum and minimun volume adjust gain in half dB */
#define ADJUST_GAIN_MAX               (OUTPUT_GAIN_MAX-OUTPUT_VOLUME_MAX)*2
#define ADJUST_GAIN_MIN               ADJUST_GAIN_MAX*-1


#define LINE_REC_LEVEL_MIN	0	/* -12dB gain: but does not match with reality which is around -10dB */
#define LINE_REC_LEVEL_MAX	0x3f	/* +35.25dB gain: but does not match with reality which is around +4dB*/

/* double the range: 1st range without boost, 2nd range with 20dB boost */
/* so the range will be from -10dB to +24 dB */ 
/* NB: the 20 dB boost is conform to the spec */
#define LINE_REC_LEVEL_MAX_WITH_BOOST	(LINE_REC_LEVEL_MAX*2)

#if defined (CONFIG_MACH_OMAP3_EVM)
	/* this board use an audio pcb based on AX05 */
	#define LINE_REC_RANGE		(LINE_REC_LEVEL_MAX_WITH_BOOST-LINE_REC_LEVEL_MIN)
#else
	/* use normal range on G6: the input audio gain issue should have been fixed */
	#define LINE_REC_RANGE		(LINE_REC_LEVEL_MAX-LINE_REC_LEVEL_MIN)
#endif

#define LINE_BOOST_REC_LEVEL_MIN	1	/* -12 dB gain */
#define LINE_BOOST_REC_LEVEL_MAX	0x7	/* +6 dB gain */
#define LINE_BOOST_REC_RANGE		(LINE_BOOST_REC_LEVEL_MAX-LINE_BOOST_REC_LEVEL_MIN)

#define MIC_REC_LEVEL_MIN	0	/* +0dB gain */
#define MIC_REC_LEVEL_MAX	1	/* +20dB gain */
/* cheat with mic range: add 1 so codec_convert_range will return 0 or 1*/
#define MIC_REC_RANGE		(MIC_REC_LEVEL_MAX-MIC_REC_LEVEL_MIN+1)

#define INPUT_VOLUME_RANGE_MAX	      LINE_REC_LEVEL_MAX

#define SET_VOLUME	1
#define SET_LINE_GAIN   2
#define SET_MIC_GAIN	3
#define SET_OUTPUT_GAIN 4
#define SET_REC_LEVEL	5


#define ADC_DAC_RATIO_GAIN_DB	3
#define USE_DSP_32BIT_MODE
#define WM87xx_I2C_NB		2

static audio_stream_t output_stream = {
        .id              = "WM8985 out",
	.input_or_output = FMODE_WRITE
};

static audio_stream_t input_stream = {
        .id              = "WM8985 in",
	.input_or_output = FMODE_READ
};

static int mixer_dev_id;
static int hp_direct_path = 0;
static int zc_switch = 1;
static int prog_vol = 0;
static int use_pga = 1;
static int dump = 0;
static int use_analog_loop = 0;
static int is_gen7;


/* To handle transfer errors.. */
static int tx_err, rx_err;

static void codec_mcbsp_dma_cb(u32 ch_status, void *arg);
static int codec_prescale(void);
static int codec_postscale(void);
static int codec_transfer(struct audio_state_s *state,
				 audio_stream_t * s, void *buffer_phy, u32 size);
static int codec_transfer_stop(audio_stream_t * s);
static int codec_transfer_posn(audio_stream_t * s);
static int codec_transfer_init(audio_state_t * state,
				      audio_stream_t * s,
				      buf_irq_handler callback);
static int codec_conf_interface(void);


static void _set_mcbsp_config_spdif(void);
static void _set_mcbsp_config_analog(void);
static void _mcbsp_stop(void);
static void _mcbsp_start(void);

static struct audio_device_config *pt_audio_device_io = NULL;

typedef enum { WM8985, CODEC_AIC31, CODEC_AIC32, CODEC_AIC33 } codec_type_t;
typedef enum { OUTPUT_HP = 1, OUTPUT_SPK = 2, OUTPUT_LINE = OUTPUT_SPK, INPUT_MIC = 4, INPUT_LINE = 8, INPUT_PHONE = 16 } codec_config_t;

static struct i2c_client *codec_handle;

static struct codec_local_info {
	/* which codec */
	const codec_type_t codec;
	/* I/O configuration */
	codec_config_t config;
	
	int read_count;
	int write_count;
	
        /* input values from ioctl() */
        int volume_left;
        int volume_right;
	int volume_adjust;
        int line_gain;
        int mic_gain;
	int rec_level_line;
	int rec_level_mic;
	int rec_level_phone;
        int output_gain;
        int sample_rate;
        int volume_speaker;

        unsigned clock_set:1;
	unsigned spdif_enabled:1;
	unsigned initialized:1;

        int mod_cnt;
	
	struct task_struct *hpmon_handle;
	int mix_refcount;

} codec_info = {
	.codec = AUDIO_CODEC,
};

/*
 * Equalizer parameters
 */ 
static wm8985_eqParams_t eq_currentParams;
static int eq_isSet = 0;

/*
 * wm8985 register default values
 */
static const unsigned int codec_regs_default[WM8985_CACHEREGNUM]= {
//  0       1       2       3       4       5       6       7       8       9     
    0x0000, 0x0000, 0x0000, 0x0000, 0x0050, 0x0000, 0x0140, 0x0080, 0x0000, 0x0000,
    0x0000, 0x00ff, 0x00ff, 0x0000, 0x0100, 0x00ff, 0x00ff, 0x0000, 0x012c, 0x002c,
    0x002c, 0x002c, 0x002c, 0x0008, 0x0032, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0038, 0x000b, 0x0032, 0x0000, 0x0008, 0x000c, 0x0093, 0x00e9,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0003, 0x0010, 0x0010, 0x0100, 0x0100, 0x0002,
    0x0001, 0x0001, 0x0039, 0x0039, 0x0039, 0x0039, 0x0001, 0x0001, 0x0000, 0x0000,
    0X0000, 0X0000
};

/*
 * wm8985 register cache
 * We can't read the WM8985 register space when we are
 * using 2 wire for device control, so we cache them instead.
 * There is no point in caching the reset register
 */
static unsigned int codec_regs[WM8985_CACHEREGNUM];


#define CODEC_UPDATE_REG(reg) wm8985_write(reg, codec_regs[reg])

struct sample_rate_reg_info {
        u32 sample_rate;
	int sr;
	int mclk_div;
	int bclk_div;
	int pllprescale;
	int plln;
	int pllk1;
	int pllk2;
	int pllk3;
};

/* DAC sampling rates (MCLK = 12 MHz) */
/* all setting are 128 fs because bclk=1 => they are all spdif compatible */
static const struct sample_rate_reg_info
sample_rate_info_mclk12[] = {
/*	 SR,  sr, mclk,bclk,  prescale, plln,  pllk1,  pllk2,  pllk3,  */

        {44100,    0,    2,    1,     0,        7,     0x021,  0x161,  0x026     },
        {22050,    2,    4,    1,     0,        7,     0x021,  0x161,  0x026     },
        {11025,    4,    6,    1,     0,        7,     0x021,  0x161,  0x026     },

        {32000,    1,    3,    1,     0,        8,     0x00c,  0x093,  0x0e8     },
        {16000,    3,    5,    1,     0,        8,     0x00c,  0x093,  0x0e8     },
        { 8000,    5,    7,    1,     0,        8,     0x00c,  0x093,  0x0e8     },
 
        {48000,    0,    2,    1,     0,        8,     0x00c,  0x093,  0x0e8     },
        {24000,    2,    4,    1,     0,        8,     0x00c,  0x093,  0x0e8     },
        {12000,    4,    6,    1,     0,        8,     0x00c,  0x093,  0x0e8     },
};

static const struct sample_rate_reg_info
sample_rate_info_mclk19[] = {
/*	 SR,   sr,mclk,bclk,prescale,plln, pllk1,pllk2,pllk3,  */

        {44100,    0,    2,    1,     1,        9,     0x01a,  0x039,  0x0af     },
        {22050,    2,    4,    1,     1,        9,     0x01a,  0x039,  0x0af     },
        {11025,    4,    6,    1,     1,        9,     0x01a,  0x039,  0x0af     },

        {32000,    1,    3,    1,     1,        0xa,   0x00f,  0x0b8,  0x0a3     },
        {16000,    3,    5,    1,     1,        0xa,   0x00f,  0x0b8,  0x0a3     },
        { 8000,    5,    7,    1,     1,        0xa,   0x00f,  0x0b8,  0x0a3     },
 
        {48000,    0,    2,    1,     1,        0xa,   0x00f,  0x0b8,  0x0a3     },
        {24000,    2,    4,    1,     1,        0xa,   0x00f,  0x0b8,  0x0a3     },
        {12000,    4,    6,    1,     1,        0xa,   0x00f,  0x0b8,  0x0a3     },
};

static struct sample_rate_reg_info *sample_rate_info;

#define NUMBER_SAMPLE_RATES_SUPPORTED SIZEOF(sample_rate_info_mclk19)

static const struct sample_rate_reg_info
spdif_sample_rate_info_mclk12[] = {
/*	 SR, sr,  mclk, bclk, prescale, plln,  pllk1,  pllk2,  pllk3	 */
        {48000,    0,    2,    1,     0,        8,     0x00c,  0x093,  0x0e8     },
        {44100,    0,    2,    1,     0,        7,     0x021,  0x161,  0x026     },
        {32000,    1,    2,    1,     1,        10,    0x03b,  0x019,  0x1e2     },
};

static const struct sample_rate_reg_info
spdif_sample_rate_info_mclk19[] = {
/*	 SR,   sr, mclk,bclk,prescale,plln,pllk1,pllk2,pllk3	 */
        {48000,    0,    2,    1,     1,        0xa,   0x00f,  0x0b8,  0x0a3     },
        {44100,    0,    2,    1,     1,        9,     0x01a,  0x039,  0x0af     },
        {32000,    1,    2,    1,     1,        6,     0x034,  0x1d0,  0x06d     },
};

static struct sample_rate_reg_info *spdif_sample_rate_info;

#define NUMBER_SPDIF_SAMPLE_RATES_SUPPORTED SIZEOF(spdif_sample_rate_info_mclk12)


// #if defined USE_DSP_32BIT_MODE
/* mode dsp */

#define	WM8985_DIGITAL_AUDIO_INTERFACE_DEFAULT (DSP_MODE | WORD_LENGTH16 | DLRSWAP)
/* falling edge */
#define TX_EDGE CLKXP

#define	WM8985_CLOCK_GEN_CTRL_DEFAULT          (MS | CLKSEL_PLL)


/* Analog config to be used with omap_mcbsp_config() for debug purpose */
#if 0
static struct omap_mcbsp_reg_cfg initial_config = {
        .spcr2 = FREE | FRST | GRST | XRST | XINTM(3),
        .spcr1 = RRST | RINTM(3),
        .rcr2  = RCOMPAND(OMAP_MCBSP_MSBFIRST) | RDATDLY(1),
        .rcr1  = RFRLEN1(0) | RWDLEN1(OMAP_MCBSP_WORD_32),
        .xcr2  = XCOMPAND(OMAP_MCBSP_MSBFIRST) | XDATDLY(1),
        .xcr1  = XFRLEN1(0) | XWDLEN1(OMAP_MCBSP_WORD_32),
        .srgr1 = 0,
        .srgr2 = 0,
#ifndef WM8985_MASTER
        /* configure McBSP to be the master */
        .pcr0  = FSXM | FSRM | CLKXM | CLKRM | CLKXP | CLKRP,
#else
        /* configure McBSP to be the slave */
        .pcr0  = CLKXP | CLKRP,
#endif /* WM8985_MASTER */
	.xccr = DXENDLY(1) | XDMAEN,
	.rccr = RFULL_CYCLE | RDMAEN,
};
#endif

static struct mcbsp_config mcbsp_config_analog_out = {
	.rx_param = {
#ifdef WM8985_MASTER
		.fsync_src = OMAP_MCBSP_RXFSYNC_EXTERNAL,
		.clk_mode = OMAP_MCBSP_CLKRXSRC_EXTERNAL,
#else
		.fsync_src = OMAP_MCBSP_RXFSYNC_INTERNAL,
		.clk_mode = OMAP_MCBSP_CLKRXSRC_INTERNAL,
#endif
		.fs_polarity = OMAP_MCBSP_FS_ACTIVE_HIGH,
		.clk_polarity = OMAP_MCBSP_CLKR_POLARITY_RISING,
		.frame_length1 = OMAP_MCBSP_FRAMELEN_N(1),
		.frame_length2 = OMAP_MCBSP_FRAMELEN_N(1),
		.word_length1 = OMAP_MCBSP_WORD_32,
		.word_length2 = OMAP_MCBSP_WORD_8,
		.justification = OMAP_MCBSP_RJUST_ZEROMSB,
		.reverse_compand = OMAP_MCBSP_MSBFIRST,
		.phase = OMAP_MCBSP_FRAME_SINGLEPHASE,
		.data_delay = OMAP_MCBSP_DATADELAY1,
	},

	.tx_param = {
#ifdef WM8985_MASTER
		.fsync_src = OMAP_MCBSP_TXFSYNC_EXTERNAL,
		.clk_mode = OMAP_MCBSP_CLKTXSRC_EXTERNAL,
#else
		.fsync_src = OMAP_MCBSP_TXFSYNC_INTERNAL,
		.clk_mode = OMAP_MCBSP_CLKTXSRC_INTERNAL,
#endif
		.fs_polarity = OMAP_MCBSP_FS_ACTIVE_HIGH,
		.clk_polarity = OMAP_MCBSP_CLKX_POLARITY_FALLING,
		.frame_length1 = OMAP_MCBSP_FRAMELEN_N(1),
		.frame_length2 = OMAP_MCBSP_FRAMELEN_N(1),
		.word_length1 = OMAP_MCBSP_WORD_32,
		.word_length2 = OMAP_MCBSP_WORD_8,
		.justification = OMAP_MCBSP_RJUST_ZEROMSB,
		.reverse_compand = OMAP_MCBSP_MSBFIRST,
		.phase = OMAP_MCBSP_FRAME_SINGLEPHASE,
		.data_delay = OMAP_MCBSP_DATADELAY1,
	},

	.srg_fsg_param = {
		.period = 0,					/* Frame period */
		.pulse_width = 0, 				/* Frame width */
		.fsgm = 0,
		.sample_rate = 0,
		.bits_per_sample = 0,
#ifdef WM8985_MASTER
		.srg_src = OMAP_MCBSP_SRGCLKSRC_CLKS,
		.sync_mode = OMAP_MCBSP_SRG_FREERUNNING,	/* SRG free running mode */
#else
		.srg_src = OMAP_MCBSP_SRGCLKSRC_CLKX,
		.sync_mode = OMAP_MCBSP_SRG_RUNNING,		/* SRG free running mode */
#endif
		.polarity = OMAP_MCBSP_CLKS_POLARITY_RISING,
		.dlb = 0,					/* digital loopback mode */
	},

	.dma_recv = {
		/* Skip the alternate element use fro stereo mode */
		.skip_alt = OMAP_MCBSP_SKIP_NONE,
		/* Automagically handle Transfer [XR]RST? */
		.auto_reset = OMAP_MCBSP_AUTO_RRST,
		/* callback function executed for every tx/rx completion */
		.callback = (omap_mcbsp_dma_cb) codec_mcbsp_dma_cb,
		/* word length of data */
		.word_length1 = OMAP_MCBSP_WORD_32,		/* Should be the same as .rx_param.word_length1 */
	},
	
	.dma_trans = {
		/* Skip the alternate element use fro stereo mode */
		.skip_alt = OMAP_MCBSP_SKIP_NONE,
		/* Automagically handle Transfer [XR]RST? */
		.auto_reset = OMAP_MCBSP_AUTO_XRST,
		/* callback function executed for every tx/rx completion */
		.callback = (omap_mcbsp_dma_cb) codec_mcbsp_dma_cb,
		/* word length of data */
		.word_length1 = OMAP_MCBSP_WORD_32,		/* Should be the same as .tx_param.word_length1 */
	},
};

static struct mcbsp_config mcbsp_config_spdif = {
	.rx_param = {
#ifdef WM8985_MASTER
		.fsync_src = OMAP_MCBSP_RXFSYNC_EXTERNAL,
		.clk_mode = OMAP_MCBSP_CLKRXSRC_EXTERNAL,
#else
		.fsync_src = OMAP_MCBSP_RXFSYNC_INTERNAL,
		.clk_mode = OMAP_MCBSP_CLKRXSRC_INTERNAL,
#endif
		.fs_polarity = OMAP_MCBSP_FS_ACTIVE_HIGH,
		.clk_polarity = OMAP_MCBSP_CLKR_POLARITY_RISING,
		.frame_length1 = OMAP_MCBSP_FRAMELEN_N(1),
		.frame_length2 = OMAP_MCBSP_FRAMELEN_N(1),
		.word_length1 = OMAP_MCBSP_WORD_32,
		.word_length2 = OMAP_MCBSP_WORD_8,
		.justification = OMAP_MCBSP_RJUST_ZEROMSB,
		.reverse_compand = OMAP_MCBSP_MSBFIRST,
		.phase = OMAP_MCBSP_FRAME_SINGLEPHASE,
		.data_delay = OMAP_MCBSP_DATADELAY1,
	},

	.tx_param = {
#ifdef WM8985_MASTER
		.fsync_src = OMAP_MCBSP_TXFSYNC_EXTERNAL,
		.clk_mode = OMAP_MCBSP_CLKTXSRC_EXTERNAL,
#else
		.fsync_src = OMAP_MCBSP_TXFSYNC_INTERNAL,
		.clk_mode = OMAP_MCBSP_CLKTXSRC_INTERNAL,
#endif
		.fs_polarity = OMAP_MCBSP_FS_ACTIVE_HIGH,
		.clk_polarity = OMAP_MCBSP_CLKX_POLARITY_FALLING,
		.frame_length1 = OMAP_MCBSP_FRAMELEN_N(4),
		.frame_length2 = OMAP_MCBSP_FRAMELEN_N(1),
		.word_length1 = OMAP_MCBSP_WORD_32,
		.word_length2 = OMAP_MCBSP_WORD_8,
		.justification = OMAP_MCBSP_RJUST_ZEROMSB,
		.reverse_compand = OMAP_MCBSP_LSBFIRST,
		.phase = OMAP_MCBSP_FRAME_SINGLEPHASE,
		.data_delay = OMAP_MCBSP_DATADELAY0,
	},

	.srg_fsg_param = {
		.period = 0,					/* Frame period */
		.pulse_width = 0, 				/* Frame width */
		.fsgm = 0,
		.sample_rate = 0,
		.bits_per_sample = 0,
#ifdef WM8985_MASTER
		.srg_src = OMAP_MCBSP_SRGCLKSRC_CLKS,
		.sync_mode = OMAP_MCBSP_SRG_FREERUNNING,	/* SRG free running mode */
#else
		.srg_src = OMAP_MCBSP_SRGCLKSRC_CLKX,
		.sync_mode = OMAP_MCBSP_SRG_RUNNING,		/* SRG free running mode */
#endif
		.polarity = OMAP_MCBSP_CLKS_POLARITY_RISING,
		.dlb = 0,					/* digital loopback mode */
	},

	.dma_recv = {
		/* Skip the alternate element use fro stereo mode */
		.skip_alt = OMAP_MCBSP_SKIP_NONE,
		/* Automagically handle Transfer [XR]RST? */
		.auto_reset = OMAP_MCBSP_AUTO_RRST,
		/* callback function executed for every tx/rx completion */
		.callback = (omap_mcbsp_dma_cb) codec_mcbsp_dma_cb,
		/* word length of data */
		.word_length1 = OMAP_MCBSP_WORD_32,		/* Should be the same as .rx_param.word_length1 */
	},
	
	.dma_trans = {
		/* Skip the alternate element use fro stereo mode */
		.skip_alt = OMAP_MCBSP_SKIP_NONE,
		/* Automagically handle Transfer [XR]RST? */
		.auto_reset = OMAP_MCBSP_AUTO_XRST,
		/* callback function executed for every tx/rx completion */
		.callback = (omap_mcbsp_dma_cb) codec_mcbsp_dma_cb,
		/* word length of data */
		.word_length1 = OMAP_MCBSP_WORD_32,		/* Should be the same as .tx_param.word_length1 */
	},
};

#if 0  /* Old configuration way */
/*
 * The WM8985 will always use stereo DSP protocol to communicate
 *
 * McBSP Configuration Required:
 * Stereo 16 bit:(default)
 * -------------
 * Single phase, FSYNC=Rising, words=1 DMA->Normal,32bit DXR
 *
 *
 */
static struct mcbsp_config plat_mcbsp_analog_out_config = {
#if defined(CONFIG_ARCH_OMAP3430)
	.mcbsp_clk_src = OMAP2_MCBSP_FCLKSRC_PRCM,
#ifdef WM8985_MASTER
	/* Driven by twl4030 */
	.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKS,
	.tx_clk_src = OMAP2_MCBSP_TXFSYNC_EXTERNAL,
	.rx_clk_src = OMAP2_MCBSP_RXFSYNC_EXTERNAL,
	.rx_ip_clk = OMAP2_MCBSP_CLKRXSRC_EXTERNAL,	
	.tx_ip_clk = OMAP2_MCBSP_CLKTXSRC_EXTERNAL,

	.srg_clk_sync = OMAP2_MCBSP_SRG_FREERUNNING,
#else
	/* Driven by OMAP */
	.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
	.tx_clk_src = OMAP2_MCBSP_TXFSYNC_INTERNAL,
	.rx_clk_src = OMAP2_MCBSP_RXFSYNC_INTERNAL,
	.rx_ip_clk = OMAP2_MCBSP_CLKRXSRC_INTERNAL,	
	.tx_ip_clk = OMAP2_MCBSP_CLKTXSRC_INTERNAL,

	.srg_clk_sync = OMAP2_MCBSP_SRG_RUNNING,
#endif
 	.tx_polarity = OMAP2_MCBSP_FS_ACTIVE_HIGH,
	.rx_polarity = OMAP2_MCBSP_FS_ACTIVE_HIGH,
	.srg_clk_pol = OMAP2_MCBSP_CLKS_POLARITY_RISING,
	.tx_clk_pol = OMAP2_MCBSP_CLKX_POLARITY_FALLING,
	.rx_clk_pol = OMAP2_MCBSP_CLKR_POLARITY_RISING,
	/* we will start with right channel and transmit the MSB of DXR */
	.fs_clk_pol = OMAP2_MCBSP_FS_ACTIVE_HIGH,
	/* DSP */
	.tx_params = {
			       .data_type = OMAP2_MCBSP_WORDLEN_NONE,
			       .skip_alt = OMAP2_MCBSP_SKIP_NONE,
			       .auto_reset = OMAP2_MCBSP_AUTO_XRST,
			       .phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
#if defined (CONFIG_MACH_OMAP3_EVM)
 			       .data_delay = OMAP2_MCBSP_DATADELAY2,	
#elif defined (CONFIG_MACH_ARCHOS)
				.data_delay = OMAP2_MCBSP_DATADELAY1,	/* 2 bit delay expected: wm8985 will sample on the 2nd rising edge, so transmit on the 2nd falling edge */
#endif
			       .reverse_compand = OMAP2_MCBSP_MSBFIRST,	/* Set msb first */
			       .word_length1 = OMAP2_MCBSP_WORDLEN_32,	/* RWDLEN1 */
			       .word_length2 = OMAP2_MCBSP_WORDLEN_8,	/* RWDLEN2 -dnt care*/
			       .frame_length1 = OMAP2_MCBSP_FRAMELEN_1,	/* RFRLEN1 */
			       .frame_length2 = OMAP2_MCBSP_FRAMELEN_1,	/* RFRLEN2 -dnt care*/
			       .justification = OMAP2_MCBSP_RJUST_ZEROMSB,	/* RJUST and fill 0s */
			       .dxena = 0,
			       .dxendly = 0,
			       .callback = codec_mcbsp_dma_cb,
			       },
	.rx_params = {
			       .data_type = OMAP2_MCBSP_WORDLEN_NONE,
			       .skip_alt = OMAP2_MCBSP_SKIP_NONE,
			       .auto_reset = OMAP2_MCBSP_AUTO_RRST,
			       .phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
			       .data_delay = OMAP2_MCBSP_DATADELAY1,	/* 1 bit delay */
			       .reverse_compand = OMAP2_MCBSP_MSBFIRST,	/* Set msb first */
			       .word_length1 = OMAP2_MCBSP_WORDLEN_32,	/* RWDLEN1 */
			       .word_length2 = OMAP2_MCBSP_WORDLEN_8,	/* RWDLEN2 -dntcare*/
			       .frame_length1 = OMAP2_MCBSP_FRAMELEN_1,	/* RFRLEN1 */
			       .frame_length2 = OMAP2_MCBSP_FRAMELEN_1,	/* RFRLEN2 */
			       .justification = OMAP2_MCBSP_RJUST_ZEROMSB,	/* RJUST and fill 0s */
			       .dxena = 0,
			       .dxendly = 0,
			       .callback = codec_mcbsp_dma_cb,
			       },
#else
#error "Unsupported Platform"
#endif
};


static struct mcbsp_config plat_mcbsp_spdif_config = {
#if defined(CONFIG_ARCH_OMAP3430)

	.mcbsp_clk_src = OMAP2_MCBSP_FCLKSRC_PRCM,
#ifdef WM8985_MASTER
	/* Driven by twl4030 */
	.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKS,
	.tx_clk_src = OMAP2_MCBSP_TXFSYNC_EXTERNAL,
	.rx_clk_src = OMAP2_MCBSP_RXFSYNC_EXTERNAL,
	.rx_ip_clk = OMAP2_MCBSP_CLKRXSRC_EXTERNAL,	
	.tx_ip_clk = OMAP2_MCBSP_CLKTXSRC_EXTERNAL,

	.srg_clk_sync = OMAP2_MCBSP_SRG_FREERUNNING,
#else
	/* Driven by OMAP */
	.srg_clk_src = OMAP2_MCBSP_SRGCLKSRC_CLKX,
	.tx_clk_src = OMAP2_MCBSP_TXFSYNC_INTERNAL,
	.rx_clk_src = OMAP2_MCBSP_RXFSYNC_INTERNAL,
	.rx_ip_clk = OMAP2_MCBSP_CLKRXSRC_INTERNAL,	
	.tx_ip_clk = OMAP2_MCBSP_CLKTXSRC_INTERNAL,

	.srg_clk_sync = OMAP2_MCBSP_SRG_RUNNING,
#endif
 	.tx_polarity = OMAP2_MCBSP_FS_ACTIVE_HIGH,
	.rx_polarity = OMAP2_MCBSP_FS_ACTIVE_HIGH,
	.srg_clk_pol = OMAP2_MCBSP_CLKS_POLARITY_RISING,
	.tx_clk_pol = OMAP2_MCBSP_CLKX_POLARITY_FALLING,
	.rx_clk_pol = OMAP2_MCBSP_CLKR_POLARITY_RISING,
	/* we will start with right channel and transmit the MSB of DXR */
	.fs_clk_pol = OMAP2_MCBSP_FS_ACTIVE_HIGH,
	/* I2S */
	.tx_params = {
			       .data_type = OMAP2_MCBSP_WORDLEN_NONE,
			       .skip_alt = OMAP2_MCBSP_SKIP_NONE,
			       .auto_reset = OMAP2_MCBSP_AUTO_XRST,
			       .phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
#if defined (CONFIG_MACH_OMAP3_EVM)
 			       .data_delay = OMAP2_MCBSP_DATADELAY1,	
			       .reverse_compand = OMAP2_MCBSP_LSBFIRST,	/* Set lsb first */
#elif defined (CONFIG_MACH_ARCHOS)
				.data_delay = OMAP2_MCBSP_DATADELAY0,   /* 1 bit delay expected: wm8985 will sample on the 2nd rising edge, so transmit on the 2nd falling edge */
			      	.reverse_compand = OMAP2_MCBSP_LSBFIRST,/* Set lsb first */
#endif


			       .word_length1 = OMAP2_MCBSP_WORDLEN_32,	/* RWDLEN1 */
			       .word_length2 = OMAP2_MCBSP_WORDLEN_8,	/* RWDLEN2 -dnt care*/
			       .frame_length1 = OMAP2_MCBSP_FRAMELEN_4,	/* RFRLEN1 */
			       .frame_length2 = OMAP2_MCBSP_FRAMELEN_1,	/* RFRLEN2 -dnt care*/
			       .justification = OMAP2_MCBSP_RJUST_ZEROMSB,	/* RJUST and fill 0s */
			       .dxena = 0,
			       .dxendly = 0,
			       .callback = codec_mcbsp_dma_cb,
			       },
	.rx_params = {
			       .data_type = OMAP2_MCBSP_WORDLEN_NONE,
			       .skip_alt = OMAP2_MCBSP_SKIP_NONE,
			       .auto_reset = OMAP2_MCBSP_AUTO_RRST,
			       .phase = OMAP2_MCBSP_FRAME_SINGLEPHASE,
			       .data_delay = OMAP2_MCBSP_DATADELAY1,	/* 1 bit delay */
			       .reverse_compand = OMAP2_MCBSP_MSBFIRST,	/* Set msb first */
			       .word_length1 = OMAP2_MCBSP_WORDLEN_32,	/* RWDLEN1 */
			       .word_length2 = OMAP2_MCBSP_WORDLEN_8,	/* RWDLEN2 -dntcare*/
			       .frame_length1 = OMAP2_MCBSP_FRAMELEN_1,	/* RFRLEN1 */
			       .frame_length2 = OMAP2_MCBSP_FRAMELEN_1,	/* RFRLEN2 */
			       .justification = OMAP2_MCBSP_RJUST_ZEROMSB,	/* RJUST and fill 0s */
			       .dxena = 0,
			       .dxendly = 0,
			       .callback = codec_mcbsp_dma_cb,
			       },
#else
#error "Unsupported Platform"
#endif
};
#endif

static struct mcbsp_config *mcbsp_config_current;

static struct stream_callback_struct in_stream_data, out_stream_data;

static int codec_initialize(struct audio_state_s * state, void *data);
static void codec_shutdown(void *data);
static int  codec_ioctl(struct inode *inode, struct file *file,
                             uint cmd, ulong arg);
static int  codec_probe(void);
static void codec_remove(void);
static int codec_suspend(struct audio_state_s * state);
static int codec_resume(struct audio_state_s * state);

static void codec_config_init(struct codec_local_info * c_info);
static void codec_config_init_first(struct codec_local_info* c_info);
static void codec_route_input(struct codec_local_info *c_info);
static void codec_route_output(struct codec_local_info *c_info);

static int codec_monitor_headphone(void *data);

static int  mixer_open(struct inode *inode, struct file *file);
static int  mixer_release(struct inode *inode, struct file *file);
static int  mixer_ioctl(struct inode *inode, struct file *file, uint cmd,
                        ulong arg);
static ssize_t mixer_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos);
static unsigned int mixer_poll(struct file * file, poll_table * wait);
static void power_down_output (void);
static int codec_update(int flag);
static int wm8985_set_equalizer( wm8985_eqParams_t *newParams );

static struct sample_rate_reg_info* _select_normal_pll_tab(int mclk, int spdif);

/* File Op structure for mixer */
static struct file_operations mixer_fops = {
        .open           = mixer_open,
        .release        = mixer_release,
        .ioctl          = mixer_ioctl,
        .read           = mixer_read,
        .poll           = mixer_poll,
        .owner          = THIS_MODULE
};

/* To store characteristic info regarding the codec for the audio driver */
static audio_state_t codec_state = {
	.owner = THIS_MODULE,
	.output_stream = &output_stream,
	.input_stream = &input_stream,
	.need_tx_for_rx = 0,
	.hw_init = codec_initialize,
	.hw_shutdown = codec_shutdown,
	.client_ioctl = codec_ioctl,
	.hw_probe = codec_probe,
	.hw_remove = codec_remove,
	.hw_suspend = codec_suspend,
	.hw_resume = codec_resume,
	.hw_prescale = codec_prescale,
	.hw_postscale = codec_postscale,
	.hw_transfer = codec_transfer,
	.hw_transfer_stop = codec_transfer_stop,
	.hw_transfer_posn = codec_transfer_posn,
	.hw_transfer_init = codec_transfer_init,
};


static void _enable_spdif_ampli(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->set_spdif)
		pt_audio_device_io->set_spdif(PLAT_ON);
#endif
}

static void _disable_spdif_ampli(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->set_spdif)
		pt_audio_device_io->set_spdif(PLAT_OFF);
#endif
}

static void _enable_master_clock(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->set_codec_master_clk_state)
		pt_audio_device_io->set_codec_master_clk_state(PLAT_ON);
#endif
}

static void _disable_master_clock(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->set_codec_master_clk_state)
		pt_audio_device_io->set_codec_master_clk_state(PLAT_OFF);
#endif
}

static inline void _enable_speaker_ampli(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->set_speaker_state)
		pt_audio_device_io->set_speaker_state(PLAT_ON);
#endif
}

static inline void _disable_speaker_ampli(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->set_speaker_state)
		pt_audio_device_io->set_speaker_state(PLAT_OFF);
#endif
}

static void _set_speaker(int val) {

	if (val == 0) {
		_disable_speaker_ampli();
	} else {
		_enable_speaker_ampli();
	}
}


static int _get_headphone_state(void) {
#if defined (CONFIG_MACH_ARCHOS)
	if (pt_audio_device_io->get_headphone_plugged)
		return pt_audio_device_io->get_headphone_plugged();
#endif
	return 0;
}

/*
void mcbsp_dump_reg(int mcbsp_id) {
	struct omap_mcbsp *mcbsp;
	void __iomem *virt_base;

	mcbsp = id_to_mcbsp_ptr(mcbsp_id);
	virt_base = mcbsp->io_base;

	printk("\n***************************\n");
	printk("\n    MCBSP ID  = %d\n", mcbsp_id);
	printk("\n***************************\n");

	printk("OMAP2_MCBSP_REG_SYSCONFIG [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_SYSCON, omap_mcbsp_read(virt_base,
								    OMAP_MCBSP_REG_SYSCON));
	printk("OMAP2_MCBSP_REG_THRSH2 [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_THRSH2, omap_mcbsp_read(virt_base,
								    OMAP_MCBSP_REG_THRSH2));
	printk("OMAP2_MCBSP_REG_THRSH1 [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_THRSH1, omap_mcbsp_read(virt_base,
								    OMAP_MCBSP_REG_THRSH1));
	printk("OMAP2_MCBSP_REG_XCCR  [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_XCCR, omap_mcbsp_read(virt_base,
								  OMAP_MCBSP_REG_XCCR));
	printk("OMAP2_MCBSP_REG_RCCR  [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_RCCR, omap_mcbsp_read(virt_base,
								  OMAP_MCBSP_REG_RCCR));
	printk("OMAP2_MCBSP_REG_RCR2  [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_RCR2, omap_mcbsp_read(virt_base,
								  OMAP_MCBSP_REG_RCR2));

	printk("OMAP2_MCBSP_REG_RCR1  [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_RCR1, omap_mcbsp_read(virt_base,
								  OMAP_MCBSP_REG_RCR1));

	printk("OMAP2_MCBSP_REG_XCR2  [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_XCR2, omap_mcbsp_read(virt_base,
								  OMAP_MCBSP_REG_XCR2));

	printk("OMAP2_MCBSP_REG_XCR1  [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_XCR1, omap_mcbsp_read(virt_base,
								  OMAP_MCBSP_REG_XCR1));

	printk("OMAP2_MCBSP_REG_SRGR2 [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_SRGR2, omap_mcbsp_read(virt_base,
								   OMAP_MCBSP_REG_SRGR2));

	printk("OMAP2_MCBSP_REG_SRGR1 [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_SRGR1, omap_mcbsp_read(virt_base,
								   OMAP_MCBSP_REG_SRGR1));

	printk("OMAP2_MCBSP_REG_PCR   [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_PCR0, omap_mcbsp_read(virt_base,
								 OMAP_MCBSP_REG_PCR0));

	printk("OMAP2_MCBSP_REG_SPCR2 [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_SPCR2, omap_mcbsp_read(virt_base,
								   OMAP_MCBSP_REG_SPCR2));

	printk("OMAP2_MCBSP_REG_SPCR1 [0x%08x] = 0x%08x\n",
	       virt_base + OMAP_MCBSP_REG_SPCR1, omap_mcbsp_read(virt_base,
								   OMAP_MCBSP_REG_SPCR1));

	printk("OMAP2_MCBSP_REG_REV   [0x%08x] = 0x%08x\n",
	       virt_base + 0x7c, omap_mcbsp_read(virt_base,
								 0x7c));

}
*/

struct wm87xx_regval {
	int reg;
	int val;
};

/*
 * read wm8985 register cache
 */
static inline unsigned int wm8985_read_reg_cache(int reg)
{
	unsigned int *cache = codec_regs;
	if (reg == WM8985_RESET)
		return 0;
	if (reg >= WM8985_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write wm8985 register cache
 */
static inline void wm8985_write_reg_cache(int reg, unsigned int value)
{
	unsigned int *cache = codec_regs;

	if (reg >= WM8985_CACHEREGNUM)
		return;

	cache[reg] = value;
}

static void wm8985_write(int reg, unsigned int data)
{
	struct wm87xx_regval regval;

 	wm8985_write_reg_cache (reg, data);

	regval.reg = reg;
	regval.val = data;

	codec_handle->driver->command(codec_handle, 0, &regval);
}

static void wm8985_field_write(int reg, unsigned int data, unsigned int mask)
{
	unsigned int next_data;

	next_data = wm8985_read_reg_cache(reg);
	next_data &= (~mask);
	next_data |= data;

	wm8985_write(reg,next_data);
}

static void wm8985_reset(void)
{
	wm8985_write(WM8985_RESET, 0);
	memcpy(codec_regs, codec_regs_default, sizeof(unsigned int) * ARRAY_SIZE(codec_regs));
}

static int wm8985_dac_mute(int mute)
{
	if (mute)
		wm8985_field_write(WM8985_DAC_CTRL, DAC_SMUTE, DAC_SMUTEM);
	else
		wm8985_field_write(WM8985_DAC_CTRL, DAC_SUNMUTE, DAC_SMUTEM);
	return 0;
}

#define wm8985_out1_mute_left() {\
	wm8985_field_write(WM8985_LOUT1V, LHP_MUTE, LHP_MUTEM);\
}\

#define wm8985_out1_mute_right() {\
	wm8985_field_write(WM8985_ROUT1V, RHP_MUTE, RHP_MUTEM);\
}\

#define wm8985_out1_unmute_left() {\
	wm8985_field_write(WM8985_LOUT1V, LHP_UNMUTE, LHP_MUTEM);\
}\

#define wm8985_out1_unmute_right() {\
	wm8985_field_write(WM8985_ROUT1V, RHP_UNMUTE, RHP_MUTEM);\
}\

#define wm8985_out2_mute_left() {\
	wm8985_field_write(WM8985_LOUT2V, LHP_MUTE, LHP_MUTEM);\
}\

#define wm8985_out2_mute_right() {\
	wm8985_field_write(WM8985_ROUT2V, RHP_MUTE, RHP_MUTEM);\
}\

#define wm8985_out2_unmute_left() {\
	wm8985_field_write(WM8985_LOUT2V, LHP_UNMUTE, LHP_MUTEM);\
}\

#define wm8985_out2_unmute_right() {\
	wm8985_field_write(WM8985_ROUT2V, RHP_UNMUTE, RHP_MUTEM);\
}\

static void wm8985_hp_mute_left(void)
{
	if (is_gen7) {
		wm8985_out1_mute_left();
	}
	else {
		wm8985_out2_mute_left();
	}
	return;
}

static void wm8985_hp_mute_right(void)
{
	if (is_gen7) {
		wm8985_out1_mute_right();
	}
	else {
		wm8985_out2_mute_right();
	}
	return;
}

static void wm8985_hp_unmute_left(void)
{
	if (is_gen7) {
		wm8985_out1_unmute_left();
	}
	else {
		wm8985_out2_unmute_left();
	}
	return;
}

static void wm8985_hp_unmute_right(void)
{
	if (is_gen7) {
		wm8985_out1_unmute_right();
	}
	else {
		wm8985_out2_unmute_right();
	}
	return;
}

static void wm8985_speaker_mute_left(void)
{
	if (is_gen7) wm8985_out2_mute_left();
	return;
}
static void wm8985_speaker_mute_right(void)
{
	if (is_gen7) wm8985_out2_mute_right();
	return;
}

static void wm8985_speaker_unmute_left(void)
{
	if (is_gen7) wm8985_out2_unmute_left();
	return;
}

static void wm8985_speaker_unmute_right(void)
{
	if (is_gen7) wm8985_out2_unmute_right();
	return;
}

static void wm8985_hp_mute(void)
{
	wm8985_hp_mute_left();
	wm8985_hp_mute_right();
	return;
}

static void wm8985_hp_unmute(void)
{
	wm8985_hp_unmute_left();
	wm8985_hp_unmute_right();
	return;
}

static void wm8985_speaker_mute(void)
{
	wm8985_speaker_mute_left();
	wm8985_speaker_mute_right();
	return;
}

static void wm8985_speaker_unmute(void)
{
	wm8985_speaker_unmute_left();
	wm8985_speaker_unmute_right();
	return;
}

static void wm8985_set_loop(int on) {

	if (use_analog_loop) {
		if (on) {
			/* left mixer: right dac to right out, input wired to output */
			wm8985_write(WM8985_LEFT_MIXER_CTRL, DACL2LMIX | BYPL2LMIX );
			/* right mixer: right dac to right out, input wired to output */
			wm8985_write(WM8985_RIGHT_MIXER_CTRL,DACR2RMIX | BYPR2RMIX);
		} else {
			wm8985_write(WM8985_LEFT_MIXER_CTRL,DACL2LMIX);
			wm8985_write(WM8985_RIGHT_MIXER_CTRL,DACR2RMIX);
		}
	}
}
#define wm8985_set_loop_on() wm8985_set_loop(1)
#define wm8985_set_loop_off() wm8985_set_loop(0)


static void wm8985_connect_input(void) {

	if (use_pga) {
		printk(KERN_DEBUG "select PGA input path\n");
// 		/* connect L2 & R2 to PGA */
// 		/* also connect LIN and RIN to the amplifier negative terminal */
// 		wm8985_write(WM8985_INPUT_CTRL,(L2_2INPPGA|R2_2INPPGA|LIN_2INPPGA|RIN_2INPPGA));


		/* OUT4 mixer already disconnect from boost */

		/* connect PGA to boost */
		wm8985_field_write(WM8985_LEFT_PGA_GAIN,INPPGAUNMUTEL,INPPGAMUTELM);
		wm8985_field_write(WM8985_RIGHT_PGA_GAIN,INPPGAUNMUTER,INPPGAMUTERM);

		/* set PGA to boost mix with 0dB */
		/* disconnect also L2/R2 and AUX */
		wm8985_write(WM8985_LEFT_ADC_BOOST,PGABOOSTL_0DB);
		wm8985_write(WM8985_RIGHT_ADC_BOOST,PGABOOSTL_0DB);

	} else {
		printk(KERN_DEBUG "select direct input path\n");
		/* disconnect R2 from PGA */
		wm8985_write(WM8985_INPUT_CTRL,0);

		/* disconnect PGA from boost */
 		wm8985_field_write(WM8985_LEFT_PGA_GAIN,(INPPGAMUTEL|INPPGAVOLL_MIN),(INPPGAMUTELM|LINVOLM));
 		wm8985_field_write(WM8985_RIGHT_PGA_GAIN,(INPPGAMUTER|INPPGAVOLL_MIN),(INPPGAMUTERM|LINVOLM));

		/* set PGA to boost mix with 0dB */
		/* disconnect also L2/R2 and AUX */
		wm8985_write(WM8985_LEFT_ADC_BOOST,0);
		wm8985_write(WM8985_RIGHT_ADC_BOOST,0);
		/* connect L2/R2 to boost by setting the level */
	}

	codec_update(SET_REC_LEVEL);

}

static void wm8985_disconnect_input(void) {
	
	/* disconnect all */

	/* disconnect R2 from PGA */
	wm8985_write(WM8985_INPUT_CTRL,0);

	/* disconnect PGA from boost */
	wm8985_field_write(WM8985_LEFT_PGA_GAIN,INPPGAMUTEL,INPPGAMUTELM);
	wm8985_field_write(WM8985_RIGHT_PGA_GAIN,INPPGAMUTER,INPPGAMUTERM);

	/* disconnect also L2/R2 and AUX */
	wm8985_write(WM8985_LEFT_ADC_BOOST,0);
	wm8985_write(WM8985_RIGHT_ADC_BOOST,0);

}

static void wm8985_power_up_input(void) {

	if (use_pga) {
		wm8985_field_write(WM8985_PWR_MGT2,
		(BOOSTENR|BOOSTENL|INPPGAENR|INPPGAENL),
		(BOOSTENRM|BOOSTENLM|INPPGAENRM|INPPGAENLM));
	} else {
		wm8985_field_write(WM8985_PWR_MGT2,
		(BOOSTENR|BOOSTENL),
		(BOOSTENRM|BOOSTENLM));
	}
}

static void wm8985_power_down_input(void) {

	wm8985_field_write(WM8985_PWR_MGT2,
	(BOOSTDISR|BOOSTDISL|INPPGADISR|INPPGADISL),
	(BOOSTENRM|BOOSTENLM|INPPGAENRM|INPPGAENLM));
}


static inline int codec_convert_range(int val, int range, int offset)
{
	 return ((val * range) / 99) + offset;
}

static void mute_sidetone(void)
{
	wm8985_set_loop_off();
}

static void set_sidetone(void)
{
	/* we do not need to set a specific vol for sidetone ampli */
	DPRINTK("set sidetone %d\n",use_analog_loop );

	wm8985_set_loop_on();
}

static int _conv_val_to_dB_att(int val) {

	return  (val >= VOL_0dB ? 0 : (VOL_0dB - val));
}

static void _setup_limiter(int val) {
	wm8985_write(WM8985_DAC_LIM2,LIMLVL(val));
	wm8985_write(WM8985_DAC_LIM1,(LIMATK_DEFAULT|LIMDCY_DEFAULT|LIMEN_EN));
}

static int codec_update(int flag)
{
	int volume_left;
	int volume_right;
	static int volume_left_prev=DEFAULT_OUTPUT_VOLUME;
	static int volume_right_prev=DEFAULT_OUTPUT_VOLUME;
	int dir=0;
	int boost = 0;
	int ret = 0;

        switch (flag) {
        case SET_VOLUME:

		volume_left = codec_info.volume_left;
		volume_right = codec_info.volume_right;

		DPRINTK_VOL("vol request: %d %d\n",volume_left,volume_right);

		if (volume_left) {
			volume_left  = codec_convert_range(volume_left, OUTPUT_VOLUME_RANGE, OUTPUT_VOLUME_MIN)
/* user volume_adjust is 1/2dB per step while wolfson volume is 1dB per step */
					- ((codec_info.volume_adjust)>>1);

			wm8985_hp_unmute_left();
			wm8985_speaker_unmute_left();
		}
		if ( volume_left > OUTPUT_GAIN_MAX ) {
			volume_left = OUTPUT_GAIN_MAX;
		}
		if ( volume_left < OUTPUT_VOLUME_MIN ) {
			volume_left = OUTPUT_VOLUME_MIN;
		}

		if (volume_right) {
			volume_right = codec_convert_range(volume_right, OUTPUT_VOLUME_RANGE, OUTPUT_VOLUME_MIN)
					- ((codec_info.volume_adjust)>>1);
			wm8985_hp_unmute_right();
			wm8985_speaker_unmute_right();
		}

		if ( volume_right > OUTPUT_GAIN_MAX ) {
			volume_right = OUTPUT_GAIN_MAX;
		}
		if ( volume_right < OUTPUT_VOLUME_MIN ) {
			volume_right = OUTPUT_VOLUME_MIN;
		}

		if (prog_vol) {
			dir = 0;
			if (volume_left_prev > volume_left)
				dir = -1;
			else if (volume_left_prev < volume_left)
				dir = +1;
			if (dir!=0) {
				while ( (volume_left_prev != volume_left) &&
					(volume_right_prev != volume_right) ) {
	
						if (volume_left_prev != volume_left) {
							volume_left_prev += dir;
							wm8985_field_write(WM8985_LOUT2V, (volume_left_prev|(zc_switch<<ZCEN_B)), (LHPVOLM|LZCENM));
							/* Gen7 => need to consider OUT1 too */
							if (is_gen7) wm8985_field_write(WM8985_LOUT1V, (volume_left_prev|(zc_switch<<ZCEN_B)), (LHPVOLM|LZCENM));
							msleep(2);
						}
	
						if (volume_right_prev != volume_right) {
							volume_right_prev += dir;
							wm8985_field_write(WM8985_ROUT2V, (volume_right_prev|(zc_switch<<ZCEN_B)), (RHPVOLM|RZCENM));
							/* Gen7 => need to consider OUT1 too */
							if (is_gen7) wm8985_field_write(WM8985_ROUT1V, (volume_right_prev|(zc_switch<<ZCEN_B)), (RHPVOLM|LZCENM));
							msleep(2);
						}
				}
			}
		} else {
			volume_left_prev = volume_left;
			volume_right_prev = volume_right;
			wm8985_field_write(WM8985_LOUT2V, (volume_left_prev|(zc_switch<<ZCEN_B)), (LHPVOLM|LZCENM));
			wm8985_field_write(WM8985_ROUT2V, (volume_right_prev|(zc_switch<<ZCEN_B)), (RHPVOLM|RZCENM));
			/* Gen7 => need to consider OUT1 too */
			if (is_gen7) {
				wm8985_field_write(WM8985_LOUT1V, (volume_left_prev|(zc_switch<<ZCEN_B)), (LHPVOLM|LZCENM));
				wm8985_field_write(WM8985_ROUT1V, (volume_right_prev|(zc_switch<<ZCEN_B)), (RHPVOLM|RZCENM));
			}
		}

		if (!volume_left) {
			wm8985_hp_mute_left();
			wm8985_speaker_mute_left();
		}

		if (!volume_right) {
			wm8985_hp_mute_right();
			wm8985_speaker_mute_right();
		}
		
		// shutdown speaker power if mute
		if ( codec_info.config & OUTPUT_SPK ) {
			if ( !volume_left && !volume_right )
				_set_speaker( 0 );
			else
				_set_speaker( 1 );
		}

		DPRINTK_VOL("OUTPUT lvol: %d\trvol: %d\n",volume_left,volume_right);

		/* get max of analog attenuation */
		/* and return the attenuation in dB */
		/* ret should be positive or zero */
		
		ret = _conv_val_to_dB_att( (volume_left > volume_right ? volume_left : volume_right) );

                break;

        case SET_LINE_GAIN:
                break;

        case SET_MIC_GAIN:
		/* Sidetone Level is the level of input we want to redirect on HP */
			if (codec_info.mic_gain >= INPUT_VOLUME_RANGE_MAX) {
				/* Mute SideTone */
				mute_sidetone();
			} else {
				set_sidetone();
			}
        	break;

	case SET_OUTPUT_GAIN:

		break;

	case SET_REC_LEVEL:

		if (codec_info.config & (INPUT_LINE|INPUT_MIC|INPUT_PHONE))  {

			if ( codec_info.config & INPUT_LINE ) 
				volume_left = codec_info.rec_level_line;
			else if ( codec_info.config & INPUT_MIC )
				volume_left = codec_info.rec_level_mic;
			else
				volume_left = codec_info.rec_level_phone;

			if (use_pga) {
				volume_left  = codec_convert_range(volume_left, LINE_REC_RANGE, LINE_REC_LEVEL_MIN);

				if (volume_left > LINE_REC_LEVEL_MAX) {
					volume_left = volume_left - LINE_REC_LEVEL_MAX + LINE_REC_LEVEL_MIN;
					boost = 1;
				}

				if (!boost) {
					wm8985_field_write(WM8985_LEFT_ADC_BOOST,PGABOOSTL_0DB,PGABOOSTL_20DB);
					wm8985_field_write(WM8985_RIGHT_ADC_BOOST,PGABOOSTL_0DB,PGABOOSTL_20DB);
				}

				wm8985_field_write(WM8985_LEFT_PGA_GAIN, volume_left, LINVOLM);
				wm8985_field_write(WM8985_RIGHT_PGA_GAIN, volume_left, RINVOLM);
	
				if (boost) {
					wm8985_field_write(WM8985_LEFT_ADC_BOOST,PGABOOSTL_20DB,PGABOOSTL_20DB);
					wm8985_field_write(WM8985_RIGHT_ADC_BOOST,PGABOOSTL_20DB,PGABOOSTL_20DB);
				}

			}
			else {
				volume_left  = codec_convert_range(volume_left, LINE_BOOST_REC_RANGE, LINE_BOOST_REC_LEVEL_MIN);
				wm8985_field_write(WM8985_LEFT_ADC_BOOST, (volume_left<<L2_2BOOSTVOLB), L2_2BOOSTVOLM);
				wm8985_field_write(WM8985_RIGHT_ADC_BOOST, (volume_left<<R2_2BOOSTVOLB), R2_2BOOSTVOLM);
			}


			if ( codec_info.config & INPUT_LINE ) {
				DPRINTK_VOL( "LINE ");
			} else if ( codec_info.config & INPUT_MIC ) {
				DPRINTK_VOL( "MIC ");
			} else {
				DPRINTK_VOL( "PHONE ");
			}

			DPRINTK_VOL( "lvol: %d\trvol: %d\tboost: %s\n",volume_left,volume_left,(boost?"ON":"OFF"));

		} else {

			/* No Mic no Line no Phone!!!*/
			DPRINTK_VOL("Don't know what you want\n");

		}

		break;
	}

        return ret;
}

static int plug_state;

static int mixer_open(struct inode *inode, struct file *file)
{

	/* Any mixer specific initialization */
	/* configure GIOv33_2 as input is done in board_init*/
	if (codec_info.mix_refcount == 0) {
		plug_state = _get_headphone_state();
		codec_info.hpmon_handle = kthread_run( codec_monitor_headphone, &codec_info, "hpmon" );
	}
	codec_info.mix_refcount++;

	return 0;
}

static int mixer_release(struct inode *inode, struct file *file)
{
	/* Any mixer specific Un-initialization */
	codec_info.mix_refcount--;
	if (codec_info.mix_refcount == 0) {
		kthread_stop( codec_info.hpmon_handle );
	}
	return 0;
}

static inline int codec_cap_volume(int val)
{
	if (val < 0)
		return 0;
	if (val > 99)
		return 99;
	return val;
}

static int codec_route_output_needed( int val )
{
	if ( (codec_info.config & OUTPUT_SPK) && (codec_info.config & OUTPUT_HP) && ((val == 1) || (val == 2)) )
			return 0;

	else if ( (codec_info.config & OUTPUT_SPK) && !(codec_info.config & OUTPUT_HP) && ((val == 3) || (val == 4)) )
			return 0;

	else if ( (codec_info.config & OUTPUT_HP) && !(codec_info.config & OUTPUT_SPK) && (val == 0) )
			return 0;

	return 1;
}

/*******************************************************
 EQUALIZER STUFF
*******************************************************/

// Enables or disables low power mode
// !! To use the 5 bands, we need to disable low power mode !!
static void set_low_power( int lowPowerEn )
{
	if ( lowPowerEn ) {
		wm8985_field_write(WM8985_ADD_CTRL, M128ENB_EN, M128ENM);
	}
	else {
		wm8985_field_write(WM8985_ADD_CTRL, M128ENB_DIS, M128ENM);
	}
}

static int wm8985_set_equalizer( wm8985_eqParams_t *newParams )
{	
	int data;
	
	DPRINTK( "Setting the equalizer\r\n" );

	DPRINTK( "Disabling low power mode for EQ operation\r\n" );
	set_low_power(0);

	// -- Band 1
	// Freq
	switch( newParams->band1_freq ) {
	case EQ_WM8985_BAND1_80HZ:	data = EQ1C_80HZ;	break;
	case EQ_WM8985_BAND1_105HZ:	data = EQ1C_105HZ;	break;
	case EQ_WM8985_BAND1_135HZ:	data = EQ1C_135HZ;	break;
	case EQ_WM8985_BAND1_175HZ:	data = EQ1C_175HZ;	break;
	default: 			return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ1, data, EQ_FREQMASK);
	// Gain
	if ( ( newParams->band1_gain < EQ_WM8985_GAIN_12DB  ) ||
	     ( newParams->band1_gain > EQ_WM8985_GAIN_m12DB )
	) {
		return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ1, newParams->band1_gain, EQ_GAINMASK);

	// -- Band 2
	// Freq
	switch( newParams->band2_freq ) {
	case EQ_WM8985_BAND2_230HZ:	data = EQ2C_230HZ;	break;
	case EQ_WM8985_BAND2_300HZ:	data = EQ2C_300HZ;	break;
	case EQ_WM8985_BAND2_385HZ:	data = EQ2C_385HZ;	break;
	case EQ_WM8985_BAND2_500HZ:	data = EQ2C_500HZ;	break;
	default: 			return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ2, data, EQ_FREQMASK);
	// Gain
	if ( ( newParams->band2_gain < EQ_WM8985_GAIN_12DB  ) ||
	     ( newParams->band2_gain > EQ_WM8985_GAIN_m12DB )
	) {
		return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ2, newParams->band2_gain, EQ_GAINMASK);
	// Bandwidth
	switch( newParams->band2_bw ) {
	case EQ_WM8985_BW_WIDE:		data = EQBW_WIDE;	break;
	case EQ_WM8985_BW_NARROW:	data = EQBW_NARROW;	break;
	default: 			return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ2, data, EQ_BWMASK);

	// -- Band 3
	// Freq
	switch( newParams->band3_freq ) {
	case EQ_WM8985_BAND3_650HZ:	data = EQ3C_650HZ;	break;
	case EQ_WM8985_BAND3_850HZ:	data = EQ3C_850HZ;	break;
	case EQ_WM8985_BAND3_1100HZ:	data = EQ3C_1100HZ;	break;
	case EQ_WM8985_BAND3_1400HZ:	data = EQ3C_1400HZ;	break;
	default: 			return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ3, data, EQ_FREQMASK);
	// Gain
	if ( ( newParams->band3_gain < EQ_WM8985_GAIN_12DB  ) ||
	     ( newParams->band3_gain > EQ_WM8985_GAIN_m12DB )
	) {
		return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ3, newParams->band3_gain, EQ_GAINMASK);
	// Bandwidth
	switch( newParams->band3_bw ) {
	case EQ_WM8985_BW_WIDE:		data = EQBW_WIDE;	break;
	case EQ_WM8985_BW_NARROW:	data = EQBW_NARROW;	break;
	default: 			return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ3, data, EQ_BWMASK);

	// -- Band 4
	// Freq
	switch( newParams->band4_freq ) {
	case EQ_WM8985_BAND4_1800HZ:	data = EQ4C_1800HZ;	break;
	case EQ_WM8985_BAND4_2400HZ:	data = EQ4C_2400HZ;	break;
	case EQ_WM8985_BAND4_3200HZ:	data = EQ4C_3200HZ;	break;
	case EQ_WM8985_BAND4_4100HZ:	data = EQ4C_4100HZ;	break;
	default: 			return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ4, data, EQ_FREQMASK);
	// Gain
	if ( ( newParams->band4_gain < EQ_WM8985_GAIN_12DB  ) ||
	     ( newParams->band4_gain > EQ_WM8985_GAIN_m12DB )
	) {
		return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ4, newParams->band4_gain, EQ_GAINMASK);
	// Bandwidth
	switch( newParams->band4_bw ) {
	case EQ_WM8985_BW_WIDE:		data = EQBW_WIDE;	break;
	case EQ_WM8985_BW_NARROW:	data = EQBW_NARROW;	break;
	default: 			return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ4, data, EQ_BWMASK);

	// -- Band 5
	// Freq
	switch( newParams->band5_freq ) {
	case EQ_WM8985_BAND5_5300HZ:	data = EQ5C_5300HZ;	break;
	case EQ_WM8985_BAND5_6900HZ:	data = EQ5C_6900HZ;	break;
	case EQ_WM8985_BAND5_9000HZ:	data = EQ5C_9000HZ;	break;
	case EQ_WM8985_BAND5_11700HZ:	data = EQ5C_11700HZ;	break;
	default: 			return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ5, data, EQ_FREQMASK);
	// Gain
	if ( ( newParams->band5_gain < EQ_WM8985_GAIN_12DB  ) ||
	     ( newParams->band5_gain > EQ_WM8985_GAIN_m12DB )
	) {
		return -EINVAL;
	}
	wm8985_field_write(WM8985_EQ5, newParams->band5_gain, EQ_GAINMASK);

	// -- 3D
	if ( ( newParams->depth_3d < 0x00 ) ||
	     ( newParams->depth_3d > 0x0F )
	) {
		return -EINVAL;
	}
	wm8985_field_write(WM8985_3D_CTRL, newParams->depth_3d, DEPTH3D_MASK);

	// -- Set the digital gain before the EQ to avoid clipping
	data = (DACVOL_00DB-newParams->dac_vol)&DACVOLM;
	wm8985_write(WM8985_LEFT_DAC_DVOL,(DACVU|data));
	wm8985_write(WM8985_RIGHT_DAC_DVOL,(DACVU|data));

	// Update current parameters
	memcpy( &eq_currentParams, newParams, sizeof(wm8985_eqParams_t) );
	eq_isSet = 1;

	DPRINTK( "Done setting the equalizer. Digital gain: %2x\r\n", data );
	return 0;
}

static int wm8985_set_equalizer_default( void )
{
	//TODO: tune
	wm8985_eqParams_t defaultParams = { EQ_WM8985_DEFAULT_PARAMS };

	DPRINTK( "setting default EQ parameters\r\n ");
	return wm8985_set_equalizer( &defaultParams );
}

static int wm8985_set_equalizer_current( void )
{
	// Use the default settings if the EQ has never been set
	if ( !eq_isSet ) {
		return wm8985_set_equalizer_default();
	}
	// Use the last used settings
	DPRINTK( "setting current EQ parameters\r\n ");
	return wm8985_set_equalizer( &eq_currentParams );
}

/*******************************************************
 / EQUALIZER STUFF
*******************************************************/

static int
mixer_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
        int val;
        int ret = 0;
        int nr = _IOC_NR(cmd);

	wm8985_eqParams_t newParams;

        /*
         * We only accept mixer (type 'M') ioctls.
         */
        if (_IOC_TYPE(cmd) != 'M')
                return -EINVAL;

	DPRINTK(" 0x%08x\n", cmd);

        if (cmd == SOUND_MIXER_INFO) {
                struct mixer_info mi;

                strncpy(mi.id, "WM8985", sizeof(mi.id));
                strncpy(mi.name, "Wolfson WM8985", sizeof(mi.name));
                mi.modify_counter = codec_info.mod_cnt;
                return copy_to_user((void *)arg, &mi, sizeof(mi));
        }


	//  5-BAND WOLFSON EQUALIZER SETTING
	//	We use this ioctl code to set the whole 5-band equalizer
	if ( (nr == SOUND_MIXER_TREBLE) && (_IOC_DIR(cmd) & _IOC_WRITE) ) {
		DPRINTK("Wolfson WM8985 - Setting the equalizer\r\n");
		// newParams_user comes from the user space -> copy it
		if ( copy_from_user( &newParams, (void *)arg, sizeof(newParams) ) ) {
			DPRINTK("Wolfson WM8985 - error during copy_from_user()\r\n");
			return -EFAULT;
		}
		return wm8985_set_equalizer( &newParams );
	}

        if (_IOC_DIR(cmd) & _IOC_WRITE) {
                if (get_user(val, (int *)arg))
                	return -EFAULT;

                switch (nr) {
// 		case SOUND_MIXER_ADJ_VOLUME:
// 			if (val < OUTPUT_VOLUME_MAX && val >= 0) {
// 				codec_info.volume_adjust = val;
// 				
// 			} else {
// 				codec_info.volume_adjust = 0;
// 			}
// 			
// 			codec_info.mod_cnt++;
// 			if ( !codec_info.spdif_enabled ) {
//                         	ret = codec_update(SET_VOLUME);
// 			} else {
// 				ret = 0;
// 			}
//                         break;
			
                case SOUND_MIXER_VOLUME:
			codec_info.volume_left = codec_cap_volume(val & 0xff);
                        codec_info.volume_right = codec_cap_volume((val >> 8) & 0xff);
                        codec_info.mod_cnt++;
			if ( !codec_info.spdif_enabled ) {
                        	ret = codec_update(SET_VOLUME);
			} else {
				ret = 0;
			}
                        break;

                case SOUND_MIXER_LINE:
                        codec_info.line_gain = val;
                        codec_info.mod_cnt++;
			ret = codec_update(SET_LINE_GAIN);
                        break;

                case SOUND_MIXER_MIC:
                        codec_info.mic_gain = val;
                        codec_info.mod_cnt++;
			ret = codec_update(SET_MIC_GAIN);
                        break;

                case SOUND_MIXER_RECSRC:
			codec_info.config &= ~(INPUT_LINE | INPUT_MIC | INPUT_PHONE);
			if (val & SOUND_MASK_LINE)
				codec_info.config |= INPUT_LINE;
			else if (val & SOUND_MASK_MIC)
				codec_info.config |= INPUT_MIC;
			else if (val & SOUND_MASK_PHONEIN)
				codec_info.config |= INPUT_PHONE;
			codec_route_input(&codec_info);
			ret = codec_update(SET_REC_LEVEL);
			break;

		case SOUND_MIXER_RECLEV:
			if (codec_info.config & INPUT_LINE)
				codec_info.rec_level_line = val;
			else if (codec_info.config & INPUT_MIC)
				codec_info.rec_level_mic = val;
			else if (codec_info.config & INPUT_PHONE)
				codec_info.rec_level_phone = val;
			codec_info.mod_cnt++;
			ret = codec_update(SET_REC_LEVEL);
			break;

                case SOUND_MIXER_OGAIN:
			if (val < ADJUST_GAIN_MAX && val >= ADJUST_GAIN_MIN) {
				codec_info.volume_adjust = val;
				
			} else {
				codec_info.volume_adjust = 0;
			}
			DPRINTK("SOUND_MIXER_OGAIN: %d\n",codec_info.volume_adjust);
			codec_info.mod_cnt++;
			if ( !codec_info.spdif_enabled ) {
                        	ret = codec_update(SET_VOLUME);
			} else {
				ret = 0;
			}
                	break;

                case SOUND_MIXER_SPEAKER:
			if ( codec_route_output_needed( val ) ) {
				codec_info.config &= ~( OUTPUT_HP | OUTPUT_SPK );
				if ( val == 1 ) {
					// mono mode					
					codec_info.config |= OUTPUT_SPK | OUTPUT_HP;
					codec_info.volume_speaker = 0;
				} else if ( val == 2 ) {
					// stereo mode
					codec_info.config |= OUTPUT_SPK | OUTPUT_HP;
					codec_info.volume_speaker = 1;
				} else if ( val == 3 ) {
					// mono mode
					codec_info.config |= OUTPUT_SPK;
					codec_info.volume_speaker = 0;
				} else if ( val == 4 ) {
					// stereo mode
					codec_info.config |= OUTPUT_SPK;
					codec_info.volume_speaker = 1;
				} else if ( val == 0){
					codec_info.config |= OUTPUT_HP;
				}

				if ( !codec_info.spdif_enabled ) {
					if (val != 5) {
						codec_route_output( &codec_info );
						_set_speaker( val);
					} else {
						power_down_output();
					}
				}
			}
			ret = 0;
                        break;

                default:
                        ret = -ENOIOCTLCMD;
                        break;
                }

        } else if (_IOC_DIR(cmd) & _IOC_READ) {
                ret = 0;

                switch (nr) {
// 		case SOUND_MIXER_ADJ_VOLUME:
// 			val = codec_info.volume_adjust;
//                         break;
                case SOUND_MIXER_VOLUME:
			val = (codec_info.volume_right << 8) | (codec_info.volume_left & 0xff);
                        break;
                case SOUND_MIXER_LINE:
                        val = codec_info.line_gain;
                        break;
                case SOUND_MIXER_MIC:
                        val = codec_info.mic_gain;
                        break;
                case SOUND_MIXER_RECSRC:
                        val = REC_MASK;
                        break;
                case SOUND_MIXER_RECMASK:
                        val = REC_MASK;
                        break;
                case SOUND_MIXER_DEVMASK:
                        val = DEV_MASK;
                        break;
                case SOUND_MIXER_CAPS:
                        val = 0;
                        break;
                case SOUND_MIXER_STEREODEVS:
                        val = 0;
                        break;
                case SOUND_MIXER_SPEAKER:
			if (codec_info.config & OUTPUT_SPK) {
				if (codec_info.config & OUTPUT_HP) val = 1;
				else val = 3;
			}
			else val = 0;
			break;
                default:
                        val = 0;
                        ret = -EINVAL;
                        break;
                }

                if (ret == 0)
                        ret = put_user(val, (int *)arg);
        }

        return ret;

}

static int codec_set_samplerate_spdif(long sample_rate)
{
        int count = 0;

	int sr;
	int mclk_div;
	int bclk_div;
	int pllprescale;
	int plln;
	int pllk1;
	int pllk2;
	int pllk3;

	if (codec_info.sample_rate == sample_rate && codec_info.clock_set)
		return 0;

	DPRINTK(" spdif speed=%li\n", sample_rate);

        /* Search for the right sample rate */
        while ((spdif_sample_rate_info[count].sample_rate != sample_rate) &&
               (count < NUMBER_SPDIF_SAMPLE_RATES_SUPPORTED)) {
                count++;
        }

        if (count == NUMBER_SPDIF_SAMPLE_RATES_SUPPORTED) {
                printk(KERN_ERR "Invalid Sample Rate %d requested\n",
                       (int)sample_rate);
                return -EINVAL;
        }

        codec_info.sample_rate = sample_rate;

	
#ifndef WM8985_MASTER
	{
	#error	WM8985 not master => to be defined
	}
#endif				/* WM8985_MASTER */

	sr		= (spdif_sample_rate_info[count].sr)<<SRB;
	mclk_div	= (spdif_sample_rate_info[count].mclk_div)<<MCLKDIVB;
	bclk_div	= (spdif_sample_rate_info[count].bclk_div)<<BCLKDIVB;
	pllprescale	= (spdif_sample_rate_info[count].pllprescale)<<PLLPRESCALEB;
	plln		= (spdif_sample_rate_info[count].plln)<<PLLNB;
	pllk1		= (spdif_sample_rate_info[count].pllk1)<<PLLK1B;
	pllk2		= (spdif_sample_rate_info[count].pllk2)<<PLLK2B;
	pllk3		= (spdif_sample_rate_info[count].pllk3)<<PLLK3B;

	wm8985_hp_mute();
	wm8985_speaker_mute();

	wm8985_field_write(WM8985_PLLK1,pllk1,PLLK1M);
	wm8985_field_write(WM8985_PLLK2,pllk2,PLLK2M);
	wm8985_field_write(WM8985_PLLK3,pllk3,PLLK3M);
	wm8985_field_write(WM8985_PLLN,(pllprescale|plln),(PLLPRESCALEM|PLLNM));
	wm8985_field_write(WM8985_CLK_GEN,(bclk_div|mclk_div),(BCLKDIVM|MCLKDIVM));
	wm8985_field_write(WM8985_ADD_CTRL,sr,SRM);

	DPRINTK("spdif rate done\n");
	codec_info.clock_set = 1;
        return 0;
}

int codec_set_samplerate(long sample_rate)
{
        int count = 0;

	int sr;
	int mclk_div;
	int bclk_div;
	int pllprescale;
	int plln;
	int pllk1;
	int pllk2;
	int pllk3;

	if (codec_info.spdif_enabled)
		return codec_set_samplerate_spdif(sample_rate);

	if (codec_info.sample_rate == sample_rate && codec_info.clock_set)
		return 0;

	DPRINTK(" speed=%li\n", sample_rate);

        /* Search for the right sample rate */
        while ((sample_rate_info[count].sample_rate != sample_rate) &&
               (count < NUMBER_SAMPLE_RATES_SUPPORTED)) {
                count++;
        }

        if (count == NUMBER_SAMPLE_RATES_SUPPORTED) {
                printk(KERN_ERR "Invalid Sample Rate %d requested\n",
                       (int)sample_rate);
                return -EINVAL;
        }

        codec_info.sample_rate = sample_rate;

	
#ifndef WM8985_MASTER
	{
	#error	WM8985 not master => to be defined
	}
#endif				/* WM8985_MASTER */

	sr		= (sample_rate_info[count].sr)<<SRB;
	mclk_div	= (sample_rate_info[count].mclk_div)<<MCLKDIVB;
	bclk_div	= (sample_rate_info[count].bclk_div)<<BCLKDIVB;
	pllprescale	= (sample_rate_info[count].pllprescale)<<PLLPRESCALEB;
	plln		= (sample_rate_info[count].plln)<<PLLNB;
	pllk1		= (sample_rate_info[count].pllk1)<<PLLK1B;
	pllk2		= (sample_rate_info[count].pllk2)<<PLLK2B;
	pllk3		= (sample_rate_info[count].pllk3)<<PLLK3B;

	wm8985_hp_mute();
	wm8985_speaker_mute();

	wm8985_field_write(WM8985_PLLK1,pllk1,PLLK1M);
	wm8985_field_write(WM8985_PLLK2,pllk2,PLLK2M);
	wm8985_field_write(WM8985_PLLK3,pllk3,PLLK3M);
	wm8985_field_write(WM8985_PLLN,(pllprescale|plln),(PLLPRESCALEM|PLLNM));
	wm8985_field_write(WM8985_CLK_GEN,(bclk_div|mclk_div),(BCLKDIVM|MCLKDIVM));
	wm8985_field_write(WM8985_ADD_CTRL,sr,SRM);

	if (!codec_info.spdif_enabled)
		codec_update(SET_VOLUME);

	DPRINTK("done\n");
	codec_info.clock_set = 1;
        return 0;
}


/*****************************************************************************
** SWITH FROM AIC TO SPDIF
*****************************************************************************/
static int codec_enable_spdif( void )
{

	DPRINTK_SPDIF("codec_enable_spdif(): BEGIN\n");

        _mcbsp_stop();

	if ( !codec_info.spdif_enabled ) {

		DPRINTK_SPDIF("codec_enable_spdif(): SPDIF wasnt enabled\n");

		/* 
		 * this will put the codec into default state, with most
	 	* clocks switched off 
	 	*/
	
		codec_info.spdif_enabled = 1;
		codec_info.clock_set = 0;
		codec_info.mod_cnt++;
	
		mute_sidetone();
		wm8985_hp_mute();
		wm8985_speaker_mute();
	}

	_enable_spdif_ampli();
 	_set_mcbsp_config_spdif();
 	_mcbsp_start();

	return 0;
}

/*****************************************************************************
** SWITH FROM SPDIF TO AIC
*****************************************************************************/
static int codec_disable_spdif( void )
{
	int ret;

	DPRINTK_SPDIF("codec_disable_spdif(): BEGIN\n");
        _mcbsp_stop();

	if ( codec_info.spdif_enabled ) {

		codec_info.spdif_enabled = 0;
		codec_info.clock_set = 0;
		codec_info.mod_cnt++;

		codec_config_init( &codec_info );

		ret = codec_update(SET_OUTPUT_GAIN);
		ret = codec_update(SET_REC_LEVEL);
		ret = codec_update(SET_MIC_GAIN);
		ret = codec_update(SET_VOLUME);

	} else {
		codec_config_init( &codec_info );
	}

	_disable_spdif_ampli();
	_set_mcbsp_config_analog();
 	_mcbsp_start();

	return 0;
}

/*****************************************************************************
** HEADPHONE PLUG MANAGER
*****************************************************************************/
static wait_queue_head_t mixer_wait_queue;
static int mixer_plug_status_available = 0;

static ssize_t mixer_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	int len = 0;

	if( count > 0 ) {
		*buffer = plug_state;
		len = 1;
	}

	mixer_plug_status_available = 0;

	return len;
}

static unsigned int mixer_poll(struct file * file, poll_table * wait)
{
	unsigned int mask = 0;

	if ( mixer_plug_status_available ) {
		mask |= POLLIN | POLLRDNORM;
	}

	poll_wait(file, &mixer_wait_queue, wait);

	return mask;
}

static void dump_reg (void) {
	int i;

	printk(" /* WM8985 regs */ \n");

	for (i=0;i<WM8985_CACHEREGNUM;i++) {
		printk("R%2d= 0x%x;\n",i,codec_regs[i]);
	}
	printk(" /***************/ \n");

}

static int codec_monitor_headphone(void *data)
{
	current->flags &= ~PF_NOFREEZE;

	while (!kthread_should_stop()) {
		int new_state;

		set_current_state(TASK_INTERRUPTIBLE);
		schedule_timeout(HZ/10);


		if (dump) {
			dump = 0;
			dump_reg();		
		}

//#ifdef CONFIG_PM
		try_to_freeze();

 		new_state = _get_headphone_state();
		if ( plug_state == new_state )
			continue;

		plug_state = new_state;
		mixer_plug_status_available = 1;
		wake_up_interruptible( &mixer_wait_queue );
	}

	return 0;
}

static int codec_initialize(struct audio_state_s * state, void *data)
{
	int ret = -1;
	int mclk;
	struct clk *clkout1;
	struct codec_local_info *c_info = data;

	printk("codec_initialize\n");
        DPRINTK("entry\n");

	if (c_info->initialized) {
		printk(KERN_DEBUG "codec already initialized\n");
		return 0;
	}

	/* Select the good architecture */
	if ( machine_is_archos_a5h() || machine_is_archos_a5s() ||
		   machine_is_archos_a5hg() || machine_is_archos_a5sg() ||
		   machine_is_archos_a5hgw() || machine_is_archos_a5sgw() ||
		   machine_is_archos_a5sc() || machine_is_archos_a5st() ||
		   machine_is_archos_a5gcam() ) {
		is_gen7=1;
	} else {
		is_gen7=0;
	}

	clkout1 = clk_get(NULL, "sys_clkout1");
	if (IS_ERR(clkout1)) {
		printk(KERN_DEBUG "audio_codec_init: cannot get codec master clock\n");
		goto initialize_exit;
	}
	mclk = clk_get_rate(clkout1);
	clk_put(clkout1);

	printk(KERN_DEBUG "master clk is: %d\n", mclk);

	sample_rate_info = _select_normal_pll_tab(mclk,0);
	spdif_sample_rate_info = _select_normal_pll_tab(mclk,1);
// 	omap_mcbsp_dump_reg(AUDIO_MCBSP);

        ret = omap_mcbsp_request(AUDIO_MCBSP);
	if (ret < 0) {
		printk(KERN_ERR " Request for MCBSP Failed[%d]\n", ret);
		goto initialize_exit;
	}

	/* Setup default codec information */
	codec_info.line_gain     = DEFAULT_INPUT_VOLUME;
	codec_info.mic_gain      = DEFAULT_INPUT_VOLUME;
	codec_info.volume_left   = DEFAULT_OUTPUT_VOLUME;
	codec_info.volume_right  = DEFAULT_OUTPUT_VOLUME;
	codec_info.volume_adjust = 0;
	codec_info.sample_rate   = AUDIO_RATE_DEFAULT;
	codec_info.rec_level_line = 0;
	codec_info.rec_level_mic = 0;
	codec_info.rec_level_phone = 0;
	codec_info.config        = OUTPUT_HP|INPUT_LINE;
	codec_info.volume_speaker = 0;
	codec_info.spdif_enabled = 0;
	codec_info.mix_refcount = 0;

	codec_state.data = &codec_info;

	// init codec in output mode here reduce scrounches at start 
	codec_config_init_first(&codec_info);
	msleep(100);

	if (state->fmode & FMODE_READ)
		c_info->read_count=1;
	else
		c_info->read_count=0;

	if (state->fmode & FMODE_WRITE)
		c_info->write_count=1;
	else
		c_info->write_count=0;

	if ( codec_info.spdif_enabled ) {
		codec_enable_spdif();
	} else {
		codec_disable_spdif();
	}

	c_info->initialized = 1;
	//mcbsp_dump_reg(AUDIO_MCBSP);
        DPRINTK("exit\n");
 	return 0;

initialize_exit:
	return ret;
}


static void power_down_output (void)
{
	/* shutdown speaker */
 	_set_speaker( 0 );
	msleep(2);

	/* 1. Disable thermal shutdown */	
	wm8985_field_write(WM8985_OUTPUT_CTRL,(TSDDIS|TSOPCTRL_OFF),(TSDENM|TSOPCTRLM));
	wm8985_hp_mute();
	wm8985_speaker_mute();
	/* 2. Disable VMID with required charge */
	wm8985_field_write(WM8985_PWR_MGT1,VMIDSEL_OFF,VMIDSELM);
	/* 3. wait VMID to discharge */
	msleep(500);
	/* 4. Power off registers R1,R2,R3 = 0x000h */
	wm8985_write(WM8985_PWR_MGT1,0);
	wm8985_write(WM8985_PWR_MGT2,0);
	wm8985_write(WM8985_PWR_MGT3,0);
}

static void codec_shutdown(void *data)
{
	struct codec_local_info* c_info = data;

	if (!c_info->initialized) {
		printk(KERN_DEBUG "codec already shut down\n");
		return;
	}

        /*
          Turn off codec after it is done.
          Can't do it immediately, since it may still have
          buffered data.

          Wait 20ms (arbitrary value) and then turn it off.
        */

	printk(KERN_DEBUG "codec shutdown\n");

        set_current_state(TASK_INTERRUPTIBLE);
        schedule_timeout(2);
        c_info->clock_set = 0;
	c_info->initialized = 0;

	/* 
	 * this will put the codec into default state, with most
	 * clocks switched off 
	 */

	omap2_mcbsp_reset(AUDIO_MCBSP);
	omap_mcbsp_free(AUDIO_MCBSP);
}

static void codec_route_output(struct codec_local_info *c_info)
{
	codec_config_t config = c_info->config;
	
	/* Mute DAC */
	wm8985_dac_mute(1);

	printk(KERN_DEBUG "<codec_configure: enumerate output\n");
	if (config & OUTPUT_HP) {
		printk(KERN_DEBUG "codec_configure:  - output on HP\n");
		/* route hp */
		if (is_gen7) wm8985_field_write(WM8985_PWR_MGT2,(ROUT1EN|LOUT1EN),(ROUT1ENM|LOUT1ENM));
		else wm8985_field_write(WM8985_PWR_MGT3,(ROUT2EN|LOUT2EN),(ROUT2ENM|LOUT2ENM));
		/* unmute hp */
		wm8985_hp_unmute();

	} else {
		/* power down hp */
		if (is_gen7) wm8985_field_write(WM8985_PWR_MGT2,0,(ROUT1ENM|LOUT1ENM));
		else wm8985_field_write(WM8985_PWR_MGT3,0,(ROUT2ENM|LOUT2ENM));
		/* mute hp */
		wm8985_hp_mute();
		
	}

	if (config & OUTPUT_SPK) {
		printk(KERN_DEBUG "codec_configure:  - output on SPK\n");
		if (is_gen7) {
			/* power up spk */
			wm8985_field_write(WM8985_PWR_MGT3,(ROUT2EN|LOUT2EN),(ROUT2ENM|LOUT2ENM));
			/* unmute spk */
			wm8985_speaker_unmute();
		}
		
	} else {
		if (is_gen7) {
			/* power down spk */
			wm8985_field_write(WM8985_PWR_MGT3,0,(ROUT2ENM|LOUT2ENM));
			/* mute spk */
			wm8985_speaker_mute();
		}
		
	}

	if(!((config & OUTPUT_HP) | (config & OUTPUT_SPK))){
		printk(KERN_DEBUG "codec_configure:  - output on nothing\n");
		/* power down output */
	}

	wm8985_dac_mute(0);


}

static void codec_route_input(struct codec_local_info *c_info)
{
	codec_config_t config = c_info->config;
	DPRINTK("Route input:");
	if (config & INPUT_LINE) {
		printk(KERN_DEBUG " LINE\n");
// 		/* connect L2 & R2 to PGA */
// 		/* also connect LIN and RIN to the amplifier negative terminal */
		wm8985_field_write(WM8985_PWR_MGT1,MIC_BIASDIS,MIC_BIASEN_M);
		wm8985_write(WM8985_INPUT_CTRL,(L2_2INPPGA|R2_2INPPGA|LIN_2INPPGA|RIN_2INPPGA));

		/* do PGA unmute */
		wm8985_connect_input();

		/* power up input */
		wm8985_power_up_input();


	} else if (config & INPUT_MIC) {
		printk(KERN_DEBUG " MIC\n");
// 		/* connect LIP & LIP to PGA */
// 		/* also connect LIN and RIN to the amplifier negative terminal */
		wm8985_write(WM8985_INPUT_CTRL,(LIP_2INPPGA|LIN_2INPPGA|RIN_2INPPGA));
		wm8985_field_write(WM8985_PWR_MGT1,MIC_BIASEN,MIC_BIASEN_M);

		/* do PGA unmute */
		wm8985_connect_input();

		/* power up input */
		wm8985_power_up_input();


	} else if (config & INPUT_PHONE) {
		printk(KERN_DEBUG " PHONE\n");
// 		/* connect LIP & LIP to PGA */
// 		/* also connect LIN and RIN to the amplifier negative terminal */
		wm8985_write(WM8985_INPUT_CTRL,(RIP_2INPPGA|LIN_2INPPGA|RIN_2INPPGA));
		wm8985_field_write(WM8985_PWR_MGT1,MIC_BIASEN,MIC_BIASEN_M);

		/* do PGA unmute */
		wm8985_connect_input();

		/* power up input */
		wm8985_power_up_input();


	} else {
		printk(KERN_DEBUG "codec_configure: input from nothing\n");
		printk(KERN_DEBUG "codec_configure: disable input amplifiers and path\n");

		/* disable all path to inpga */
		/* Mute PGAs */
		wm8985_disconnect_input();

		/* disable MIC bias */
		wm8985_field_write(WM8985_PWR_MGT1,MIC_BIASDIS,MIC_BIASEN_M);

		/* power down input */
		wm8985_power_down_input();

	}
}

static void codec_config_init(struct codec_local_info* c_info)
{
	DPRINTK( "codec_config_init: BEGIN\r\n ");

	/* route output */
 	codec_route_output(c_info);

	if (c_info->read_count == 1) {

		DPRINTK( "codec_config_init: c_info->read_count == 1\r\n ");

		/* power down ADC */
		wm8985_field_write(WM8985_PWR_MGT2,(ADCDISR|ADCDISL),(ADCENRM|ADCENLM));

		/* power down DAC */
		wm8985_field_write(WM8985_PWR_MGT3,(DACDISR|DACDISL),(DACENRM|DACENLM));

		/* power up input */
		printk(KERN_DEBUG "power up input\n");

		/* set eq to DAC */
		wm8985_field_write(WM8985_EQ1,EQ3DMODE_DAC,EQ3DMODEM);

		/* power up ADC */
		wm8985_field_write(WM8985_PWR_MGT2,(ADCENR|ADCENL),(ADCENRM|ADCENLM));

		/* set EQ */
		wm8985_set_equalizer_current();

		/* power up DAC */
		wm8985_field_write(WM8985_PWR_MGT3,(DACENR|DACENL),(DACENRM|DACENLM));

		set_sidetone();
		
	} else {

		DPRINTK( "codec_config_init: c_info->read_count != 1\r\n ");

		mute_sidetone();


		/* set eq to ADC */
		wm8985_field_write(WM8985_EQ1,EQ3DMODE_ADC,EQ3DMODEM);

		/* power down ADC */		
		wm8985_field_write(WM8985_PWR_MGT2,(ADCDISR|ADCDISL),(ADCENRM|ADCENLM));

		/* power down DAC */
		wm8985_field_write(WM8985_PWR_MGT3,(DACDISR|DACDISL),(DACENRM|DACENLM));

		/* set eq to DAC */
		wm8985_field_write(WM8985_EQ1,EQ3DMODE_DAC,EQ3DMODEM);

		/* power up DAC */
		wm8985_field_write(WM8985_PWR_MGT3,(DACENR|DACENL),(DACENRM|DACENLM));

	}

}

static void codec_config_init_first(struct codec_local_info* c_info)
{
 	_set_speaker( 0 );
	msleep(2);
	/** power up **/
	DPRINTK("codec_config_init_first\n");

	wm8985_reset();

	/* Set low analog bias mode */
	wm8985_write(WM8985_BIAS_CTRL,BIASCUT);

	/* Enable thermal shutdown */	
	wm8985_write(WM8985_OUTPUT_CTRL,(TSDEN|TSOPCTRL));

	/* Enable Internal Bias*/
	wm8985_write(WM8985_PWR_MGT1,(BIASEN| BUFIOEN));

	/* Mute all outputs, and PGA to minimum */	
	/* LOUT1 and ROUT1 to minimum*/
	wm8985_write(WM8985_LOUT1V,(LHP_MUTE|OUTVU));
	wm8985_write(WM8985_ROUT1V,(RHP_MUTE|OUTVU));
	/* LOUT2 and ROUT2 to minimum*/
	wm8985_write(WM8985_LOUT2V,(LHP_MUTE|OUTVU));
	wm8985_write(WM8985_ROUT2V,(RHP_MUTE|OUTVU));
	/* Input PGA to minimum and mute */
	wm8985_write(WM8985_LEFT_PGA_GAIN,(INPPGAMUTEL|INPPGAVU));
	wm8985_write(WM8985_RIGHT_PGA_GAIN,(INPPGAMUTER|INPPGAVU));

	/* set PGA boost gain to 0 dB */
	/* and disconnect all from boost */
	wm8985_write(WM8985_LEFT_ADC_BOOST,0);
	wm8985_write(WM8985_RIGHT_ADC_BOOST,0);

	/* mute and disable out3 mixer */
//  	wm8985_write(WM8985_OUT3_MIXER_CTRL,(OUT3MUTE|LDAC2OUT3_DIS));
  	wm8985_write(WM8985_OUT3_MIXER_CTRL,0);
	/* mute and disable out4 mixer */
//  	wm8985_write(WM8985_OUT4_MIXER_CTRL,(OUT4MUTE|RDAC2OUT4_DIS));
  	wm8985_write(WM8985_OUT4_MIXER_CTRL,0);

	/* Enable Vmid indpendent current bias */
	wm8985_write(WM8985_OUT42DAC,POBCTRL);

	/* Enable required output */
	/* left mixer default is good: left dac to left out, all input not wired to output */
	wm8985_write(WM8985_LEFT_MIXER_CTRL,DACL2LMIX);
	/* right mixer default is good: right dac to right out, all input not wired to output */
	wm8985_write(WM8985_RIGHT_MIXER_CTRL,DACR2RMIX);
	/* output ctrl default is good */

	/* Enable VMID with required charge */
	wm8985_field_write(WM8985_PWR_MGT1,VMIDSEL_300k,VMIDSELM);

	/* wait 30 ms */
	msleep(30);
	
	/* setup DAI */
	wm8985_write(WM8985_IFACE,WM8985_DIGITAL_AUDIO_INTERFACE_DEFAULT);
	wm8985_write(WM8985_CLK_GEN,WM8985_CLOCK_GEN_CTRL_DEFAULT);

	/* setup path input: all inputs are disconnected from PGA */
	wm8985_write(WM8985_INPUT_CTRL,0);

	/* set sample rate */
 	codec_set_samplerate(c_info->sample_rate);

	/* set eq to DAC */
	wm8985_field_write(WM8985_EQ1,EQ3DMODE_DAC,EQ3DMODEM);

	/* enable PLL, ADC and DAC */
	wm8985_field_write(WM8985_PWR_MGT1,PLLEN,PLLENM);
	wm8985_write(WM8985_PWR_MGT2,(ADCENR|ADCENL));
	wm8985_write(WM8985_PWR_MGT3,(DACENR|DACENL|RMIXEN|LMIXEN));
	
	/* set DAC vol once => use analog vol */
	wm8985_write(WM8985_LEFT_DAC_DVOL,(DACVU|DACVOL_00DB));
	wm8985_write(WM8985_RIGHT_DAC_DVOL,(DACVU|DACVOL_00DB));
	
	/* set DAC best SNR and mute*/
	wm8985_write(WM8985_DAC_CTRL,(DACOSR128|DAC_SMUTE));

	/* set ADC best SNR and high pass filter*/
	wm8985_write(WM8985_ADC_CTRL,(HPFEN|ADCOSR));

	/* set slow clk for zero crossing */
	wm8985_write(WM8985_ADD_CTRL,SLOWCLKEN);	

	/* Disable Vmid independent */
	wm8985_field_write(WM8985_OUT42DAC,POBCTRL_DIS,POBCTRLM);

	/* wait 500 ms */
	msleep(500);
	/* enable PGA out */
	wm8985_field_write(WM8985_PWR_MGT3,(ROUT2EN|LOUT2EN),(ROUT2ENM|LOUT2ENM));
	/* Gen7 => need to consider OUT1 too */
	if (is_gen7) wm8985_field_write(WM8985_PWR_MGT2,(ROUT1EN|LOUT1EN),(ROUT1ENM|LOUT1ENM));

	/* */
//	wm8985_write(WM8985_BIAS_CTRL,BIAS_NORMAL);
	msleep(200);

 	_mcbsp_stop();
 	_set_mcbsp_config_analog();
 	_mcbsp_start();
}

/*****************************************************************************
** CODEC FILE OPERATIONS
*****************************************************************************/

static int codec_ioctl(struct inode *inode, struct file *file, uint cmd, ulong arg)
{
        long val;
        int ret = 0;

        //DPRINTK(" 0x%08x\n", cmd);

        /*
         * These are platform dependent ioctls which are not handled by the
         * generic omap-audio module.
         */
        switch (cmd) {
        case SNDCTL_DSP_STEREO:
                ret = get_user(val, (int *)arg);
                if (ret)
                        return ret;
                /* stereo only for now */
                ret = (val == 0) ? -EINVAL : 1;
                return put_user(ret, (int *)arg);

        case SNDCTL_DSP_CHANNELS:
        case SOUND_PCM_READ_CHANNELS:
                /* stereo only for now */
                return put_user(2, (long *)arg);

        case SNDCTL_DSP_SPEED:
                ret = get_user(val, (long *)arg);
                if (ret)
                        break;

		_mcbsp_stop();

		ret = codec_set_samplerate(val);

		// What's the point of these 2 lines?
		/*
		codec_route_output(&codec_info);
		codec_route_input(&codec_info);
		*/
		
		_mcbsp_start();

                if (ret)
                        break;
                /* fall through */

        case SOUND_PCM_READ_RATE:
                return put_user(codec_info.sample_rate, (long *)arg);

        case SOUND_PCM_READ_BITS:
        case SNDCTL_DSP_SETFMT:
        case SNDCTL_DSP_GETFMTS:
                /* we can do 16-bit only */
                return put_user(AFMT_S16_LE, (long *)arg);

	case SNDCTL_DSP_BIND_CHANNEL:
		ret = get_user(val, (long *)arg);
                if (ret)
                        break;

		if (val & DSP_BIND_SPDIF) {
			codec_enable_spdif();
		} else {
			codec_disable_spdif();
		}

		break;


        default:
                /* Maybe this is meant for the mixer (As per OSS Docs) */
                return mixer_ioctl(inode, file, cmd, arg);
        }

        return ret;
}

extern struct i2c_client * i2c_get_wm87xx_client(void);

static int codec_probe(void)
{
        codec_handle = i2c_get_wm87xx_client();
	//codec_handle = i2c_get_client(I2C_DRIVERID_WM8731,WM87xx_I2C_NB,NULL);
        if (codec_handle == 0) {
		printk(KERN_ERR "No WM8985 codec found\n");
        	return -ENODEV;
	}
#if defined (CONFIG_MACH_ARCHOS)
	pt_audio_device_io = archos_audio_get_io();
	_enable_master_clock();
#endif
        mixer_dev_id = register_sound_mixer(&mixer_fops, -1);

	init_waitqueue_head( &mixer_wait_queue );

        /* Announcement Time */
        printk(KERN_INFO CODEC_NAME " version %s audio support initialized\n",DRIVER_VERSION);

        return 0;
}

static void codec_remove(void)
{
        unregister_sound_mixer(mixer_dev_id);

#if defined (CONFIG_MACH_ARCHOS)
	_disable_master_clock();
#endif
}

static int codec_suspend(struct audio_state_s * state)
{

	/* suspend in silence */
	power_down_output();
	codec_shutdown (&codec_info);

        return 0;
}

static int codec_resume(struct audio_state_s * state)
{
	printk("codec_resume\n");
	return codec_initialize(state, &codec_info);
}


/** 
 * @brief codec_prescale
 * 
 * @return  0 if successful
 */
static int codec_prescale(void)
{
	int ret = 0;
	FN_IN;
	/* 
	 * This is a very effective,efficient and a pretty dirty way of 
	 * doing things. Mux out the mcbsp2 signals out, no signals, no
	 * dmareq and no transfers
	 */
#if defined (CONFIG_ARCH_OMAP243X)
	ret = omap2_cfg_reg(AC10_2430_MCBSP2_FSX_OFF);
	ret = omap2_cfg_reg(AD16_2430_MCBSP2_CLX_OFF);
	ret = omap2_cfg_reg(AE13_2430_MCBSP2_DX_OFF);
	ret = omap2_cfg_reg(AD13_2430_MCBSP2_DR_OFF);
	/* Give time for these to settle down empirical vals */
	udelay(TWL4030_MCBSP2_2430SDP_PRESCALE_TIME);
#else
	/* Find other ways to shut off signals to mcbsp */
#endif
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief codec_postscale
 * 
 * @return  0 if successful
 */
static int codec_postscale(void)
{
	int ret = 0;
	FN_IN;
	/* 
	 * This is a very effective,efficient and a pretty dirty way 
	 * of doing things. Mux back the mcbsp2 signals out
	 */
#if defined (CONFIG_ARCH_OMAP243X)
	ret = omap2_cfg_reg(AC10_2430_MCBSP2_FSX);
	ret = omap2_cfg_reg(AD16_2430_MCBSP2_CLX);
	ret = omap2_cfg_reg(AE13_2430_MCBSP2_DX);
	ret = omap2_cfg_reg(AD13_2430_MCBSP2_DR);
	/* Give time for these to settle down empirical vals */
	udelay(TWL4030_MCBSP2_2430SDP_PRESCALE_TIME);
#else
	/* Find other ways to shut off signals to mcbsp */
#endif
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief codec_transfer [hw_transfer]
 *  - Do the transfer to a said stream
 * @param s - stream for which the transfer occurs
 * @param buffer_phy - the physical address of the buffer
 * @param size - num bytes to transfer
 * 
 * @return 0 if success else return value -EBUSY implies retry later..
 */
static int codec_transfer(struct audio_state_s *state,
				 audio_stream_t * s, void *buffer_phy, u32 size)
{
	int ret = 0;
	int restart = 0;
	FN_IN;
	if (unlikely(!s || !state || !buffer_phy)) {
		printk(KERN_ERR "Stream/State/phy_buf NULL!!\n");
		return -EPERM;
	}
	/* DEBUG */
#ifdef TWL_DUMP_REGISTERS_MCBSP
	printk(KERN_INFO "TRANSFER");
	omap_mcbsp_dump_reg(AUDIO_MCBSP);
#endif
	/* Error Recovery Strategy -
	 * 1. stop codec
	 * 2. stop mcbsp tx/rx
	 * 3. stop dma tx/rx
	 * ---
	 * Restart:
	 * 1. send/recievedata (starts mcbsp,dma)
	 * 2. start codec
	 */
	if (s->input_or_output == FMODE_READ) {
		DPRINTK("RX-%d", size);
		if (rx_err) {
			/* Detected failure in mcbsp - try to live */
			DPRINTK("DETECTED MCBSP RX FAILURE.. \n");
			if (unlikely
			    (ret =
			     omap2_mcbsp_set_rrst(AUDIO_MCBSP,
						  OMAP_MCBSP_RRST_DISABLE))) {
				printk(KERN_ERR
				       " mcbsp rrst disable failed [%d]\n",
				       ret);
				goto transfer_exit;
			}
			if (unlikely
			    (ret = omap2_mcbsp_stop_datarx(AUDIO_MCBSP))) {
				printk(KERN_ERR
				       " Stop Data Rx for MCBSP Failed[%d]\n",
				       ret);
				goto transfer_exit;
			}
			restart = 1;
			rx_err = 0;
		}

		ret =
		    omap2_mcbsp_receive_data(AUDIO_MCBSP,
					     &in_stream_data,
					     (dma_addr_t) buffer_phy, size);
		DPRINTK("rx-%d\n", ret);
	} else {
		DPRINTK("TX-%d", size);
		if (tx_err) {
			/* Detected failure in mcbsp - try to live */
			DPRINTK("DETECTED MCBSP TX FAILURE.. \n");
			if (unlikely
			    (ret =
			     omap2_mcbsp_set_xrst(AUDIO_MCBSP,
						  OMAP_MCBSP_XRST_DISABLE))) {
				printk(KERN_ERR
				       " mcbsp xrst disable failed [%d]\n",
				       ret);
				goto transfer_exit;
			}
			if (unlikely
			    (ret = omap2_mcbsp_stop_datatx(AUDIO_MCBSP))) {
				printk(KERN_ERR
				       " Stop Data Tx for MCBSP Failed[%d]\n",
				       ret);
				goto transfer_exit;
			}
			restart = 1;
			tx_err = 0;
		}
		ret =
		    omap2_mcbsp_send_data(AUDIO_MCBSP,
					  &out_stream_data,
					  (dma_addr_t) buffer_phy, size);
	}
//	omap_mcbsp_dump_reg(AUDIO_MCBSP);
	if (restart) {
#ifdef TWL_DUMP_REGISTERS_MCBSP
		printk(KERN_INFO "restart TRANSFER");
		omap_mcbsp_dump_reg(AUDIO_MCBSP);
#endif
// 		twl4030_codec_on();
// 		twl4030_ext_mut_off();
	}

      transfer_exit:
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief codec_transfer_stop [hw_transfer_stop]
 *  - Stop Transfers on the current stream
 * @param s - stream for which the transfer occurs
 * 
 * @return 0 if success else return value
 */
static int codec_transfer_stop(audio_stream_t * s)
{
	int ret = 0;
	FN_IN;
	if (unlikely(!s)) {
		printk(KERN_ERR "Stream IS NULL!!\n");
		return -EPERM;
	}
	if (s->input_or_output == FMODE_READ) {
		ret = omap2_mcbsp_stop_datarx(AUDIO_MCBSP);
		(void)omap2_mcbsp_set_rrst(AUDIO_MCBSP,
					   OMAP_MCBSP_RRST_DISABLE);
	} else {
		ret = omap2_mcbsp_stop_datatx(AUDIO_MCBSP);
		(void)omap2_mcbsp_set_xrst(AUDIO_MCBSP,
					   OMAP_MCBSP_XRST_DISABLE);
	}
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief omap_twl4030_transfer_posn [hw_transfer_posn]
 *  - Where am i?
 * @param s - stream for which the transfer occurs
 * 
 * @return posn of the transfer
 */
static int codec_transfer_posn(audio_stream_t * s)
{
	int ret = 0;
	int fi, ei;
	FN_IN;
	if (unlikely(!s)) {
		printk(KERN_ERR "Stream IS NULL!!\n");
		return -EPERM;
	}
	/* we always ask only one frame to transmit/recieve,
	 * variant is the element num 
	 */
	if (s->input_or_output == FMODE_READ) {
		ret = omap2_mcbsp_receiver_index(AUDIO_MCBSP, &ei, &fi);
		ret = (ei * (mcbsp_config_current->dma_recv.word_length1));
	} else {
		ret = omap2_mcbsp_transmitter_index(AUDIO_MCBSP, &ei, &fi);
		ret = (ei * (mcbsp_config_current->dma_trans.word_length1));
	}
	if (ret < 0) {
		printk(KERN_ERR
		       "codec_transfer_posn: Unable to find index of "
		       "transfer\n");
	}
	FN_OUT(ret);
	return ret;
}

/** 
 * @brief codec_transfer_init [hw_transfer_init]
 *  - initialize the current stream
 * @param state -codec state for which this transfer is to be done
 * @param s - stream for which the transfer occurs
 * @param callback - call me back when I am done with the transfer
 * 
 * @return 0 if success else return value
 */
static int codec_transfer_init(audio_state_t * state,
				      audio_stream_t * s,
				      buf_irq_handler callback)
{
	int ret = 0;
	struct stream_callback_struct *stream;
	FN_IN;
	if (unlikely(!s || !callback)) {
		printk(KERN_ERR "Stream/CALLBACK IS NULL!!\n");
		return -EPERM;
	}
	if (s->input_or_output == FMODE_READ) {
		stream = &in_stream_data;
	} else {
		stream = &out_stream_data;
	}
	memset(stream, 0, sizeof(struct stream_callback_struct));
	stream->s = s;
	stream->state = state;
	stream->callback = callback;
	if (s->input_or_output == FMODE_READ) {
		ret = omap2_mcbsp_dma_recv_params(AUDIO_MCBSP,
				&(mcbsp_config_current->dma_recv));
		if (ret < 0) {
			printk(KERN_ERR "RECV params failed");
			goto transfer_exit;
		}
	} else {
		ret = omap2_mcbsp_dma_trans_params(AUDIO_MCBSP,
				 &(mcbsp_config_current->dma_trans));
		if (ret < 0) {
			printk(KERN_ERR "TX params failed");
			goto transfer_exit;
		}
	}

 transfer_exit:
	FN_OUT(ret);
	return ret;
}


/** 
 * @brief codec_mcbsp_dma_cb - the mcbsp call back
 * 
 * @param ch_status - dma channel status value
 * @param arg - the stream_callback structure
 */
static void codec_mcbsp_dma_cb(u32 ch_status, void *arg)
{
	struct stream_callback_struct *stream =
	    (struct stream_callback_struct *)arg;
	FN_IN;
	if (unlikely(!stream)) {
		printk(KERN_ERR "No Stream information!!\n");
		return;
	}
	if (unlikely(!stream->callback)) {
		printk(KERN_ERR "No Callback information!!\n");
		return;
	}
	if (ch_status) {
		printk(KERN_ERR "Error happend[%d 0x%x]!!\n", ch_status,
		       ch_status);
		return;

	}
	stream->callback(stream->state, stream->s);
	FN_OUT(0);
}
static inline void _mcbsp_stop_tx(void) {
// 	omap2_mcbsp_set_xen(AUDIO_MCBSP,OMAP2_MCBSP_TX_DISABLE);
 	(void)omap2_mcbsp_set_xrst(AUDIO_MCBSP,OMAP_MCBSP_XRST_DISABLE);
}

static inline void _mcbsp_start_tx(void) {
// 	omap2_mcbsp_set_xen(AUDIO_MCBSP,OMAP2_MCBSP_TX_ENABLE);
 	(void)omap2_mcbsp_set_xrst(AUDIO_MCBSP,OMAP_MCBSP_XRST_ENABLE);
}

static inline void _mcbsp_stop_rx(void) {
	(void)omap2_mcbsp_set_rrst(AUDIO_MCBSP,OMAP_MCBSP_RRST_DISABLE);
}

static inline void _mcbsp_start_rx(void) {
	(void)omap2_mcbsp_set_rrst(AUDIO_MCBSP,OMAP_MCBSP_RRST_ENABLE);
}

static void _mcbsp_stop(void) {
	_mcbsp_stop_tx();
	_mcbsp_stop_rx();
}

static void _mcbsp_start(void) {
	_mcbsp_start_tx();
	_mcbsp_start_rx();
}

/* Don't use these functions, they are enabling SRG and FRG (GRST and FRST in SPCR2), which is quiet bad for us (sound distortions) */
/*
static void _mcbsp_stop(void) {
	omap_mcbsp_stop(AUDIO_MCBSP);

}

static void _mcbsp_start(void) {
	omap_mcbsp_start(AUDIO_MCBSP);
}
*/

static void _set_mcbsp_config_spdif(void) {
	mcbsp_config_current = &mcbsp_config_spdif;
	codec_conf_interface();
}

static void _set_mcbsp_config_analog(void) {
	mcbsp_config_current = &mcbsp_config_analog_out;
	codec_conf_interface();
}


static struct sample_rate_reg_info* _select_normal_pll_tab(int mclk, int spdif) {

	struct sample_rate_reg_info *ret;

	switch(mclk) {
		case 12000000:
			if (!spdif)
				ret = (struct sample_rate_reg_info *)sample_rate_info_mclk12;
			else
				ret = (struct sample_rate_reg_info *)spdif_sample_rate_info_mclk12;
		break;
		case 19200000:
			if (!spdif)
				ret = (struct sample_rate_reg_info *)sample_rate_info_mclk19;
			else
				ret = (struct sample_rate_reg_info *)spdif_sample_rate_info_mclk19;
		break;
		default:
			ret = (struct sample_rate_reg_info *)sample_rate_info_mclk19;
			printk("Unknown sysclock\n");
		break;
	}
	return ret;
}

static int codec_conf_interface(void) {
	int ret = 0;
	struct omap_mcbsp *mcbsp;
	void __iomem *io_base;

	FN_IN;
	mcbsp = id_to_mcbsp_ptr(AUDIO_MCBSP);
	io_base = mcbsp->io_base;

	/* reset the McBSP registers so that we can 
	 * configure it 
	 */
	if (unlikely(ret = omap2_mcbsp_reset(AUDIO_MCBSP))) {
		printk(KERN_ERR "conf_data Reset for MCBSP Failed[%d]\n", ret);
		/* Dont care abt result */
		return ret;
	}

	/* Disable SRG */
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2, omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR2) & ~(GRST));
	/* Disable FSG */
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2, omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR2) & ~(FRST));

	omap2_mcbsp_params_cfg(AUDIO_MCBSP,OMAP_MCBSP_SLAVE,
				&(mcbsp_config_current->rx_param),
				&(mcbsp_config_current->tx_param),
				&(mcbsp_config_current->srg_fsg_param));
	/* Can be used instead of omap2_mcbsp_params_cfg(), spdif reg config not written for this function */
	//omap_mcbsp_config(AUDIO_MCBSP, &initial_config);

	/* Set XCCR and RCCR to default (DMA enabled) */
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_XCCR, omap_mcbsp_read(io_base, OMAP_MCBSP_REG_XCCR) | DXENDLY(1) | XDMAEN);
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_RCCR, omap_mcbsp_read(io_base, OMAP_MCBSP_REG_RCCR) | RFULL_CYCLE | RDMAEN);

	/* Transmit interrupt generated by XSYNCERR, Receive Interrupt generated by RSYNCERR */
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2, omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR2) | XINTM(3));
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR1, omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR1) | RINTM(3));

	/* Set DMA stuff, already done by codec_transfer_init, but needed when
		the config changes dynamicaly (switch between SPDIF/ANALOG) */
	ret = omap2_mcbsp_dma_recv_params(AUDIO_MCBSP,
			&(mcbsp_config_current->dma_recv));
	if (ret < 0) {
		printk(KERN_ERR "Set RX DMA params failed");
		return ret;
	}
	ret = omap2_mcbsp_dma_trans_params(AUDIO_MCBSP,
			 &(mcbsp_config_current->dma_trans));
	if (ret < 0) {
		printk(KERN_ERR "Set TX DMA params failed");
		return ret;
	}

#ifndef WM8985_MASTER
	/* Enable SRG */
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2, omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR2) | GRST);
	/* Enable FSG */
	omap_mcbsp_write(io_base, OMAP_MCBSP_REG_SPCR2, omap_mcbsp_read(io_base, OMAP_MCBSP_REG_SPCR2) | FRST);
#endif

	FN_OUT(ret);
	return ret;
}

#if 0 /* Old configuration way */
/**
 * @brief codec_conf_data_interface
 * *NOTE* Call only from dsp device
 * 
 * @return  0 if successful
 */
static int codec_conf_data_interface(struct mcbsp_config *config)
{
	int ret = 0;
	int line = 0;

	FN_IN;
	/* reset the McBSP registers so that we can 
	 * configure it 
	 */
	if (unlikely(ret = omap2_mcbsp_interface_reset(AUDIO_MCBSP))) {
		printk(KERN_ERR "conf_data Reset for MCBSP Failed[%d]\n", ret);
		/* Dont care abt result */
		return ret;
	}

	//ret = omap2_mcbsp_set_srg(AUDIO_MCBSP, OMAP2_MCBSP_SRG_DISABLE);
	ret = omap2_mcbsp_set_srg(AUDIO_MCBSP, OMAP_MCBSP_SRG_DISABLE);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	//ret = omap2_mcbsp_set_fsg(AUDIO_MCBSP, OMAP2_MCBSP_FSG_DISABLE);
	ret = omap2_mcbsp_set_fsg(AUDIO_MCBSP, OMAP_MCBSP_FSG_DISABLE);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}

	/* Set the sample rate in mcbsp */
	/* PRCM Clock used - dont care abt clock - mcbsp, find it out */
	/* Setup default codec information */
        codec_info.line_gain     = DEFAULT_INPUT_VOLUME;
        codec_info.mic_gain      = DEFAULT_INPUT_VOLUME;
	codec_info.volume_left   = DEFAULT_OUTPUT_VOLUME;
	codec_info.volume_right  = DEFAULT_OUTPUT_VOLUME;
	codec_info.volume_adjust = 0;
	codec_info.sample_rate   = AUDIO_RATE_DEFAULT;
	codec_info.rec_level_line = 0;
	codec_info.rec_level_mic = 0;
	codec_info.rec_level_phone = 0;
	codec_info.config        = OUTPUT_HP|INPUT_LINE;
	codec_info.volume_speaker = 0;
	codec_info.spdif_enabled = 0;
	codec_info.mix_refcount = 0;

	codec_state.data = &codec_info;

#ifdef WM8985_MASTER
	ret = omap2_mcbsp_srg_cfg(AUDIO_MCBSP,
				  1,
				  1,
				  config->srg_clk_src,
				  1,
				  config->srg_clk_sync,
				  config->srg_clk_pol);
#else
	ret = omap2_mcbsp_srg_cfg(AUDIO_MCBSP,
				  audio_samplerate,
				  current_bitspersample,
				  config->srg_clk_src,
				  1,
				  config->srg_clk_sync,
				  config->srg_clk_pol);
#endif
	if (unlikely(ret < 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}

	/* Setup the framesync clocks */
#ifdef WM8985_MASTER
	ret =
	    omap2_mcbsp_fsync_cfg(AUDIO_MCBSP,
				  config->tx_clk_src,
				  config->rx_clk_src,
				  config->tx_polarity,
				  config->rx_polarity, 0, 0, 0);
#else
	ret =
	    omap2_mcbsp_fsync_cfg(AUDIO_MCBSP,
				  config->tx_clk_src,
				  config->rx_clk_src,
				  config->tx_polarity,
				  config->rx_polarity,
				  current_bitspersample * 2 - 1,
				  current_bitspersample - 1, 1);
#endif
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}

	ret =
	    omap2_mcbsp_txclk_cfg(AUDIO_MCBSP,
				  config->tx_ip_clk,
				  config->tx_clk_pol);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	ret =
	    omap2_mcbsp_rxclk_cfg(AUDIO_MCBSP,
				  config->rx_ip_clk,
				  config->rx_clk_pol);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
#ifndef WM8985_MASTER
	ret = omap2_mcbsp_set_srg(AUDIO_MCBSP, OMAP2_MCBSP_SRG_ENABLE);
	if (unlikely(ret != 0)) {
		line = __LINE__;
		goto mcbsp_config_exit;
	}
	ret = omap2_mcbsp_set_fsg(AUDIO_MCBSP, OMAP2_MCBSP_FSG_ENABLE);
	if (unlikely(ret != 0)) {
		line = __LINE__;
	}
#endif

	omap2_mcbsp_set_trans_params(AUDIO_MCBSP, &(plat_mcbsp_config->tx_params));
	omap2_mcbsp_set_recv_params(AUDIO_MCBSP,&(plat_mcbsp_config->rx_params));

      mcbsp_config_exit:
	if (unlikely(ret != 0)) {
		printk(KERN_ERR
		       "Unable to configure Mcbsp ret=%d @ line %d.", ret,
		       line);
	}
	FN_OUT(ret);
	return ret;
}
#endif

static int __init audio_codec_init(void)
{
        int ret;

	/* Setup default codec information */
	codec_info.line_gain     = DEFAULT_INPUT_VOLUME;
	codec_info.mic_gain      = DEFAULT_INPUT_VOLUME;
	codec_info.volume_left   = DEFAULT_OUTPUT_VOLUME;
	codec_info.volume_right  = DEFAULT_OUTPUT_VOLUME;
	codec_info.volume_adjust = 0;
	codec_info.sample_rate   = AUDIO_RATE_DEFAULT;
	codec_info.rec_level_line = 0;
	codec_info.rec_level_mic = 0;
	codec_info.rec_level_phone = 0;
	codec_info.config        = OUTPUT_HP|INPUT_LINE;
	codec_info.volume_speaker = 0;
	codec_info.spdif_enabled = 0;
	codec_info.mix_refcount = 0;

	codec_state.data = &codec_info;

        /* register the codec with the audio driver */
        if ((ret = audio_register_codec(&codec_state)) != 0) {
                printk(KERN_ERR "Failed to register WM8985 driver with Audio OSS Driver\n");
        }
	return ret;
}

static void __exit audio_codec_exit(void)
{
        audio_unregister_codec(&codec_state);
        return;
}

module_init(audio_codec_init);
module_exit(audio_codec_exit);

module_param(hp_direct_path, int, S_IRUGO|S_IWUSR);
module_param(zc_switch, int, S_IRUGO|S_IWUSR);
module_param(prog_vol, int, S_IRUGO|S_IWUSR);
module_param(use_pga, int, S_IRUGO|S_IWUSR);
module_param(dump, int, S_IRUGO|S_IWUSR);
module_param(use_analog_loop, int, S_IRUGO|S_IWUSR);

MODULE_AUTHOR("ARCHOS");
MODULE_DESCRIPTION("Glue audio driver for the WM8985 codec.");
MODULE_LICENSE("GPL");
