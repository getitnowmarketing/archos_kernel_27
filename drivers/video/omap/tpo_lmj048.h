#ifndef _LINUX_TPOLMJ_H
#define _LINUX_TPOLMJ_H

// TPO LMJ0T001A registers for 4"8 lcd

#define  TPO_CHIPID			0x00
#define  TPO_REVID			0x01

#define  TPO_COLORMODE			0x03
#define  TPO_POL			0x04
#define  TPO_OUTPUT			0x05
#define  TPO_PARS			0x06
#define  TPO_CKH1			0x07
#define  TPO_CKH2			0x08
#define  TPO_ENBCKH			0x09
#define  TPO_ENBCKV			0x0a
#define  TPO_ENBDE			0x0b
#define  TPO_RGAIN			0x0c
#define  TPO_GGAIN			0x0d
#define  TPO_BGAIN			0x0e
#define  TPO_ROFFSET			0x0f
#define  TPO_GOFFSET			0x10
#define  TPO_BOFFSET			0x11
#define  TPO_PVH			0x12
#define  TPO_NVH			0x13

#define  TPO_RGAMMA16			0x14
#define  TPO_RGAMMA64			0x15
#define  TPO_RGAMMA192			0x16
#define  TPO_RGAMMA256			0x17

#define  TPO_GAMMA0			0x18
#define  TPO_GAMMA1			0x19
#define  TPO_GAMMA2			0x1a
#define  TPO_GAMMA3			0x1b
#define  TPO_GAMMA4			0x1c
#define  TPO_GAMMA5			0x1d
#define  TPO_GAMMA6			0x1e

#define  TPO_GAMMAC64			0x1f
#define  TPO_GAMMAC128			0x20
#define  TPO_GAMMAC192			0x21
#define  TPO_GAMMAC224			0x22
#define  TPO_GAMMAC240			0x23
#define  TPO_GAMMAC248			0x24
#define  TPO_GAMMAC252			0x25
#define  TPO_GAMMAC255			0x26

#define  TPO_VCOM			0x3a
#define  TPO_HBP_ADJ			0x3c
#define  TPO_HTOTAL			0x3d
#define  TPO_VBP_ADJ			0x3e
#define  TPO_VTOTAL			0x3f
#define  TPO_POWER			0x42
#define  TPO_PWM			0x43

#endif /* _LINUX_TPOLMJ_H */
