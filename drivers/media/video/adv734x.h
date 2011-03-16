#ifndef _ADV734X_H_
#define _ADV734X_H_

/* reg 0 */
#define PLL_ON	(0<<1)
#define PLL_OFF	(1<<1)
#define PLL_VAL	PLL_ON

#define DAC3 (1<<2)
#define DAC2 (1<<3)
#define DAC1 (1<<4)
#define DAC6 (1<<5)
#define DAC5 (1<<6)
#define DAC4 (1<<7)

#define COMPOSITE_DAC	(DAC4)
#define RGB_DAC		(DAC1|DAC2|DAC3|DAC4)
#define COMPONENT_DAC	(DAC1|DAC2|DAC3)
#define SVIDEO_DAC	(DAC5|DAC6)

#endif
