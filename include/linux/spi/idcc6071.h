/* linux/spi/idcc6071.h */

/* Touchscreen characteristics vary between boards and models.  The
 * platform_data for the device's "struct device" holds this information.
 *
 * It's OK if the min/max values are zero.
 */
#include <mach/mux.h>
#ifdef ZOOM1
#define SPI_GPIO_MRDY	136
#define SPI_GPIO_SRDY	137

#define MRDY_CFG_REG EA4_3430_GPIO_136
#define SRDY_CFG_REG AH3_3430_GPIO_137
#endif

/* IDCC */
#define SPI_GPIO_MRDY     29
#define SPI_GPIO_SRDY     0
#define MRDY_CFG_REG     AH8_3430_GPIO29       /* configure MRDY */
#define SRDY_CFG_REG     AF26_3430_GPIO0       /* configure SRDY */

/*
 * FIXME: Move this file to a non-SPI location
 */
#define MODEM_GPIO_AUDIO         95
#define MODEM_GPIO_RESET         52
#define MODEM_GPIO_PWRON         127
