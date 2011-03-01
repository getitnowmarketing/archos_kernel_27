
#ifndef _LINUX_ATMEGA_H
#define _LINUX_ATMEGA_H

#define ATMEGA_EEPROM_TABLE 16

struct atmega_exchange_table {
        unsigned long	value;
        unsigned char	control_byte;
        unsigned char	p1;
        unsigned char	p2;
        unsigned char	p3;
};

// MSP CMD register bit mask
#define ATMEGA_I2C_CTRL_CMD_NONE		0
#define ATMEGA_I2C_CTRL_CMD_SET_RTC		1
#define ATMEGA_I2C_CTRL_CMD_SET_ALARM		2
#define ATMEGA_I2C_CTRL_CMD_SLEEP		3
#define ATMEGA_I2C_CTRL_CMD_NOT_USED		4
#define ATMEGA_I2C_CTRL_CMD_RESET_ALARM	5
#define ATMEGA_I2C_CTRL_CMD_RESET_DAVINCI	6
#define ATMEGA_I2C_CTRL_CMD_SHUTDOWN		7
#define ATMEGA_I2C_CTRL_CMD_CHARGEMODE		8	
#define ATMEGA_I2C_CTRL_CMD_CHG_LED_TOGGLE	9
#define ATMEGA_I2C_CTRL_CMD_EEPROM		11

// MSP charge mode value
#define CHARGER_OFF		0x000
#define CHARGER_ON_LOW		0x200
#define CHARGER_ON_HIGH	0x300

#define  ATMEGA_I2C_COMMAND_MAGIC		0x5a

// MSP read registers
#define  ATMEGA_I2C_READ_REG_BASE		0x00
#define  ATMEGA_I2C_TIME_REG			0x00
#define  ATMEGA_I2C_TIME_SIZE			4
#define  ATMEGA_I2C_BOARDID_REG		0x04
#define  ATMEGA_I2C_BOARDID_SIZE		2
#define  ATMEGA_I2C_VERSION_REG		0x06
#define  ATMEGA_I2C_VERSION_SIZE		2
#define  ATMEGA_I2C_STATUS_REG			0x08
#define  ATMEGA_I2C_STATUS_SIZE		2
#define  ATMEGA_I2C_CURRENTSTATE_REG		0x0a
#define  ATMEGA_I2C_CURRENTSTATE_SIZE		1
#define  ATMEGA_I2C_NEXTSTATE_REG		0x0b
#define  ATMEGA_I2C_NEXTSTATE_SIZE		1
#define  ATMEGA_I2C_ADC0_REG			0x0c
#define  ATMEGA_I2C_ADC0_SIZE			2
#define  ATMEGA_I2C_ADC1_REG			0x0e
#define  ATMEGA_I2C_ADC1_SIZE			2
#define  ATMEGA_I2C_ADC2_REG			0x10
#define  ATMEGA_I2C_ADC2_SIZE			2
#define  ATMEGA_I2C_ADC3_REG			0x12
#define  ATMEGA_I2C_ADC3_SIZE			2
#define  ATMEGA_I2C_ALARM_REG			0x14
#define  ATMEGA_I2C_ALARM_SIZE			4
#define  ATMEGA_I2C_CS_REG			0x18
#define  ATMEGA_I2C_CS_SIZE			2

#define  ATMEGA_I2C_REREAD_REG_BASE		0x1a

#define  ATMEGA_I2C_READ_EEPROM_REG		0x24
#define  ATMEGA_I2C_READ_EEPROM_SIZE		ATMEGA_EEPROM_TABLE

// MSP write registers
#define  ATMEGA_I2C_WRITE_REG_BASE		0x00
#define  ATMEGA_I2C_COMMAND_REG		0x04
#define  ATMEGA_I2C_AVAILABLE_REG		0x08
#define  ATMEGA_I2C_AVAILABLE_SIZE		1
#define  ATMEGA_I2C_WRITE_EEPROM_REG		0x0a
#define  ATMEGA_I2C_EXCHANGE_REG		ATMEGA_I2C_WRITE_REG_BASE
#define  ATMEGA_I2C_EXCHANGE_SIZE		10

#ifdef __KERNEL__
extern int atmega_write_value( unsigned int reg, unsigned char *value, int size );
extern int atmega_write_table( struct  atmega_exchange_table * );
extern int atmega_read_value( unsigned int reg, unsigned char *value, int size );
extern int atmega_read_eeprom(unsigned char *buf, int offs, int size);
extern int atmega_write_eeprom(const unsigned char *buf, int offs, int size);
#endif

#endif /* _LINUX_ATMEGA_H */
