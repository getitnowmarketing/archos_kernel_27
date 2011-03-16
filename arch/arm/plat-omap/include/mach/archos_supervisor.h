#ifndef _ARCHOS_SUPERVISOR_H
#define _ARCHOS_SUPERVISOR_H

#include <linux/device.h>
#include <linux/poll.h>

// must be in the same order of module_resistors!!
// see common/Include/sys_atmega.h
#define MODULE_ID_TV_CRADLE			1
#define	MODULE_ID_REMOTE_FM			2
#define MODULE_ID_VRA_GEN6			3
#define	MODULE_ID_GPS_WITHOUT_TMC		4
#define MODULE_ID_USB_HOST_CABLE		5
#define MODULE_ID_BATTERY_DOCK_GEN6		6
#define MODULE_ID_POWER_CABLE			7
#define MODULE_ID_SERIAL_ADAPTER		8
#define MODULE_ID_PC_CABLE			9
#define MODULE_ID_UNUSED10			10
#define MODULE_ID_GPS_WITH_TMC			11
#define MODULE_ID_HDMI_DOCK			12
#define MODULE_ID_MINI_DOCK			13
#define MODULE_ID_DVBT_SNAP_ON			14
#define MODULE_ID_MUSIC_DOCK			15

enum archos_sv_event_id {
	ARCHOS_SV_EVENT_STATUS_CHANGED,
	ARCHOS_SV_EVENT_KEY,
};

struct archos_sv_event_key {
	int key;
	unsigned long flags;
};

struct archos_sv_event {
	enum archos_sv_event_id id;
	void *data;
	int size;
};

struct archos_sv;

struct archos_sv_ops {
	int (*ioctl)(struct archos_sv *sv, unsigned int cmd, unsigned long arg);
	int (*read)(struct archos_sv *sv, char __user *buffer, size_t count, loff_t *ppos);
	unsigned int (*poll)(struct archos_sv *sv, poll_table * wait);
};

struct archos_sv {
	const char *name;
	struct device *parent;
	struct device class_dev;
	int index;
	wait_queue_head_t wait_queue;
	struct archos_sv_ops *ops;
	
	unsigned long private[0] ____cacheline_aligned;
};

struct archos_sv_client {
	struct archos_sv *sv;
	struct device dev;
};

struct archos_sv_client_driver {
	const char *name;
	struct device_driver drv;
	int (*probe) (struct archos_sv_client *sv_client);
	int (*remove) (struct archos_sv_client *sv_client);
	void (*event) (struct archos_sv_client *sv_client, struct archos_sv_event *ev);
};

int archos_sv_register_client(struct archos_sv_client_driver *c);
void archos_sv_unregister_client(struct archos_sv_client_driver *c);
struct archos_sv *archos_sv_alloc(int sizeof_private, struct device *dev);
void archos_sv_free(struct archos_sv *sv);
int archos_sv_register(struct archos_sv *sv);
void archos_sv_unregister(struct archos_sv *sv);

int archos_sv_set_charge_mode(struct archos_sv_client *sv_client, unsigned long arg);
int archos_sv_get_usb_type(struct archos_sv_client *sv_client);
int archos_sv_battery_dock_check_dcin(struct archos_sv_client *sv_client);

int archos_sv_signal_event(struct archos_sv *sv, struct archos_sv_event *ev);

#endif /* _ARCHOS_SUPERVISOR_H */
