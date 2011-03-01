#ifndef __DRIVERS_USB_STORAGE_DEVDB_H
#define __DRIVERS_USB_STORAGE_DEVDB_H

extern struct us_data *usb_stor_devdb_find(const char *devpath);
extern int usb_stor_devdb_store(struct us_data *us, const char* devpath);
extern int usb_stor_devdb_remove(const struct us_data *us);

#endif /* __DRIVERS_USB_STORAGE_DEVDB_H */
