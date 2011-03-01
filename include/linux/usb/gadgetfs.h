/*
 * Filesystem based user-mode API to USB Gadget controller hardware
 *
 * Other than ep0 operations, most things are done by read() and write()
 * on endpoint files found in one directory.  They are configured by
 * writing descriptors, and then may be used for normal stream style
 * i/o requests.  When ep0 is configured, the device can enumerate;
 * when it's closed, the device disconnects from usb.  Operations on
 * ep0 require ioctl() operations.
 *
 * Configuration and device descriptors get written to /dev/gadget/$CHIP,
 * which may then be used to read usb_gadgetfs_event structs.  The driver
 * may activate endpoints as it handles SET_CONFIGURATION setup events,
 * or earlier; writing endpoint descriptors to /dev/gadget/$ENDPOINT
 * then performing data transfers by reading or writing.
 */

#ifndef __LINUX_USB_GADGETFS_H
#define __LINUX_USB_GADGETFS_H

#include <asm/types.h>
#include <asm/ioctl.h>

#include <linux/usb/ch9.h>

/*
 * Events are delivered on the ep0 file descriptor, when the user mode driver
 * reads from this file descriptor after writing the descriptors.  Don't
 * stop polling this descriptor.
 */

enum usb_gadgetfs_event_type {
	GADGETFS_NOP = 0,

	GADGETFS_CONNECT,
	GADGETFS_DISCONNECT,
	GADGETFS_SETUP,
	GADGETFS_SUSPEND,
	/* and likely more ! */
};

/* NOTE:  this structure must stay the same size and layout on
 * both 32-bit and 64-bit kernels.
 */
struct usb_gadgetfs_event {
	union {
		/* NOP, DISCONNECT, SUSPEND: nothing
		 * ... some hardware can't report disconnection
		 */

		/* CONNECT: just the speed */
		enum usb_device_speed	speed;

		/* SETUP: packet; DATA phase i/o precedes next event
		 *(setup.bmRequestType & USB_DIR_IN) flags direction
		 * ... includes SET_CONFIGURATION, SET_INTERFACE
		 */
		struct usb_ctrlrequest	setup;
	} u;
	enum usb_gadgetfs_event_type	type;
};

#define GADGETFS_IO_PENDING	0
#define GADGETFS_IO_DONE	1

struct gadget_io_buffer{
	int		size;

	unsigned char	*data;		// user space buffer
	int		length;		// amount of data to be transferred
	unsigned char	*kdata;		// copy in kernel space

	int		zero;		// whether to send a null packet

	int		transferred;
	volatile int	status;

	void		*req;
	int		direction;
};

/* The 'g' code is also used by printer gadget ioctl requests.
 * Don't add any colliding codes to either driver, and keep
 * them in unique ranges (size 0x20 for now).
 */

/* endpoint ioctls */

/* IN transfers may be reported to the gadget driver as complete
 *	when the fifo is loaded, before the host reads the data;
 * OUT transfers may be reported to the host's "client" driver as
 *	complete when they're sitting in the FIFO unread.
 * THIS returns how many bytes are "unclaimed" in the endpoint fifo
 * (needed for precise fault handling, when the hardware allows it)
 */
#define	GADGETFS_FIFO_STATUS	_IO('g', 1)

/* discards any unclaimed data in the fifo. */
#define	GADGETFS_FIFO_FLUSH	_IO('g', 2)

/* resets endpoint halt+toggle; used to implement set_interface.
 * some hardware (like pxa2xx) can't support this.
 */
#define	GADGETFS_CLEAR_HALT	_IO('g', 3)

#define GADGETFS_CLEAR_QUEUE    _IO('g',4)

#define GADGETFS_IO_SUBMIT        _IO('g',5)
#define GADGETFS_IO_CANCEL        _IO('g',6)
#define GADGETFS_IO_GET_STATUS    _IO('g',7)
#define GADGETFS_IO_DEVICE_STATUS    _IO('g',8)

#define GADGETFS_IN 		1
#define GADGETFS_OUT 		2

#endif /* __LINUX_USB_GADGETFS_H */
