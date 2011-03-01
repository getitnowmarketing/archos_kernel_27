#include <linux/device.h>
#include <linux/idr.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <mach/archos_supervisor.h>

#define cls_dev_to_archos_sv(d)		container_of(d, struct archos_sv, class_dev)
#define dev_to_archos_sv_client(d)	container_of(d, struct archos_sv_client, dev)
#define to_archos_sv_driver(d)		container_of(d, struct archos_sv_client_driver, drv)

static DEFINE_IDR(archos_sv_idr);
static DEFINE_SPINLOCK(archos_sv_lock);

static struct archos_sv_global_data {
	struct archos_sv_client *client;
	struct archos_sv *sv;
} sv_global;

static void archos_sv_classdev_release(struct device *dev)
{
	struct archos_sv *sv = cls_dev_to_archos_sv(dev);
	kfree(sv);
}

static int archos_sv_client_match(struct device *dev, struct device_driver *drv)
{
printk("%s\n", __FUNCTION__);
	return 1;
}

static int archos_sv_client_probe(struct device *dev)
{
	struct archos_sv_client_driver *drv = to_archos_sv_driver(dev->driver);
	struct archos_sv_client *client = dev_to_archos_sv_client(dev);

	if (drv->probe)
		return drv->probe(client);

	return 0;
}

static int archos_sv_client_remove(struct device *dev)
{
	struct archos_sv_client_driver *drv = to_archos_sv_driver(dev->driver);
	struct archos_sv_client *client = dev_to_archos_sv_client(dev);

	if (drv->remove)
		return drv->remove(client);

	return 0;
}

static struct class archos_sv_class = {
	.name		= "archos_sv",
	.dev_release	= archos_sv_classdev_release,
};

static struct bus_type archos_sv_bus_type = {
	.name		= "archos_sv",
	.probe		= archos_sv_client_probe,
	.remove		= archos_sv_client_remove,
	.match		= archos_sv_client_match,
};

static struct device_type archos_sv_type = {
	//.groups = archos_sv_attr_groups,
};

int archos_sv_set_charge_mode(struct archos_sv_client *sv_client, unsigned long arg )
{
	return 0;
}

int archos_sv_get_usb_type(struct archos_sv_client *sv_client)
{
	return 0;
}

int archos_sv_battery_dock_check_dcin(struct archos_sv_client *sv_client)
{
	return 0;
}

static void archos_sv_client_release(struct device *dev)
{
	struct archos_sv_client *client = dev_to_archos_sv_client(dev);

	kfree(client);
}

static struct archos_sv_client *archos_sv_client_alloc(void)
{
	struct archos_sv_client *sv_client = kzalloc(sizeof(struct archos_sv_client), GFP_KERNEL);
	if (!sv_client)
		return NULL;

	device_initialize(&sv_client->dev);

	sv_client->dev.parent = NULL;
	sv_client->dev.bus = &archos_sv_bus_type;
	sv_client->dev.release = archos_sv_client_release;
	//sv_client->dev.type = &archos_sv_type;

	return sv_client;
}

static void archos_sv_client_free(struct archos_sv_client *sv_client)
{
	if (!sv_client)
		return;
		
	kfree(sv_client);
}

int archos_sv_register_client(struct archos_sv_client_driver *sv_client_driver)
{
	int err;
	struct archos_sv_client *sv_client;
	
printk("archos_sv_register_client\n");		
	sv_client_driver->drv.bus = &archos_sv_bus_type;
	
	err = driver_register(&sv_client_driver->drv);
	if (err < 0)
		return err;
			
	sv_client = archos_sv_client_alloc();
	if (!sv_client){
		err = -ENOMEM;
		goto fail;
	}
	
	snprintf(sv_client->dev.bus_id, sizeof(sv_client->dev.bus_id),
		 "svc%d", 0);

	err = device_add(&sv_client->dev);
	if (err) {
		goto fail2;
	}
	
	sv_global.client = sv_client;

	printk(KERN_DEBUG "%s: supervisor client device \'%s\' registered\n", sv_client->dev.bus_id, sv_client_driver->name ? sv_client_driver->name : "Unknown");
	return 0;
fail2:
	archos_sv_client_free(sv_client);
fail:
	driver_unregister(&sv_client_driver->drv);
	return err;
}

void archos_sv_unregister_client(struct archos_sv_client_driver *sv_client_driver)
{
	device_del(&sv_global.client->dev);
	
	sv_client_driver->drv.bus = &archos_sv_bus_type;
	
	driver_unregister(&sv_client_driver->drv);
	
	archos_sv_client_free(sv_global.client);
	
	printk(KERN_DEBUG "supervisor client device \'%s\' unregistered\n", sv_client_driver->name ? sv_client_driver->name : "Unknown");
}

static int archos_sv_client_signal_event(struct device *dev, void *_ev)
{
	struct archos_sv_client_driver *drv = to_archos_sv_driver(dev->driver);	
	struct archos_sv_event *ev = (struct archos_sv_event *)_ev;
	
	if (drv->event)
		drv->event(dev_to_archos_sv_client(dev), ev);
		
	return 0;
}

int archos_sv_signal_event(struct archos_sv *sv, struct archos_sv_event *ev) 
{
printk("%s\n", __FUNCTION__);
	return bus_for_each_dev(&archos_sv_bus_type, NULL, (void *)ev, archos_sv_client_signal_event);
}

static int archos_sv_open(struct inode *inode, struct file *file)
{

	return 0;
}

static int archos_sv_release(struct inode *inode, struct file *file)
{

	return 0;
}

static int archos_sv_ioctl( struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg )
{
	return 0;
}

static ssize_t archos_sv_read(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	int len = 0;
	return len;
}

static unsigned int archos_sv_poll(struct file * file, poll_table * wait)
{
	unsigned int mask = 0;

	//poll_wait(file, &sv->wait_queue, wait);

	return mask;
}

static struct file_operations archos_sv_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= archos_sv_ioctl,
	.open		= archos_sv_open,
	.release	= archos_sv_release,
	.read		= archos_sv_read,
	.poll		= archos_sv_poll,
};

static struct miscdevice archos_sv_dev =
{
	.minor		= ATMEGA_IO_MINOR,
	.name		= "atmg",
	.fops		= &archos_sv_fops,
};

struct archos_sv *archos_sv_alloc(int sizeof_private, struct device *dev)
{
	int err;
	struct archos_sv *sv;

	if (!idr_pre_get(&archos_sv_idr, GFP_KERNEL))
		return NULL;

	sv = kzalloc(sizeof(struct archos_sv)+sizeof_private, GFP_KERNEL);
	if (!sv)
		return NULL;
		
	spin_lock(&archos_sv_lock);
	err = idr_get_new(&archos_sv_idr, sv, &sv->index);
	spin_unlock(&archos_sv_lock);
	if (err)
		goto free;

	snprintf(sv->class_dev.bus_id, BUS_ID_SIZE,
		 "sv%d", sv->index);

	sv->parent = dev;
	sv->class_dev.parent = dev;
	sv->class_dev.class = &archos_sv_class;
	device_initialize(&sv->class_dev);

	init_waitqueue_head(&sv->wait_queue);

	return sv;
free:
	kfree(sv);
	return NULL;
}

void archos_sv_free(struct archos_sv *sv)
{
	spin_lock(&archos_sv_lock);
	idr_remove(&archos_sv_idr, sv->index);
	spin_unlock(&archos_sv_lock);

	put_device(&sv->class_dev);
}

int archos_sv_register(struct archos_sv *sv)
{
	int err;	

	err = misc_register( &archos_sv_dev );
	if (err)
		return err;

	err = device_add(&sv->class_dev);
	if (err) {
		misc_deregister( &archos_sv_dev );
		return err;
	}

	printk(KERN_DEBUG "%s: supervisor device \'%s\' registered\n",
		       sv->class_dev.bus_id, sv->name ? sv->name : "Unknown");

	sv_global.sv = sv;
	
	return 0;
}

void archos_sv_unregister(struct archos_sv *sv)
{
	device_del(&sv->class_dev);
	misc_deregister( &archos_sv_dev );
}

int __init archos_supervisor_init(void)
{
	int err;
	printk(KERN_INFO "Archos supervisor system\n");

	err = bus_register(&archos_sv_bus_type);
	if (err < 0) {
		printk(KERN_ERR "Archos supervisor system initialization failed\n");
		return err;
	}
	
	err = class_register(&archos_sv_class);
	if (err < 0) {
		printk(KERN_ERR "Archos supervisor system initialization failed\n");
		bus_unregister(&archos_sv_bus_type);
		return err;
	}

	printk(KERN_DEBUG "Archos supervisor system initialized\n");
	
	return 0;	
}

void __exit  archos_supervisor_exit(void)
{
	class_unregister(&archos_sv_class);
	bus_unregister(&archos_sv_bus_type);
}

subsys_initcall( archos_supervisor_init );
module_exit( archos_supervisor_exit );

/* Module information */
MODULE_DESCRIPTION("ARCHOS system supervisor");
MODULE_LICENSE("GPL");
