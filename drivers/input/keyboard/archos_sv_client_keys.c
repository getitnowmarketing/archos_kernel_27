#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <mach/archos_supervisor.h>

void archos_sv_keys_handle_event(struct archos_sv_client *sv_client, struct archos_sv_event *ev)
{
printk("%s\n", __FUNCTION__);
	switch(ev->id)
	{
		case ARCHOS_SV_EVENT_STATUS_CHANGED:
			break;
		
		default:
			break;
	}
}

static int archos_sv_keys_probe(struct archos_sv_client *sv_client)
{
printk("archos_sv_client_battery_probe\n");
	
	return 0;
}

static int archos_sv_keys_remove(struct archos_sv_client *sv_client)
{	

	return 0;
}

static struct archos_sv_client_driver archos_sv_keys_driver = {
	.probe 	= archos_sv_keys_probe,	
	.remove = archos_sv_keys_remove,
	.event 	= archos_sv_keys_handle_event,	
	.drv		= {
		.name	= "archos-sv-keys",
	},
};

static int __init archos_sv_keys_init(void)
{
	return archos_sv_register_client(&archos_sv_keys_driver);
}

static void __exit archos_sv_keys_exit(void)
{
	archos_sv_unregister_client(&archos_sv_keys_driver);
}

module_init(archos_sv_keys_init);
module_exit(archos_sv_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Niklas Schroeter <schroeter@archos.com>");
MODULE_DESCRIPTION("Keyboard driver for input events generated from Archos supervisor devices");
