#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <mach/board-archos.h>
#include <linux/gpio_keys.h>
#include <asm/gpio.h>

static int is_avos(struct task_struct * p)
{
	if (p->pid) {
		char *c, *d = "avos";
		for (c = p->comm; *c != 0 ; ++c){
			if (*c == *d){
				++d;
			}
			if (*d == 0 ) {
				if (*(c+1) == 0) {
					// avos$, so either arm_avos or avos
					return 1;
				} else {
					// avos.+, so probably avos_helper.sh
					return 0;
				}
			}
		}
	}
	return 0;
}

static void avos_kill(struct work_struct *work)
{
	struct task_struct *g, *p;
	struct mm_struct *mm = NULL;
printk("avos kill\n");
	read_lock(&tasklist_lock);

	do_each_thread(g, p)
		if (is_avos(p)) {
			mm = get_task_mm(p);
			printk(KERN_ERR "Killing process %d (%s).\n", p->pid, p->comm);
			// we kill the process group, so no need to send signal to all threads
			group_send_sig_info(SIGABRT, (void*)1L, p);
			goto done;
		}
	while_each_thread(g, p);
done:
	read_unlock(&tasklist_lock);
	if (mm)
		mmput(mm);

	return;
}

static DECLARE_WORK(kill_work,(work_func_t)avos_kill);

static void magic_keypress(int pressed)
{
	static int avos_killed = 0;

	/* if both keys pressed ...*/
	if (pressed) {
		if (!avos_killed)
			schedule_work(&kill_work);
		avos_killed = 1;
	} else
		avos_killed = 0;
}

static irqreturn_t core_dump_keys_isr(int irq, void *dev_id)
{
	struct platform_device *pdev = dev_id;
	struct archos_core_platform_data *pdata = pdev->dev.platform_data;
	int i;
	int key_down = 0;
	
	for (i = 0; i < pdata->gpio_keys->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->gpio_keys->buttons[i];
		int j;
		for (j = 0; j < pdata->ncodes; j++) {
			if (pdata->gpio_keys->buttons[i].code == pdata->codes[j] 
					&& ((gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low)) {
				key_down++;
				break;
			}
		}
	}
	
	magic_keypress(key_down == pdata->ncodes);
	
	return IRQ_HANDLED;
}

static int __devinit core_dump_probe(struct platform_device *pdev)
{
	struct archos_core_platform_data *pdata = pdev->dev.platform_data;
	int i, error;

	for (i = 0; i < pdata->gpio_keys->nbuttons; i++) {
		int j, irq;
		struct gpio_keys_button *button = NULL;
		for (j = 0; j < pdata->ncodes; j++) {
			if (pdata->gpio_keys->buttons[i].code == pdata->codes[j]) {
				button = &pdata->gpio_keys->buttons[i];	
				break;
			}
		}
		
		if (button == NULL)
			continue;
			
#ifndef CONFIG_KEYBOARD_GPIO
		error = gpio_request(button->gpio, button->desc ?: "core_dump_keys");
		if (error < 0) {
			pr_err("core-dump: failed to request GPIO %d,"
				" error %d\n", button->gpio, error);
			goto fail;
		}

		error = gpio_direction_input(button->gpio);
		if (error < 0) {
			pr_err("core-dump: failed to configure input"
				" direction for GPIO %d, error %d\n",
				button->gpio, error);
			gpio_free(button->gpio);
			goto fail;
		}

#endif
		irq = gpio_to_irq(button->gpio);
		if (irq < 0) {
			error = irq;
			pr_err("core-dump: Unable to get irq number"
				" for GPIO %d, error %d\n",
				button->gpio, error);
			goto fail;
		}

		error = request_irq(irq, core_dump_keys_isr,
				    IRQF_SAMPLE_RANDOM | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED,
				    button->desc ? button->desc : "core_dump_keys",
				    pdev);
		if (error) {
			pr_err("core-dump: Unable to claim irq %d; error %d\n",
				irq, error);
			goto fail;
		}
	}

	return 0;

 fail:
	while (--i >= 0) {
		int j;
		for (j = 0; j < pdata->ncodes; j++) {
			if (pdata->gpio_keys->buttons[i].code == pdata->codes[j]) {
				int irq = gpio_to_irq(pdata->gpio_keys->buttons[i].gpio);
				free_irq(irq, pdev);
#ifndef CONFIG_KEYBOARD_GPIO
				gpio_free(pdata->gpio_keys->buttons[i].gpio);
#endif
				break;
			 }
		}
	}

	platform_set_drvdata(pdev, NULL);
	
	return -1;
}

static int __devexit core_dump_remove(struct platform_device *pdev)
{
	struct archos_core_platform_data *pdata = pdev->dev.platform_data;
	int i, j;

	for (i = 0; i <pdata->gpio_keys->nbuttons; i++) {
		for (j = 0; j < pdata->ncodes; j++) {
			if (pdata->gpio_keys->buttons[i].code == pdata->codes[j]) {
				int irq = gpio_to_irq(pdata->gpio_keys->buttons[i].gpio);
				free_irq(irq, pdev);
#ifndef CONFIG_KEYBOARD_GPIO
				gpio_free(pdata->gpio_keys->buttons[i].gpio);
#endif
			 }
		}
	}

	return 0;
}

static struct platform_driver core_dump_device_driver = {
	.probe		= core_dump_probe,
	.remove		= __devexit_p(core_dump_remove),
	.suspend	= NULL,
	.resume		= NULL,
	.driver		= {
		.name	= "core-dump",
		.owner	= THIS_MODULE,
	}
};

static int __init core_dump_init(void)
{
	return platform_driver_register(&core_dump_device_driver);
}

static void __exit core_dump_exit(void)
{
	platform_driver_unregister(&core_dump_device_driver);	
}
module_init(core_dump_init);
module_exit(core_dump_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Niklas Schroeter <schroeter@archos.com>");
MODULE_DESCRIPTION("Core Dumper");
MODULE_ALIAS("platform:core-dump");

