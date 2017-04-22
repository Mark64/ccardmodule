// implementation for the init and deinit functions for the kernel
//   module build
// implements sysfs interface for exposing system
//   calls to scripting languages


#include<linux/module.h>
#include<linux/kernel.h>
#include<linux/init.h>
#include<linux/gpio.h>
#include<linux/semaphore.h>
#include "ccard.h"

static int __init start_ccard(void);
static void __exit poweroff_ccard(void);

static struct semaphore sem3v3power;
static struct semaphore sem5v0power;

module_init(start_ccard);
module_exit(poweroff_ccard);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mark Hill <markleehill@gmail.com>");
MODULE_VERSION("0.0.0");
MODULE_SUPPORTED_DEVICE("intrepid-based ccard2");


static int __init start_ccard(void)
{
	sema_init(&sem3v3power, 0);
	sema_init(&sem5v0power, 0);

	int pleasegive = gpio_request(102, "3v3ccard");
	if (pleasegive)
		printk(KERN_DEBUG "stop exporting gpio 102\n");
	pleasegive = gpio_request(103, "5v0ccard");
	if (pleasegive)
		printk(KERN_DEBUG "stop exporting gpio 103\n");

	// start the i2c driver which will start up all the
	//   components attached to the i2c bus
//	if (ccard_init_i2c()) {
//		printk(KERN_ERR "failed to initialize i2c driver. \
				c card module failed to load\n");
//		return 1;
//	}

	printk(KERN_NOTICE "hi irvine02\nit's nice and warm here \
			in kernelspace\nkeep an eye out for \
			messages about i2c initialization\n");

	return 0;
}

static void __exit poweroff_ccard(void) {
//	ccard_cleanup_i2c();

	gpio_free(102);
	gpio_free(103);

	printk(KERN_NOTICE "bye irvine02\n");
}



static inline void set_power(u8 gpio, u8 state, s8 flags) {
	struct semaphore sem = (gpio == 102) ? sem3v3power : sem5v0power;
	if (state == 0 && (down_trylock(&sem) || flags)) {
		gpio_direction_output(102, 0);
		printk(KERN_NOTICE "turning off gpio %i\n", gpio);
	} else {
		up(&sem);
		gpio_direction_output(102, 1);
		printk(KERN_NOTICE "turning on gpio %i\n", gpio);
	}
}

void set_3v3_state(u8 state, s8 flags) {
	set_power(102, state, flags);
}

void set_5v0_state(u8 state, s8 flags) {
	set_power(103, state, flags);
}



