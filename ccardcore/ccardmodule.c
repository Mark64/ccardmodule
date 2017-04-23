// implementation for the init and deinit functions for the kernel
//   module build
// implements sysfs interface for exposing hardware function
//   calls to scripting languages and userspace applications
// by Mark Hill

#include<linux/module.h>
#include<linux/kernel.h>
#include<linux/init.h>
#include<linux/gpio.h>
#include<linux/semaphore.h>
#include "ccard.h"
#include "i2c_ccard.c"
#include "magnetorquer.c"
#include "dsa.c"


#define ccard_3v3_gpio 102
#define ccard_5v0_gpio 103

static int __init start_ccard(void);
static void __exit poweroff_ccard(void);

static struct semaphore sem3v3power;
static struct semaphore sem5v0power;

module_init(start_ccard);
module_exit(poweroff_ccard);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mark Hill <markleehill@gmail.com>");
MODULE_VERSION("0.1.0");
MODULE_DESCRIPTION("controls the LED, GPS, thruster, DSAs, and magnetorquers");
MODULE_SUPPORTED_DEVICE("intrepid-based ccard2");

// init function
static int __init start_ccard(void)
{
	sema_init(&sem3v3power, 0);
	sema_init(&sem5v0power, 0);

	int pleasegive = gpio_request(ccard_3v3_gpio, "3v3ccard");
	if (pleasegive)
		printk(KERN_DEBUG "stop exporting gpio %i\n", ccard_3v3_gpio);
	pleasegive = gpio_request(ccard_5v0_gpio, "5v0ccard");
	if (pleasegive)
		printk(KERN_DEBUG "stop exporting gpio %i\n", ccard_5v0_gpio);

	set_5v0_pwr(1, 0);

	// start the i2c driver which will start up all the
	//   components attached to the i2c bus
	if (ccard_init_i2c()) {
		printk(KERN_ERR "failed to initialize i2c driver\n");
		return 1;
	}


	printk(KERN_NOTICE "hi irvine02\n");
	printk(KERN_NOTICE "it's nice to be here in kernelspace\n");
	printk(KERN_NOTICE "see the sysfs (/sys) for my interface\n");

	return 0;
}

// is only a concern when built as a loadable module (debugging)
static void __exit poweroff_ccard(void) {
	ccard_cleanup_i2c();

	set_dsa_pwr(0, 1);
	set_5v0_pwr(0, 1);

	gpio_free(ccard_3v3_gpio);
	gpio_free(ccard_5v0_gpio);

	printk(KERN_NOTICE "looks like it's time to go. bye!\n");
}



//
// power section
//

static inline void set_power(u8 gpio, u8 state, s8 flags) {
	struct semaphore *sem = (gpio == ccard_3v3_gpio) ? \
			       &sem3v3power : &sem5v0power;
	if (state == 0 && (down_trylock(sem) || flags)) {
		gpio_direction_output(gpio, 0);
		printk(KERN_NOTICE "turning off gpio %i\n", gpio);
	} else {
		up(sem);
		gpio_direction_output(gpio, 1);
		printk(KERN_NOTICE "turning on gpio %i\n", gpio);
	}
}

void set_dsa_pwr(u8 state, s8 flags) {
	set_power(ccard_3v3_gpio, state, flags);
}

void set_5v0_pwr(u8 state, s8 flags) {
	set_power(ccard_5v0_gpio, state, flags);
}



