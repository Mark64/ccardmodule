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
#include<linux/device.h>
#include "ccard.h"
#include "spi_ccard.c"
#include "i2c_ccard.c"
#include "magnetorquer.c"
#include "dsa.c"
#include "gps.c"
#include "thruster.c"


#define ccard_3v3_gpio 102
#define ccard_5v0_gpio 103

static int __init start_ccard(void);
static void __exit poweroff_ccard(void);

static struct semaphore sem3v3power;
static struct semaphore sem5v0power;

// holds the class for the navigation devices
static struct class _nav_class;
// creates the _nav_class object
static inline s8 create_ccard_nav_class(void);
// unregisters the _nav_class object
static inline void remove_ccard_nav_class(void);

module_init(start_ccard);
module_exit(poweroff_ccard);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Mark Hill <markleehill@gmail.com>");
MODULE_VERSION("0.3.1");
MODULE_DESCRIPTION("controls DSAs and magnetorquers");
MODULE_SUPPORTED_DEVICE("IRVINE02 ccard");

// init function
static int __init start_ccard(void)
{
	sema_init(&sem3v3power, 0);
	sema_init(&sem5v0power, 0);

	int pleasegive = gpio_request(ccard_3v3_gpio, "3v3ccard");
	//if (pleasegive)
	//	printk(KERN_DEBUG "stop exporting gpio %i\n", ccard_3v3_gpio);
	pleasegive = gpio_request(ccard_5v0_gpio, "5v0ccard");
	//if (pleasegive)
	//	printk(KERN_DEBUG "stop exporting gpio %i\n", ccard_5v0_gpio);

	set_5v0_pwr(1, 0);

	// start the i2c driver which will start up all the
	//   components attached to the i2c bus
	if (ccard_init_i2c()) {
		printk(KERN_ERR "failed to initialize i2c driver\n");
		return 1;
	}

	//if (create_ccard_nav_class()) {
	//	printk(KERN_ERR "failed to create the navigation class\n");
	//	return 1;
	//}

	//if (ccard_init_spi()) {
	//	printk(KERN_ERR "failed to initialize the spi driver\n");
	//	return 1;
	//}


	printk(KERN_NOTICE "c card driver loaded\n");

	return 0;
}

// is only a concern when built as a loadable module (debugging)
static void __exit poweroff_ccard(void) {
	ccard_cleanup_i2c();
	//ccard_cleanup_spi();

	//remove_ccard_nav_class();

	set_dsa_pwr(0, 1);
	set_5v0_pwr(0, 1);

	gpio_free(ccard_3v3_gpio);
	gpio_free(ccard_5v0_gpio);

	printk(KERN_NOTICE "exiting c card driver\n");
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


//
// sysfs section
//

static void ccard_release_nav_dev(struct device *dev)
{
	printk(KERN_DEBUG "releasing nav device file\n");
}

static inline s8 create_ccard_nav_class()
{
	struct class nav = {
		.name = "navigation",
		.owner = THIS_MODULE,
		.dev_release = ccard_release_nav_dev,
	};
	_nav_class = nav;

	return class_register(&_nav_class);
}

static inline void remove_ccard_nav_class()
{
	class_unregister(&_nav_class);
}

struct class *ccard_nav_class()
{
	return &_nav_class;
}




