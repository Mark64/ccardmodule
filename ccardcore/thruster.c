// implementation for the thruster code
//
// by Mark Hill

#include<linux/kernel.h>
#include<linux/i2c.h>
#include<linux/types.h>
#include<linux/fs.h>
#include<linux/sysfs.h>
#include<linux/device.h>
#include<linux/string.h>

#include "ccard.h"

// defines the number of thrusters present on the device
#define THRUSTER_COUNT 1

// creates and removes the thruster sysfs object
static s8 create_thruster_devices(void);
static void remove_thruster_devices(void);

// definitions for the thruster attributes sysfs callbacks
static ssize_t read_thruster_percent(struct device *dev, \
				     struct device_attribute *attr, char *buf);
static ssize_t write_thruster_percent(struct device *dev, \
				      struct device_attribute *attr, \
				      const char *buf, size_t count);


// stores the thruster class
static struct class _thruster_class;
// stores the device numbers
static dev_t _dev_thruster[THRUSTER_COUNT];
// stores the device structs
static struct device *_thruster_devices[THRUSTER_COUNT];
// device attribute for the thrusters
static DEVICE_ATTR(thrust, S_IRUSR | S_IWUSR, read_thruster_percent, \
		   write_thruster_percent);

// stores a flag indicating if the thruster has been initialized
// 0 = uninitialized, 1 = initialized
static s8 _thruster_initialized = 0;
#define THRUST_RESOLUTION 10000
// stores the current value written to the DAC since the device
//   is read only hardware
// value is equal to (true percent) * THRUST_RESOLUTION
static u16 _thrust_percents[THRUSTER_COUNT] = {0};


s8 init_thruster()
{
	// checks if the hardware has been initialized
	if (_thruster_initialized)
		return 0;

	// creates the thrust device files
	if (create_thruster_devices())
		goto init_failure;

	for (int i = 0; i < THRUSTER_COUNT; i++) {
		if (set_thrust(i, 0))
		    goto init_failure;
	}

	printk(KERN_DEBUG "thruster DAC initialization successful\n");
	_thruster_initialized = 1;
	return 0;

init_failure:
	printk(KERN_ERR "failed to initialize thruster DAC\n");
	return 1;
}


void cleanup_thruster()
{
	if (!_thruster_initialized)
		return;

	remove_thruster_devices();

	for (int i = 0; i < THRUSTER_COUNT; i++) {
		set_thrust(i, 0);
	}

	_thruster_initialized = 0;
}



s32 current_thrust(u8 thruster_num)
{
	if (thruster_num >= THRUSTER_COUNT) {
		printk(KERN_ERR "thruster %i does not exist\n", thruster_num);
		return -1;
	}

	return _thrust_percents[thruster_num];
}


s8 set_thrust(u8 thruster_num, u16 thrust)
{
	if (thruster_num >= THRUSTER_COUNT) {
		printk(KERN_ERR "thruster %i does not exist\n", thruster_num);
		return -1;
	} else if (thrust > THRUST_RESOLUTION) {
		printk(KERN_ERR "thrust %i larger than maximum %i\n", thrust, \
				THRUST_RESOLUTION);
		return 1;
	}

	u16 max = 65536 - 1;
	u16 min = 65536 * (7 / 27);
	u16 rawThrust = (max - min) * thrust / THRUST_RESOLUTION + min;
	if (rawThrust > max || rawThrust < min) {
		printk(KERN_ERR "math error when calculating thruster thrust %i\
				\n", thrust);
		return 1;
	}
	if (thrust == 0)
		rawThrust = 0;

	s8 command = 0b0011 << 4;
	s8 outbuf[] = {command, (rawThrust & 0xff00) >> 8, rawThrust &0xff};
	if (ccard_lock_bus()) {
		printk(KERN_ERR "unable to lock i2c bus\n");
		return 1;
	} else if (i2c_master_send(thruster_dac(), outbuf, 3) < 3) {
		printk(KERN_ERR "setting thruster to thrust %i init_failed\n", \
				thrust);
		ccard_unlock_bus();
		return 1;
	}
	ccard_unlock_bus();

	return 0;
}



//
// sysfs section
//

static ssize_t read_thruster_percent(struct device *dev, \
				     struct device_attribute *attr, char *buf)
{
	s8 thrust_num = -1;
	for (int i = 0; i < THRUSTER_COUNT; i++) {
		if (dev == _thruster_devices[i]) {
			thrust_num = i;
			break;
		}
	}

	if (thrust_num < 0 || thrust_num >= THRUSTER_COUNT) {
		printk(KERN_DEBUG "invalid thruster number %i\n", thrust_num);
		return scnprintf(buf, 50, "thruster not recognized\n");
	}

	printk(KERN_DEBUG "reading thrust for thruster %i\n", thrust_num);
	return scnprintf(buf, 20, "%i%%\n", current_thrust(thrust_num) / \
						THRUST_RESOLUTION * 100);
}

static ssize_t write_thruster_percent(struct device *dev, \
				      struct device_attribute *attr, \
				      const char *buf, size_t count)
{
	s8 thrust_num = -1;
	for (int i = 0; i < THRUSTER_COUNT; i++) {
		if (dev == _thruster_devices[i]) {
			thrust_num = i;
			break;
		}
	}

	if (thrust_num < 0 || thrust_num >= THRUSTER_COUNT)
		printk(KERN_DEBUG "invalid thruster number %i\n", thrust_num);

	unsigned long value = 0;
	if (strict_strtoul(buf, 10, &value))
		printk(KERN_ERR "%s is an invalid thrust value\n", buf);

	if (set_thrust(thrust_num, value & 0xffff))
		printk(KERN_ERR "unable to set thrust to %lu\n", \
				value & 0xffff);

	printk(KERN_DEBUG "successfully set thrust for thruster %i to %lu\n", \
			thrust_num, value & 0xffff);

	return count;
}

static void ccard_release_thruster(struct device *dev)
{
	printk(KERN_DEBUG "releasing thruster device file\n");
}

static s8 create_thruster_devices()
{
	printk(KERN_DEBUG "creating thruster sysfs files\n");

	struct device *parent = &thruster_dac()->dev;

	struct class thruster_class = {
		.name = "thruster",
		.owner = THIS_MODULE,
		.dev_release = ccard_release_thruster,
	};
	_thruster_class = thruster_class;

	if (class_register(&_thruster_class)) {
		printk(KERN_ERR "failed to create thruster class\n");
		return 1;
	}

	if (alloc_chrdev_region(_dev_thruster, 0, THRUSTER_COUNT, "thruster")) {
		printk(KERN_ERR "couldn't create thruster dev_t's\n");
		return 1;
	}

	for (int i = 0; i < THRUSTER_COUNT; i++) {
		char name[15];
		scnprintf(name, 15, "thruster%i", i);

		_dev_thruster[i] = MKDEV(MAJOR(_dev_thruster[0]), \
					 MINOR(_dev_thruster[0]) + i);

		_thruster_devices[i] = device_create(&_thruster_class, \
						     parent, _dev_thruster[i], \
						     NULL, name);

		if (device_create_file(_thruster_devices[i], \
					&dev_attr_thrust)) {
			printk(KERN_ERR "error making sysfs files\n");
			return 1;
		}
	}

	printk(KERN_DEBUG "created thruster sysfs files\n");
	return 0;
}

static void remove_thruster_devices()
{
	for (int i = 0; i < THRUSTER_COUNT; i++) {
		device_remove_file(_thruster_devices[i], &dev_attr_thrust);
		device_destroy(&_thruster_class, _dev_thruster[i]);
	}

	unregister_chrdev_region(_dev_thruster[0], THRUSTER_COUNT);

	class_unregister(&_thruster_class);
}









