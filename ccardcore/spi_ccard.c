// implementation of the i2c driver
// by Mark Hill

#include<linux/kernel.h>
#include<linux/spi/spi.h>
#include<linux/device.h>
#include<linux/types.h>

#include "ccard.h"

// function definition for the spi_driver struct
static int ccard_spi_probe(struct spi_device *device);
static int ccard_spi_remove(struct spi_device *device);


#define _thruster_cs 2
#define _thruster_mode 1
#define _thruster_speed 1
#define _thruster_bus 1
#define _spi_drvr_name "ccard_spi_drvr"

// struct containing spi driver info for the c card
static struct spi_driver _spi_drvr = {
	.driver = {
		.name = _spi_drvr_name,
		.owner = THIS_MODULE,
	},
	.probe = ccard_spi_probe,
	.remove = ccard_spi_remove,
};

struct spi_board_info _spi_board_info[] = {
	{
		.modalias = _spi_drvr_name,
		.max_speed_hz = _thruster_speed,
		.bus_num = _thruster_bus,
		.chip_select = _thruster_cs,
		.mode = _thruster_mode,
	},
};

static struct spi_device *_thruster;

// initializes the spi driver for the c card
s8 ccard_init_spi()
{
	// for a loadable module, registering the devices must be
	//   done with spi_new_device
	spi_new_device(spi_busnum_to_master(_thruster_bus), _spi_board_info);
	// for a builtin module use spi_register_board_info
	//_thruster = spi_register_board_info(_spi_board_info, 1);
	// initialize the driver
	if (spi_register_driver(&_spi_drvr)) {
		printk(KERN_ERR "failed to add spi driver to kernel\n");
		return 1;
	}

	printk(KERN_NOTICE "successfully added spi driver to kernel\n");

	return 0;
}

// cleans up the spi driver and removes it from the runtime
void ccard_cleanup_spi()
{
	printk(KERN_NOTICE "removing spi driver from kernel\n");
	if (_thruster != NULL)
		spi_unregister_device(_thruster);
	spi_unregister_driver(&_spi_drvr);
}

// probe function called by the kernel when a matching spi_device is found
static int ccard_spi_probe(struct spi_device *device)
{
	if (device->chip_select == _thruster_cs) {
		printk(KERN_NOTICE "found thruster\n");
		_thruster = device;
		return init_thruster();
	} else {
		printk(KERN_ERR "found unknown spi slave at chip select %i", \
				device->chip_select);
		return 1;
	}
}

// remove function called by the kernel when the spi_device must be removed
static int ccard_spi_remove(struct spi_device *device)
{
	if (device->chip_select == _thruster_cs) {
		printk(KERN_NOTICE "kernel wants to remove thruster device\n");
		cleanup_thruster();
		_thruster = NULL;
	} else {
		printk(KERN_ERR "anyone know why the kernel wants to remove \
		       spi slave at chip select %i and asked the c card driver \
		       to take of it?\n", device->chip_select);
		return 1;
	}
	return 0;
}


// returns the spi_device struct for the thruster
struct spi_device *thruster(void)
{
	return _thruster;
}






