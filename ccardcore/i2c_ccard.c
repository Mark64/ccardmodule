// implementation of the i2c driver
// by Mark Hill

#include<linux/kernel.h>
#include<linux/i2c.h>
#include<linux/types.h>

#include "ccard.h"

// function definition for the i2c_driver struct
static int ccard_i2c_probe(struct i2c_client *client, \
			   const struct i2c_device_id *id);
static int ccard_i2c_remove(struct i2c_client *client);


#define _dsa_addr 0x38
#define _mt_addr 0x38
#define _thruster_dac_addr 0x0f
#define _i2c_bus 1

// holds the board info to pass to the i2c subsystem
static struct i2c_board_info ccard_board_info[] = {
	{I2C_BOARD_INFO("ccard_dsa", _dsa_addr),},
	{I2C_BOARD_INFO("ccard_mt", _mt_addr),},
	{I2C_BOARD_INFO("ccard_thruster_dac", _thruster_dac_addr),},
};

// array containing i2c board ids for use with the i2c subsystem detection
//   mechanism
#define _dsa_id 512
#define _mt_id 1024
#define _dac_id 768
static struct i2c_device_id ccard_i2c_ids[] = {
	{"ccard_dsa", _dsa_id},
	{"ccard_mt", _mt_id},
	{"ccard_thruster_dac", _dac_id},
};

// struct containing i2c driver info for the c card
static struct i2c_driver _drvr = {
	.id = 64,
	.driver = {
		.name = "ccard_i2c_drvr",
		.owner = THIS_MODULE,
	},
	.probe = ccard_i2c_probe,
	.remove = ccard_i2c_remove,
	.id_table = ccard_i2c_ids,
};

static struct i2c_client *_dsa;
static struct i2c_client *_mt;
static struct i2c_client *_thruster_dac;

static struct mutex *_ccard_i2c_lock;

// creates the controller devices for the corresponding
//   gpio expander chip
static inline void create_dsa_expdr_device(void);
static inline void create_mt_expdr_device(void);
static inline void create_thruster_dac_device(void);

// initializes the i2c driver for the c card
s8 ccard_init_i2c()
{
	// for a loadable module, registering the devices must be
	//   done with i2c_new_device
	// get the adapter for i2c bus 1
	struct i2c_adapter *a = i2c_get_adapter(_i2c_bus);
	_ccard_i2c_lock = &a->clist_lock;
	_dsa = i2c_new_device(a , &ccard_board_info[0]);
	_mt = i2c_new_device(a , &ccard_board_info[1]);
	_thruster_dac = i2c_new_device(a, &ccard_board_info[2]);
	// for a builtin module, register the i2c devices with the kernel
	//i2c_register_board_info(_i2c_bus, ccard_board_info,
	//			ARRAY_SIZE(ccard_board_info));
	// initialize the driver
	if (i2c_add_driver(&_drvr)) {
		printk(KERN_ERR "failed to add i2c driver to kernel\n");
		return 1;
	}

	printk(KERN_NOTICE "successfully added i2c driver to kernel\n");

	return 0;
}

// cleans up the i2c driver and removes it from the runtime
void ccard_cleanup_i2c()
{
	printk(KERN_NOTICE "removing i2c driver from kernel\n");
	if (_mt != NULL)
		i2c_unregister_device(_mt);
	if (_dsa != NULL)
		i2c_unregister_device(_dsa);
	if (_thruster_dac != NULL)
		i2c_unregister_device(_thruster_dac);
	i2c_del_driver(&_drvr);
}

// probe function called by the kernel when a matching i2c_client is found
static int ccard_i2c_probe(struct i2c_client *client, \
			   const struct i2c_device_id *id)
{
	if (client->addr == _dsa_addr) {
		printk(KERN_NOTICE "found dsa controller\n");
		_dsa = client;
		create_dsa_expdr_device();
		return init_dsa();
	} else if (client->addr == _mt_addr) {
		printk(KERN_NOTICE "found magnetorquer controller\n");
		_mt = client;
		create_mt_expdr_device();
		return init_mt();
	} else if (client->addr == _thruster_dac_addr) {
		printk(KERN_NOTICE "found thruster dac\n");
		_thruster_dac = client;
		create_thruster_dac_device();
		return init_thruster();
	} else {
		printk(KERN_ERR "found unknown i2c slave at address %x", \
				client->addr);
		return 1;
	}
}

// remove function called by the kernel when the i2c_client must be removed
static int ccard_i2c_remove(struct i2c_client *client)
{
	if (client->addr == _dsa_addr) {
		printk(KERN_NOTICE "kernel wants to remove dsa controller\n");
		cleanup_dsa();
		_dsa = NULL;
	} else if (client->addr == _mt_addr) {
		printk(KERN_NOTICE "kernel wants to remove magnetorquer \
				controller\n");
		cleanup_mt();
		_mt = NULL;
	} else if (client->addr == _thruster_dac_addr) {
		printk(KERN_NOTICE "kernel wants to remove thruster dac\n");
		cleanup_thruster();
		_thruster_dac = NULL;
	} else {
		printk(KERN_ERR "anyone know why the kernel wants to remove \
		       i2c slave at address %x and asked the c card driver \
		       to take of it?\n", client->addr);
		return 1;
	}
	return 0;
}


// locks the i2c bus
int ccard_lock_bus()
{
	return mutex_lock_interruptible(_ccard_i2c_lock);
}

// unlocks the i2c bus
void ccard_unlock_bus()
{
	mutex_unlock(_ccard_i2c_lock);
}


// returns the i2c_client struct for the magnetorquer GPIO expdr
struct i2c_client *mt_expdr()
{
	return _mt;
}

// returns the i2c_client struct for the dsa GPIO expdr
struct i2c_client *dsa_expdr()
{
	return _dsa;
}

// returns the i2c_client struct for the DAC controlling the thruster
struct i2c_client *thruster_dac()
{
	return _thruster_dac;
}



//
// sysfs section
//

static inline void name_i2c_client(struct i2c_client *client, \
				   const char name[])
{
	scnprintf(client->name, I2C_NAME_SIZE, name);
}

static inline void create_dsa_expdr_device()
{
	name_i2c_client(_dsa, "dsa_expdr");
}

static inline void create_mt_expdr_device()
{
	name_i2c_client(_mt, "mt_expdr");
}

static inline void create_thruster_dac_device()
{
	name_i2c_client(_thruster_dac, "thruster_dac");
}




