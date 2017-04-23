// implemtation for the magnetorquer functions
// http://rohmfs.rohm.com/en/products/databook/datasheet/ic/motor/stepper/bd63510aefv-e.pdf - datasheet
//
// by Mark Hill
#include<linux/kernel.h>
#include<linux/delay.h>
#include<linux/types.h>
#include<linux/i2c.h>
#include<linux/device.h>
#include<linux/sysfs.h>
#include<linux/string.h>

#include "ccard.h"


// defines the number of magnetorquers connected to the ccard
#define MT_COUNT 3

// basically its like this [forwardPinMT1, forwardPinMT2, ... forwardPinMTn]
static const u8 _forwardBits[] = {0, 2, 4};
static const u8 _reverseBits[] = {1, 3, 5};

// flag that indicates if the magnetorquer hardware has been
//   initialized properly
// 1 == initialized, 0 = uninitialized
static s8 _mt_initialized = 0;

// function to create the device structs fro the magnetorquers and
//   add them to the sysfs
static inline void create_mt_devices(void);
// removes the magnetorquer devices from sysfs
static inline void remove_mt_devices(void);

// definitions for the magnetorquer sysfs callbacks
static ssize_t read_mt_state(struct device *dev, \
				struct device_attribute *attr, char *buf);
static ssize_t write_mt_state(struct device *dev, \
				 struct device_attribute *attr, \
				 const char *buf, size_t count);

// stores the device structs
static struct device _mt_devices[MT_COUNT];
// device attributes for the magnetorquers
static DEVICE_ATTR(state, S_IRUSR | S_IWUSR, read_mt_state, \
		   write_mt_state);



// sets magnetorquer hardware into a default state and prepares
//   for subsequent state changes
s8 init_mt() {
	// checks if the hardware has already been initialized
	if (_mt_initialized)
		return 0;

	create_mt_devices();
	return 0;

	// configure all pins as outputs
	s8 cfgreg = 0x03;
	s8 cfgval = 0x00;
	const char buf[] = {cfgreg, cfgval};
	if (ccard_lock_bus()) {
		printk(KERN_ERR "unable to lock i2c bus\n");
		return 1;
	}
	s8 failure = i2c_master_send(mt_expdr(), buf, 2) < 2;
	// write all off to the i2c device
	s8 outreg = 0x01;
	s8 outval = 0x00;
	const char buf2[] = {outreg, outval};
	failure |= i2c_master_send(mt_expdr(), buf2, 2) < 2;
	ccard_unlock_bus();

	if (failure) {
		printk(KERN_ERR "failed to configure magnetorquer GPIO expander\n");
		return -1;
	}

	create_mt_devices();

	printk(KERN_NOTICE "magnetorquer initialization successful\n");
	// initializaton was successful
	_mt_initialized = 1;
	return 0;
}

// cleans up and powers off the magnetorquer hardware
void cleanup_mt() {
	if (!_mt_initialized)
		return;

	remove_mt_devices();
	return;

	// allows the magnetic field to be discharged before
	//   shutting the hardware off
	for (s8 i = 0; i < MT_COUNT; i++) {
		set_mt_state(i, off);
	}

	// resets the isInitialized flag
	_mt_initialized = 0;
}

// retrieves the state of magnetorquer <mt_num>
enum mt_state get_mt_state(u8 mt_num) {
	if (!_mt_initialized)
		return off;
	// read the current value from the GPIO expander
	u8 valbuf[1];
	s8 valregbuf[] = {0x01};
	if (ccard_lock_bus()) {
		printk(KERN_ERR "unable to lock i2c bus\n");
		return 1;
	} else if (i2c_master_send(mt_expdr(), valregbuf, 1) < 1 || \
	    i2c_master_recv(mt_expdr(), valbuf, 1) < 1) {
		printk(KERN_ERR "error reading magnetorquer expander\n");
		ccard_unlock_bus();
		return off;
	}
	ccard_unlock_bus();

	u8 value = (u8)valbuf[0];
	// flags that indicate the state of their corresponding
	//   direction, where 1 = on and 0 = off
	u8 forwardOn = (value >> _forwardBits[mt_num]) & 0x01;
	u8 reverseOn = (value >> _reverseBits[mt_num]) & 0x01;

	// the state is actually a combination of the two bits, with the
	//   reverse bit being the high bit
	return (reverseOn << 1) | forwardOn;
}

// sets the state of magnetorquer <mt_num> to the
//   desired state, after entering the transition state if
//   needed for a brief period of time
// returns 0 if successful and -1 if not successful
s8 set_mt_state(u8 mt_num, enum mt_state desired_state) {
	if (!_mt_initialized)
		return 1;
	// get the current state first to determine whether a transition state
	//   is needed to prevent large back emf, and to determine if a
	//   state change is even necessary
	enum mt_state currentState = get_mt_state(mt_num);
	// determine if state change is needed
	if (currentState == desired_state) {
		printk(KERN_DEBUG "cur state already equals desired state\n");
		return 0;
	}

	// i2c operation values
	if (ccard_lock_bus()) {
		printk(KERN_ERR "unable to lock i2c bus\n");
		return 1;
	}
	s8 outregbuf[] = {0x01};
	s8 valbuf[1];
	if (i2c_master_send(mt_expdr(), outregbuf, 1) < 0 || \
	    i2c_master_recv(mt_expdr(), valbuf, 1) < 0) {
		printk(KERN_ERR "error reading mt state for mt %i\n", mt_num);
		ccard_unlock_bus();
		return 1;
	}
	ccard_unlock_bus();

	u8 value = (s8)valbuf[0];
	// enter transition state if needed
	if (currentState != off) {
		printk(KERN_NOTICE "enabling brake mode for mt %i\n", mt_num);

		// mask used for setting the proper bits
		u8 mask = (0x01 << _forwardBits[mt_num]) | \
			  (0x01 << _reverseBits[mt_num]);

		// modify the correct values with bitwise operations
		value |= mask;

		s8 outbuf[] = {outregbuf[0], (s8)value};
		if (ccard_lock_bus()) {
			printk(KERN_ERR "unable to lock i2c bus\n");
			return 1;
		} else if (i2c_master_send(mt_expdr(), outbuf, 2) < 2) {
			printk(KERN_ERR "setting magnetorquer to state %i failed\n", \
					desired_state);
			ccard_unlock_bus();
			return 1;
		}
		ccard_unlock_bus();

		// give the magnetic field time to collapse
		msleep(100);

		// if the desired state is transitioning, then this function
		//   has already done it's work
		if (desired_state == transitioning)
			return 0;
	}

	// write the desired state
	printk(KERN_DEBUG "updating MT state\n");
	// set the proper bits using bitwise operations
	u8 forwardValue = (desired_state == forward) ? 1 : 0;
	u8 reverseValue = (desired_state == reverse) ? 1 : 0;
	value &= forwardValue << _forwardBits[mt_num];
	value &= reverseValue << _reverseBits[mt_num];
	// modify the correct values with bitwise operations
	s8 outbuf[] = {outregbuf[0], (s8)value};
	if (ccard_lock_bus()) {
		printk(KERN_ERR "unable to lock i2c bus\n");
		return 1;
	} else if (i2c_master_send(mt_expdr(), outbuf, 2) < 2) {
		printk(KERN_ERR "failed to set magnetorquer state\n");
		ccard_unlock_bus();
		return 1;
	}
	ccard_unlock_bus();

	return 0;
}



// if the user writes any of these strings to the 'state' file for
//   the magnetorquers, they will be treated the same
// only the first one is returned by read_mt_state, so these are undocumented
//   easter eggs for those who read the source code
static char *possible_off_str[16] = {
	"off\n", "OFF\n", "Off\n", "OfF\n", "oFf\n", "oFF\n", "ofF\n", "OFf\n",
	"stop\n", "STOP\n", "dont\n", "I honestly cant rn\n", "please stop\n",
	"end\n", "quit\n", "0\n"
};
static char *possible_fwd_str[16] = {
	"foward\n", "1\n", "FORWARD\n", "Forward\n", "fwd\n", "FWD\n", "progress\n",
	"life\n", "towards the goal\n", "ahead\n", "forwards\n", "straight\n",
	"positive\n", "up\n", "+\n", "->\n"
};
static char *possible_bwd_str[16] = {
	"backward\n", "back\n", "reverse\n", "bwd\n", "bkwd\n", "rvrs\n",
	"rear\n", "-1\n", "undo\n", "other way\n", "2\n", "negative\n",
	"-\n", "<-\n", "down\n", "BACK\n"
};
static char *possible_trans_str[16] = {
	"brake\n", "I like both equally\n", "lets be friends\n", "both\n",
	"3\n", "equality\n", "coast\n", "easy\n", "nothing\n",
	"the universe is large\n", "void\n", "null\n", "done\n", "equalize\n",
	"transitioning\n", "transition\n"
};


static ssize_t read_mt_state(struct device *dev, \
				struct device_attribute *attr, char *buf)
{
	printk(KERN_DEBUG "reading magnetorquer state\n");

	s8 mt_num = 0;
	for (int i = 1; i < MT_COUNT; i++) {
		if (dev == &_mt_devices[i]) {
			mt_num = i;
			break;
		}
	}

	enum mt_state cur_state = mt_num;//get_mt_state(mt_num);

	char **cur_str_list = possible_off_str;
	char **other_str_list1 = possible_fwd_str;
	char **other_str_list2 = possible_bwd_str;
	char **other_str_list3 = possible_trans_str;
	switch (cur_state) {
	case forward:
		cur_str_list = possible_fwd_str;
		other_str_list1 = possible_bwd_str;
		other_str_list2 = possible_trans_str;
		other_str_list3 = possible_off_str;
		break;
	case reverse:
		cur_str_list = possible_bwd_str;
		other_str_list2 = possible_trans_str;
		other_str_list3 = possible_off_str;
		break;
	case transitioning:
		cur_str_list = possible_trans_str;
		other_str_list2 = possible_off_str;
	default:
		break;
	}

	return scnprintf(buf, 100, "[%s] %s %s %s\n", cur_str_list[0], \
		  other_str_list1[0], other_str_list2[0], other_str_list3[0]);

}



static ssize_t write_mt_state(struct device *dev, \
				 struct device_attribute *attr, \
				 const char *buf, size_t count)
{
	printk(KERN_DEBUG "asked to write %s\n", buf);

	s8 mt_num = 0;
	for (int i = 1; i < MT_COUNT; i++) {
		if (dev == &_mt_devices[i]) {
			mt_num = i;
			break;
		}
	}

	enum mt_state state = off;
	char **arrays[4] = {
		possible_off_str, possible_fwd_str, possible_bwd_str,
		possible_trans_str};
	for (int i = 1; i < 4; i++) {
		char **array = arrays[i];
		for (int j = 0; j < 16; j++) {
			if (strcmp(array[j], buf) == 0) {
				state = i;
				break;
			}
		}
	}

	printk(KERN_DEBUG "setting mt %i to state %i\n", mt_num, state);
	//set_mt_state(mt_num, state);

	return count;
}


static void ccard_release_mt(struct device *dev)
{
	printk(KERN_DEBUG "releasing dsa device file\n");
}


static inline void create_mt_devices()
{
	struct device *parent = &dsa_expdr()->dev;
	for (int i = 0; i < MT_COUNT; i++) {
		char name[15];
		scnprintf(name, 15, "magnetorquer%i", i);
		struct device mt = {
			.parent = parent,
			.init_name = name,
			.driver = parent->driver,
			.release = ccard_release_mt,
		};
		_mt_devices[i] = mt;


		if (device_register(&_mt_devices[i]) || \
		    device_create_file(&_mt_devices[i], &dev_attr_state)) {
			printk(KERN_ERR "error creating sysfs files\n");
			return;
		}
	}

	printk(KERN_DEBUG "created magnetorquer sysfs files\n");
}

static inline void remove_mt_devices()
{
	for (int i = 0; i < MT_COUNT; i++) {
		device_remove_file(&_mt_devices[i], &dev_attr_state);
		device_unregister(&_mt_devices[i]);
	}
}








