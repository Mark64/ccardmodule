// implementation for the DSA (deployable solar array) system calls
//
// using the TCA9554A GPIO expander
// http://www.ti.com/lit/ds/symlink/tca9554.pdf - datasheet
//
// by Mark Hill
#include<linux/kernel.h>
#include<linux/semaphore.h>
#include<linux/kthread.h>
#include<linux/err.h>
#include<linux/sched.h>
#include<linux/slab.h>
#include<linux/delay.h>
#include<linux/time.h>
#include<linux/gpio.h>
#include<linux/i2c.h>
#include<linux/device.h>
#include<linux/sysfs.h>
#include<linux/string.h>
#include<linux/fs.h>

#include "ccard.h"

#define ccard_rel_dfl_timeout 12
#define ccard_dep_dfl_timeout 10
#define DSA_COUNT 2

// defines the pin location for each value, which corresponds to the
//   bit number on the device's registers
// the value at index 0 is the corresponding value for DSA 1, and
//   the value at each subsequent index 'n' is the corresponding value
//   for DSA 'n + 1'
// its a useless feature, because I don't think the cubesat will ever
//   have more than 1 DSA, but who knows what the EXA has planned
static const u8 _dsa_res_out[] = {0, 2};
static const u8 _dsa_dep_out[] = {1, 3};
static const u8 _dsa_res_in[] = {5, 7};
static const u8 _dsa_dep_in[] = {6, 8};

// flag indicating whether the DSA hardware has been properly
//   configured
// !0 = initialized, 0 = uninitialzed
static int _dsa_initialized = 0;

// these store the user configured timeout values used by the system for the
//   DSA operations
// they are set to the defined value from ccard.h
static u32 _userReleaseTimeout = ccard_rel_dfl_timeout;
static u32 _userDeployTimeout = ccard_dep_dfl_timeout;

// stores the desired values for the DSAs
static enum dsa_state _desiredDSAStates[DSA_COUNT];
// stores the current state of the DSAs as determined by reading
//   hardware state registers. see get_dsa_state(dsa)
static enum dsa_state _currentDSAStates[DSA_COUNT];

// this function is called when a discrepancy
//   in the desired and current DSA states is found for
//   DSA <dsa>
// this function determines the proper method of correcting the
//   discrepancy and passes that onto a separate thread if needed
// returns 0 on success, indicating that it will (or has) attempted
//   to correct the discrepancy
// if it fails for any reason, it will return -1
static int correct_dsa(u8 dsa);

// holds the class object
static struct class _dsa_class;
// holds the dsa device structs
static struct device *_dsa0;
static struct device *_dsa1;
// holds the device numbers
static dev_t _dev_dsa0;
static dev_t _dev_dsa1;
// callback function for attributes
static ssize_t read_dsa_state(struct device *dev, \
			      struct device_attribute *attr, \
			      char *buf);
static ssize_t read_target_dsa_state(struct device *dev, \
				     struct device_attribute *attr, \
				     char *buf);
static ssize_t write_target_dsa_state(struct device *dev, \
				      struct device_attribute *attr, \
				      const char *buf, size_t count);

// stores the device attributes
static DEVICE_ATTR(current_state, S_IRUSR, read_dsa_state, \
		   write_target_dsa_state);
static DEVICE_ATTR(desired_state, S_IRUSR | S_IWUSR, read_target_dsa_state, \
		   write_target_dsa_state);
// callback function for the dsa attributes
static ssize_t read_dsa_release_timeout(struct class *class, char *buf);
static ssize_t write_dsa_release_timeout(struct class *class, const char *buf, \
					 size_t count);

static ssize_t read_dsa_deploy_timeout(struct class *class, char *buf);

static ssize_t write_dsa_deploy_timeout(struct class *class, const char *buf, \
					size_t count);
// stores the class attributes
static CLASS_ATTR(release_timeout, S_IRUSR | S_IWUSR, \
		  read_dsa_release_timeout, write_dsa_release_timeout);
static CLASS_ATTR(deploy_timeout, S_IRUSR | S_IWUSR, \
		  read_dsa_deploy_timeout, write_dsa_deploy_timeout);

// creates the sysfs interface for the dsas
static inline void create_dsa_devices(void);
// removes the sysfs objects
static inline void remove_dsa_devices(void);




// sets the initial state of the GPIO expander and initializes configuration values
s8 init_dsa()
{
	printk(KERN_DEBUG "initializing dsa hardware\n");
	// skip initialization if it has already been done
	if (_dsa_initialized)
		return 0;

	// make sure the 3V3 power supply is off
	set_dsa_pwr(0, 1);

	// initializes the pins as inputs or outputs, respectively
	// setting a pin as an input requires a write of 1, while setting
	//   it as an output requires a write of 0
	s8 cfgval = 0xf0;
	s8 cfgreg = 0x03;
	s8 cfgbuf[] = {cfgreg, cfgval};
	s8 outval = 0x00;
	s8 outreg = 0x01;
	s8 outbuf[] = {outreg, outval};
	if (ccard_lock_bus()) {
		printk(KERN_ERR "unable to lock i2c bus\n");
		return 1;
	} else if (i2c_master_send(dsa_expdr(), cfgbuf, 2) < 2 || \
	    i2c_master_send(dsa_expdr(), outbuf, 2) < 2) {
		printk(KERN_ERR "failed to configure DSA GPIO expander\n");
		ccard_unlock_bus();
		return -1;
	}
	ccard_unlock_bus();

	create_dsa_devices();

	printk(KERN_NOTICE "dsa initialization successful\n");

	_dsa_initialized = 1;
	return 0;
}

// cleans up and powers off the DSA hardware
void cleanup_dsa()
{
	if (!_dsa_initialized)
		return;
	// since the update threads will exit when they detect a
	//   change in the desired state, the desired state is set to to
	//   stowed so that they can be closed
	remove_dsa_devices();

	for (int i = 0; i < DSA_COUNT; i++) {
		set_dsa_state(i, stowed);
	}

	// now wait for the threads to close
	msleep(1000);

	// turn off the outputs
	s8 offreg = 0x01;
	s8 offval = 0x00;
	s8 offbuf[] = {offreg, offval};
	if (ccard_lock_bus())
		printk(KERN_ERR "unable to lock i2c bus\n");
	i2c_master_send(dsa_expdr(), offbuf, 2);
	ccard_unlock_bus();

	set_dsa_pwr(0, 1);

	// set the isInitialized flag off
	_dsa_initialized = 0;
}


// since the only 4 bits describe each dsa, but all registers have to be read,
//   it is more efficient to update them all at the same time
static inline void update_dsa_state(void)
{
	// the TCA9554A represents the current state of its
	//   input registers as one byte, and the value of its
	//   output registers as a second byte
	// both are needed to determine the state of the software
	s8 inreg[] = {0x00};
	s8 outreg[] = {0x01};
	s8 inbuf[1];
	s8 outbuf[1];
	// the first value in this array is the input value, second
	//   value is the output value
	u8 gpioState[2] = {};

	if (ccard_lock_bus()) {
		printk(KERN_ERR "unable to lock i2c bus\n");
	} else if (i2c_master_send(dsa_expdr(), inreg, 1) < 1 || \
	    i2c_master_recv(dsa_expdr(), inbuf, 1) < 1 || \
	    i2c_master_send(dsa_expdr(), outreg, 1) < 1 || \
	    i2c_master_recv(dsa_expdr(), outbuf, 1) < 1) {
		printk(KERN_ERR "couldn't read dsa pins in update_dsa_state\n");
	} else {
		gpioState[0] = inbuf[0];
		gpioState[1] = outbuf[0];
	}
	ccard_unlock_bus();

	for (int i = 0; i < DSA_COUNT; i++) {
		// uses the bit number (big endian format) to shift the bits
		//   and determine the corresponding value
		u16 in_res_val = (gpioState[0] >> \
				  _dsa_res_in[i]) & 0b0001;
		u16 in_dep_val = (gpioState[0] >> \
				  _dsa_dep_in[i]) & 0b0001;
		u16 out_res_val = (gpioState[1] >> \
				   _dsa_res_out[i]) & 0b0001;
		u16 out_dep_val = (gpioState[1] >> \
				   _dsa_dep_out[i]) & 0b0001;

		// because its technically possible to run a deploy operation without
		//   having first released the DSAs, the raw enumvalue for deploying
		//   ignores the input release value, so when a deploy operation is
		//   running, the input release value must be set to zero regardless
		//   of its true value
		if (out_dep_val == 1)
			in_res_val = 0;

		// now, thanks to bitwise operations and a clever setup of the DSAState enum
		//   raw values, creating the proper state value is easy
		// to understand why the bits are shifted by the amount here, see
		//   the raw enum values in cleanccard.h
		enum dsa_state state = (in_res_val << 1) + \
				       (in_dep_val << 1) + \
				       (in_dep_val << 3) + \
				       out_res_val + (out_dep_val << 2);

		// now set the corresponding current state value
		_currentDSAStates[i] = state;
	}
}

// gets the current dsa state after calling update_dsa_state
enum dsa_state get_dsa_state(u8 dsa)
{
	if (!_dsa_initialized)
		return stowed;

	// check to make sure the dsa isn't out of bounds
	if (dsa >= DSA_COUNT) {
		printk(KERN_ERR "dsa %i does not exist\n", dsa);
		return -1;
	}

	update_dsa_state();

	return _currentDSAStates[dsa];
}

s8 set_dsa_state(u8 dsa, enum dsa_state desiredState)
{
	if (!_dsa_initialized)
		return -1;
	// first, we need to check the input to ensure it makes sense
	//   and isn't going to put the system in an invalid state
	// lets get the current state
	// and if you're wondering why this isn't in the lock,
	//   its because the lock is handled by get_dsa_state
	if (dsa >= DSA_COUNT) {
		printk(KERN_ERR "dsa %i does not exist\n", dsa);
		return -1;
	}

	enum dsa_state currentState = get_dsa_state(dsa);
	// stores the return value, which is determined by whether or
	//   not the input makes sense
	int returnValue = -1;
	if (desiredState != released && desiredState != deployed) {
		printk(KERN_ERR "impossible desired state in set_dsa_state\n");
		return returnValue;
	}
	if (desiredState == deployed && currentState == stowed) {
		printk(KERN_ERR "performing dply op while dsa %i is stowed\n", \
				dsa);
		returnValue = 3;
	}

	// write the desired state to the appropriate array index
	_desiredDSAStates[dsa] = desiredState;
	correct_dsa(dsa);

	return returnValue;
}


static s8 shutoff_dsa(u8 dsa)
{
	// flag indicating failure condition
	s8 failure = 0;

	s8 valreg[] = {0x01};
	// read the existing value so that only the bit for this operation
	//   is changed
	s8 valbuf[1];
	if (ccard_lock_bus()) {
		printk(KERN_ERR "unable to lock i2c bus\n");
		return 1;
	}
	if (i2c_master_send(dsa_expdr(), valreg, 1) > 1 || \
		 i2c_master_recv(dsa_expdr(), valbuf, 1) > 1) {
		// mask used to set proper bits off
		u8 mask = (0x01 << _dsa_res_out[dsa]) + \
			  (0x01 << _dsa_dep_out[dsa]);
		// now change the proper bit to enable release
		valbuf[0] = (s8)((u8)valbuf[0] & mask);
		// write the changed value
		s8 writebuf[] = {valreg[0], valbuf[0]};
		failure = i2c_master_send(dsa_expdr(), writebuf, 2) == 2;
	} else {
		printk(KERN_EMERG "failed to shut off power to dsa %i\n", dsa);
		printk(KERN_EMERG "disabling 3v3 to protect c card\n");
	}
	ccard_unlock_bus();

	return failure;
}


// for op, 0 = release, 1 = deploy
// inlined to allow compiler to optimize away unnessecary variables
static inline int exec_dsa_op(u8 dsa, u8 op)
{
	char *opstr = (op == 0) ? "release" : "deploy";
	// log that the thread was started
	printk(KERN_NOTICE "%s thread successfully created\n", opstr);

	const s8 timeout = (op == 0) ? _userReleaseTimeout : _userDeployTimeout;
	const enum dsa_state desired = (op == 0) ? released : deployed;
	const u8 *pins = (op == 0) ? \
			 _dsa_res_out : _dsa_dep_out;

	// get the start time, which will be used to determine if the operation has timed out
	struct timespec start = current_kernel_time();

	// turn on 3V3 supply
	set_dsa_pwr(1, 0);

	// turn on the GPIO on the expander which will enable power to
	//   the proper switch for the operation
	s8 valreg[] = {0x01};
	// read the existing value so that only the bit for this operation
	//   is changed
	s8 valbuf[1];
	if (ccard_lock_bus()) {
		printk(KERN_ERR "unable to lock i2c bus\n");
		return 1;
	} else if (i2c_master_send(dsa_expdr(), valreg, 1) < 1 || \
	    i2c_master_recv(dsa_expdr(), valbuf, 1) < 1) {
		printk(KERN_ERR "error reading dsa state for dsa %i", dsa);
		ccard_unlock_bus();
		return 1;
	}

	// now change the proper bit to enable release
	u8 mask = (0x01 << pins[dsa]);
	valbuf[0] = (s8)((u8)valbuf[0] | mask);
	// write the changed value
	s8 writebuf[] = {valreg[0], valbuf[0]};
	if (i2c_master_send(dsa_expdr(), writebuf, 2) < 2) {
		printk(KERN_ERR "dsa %i %s operation failed\n", dsa, opstr);
		ccard_unlock_bus();
		return 1;
	}
	ccard_unlock_bus();

	s8 retval = 0;
	// loop that runs until the operaton has completed
	while (1) {
		// check the current time
		struct timespec currentTime = current_kernel_time();
		if (currentTime.tv_sec - start.tv_sec > timeout) {
			printk(KERN_NOTICE "dsa %i %s operation timed out", \
					dsa, opstr);
			retval = 1;
			break;
		}
		// check the current state
		if (_currentDSAStates[dsa] == desired) {
			printk(KERN_NOTICE "dsa %i %s operation successful", \
					dsa, opstr);
			retval = 0;
			break;
		}
		// check if the user no longer wants a deploy operation to occur
		if (_desiredDSAStates[dsa] != desired) {
			printk(KERN_NOTICE "dsa %i %s operation terminated", \
					dsa, opstr);
			retval = 1;
			break;
		}

		// no need to hog resources
		msleep(200);
	}

	// turn the switch off
	set_dsa_pwr(0, shutoff_dsa(dsa));

	return retval;
}

// this function is run in its own thread and handles releasing the specified DSA
// it will automatically exit upon success, timeout, or if the desiredState changes to something
//   other than released, which usually means a termination request
// expects a pointer to an int containing the DSA number to release
// frees the memory associated with the pointer after recording the value
// returns the result of the operation
static int res_dsa(void *dsa)
{
	// store the dsa and free the pointer
	u8 d = *(u8 *)dsa;
	kfree(dsa);

	s8 flag = exec_dsa_op(d, 0);

	if (flag)
		set_dsa_state(d, stowed);

	return flag;
}

// this function is run in its own thread and handles deploying the specified DSA
// it will automatically exit upon success, timeout, or if the desiredState changes to something
//   other than released, which usually means a termination request
// expects a pointer to an int containing the DSA number to deploy
// frees the memory associated with the pointer after recording the value
// returns the result of the operation
static int dep_dsa(void *dsa)
{
	// store the dsa and free the pointer
	u8 d = *(u8 *)dsa;
	kfree(dsa);

	s8 flag = exec_dsa_op(d, 1);

	if (flag)
		set_dsa_state(d, stowed);

	return flag;
}

// this is called by dsa_monitor when a discrepancy is found
static int correct_dsa(u8 dsa)
{
	// check that dsa is in bounds
	if (dsa >= DSA_COUNT) {
		printk(KERN_ERR "invalid dsa in correct_dsa\n");
		return -1;
	}

	// determines the discrepancy and schedules an operation if needed
	enum dsa_state cur = _currentDSAStates[dsa];
	enum dsa_state des = _desiredDSAStates[dsa];
	if (cur == releasing || cur == deploying) {
		printk(KERN_ERR "possible duplicate call to correct_dsa. exiting\n");
		return 0;

	} else if (des == cur) {
		printk(KERN_DEBUG "dsa %i needs no correction\n", dsa);
		return 0;
	} else if (1 || (des == released && cur == stowed)) {
		printk(KERN_DEBUG "scheduling release operation\n");

		// creates a thread to release the specified DSA and manage timeouts
		u8 *d = (u8 *)kmalloc(sizeof(u8), GFP_KERNEL);
		*d = dsa;
		struct task_struct *res = kthread_run(&res_dsa, d, \
						      "res_dsa%i", *d);
		if (IS_ERR(res)) {
			printk(KERN_ERR "failed to create res_dsa thread\n");
			return 1;
		} else {
			return 0;
		}
	} else if (des == deployed && (cur == released || cur == stowed)) {
		printk(KERN_DEBUG "scheduling deploy operation\n");

		// creates a thread to deploy the specified DSA and manage timeouts
		u8 *d = (u8 *)kmalloc(sizeof(u8), GFP_KERNEL);
		*d = dsa;
		struct task_struct *dply = kthread_run(&dep_dsa, d, \
						       "dply_dsa%i", *d);
		if (IS_ERR(dply)) {
			printk(KERN_ERR "failed to create dep_dsa thread\n");
			return 1;
		}
		else {
			return 0;
		}
	} else if (des == stowed && (cur != released || cur != deployed)) {
		printk(KERN_ERR "power cut to DSA %i based on desired state = stowed\n", dsa);
		shutoff_dsa(dsa);
		return 0;
	}

	return 1;
}


//
// sysfs section
//

// all strings here are treated the same during a write
static char *possible_stowed_str[10] = {
	"stowed\n", "stow\n", "off\n", "stop\n", "dont\n", "undo\n",
	"actually no\n", "he called us first\n", "STOP\n", "cancel\n"
};
static char *possible_rlsed_str[10] = {
	"released\n", "release\n", "drop\n", "pull the pin\n", "prepare\n",
	"heat up\n", "get ready\n", "relinquish\n", "Ronnie Nader\n", "unlatch\n"
};
static char *possible_dplyed_str[10] = {
	"deployed\n", "deploy\n", "launch\n", "expand\n", "final position\n",
	"fold out\n", "reveal\n", "shine\n", "collect light\n", "finish\n"
};

static char *possible_dplying_str[1] = {"deploying\n"};
static char *possible_rlsing_str[1] = {"releasing\n"};


static ssize_t read_dsa_state(struct device *dev, \
				 struct device_attribute *attr, char *buf)
{
	printk("reading dsa state\n");

	s8 dsa = (dev == _dsa0) ? 0 : 1;
	enum dsa_state state = get_dsa_state(dsa);
	char *state_str;

	switch (state) {
	case stowed:
		state_str = possible_stowed_str[0];
		break;
	case releasing:
		state_str = possible_rlsing_str[0];
		break;
	case released:
		state_str = possible_rlsed_str[0];
		break;
	case deploying:
		state_str = possible_dplying_str[0];
		break;
	case deployed:
		state_str = possible_dplyed_str[0];
		break;
	default:
		state_str = "invalid internal state\n";
		break;
	}

	// remove the trailing return char
	char str[30];
	scnprintf(str, strlen(state_str), "%s", state_str);

	return scnprintf(buf, 92, "[%s] stowed releasing released deploying deployed\n", str);
}


static ssize_t read_target_dsa_state(struct device *dev, \
					struct device_attribute *attr, \
					char *buf)
{
	printk("reading target dsa state\n");

	s8 dsa = (dev == _dsa0) ? 0 : 1;
	enum dsa_state state = _desiredDSAStates[dsa];
	char *state_str;

	switch (state) {
	case stowed:
		state_str = possible_stowed_str[1];
		break;
	case released:
		state_str = possible_rlsed_str[1];
		break;
	case deployed:
		state_str = possible_dplyed_str[1];
		break;
	default:
		state_str = "invalid internal state\n";
		break;
	}

	// remove the trailing return char
	char str[30];
	scnprintf(str, strlen(state_str), "%s", state_str);

	return scnprintf(buf, 50, "[%s] stow release deploy\n", str);
}

static ssize_t write_target_dsa_state(struct device *dev, \
					struct device_attribute *attr, \
					const char *buf, size_t count)
{
	printk(KERN_DEBUG "asked to write %s\n", buf);

	if (strcmp(buf, "he called us first\n") == 0 || \
	    strcmp(buf, "Ronnie Nader\n") == 0)
		printk(KERN_NOTICE "I'm not your girlfriend\n");

	s8 dsa = (dev == _dsa0) ? 0 : 1;
	enum dsa_state state = stowed;

	char **arrays[3] = {
		possible_stowed_str, possible_rlsed_str, possible_dplyed_str
	};

	for (int i = 1; i < 3; i++) {
		char **array = arrays[i];
		for (int j = 0; j < 10; j++) {
			if (strcmp(array[j], buf) == 0) {
				state = (i == 0) ? 0 : (i == 1) ? 2 : 10;
				break;
			}
		}
	}

	correct_dsa(dsa);
	set_dsa_state(dsa, state);
	printk(KERN_DEBUG "setting dsa %i to state %i\n", dsa, state);

	return count;

}

static __always_inline ssize_t read_timeout(u32 timeout, char *buf)
{
	printk(KERN_DEBUG "reading timeout\n");
	return scnprintf(buf, 20, "%i seconds\n", timeout);
}

static __always_inline ssize_t write_timeout(u32 *timeout, const char *buf, \
					     size_t count)
{
	printk("writing %s to timeout with value %i\n", buf, *timeout);
	unsigned long value = 0;
	if (strict_strtoul(buf, 10, &value))
		printk(KERN_WARNING "%s is an invalid timeout value\n", buf);
	else
		*timeout = (u32)value;

	if (*timeout == value)
		printk("wrote %lu to timeout", value);

	return count;
}

static ssize_t read_dsa_release_timeout(struct class *class, char *buf)
{
	return read_timeout(_userReleaseTimeout, buf);
}

static ssize_t write_dsa_release_timeout(struct class *class, const char *buf, \
					 size_t count)
{
	return write_timeout(&_userReleaseTimeout, buf, count);
}

static ssize_t read_dsa_deploy_timeout(struct class *class, char *buf)
{
	return read_timeout(_userDeployTimeout, buf);
}

static ssize_t write_dsa_deploy_timeout(struct class *class, const char *buf, \
					size_t count)
{
	return write_timeout(&_userDeployTimeout, buf, count);
}

static void ccard_release_dsa(struct device *dev)
{
	printk(KERN_NOTICE "releasing dsa device file triggers cleanup\n");
}

static inline void create_dsa_devices()
{
	printk(KERN_DEBUG "creating dsa sysfs files\n");

	struct device *parent = &dsa_expdr()->dev;

	struct class dsa_class = {
		.name = "dsa",
		.owner = THIS_MODULE,
		.dev_release = ccard_release_dsa,
	};
	_dsa_class = dsa_class;

	if (class_register(&_dsa_class)) {
		printk(KERN_ERR "couldn't create dsa class\n");
		return;
	}

	if (class_create_file(&_dsa_class, &class_attr_release_timeout) || \
	    class_create_file(&_dsa_class, &class_attr_deploy_timeout)) {
		printk(KERN_ERR "couldn't create dsa class attributes\n");
		return;
	}

	if (alloc_chrdev_region(&_dev_dsa0, 0, 2, "dsa")) {
		printk(KERN_ERR "couldn't create dsa device numbers\n");
		return;
	}
	_dev_dsa1 = MKDEV(MAJOR(_dev_dsa0), MINOR(_dev_dsa0) + 1);

	_dsa0 = device_create(&_dsa_class, parent, _dev_dsa0, NULL, "dsa0");
	_dsa1 = device_create(&_dsa_class, parent, _dev_dsa1, NULL, "dsa1");

	if (device_create_file(_dsa0, &dev_attr_current_state) || \
	    device_create_file(_dsa0, &dev_attr_desired_state)) {
		printk(KERN_ERR "couldn't create dsa0 device files\n");
		return;
	}
	if (device_create_file(_dsa1, &dev_attr_current_state) || \
	    device_create_file(_dsa1, &dev_attr_desired_state)) {
		printk(KERN_ERR "couldn't create dsa1 device files\n");
		return;
	}


	printk(KERN_DEBUG "created sysfs dsa files\n");
}

static inline void remove_dsa_devices()
{
	device_remove_file(_dsa0, &dev_attr_current_state);
	device_remove_file(_dsa0, &dev_attr_desired_state);
	device_destroy(&_dsa_class, _dev_dsa0);

	device_remove_file(_dsa1, &dev_attr_current_state);
	device_remove_file(_dsa1, &dev_attr_desired_state);
	device_destroy(&_dsa_class, _dev_dsa1);

	unregister_chrdev_region(_dev_dsa0, 2);

	class_unregister(&_dsa_class);
}












