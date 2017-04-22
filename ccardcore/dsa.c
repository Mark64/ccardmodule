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

#include "ccard.h"

#define DSA_COUNT 2

// defines the i2c address at which the GPIO expander is located
static const u8 _dsaControllerI2CAddress = 0x38;
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
// 0 = initialized
// -1 = uninitialzed
static int _dsa_initialized = -1;

// these store the user configured timeout values used by the system for the
//   DSA operations
// they are currently -1, and will be set to the proper value in initialization
static s8 _userReleaseTimeout = -1;
static s8 _userDeployTimeout = -1;

// stores the desired values for the DSAs
static enum dsa_state _desiredDSAStates[DSA_COUNT];
// stores the current state of the DSAs as determined by reading
//   hardware state registers. see get_dsa_state(dsa)
static enum dsa_state _currentDSAStates[DSA_COUNT];

// this is the main update function which ensures that the hardware values
//   for the dsa state reflect the settings in desiredDSAStates
static int dsa_monitor(void *junkInput);
// this function is called by dsa_monitor when a discrepancy
//   in the desired and current DSA states is found for
//   DSA <dsa>
// this function determines the proper method of correcting the
//   discrepancy and passes that onto a separate thread if needed
// returns 0 on success, indicating that it will (or has) attempted
//   to correct the discrepancy
// if it fails for any reason, it will return -1
static int correct_dsa(u8 dsa);


// stores the thread used to update the DSA states
static struct task_struct *_dsa_mntrd;


// sets the initial state of the GPIO expander and initializes configuration values
s8 init_dsa()
{
	// skip initialization if it has already been done
	if (_dsa_initialized == 0)
		return 0;

	// make sure the 3V3 power supply is off
	set_3v3_state(0, 0);

	// initializes the timeout values to the default included in the
	//   header file
	_userReleaseTimeout = ccard_rel_timeout;
	_userDeployTimeout = ccard_dep_timeout;

	// initializes the pins as inputs or outputs, respectively
	// setting a pin as an input requires a write of 1, while setting
	//   it as an output requires a write of 0
	s8 cfgval = 0xf0;
	s8 cfgreg = 0x03;
	s8 cfgbuf[] = {cfgreg, cfgval};
	s8 outval = 0x00;
	s8 outreg = 0x01;
	s8 outbuf[] = {outreg, outval};

	if (i2c_master_send(dsa_expdr(), cfgbuf, 2) || \
	    i2c_master_send(dsa_expdr(), outbuf, 2)) {
		printk(KERN_ERR "failed to configure DSA GPIO expander. \
				Is the C Card connected?\n");
		return -1;
	}


	// spawns the update thread, which ensures that the hardware state
	//   reflects the values in desiredDSAStates
	_dsa_mntrd = kthread_create(&dsa_monitor, 0, "dsa_mntr");
	if (IS_ERR(_dsa_mntrd)) {
		printk(KERN_ERR "failed to create dsa_mntr thread\n");
		return -1;
	}
	wake_up_process(_dsa_mntrd);

	printk(KERN_NOTICE "dsa initialization successful\n");

	_dsa_initialized = 0;
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
	for (int i = 0; i < DSA_COUNT; i++) {
		set_dsa_state(i, stowed);
	}

	// now wait for the threads to close
	msleep(1000);

	// kill the main updating thread
	kthread_stop(_dsa_mntrd);

	// turn off the outputs
	s8 offreg = 0x01;
	s8 offval = 0x00;
	s8 offbuf[] = {offreg, offval};
	i2c_master_send(dsa_expdr(), offbuf, 2);

	set_3v3_state(0, 0);

	// set the isInitialized flag off
	_dsa_initialized = -1;
}


// this sets the user-configurable timeout value, resulting in the software using that
//   value rather than the value included in the header
void set_usr_ccard_rel_timeout(u8 desiredTimeout)
{
	_userReleaseTimeout = desiredTimeout;
}

// this sets the user-configurable timeout value, resulting in the software using that
//   value rather than the value included in the header
void set_usr_ccard_dep_timeout(u8 desiredTimeout)
{
	_userDeployTimeout = desiredTimeout;
}


enum dsa_state get_dsa_state(u8 dsa)
{
	if (!_dsa_initialized)
		return stowed;

	// check to make sure the dsa isn't out of bounds
	if (dsa >= DSA_COUNT) {
		printk(KERN_ERR "dsa %i does not exist\n", dsa);
		return num_error;
	}

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
	if (desiredState != released || desiredState != deployed) {
		printk(KERN_ERR "impossible desired state in set_dsa_state\n");
		return returnValue;
	}
	if (desiredState == deployed && currentState == stowed) {
		printk(KERN_ERR "performing deploy operation while DSA %i is \
				currently stored. Possible error\n", dsa);
		returnValue = 3;
	}

	// write the desired state to the appropriate array index
	_desiredDSAStates[dsa] = desiredState;

	return returnValue;
}


struct task_struct *op_threads[DSA_COUNT * 2];
struct semaphore *op_act[DSA_COUNT * 2];
// it monitors the 'desiredDSAStates' array for changes as compared
//   to the current value obtained reported by the hardware, and if it finds
//   a discrepancy, it spawns a thread to handle the change
// if all the DSA's have been deployed, this function exits to
//   prevent wasted resources
static int dsa_monitor(void *junkInput)
{
	// this if statement is useless and should be removed, but
	//   in YouCompleteMe, the vim plugin I use to turn vim into
	//   a fully functional IDE, it flags the input as unused, so
	//   thats why this is here
	if (junkInput != 0) {
		printk(KERN_ERR "possible error in dsa updating thread\n");
	}

	// create semaphores for keeping track of thread activity
	for (int i = 0; i < DSA_COUNT * 2; i++) {
		op_act[i] = (struct semaphore *)\
			    kmalloc(sizeof(struct semaphore), GFP_KERNEL);
		sema_init(op_act[i], 0);
	}

	// this array holds a flag for each DSA indicating
	//   whether the state for each is acceptable (0) or
	//   if it requires correction (1)
	u8 dsaDiscrepancies[DSA_COUNT] = {0};

	// infinite loop designed to check values and ensure proper
	//   function and updating of the DSA state
	while (1) {
		// state update section
		//
		if (kthread_should_stop())
			break;
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


		if (i2c_master_send(dsa_expdr(), inreg, 1) || \
		    i2c_master_recv(dsa_expdr(), inbuf, 1) || \
		    i2c_master_send(dsa_expdr(), outreg, 1) || \
		    i2c_master_recv(dsa_expdr(), outbuf, 1)) {
			printk(KERN_ERR "failed to read dsa state. is c card \
			       unplugged?");
		} else {
			gpioState[0] = inbuf[0];
			gpioState[1] = outbuf[0];
		}

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

		// discrepancy check section
		//

		// check all the desired states against the current states and flag
		//   any discrepancies by setting the appropriate value in the
		//   dsaDiscrepancies array
		for (int i = 0; i < DSA_COUNT; i++) {
			// reset the value so that it is only set to 1 when a
			//   discrepancy exists
			dsaDiscrepancies[i] = 0;
			enum dsa_state des = _desiredDSAStates[i];
			enum dsa_state cur = _currentDSAStates[i];
			// check if the states differ
			// the states will differ while a correction is in progress,
			//   so this avoids a false flag (note: does not avoid 9/11)
			if (des != cur) {
				if (op_threads[i * 2] != NULL \
				    && down_trylock(op_act[i * 2])) {
					kthread_stop(op_threads[i * 2]);
					op_threads[i * 2] = NULL;
					dsaDiscrepancies[i] = 1;
				}
				if (op_threads[i * 2 + 1] != NULL && \
				    down_trylock(op_act[i * 2 + 1])) {
					kthread_stop(op_threads[i * 2 + 1]);
					op_threads[i * 2 + 1] = NULL;
					dsaDiscrepancies[i] = 1;
				}
				if (op_threads[i * 2] == NULL || \
				    op_threads[i * 2 + 1] == NULL)
					dsaDiscrepancies[i] = 1;
			}
		}

		// fix discrepancy section
		//

		// if no discrepancies are found, and all DSA's have been deployed,
		//   then this flags that condition so that the infinite loop,
		//   and the current thread, can be exited
		// 0 = all DSAs deployed, 1 = not all DSAs deployed
		u8 allDeployedFlag = 0;
		for (int i = 0; i < DSA_COUNT; i++) {
			if (dsaDiscrepancies[i] == 1) {
				correct_dsa(i);
			}
			if (_currentDSAStates[i] != deployed) {
				allDeployedFlag = 1;
			}
		}

		// if all DSAs are deployed, break from the infinite loops
		//   and end the thread
		if (allDeployedFlag == 0) {
			printk(KERN_NOTICE "exiting DSA update thread. all DSAs were deployed\n");
			break;
		}

		// only runs every 0.1 seconds (plus execution time), otherwise it would
		//   be very bad for performance
		// plus its not like you need super high resolution here
		msleep(100);
	}

	// returns only once all DSA's are deployed
	return 0;
}


static int shutoff_dsa(u8 dsa)
{
	// flag indicating failure condition
	s8 failure = 0;

	s8 valreg[] = {0x01};
	// read the existing value so that only the bit for this operation
	//   is changed
	s8 valbuf[1];
	if (i2c_master_send(dsa_expdr(), valreg, 1) || \
	    i2c_master_recv(dsa_expdr(), valbuf, 1)) {
		printk(KERN_ERR "error reading dsa state for dsa %i", dsa);
		failure = 1;
	} else {
		// mask used to set proper bits off
		u8 mask = (0x01 << _dsa_res_out[dsa]) + \
			  (0x01 << _dsa_dep_out[dsa]);
		// now change the proper bit to enable release
		valbuf[0] = (s8)((u8)valbuf[0] & mask);
		// write the changed value
		s8 writebuf[] = {valreg[0], valbuf[0]};
		if (i2c_master_send(dsa_expdr(), writebuf, 2)) {
			printk(KERN_ERR "error shutting off dsa %i", dsa);
			failure = 1;
		}
	}

	if (failure)
		printk(KERN_EMERG "failed to shut off power to dsa %i. \
				shutting off 3v3 supply to minimizei \
				damage to c card", dsa);

	set_3v3_state(0, failure);
	return failure;
}


// for op, 0 = release, 1 = deploy
// inlined to allow compiler to optimize away unnessecary variables
static inline int exec_dsa_op(u8 dsa, u8 op)
{
	const char *opstr = (op == 0) ? "release" : "deploy";
	const s8 timeout = (op == 0) ? _userReleaseTimeout : _userDeployTimeout;
	const enum dsa_state desired = (op == 0) ? released : deployed;
	const u8 *pins = (op == 0) ? \
			 _dsa_res_out : _dsa_dep_out;

	// up the semaphore to show activity
	for (u8 i = 0; i < 10; i++) {
		up(op_act[dsa * 2 + op]);
	}

	// log that the thread was started
	printk(KERN_NOTICE "%s thread successfully created\n", opstr);

	// get the start time, which will be used to determine if the operation has timed out
	struct timespec start = current_kernel_time();

	// don't want to turn the 3V3 on if the operation has already been done
	if (_currentDSAStates[dsa] == desired)
		return 0;

	// turn on 3V3 supply
	set_3v3_state(1, 0);

	// turn on the GPIO on the expander which will enable power to
	//   the proper switch for the operation
	s8 valreg[] = {0x01};
	// read the existing value so that only the bit for this operation
	//   is changed
	s8 valbuf[1];
	if (i2c_master_send(dsa_expdr(), valreg, 1) || \
	    i2c_master_recv(dsa_expdr(), valbuf, 1)) {
		printk(KERN_ERR "error reading dsa state for dsa %i", dsa);
		return 1;
	}

	// now change the proper bit to enable release
	u8 mask = (0x01 << pins[dsa]);
	valbuf[0] = (s8)((u8)valbuf[0] | mask);
	// write the changed value
	s8 writebuf[] = {valreg[0], valbuf[0]};
	if (i2c_master_send(dsa_expdr(), writebuf, 2)) {
		printk(KERN_ERR "dsa %i %s operation failed\n", dsa, opstr);
		return 1;
	}

	// loop that runs until the operaton has completed
	while (1) {
		// up the semaphore to show activity
		for (u8 i = 0; i < 10; i++) {
			up(op_act[dsa * 2 + op]);
		}

		// check the current time
		struct timespec currentTime = current_kernel_time();
		if (currentTime.tv_sec - start.tv_sec > timeout) {
			printk(KERN_NOTICE "dsa %i %s operation \
					timed out", dsa, opstr);
			break;
		}
		// check the current state
		if (_currentDSAStates[dsa] == desired) {
			printk(KERN_NOTICE "dsa %i %s operation \
					successful", dsa, opstr);
			break;
		}
		// check if the user no longer wants a deploy operation to occur
		if (_desiredDSAStates[dsa] != desired) {
			printk(KERN_NOTICE "user requested dsa %i %s operation \
					termination", dsa, opstr);
			break;
		}

		// no need to hog resources
		msleep(200);
	}

	// turn the switch off
	shutoff_dsa(dsa);

	int done;
	do {
		done = down_trylock(op_act[dsa * 2 + op]);
	} while (!done);

	return 0;
}

// this function is run in its own thread and handles releasing the specified DSA
// it will automatically exit upon success, timeout, or if the desiredState changes to something
//   other than released, which usually means a termination request
// it will always shut off power before exiting
// expects a pointer to an int containing the DSA number to release
// frees the memory associated with the pointer after recording the value
// returns NULL
static int res_dsa(void *dsa)
{
	// store the dsa and free the pointer
	u8 d = *(u8 *)dsa;
	kfree(dsa);

	return exec_dsa_op(d, 0);
}

// this function is run in its own thread and handles deploying the specified DSA
// it will automatically exit upon success, timeout, or if the desiredState changes to something
//   other than released, which usually means a termination request
// it will always shut off power before exiting
// expects a pointer to an int containing the DSA number to deploy
// frees the memory associated with the pointer after recording the value
// returns NULL
static int dep_dsa(void *dsa)
{
	// store the dsa and free the pointer
	u8 d = *(u8 *)dsa;
	kfree(dsa);

	return exec_dsa_op(d, 1);
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

	} else if (des == released && cur == stowed) {
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












