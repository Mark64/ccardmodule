// implemtation for the magnetorquer functions
// http://rohmfs.rohm.com/en/products/databook/datasheet/ic/motor/stepper/bd63510aefv-e.pdf - datasheet
//
// by Mark Hill
#include<linux/kernel.h>
#include<linux/delay.h>
#include<linux/types.h>
#include<linux/i2c.h>
#include<linux/device.h>

#include "ccard.h"


// defines the number of magnetorquers connected to the ccard
#define MT_COUNT 3

// these arrays store the bit number which corresponds to the GPIO pin
//   that is connected to the corresponding direction for the magnetorquer
//   whose number is the index of the array + 1
// ^terribly worded sentence
// basically its like this [forwardPinMT1, forwardPinMT2,... forwardPinMTn]
static const u8 _forwardBits[] = {0, 2, 4};
static const u8 _reverseBits[] = {1, 3, 5};

// flag that indicates if the magnetorquer hardware has been
//   initialized properly
// 0 == initialized, -1 = uninitialized
static s8 _isInitialized = -1;


// sets magnetorquer hardware into a default state and prepares
//   for subsequent state changes
s8 init_mt() {
	// checks if the hardware has already been initialized
	if (!_isInitialized)
		return 0;

	// configure all pins as outputs
	s8 cfgreg = 0x03;
	s8 cfgval = 0x00;
	const char buf[] = {cfgreg, cfgval};
	s8 success = i2c_master_send(mt_expdr(), buf, 2);
	// write all off to the i2c device
	s8 outreg = 0x01;
	s8 outval = 0x00;
	const char buf2[] = {outreg, outval};
	success |= i2c_master_send(mt_expdr(), buf2, 2);

	if (success) {
		printk(KERN_ERR "failed to configure magnetorquer GPIO expander \
				is c card plugged in?\n");
		return -1;
	}

	printk(KERN_NOTICE "magnetorquer initialization successful\n");
	// initializaton was successful
	_isInitialized = 0;
	return 0;
}

// cleans up and powers off the magnetorquer hardware
void cleanup_mt() {
	// allows the magnetic field to be diss8ged before
	//   shutting the hardware off
	for (s8 i = 0; i < MT_COUNT; i++) {
		set_mt_state(i, off);
	}

	// resets the isInitialized flag
	_isInitialized = -1;
}

// retrieves the state of magnetorquer <mt_num>
enum mt_state get_mt_state(u8 mt_num) {
	if (!_isInitialized)
		return off;
	// read the current value from the GPIO expander
	u8 valbuf[1];
	s8 valregbuf[] = {0x01};
	if (i2c_master_send(mt_expdr(), valregbuf, 1) || \
	    i2c_master_recv(mt_expdr(), valbuf, 1)) {
		printk(KERN_ERR "error reading magnetorquer expdr. is \
				the c card plugged in?\n");
		return off;
	}

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
	if (!_isInitialized)
		return 1;
	// get the current state first to determine whether a transition state
	//   is needed to prevent large back emf, and to determine if a
	//   state change is even necessary
	enum mt_state currentState = get_mt_state(mt_num);
	// determine if state change is needed
	if (currentState == desired_state) {
		printk(KERN_DEBUG"unnecessary call to set_mt_state. \
				currentState already equals desired_state\n");
		return 0;
	}

	// i2c operation values

	s8 outregbuf[] = {0x01};
	s8 valbuf[1];
	if (i2c_master_send(mt_expdr(), outregbuf, 1) || \
	    i2c_master_recv(mt_expdr(), valbuf, 1)) {
		printk(KERN_ERR "error reading mt state when attempting \
				to set state for mt %i\n", mt_num);
		return 1;
	}

	u8 value = (s8)valbuf[0];
	// enter transition state if needed
	if (currentState != off) {
		printk(KERN_NOTICE "enabling both MT ouputs to allow the \
				magnetic field to collapse and prevent back \
				emf-induced damaged\n");

		// mask used for setting the proper bits
		u8 mask = (0x01 << _forwardBits[mt_num]) | \
			  (0x01 << _reverseBits[mt_num]);

		// modify the correct values with bitwise operations
		value |= mask;

		s8 outbuf[] = {outregbuf[0], (s8)value};
		if (i2c_master_send(mt_expdr(), outbuf, 2)) {
			printk(KERN_ERR "setting magnetorquer to state %i \
					failed\n", desired_state);
			return 1;
		}

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

	if (i2c_master_send(mt_expdr(), outbuf, 2)) {
		printk(KERN_ERR "failed to set magnetorquer state\n");
		return 1;
	}

	return 0;
}






