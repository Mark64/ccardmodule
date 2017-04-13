// implemtation for the magnetorquer functions
//
// by Mark Hill
#include<stdlib.h>
#include<unistd.h>
#include "i2cctl.h"
#include<stdint.h>

#include "ccard.h"

// defines the number of magnetorquers connected to the ccard
#define MT_COUNT 3

// stores the i2c address of the magnetorquer GPIO
//   expander
// http://rohmfs.rohm.com/en/products/databook/datasheet/ic/motor/stepper/bd63510aefv-e.pdf - datasheet
static const uint16_t _magnetorquerControllerAddress = 0x39;

// these arrays store the bit number which corresponds to the GPIO pin
//   that is connected to the corresponding direction for the magnetorquer
//   whose number is the index of the array + 1
// ^terribly worded sentence
// basically its like this [forwardPinMT1, forwardPinMT2,... forwardPinMTn]
static const uint8_t _forwardBits[] = {0, 2, 4};
static const uint8_t _reverseBits[] = {1, 3, 5};

// flag that indicates if the magnetorquer hardware has been
//   initialized properly
// 0 == initialized, -1 = uninitialized
static int _isInitialized = -1;

// sets magnetorquer hardware into a default state and prepares
//   for subsequent state changes
int initializeMagnetorquerHardware() {
	// checks if the hardware has already been initialized
	if (_isInitialized == 0) {
		return 0;
	}

	// configure all pins as outputs
	uint8_t configRegister = 0x03;
	int success = i2cWrite(_magnetorquerControllerAddress, &configRegister, 1, 0x00, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
	// write all off to the i2c device
	uint8_t outputRegister = 0x01;
	success |= i2cWrite(_magnetorquerControllerAddress, &outputRegister, 1, 0x00, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
	if (success != 0) {
		log("failed to configure magnetorquer GPIO expander. is c card plugged in?\n", 2);
		return -1;
	}

	// initializaton was successful
	_isInitialized = 0;
	return 0;
}

// cleans up and powers off the magnetorquer hardware
void deinitializedMagnetorquerHardware() {
	// allows the magnetic field to be discharged before
	//   shutting the hardware off
	for (int i = 0; i < MT_COUNT; i++) {
		setMagnetorquerState(i, off);
	}
	
	// resets the isInitialized flag
	_isInitialized = -1;
}

// retrieves the state of magnetorquer <mtNumber>
enum MTState getMagnetorquerState(unsigned char mtNumber) {
	// initialize the hardware if it hasn't already been
	initializeMagnetorquerHardware();

	// read the current value from the GPIO expander
	uint32_t value = 0;
	uint8_t valueRegister = 0x01;
	int success = i2cWordRead(_magnetorquerControllerAddress, &valueRegister, 1, &value, WORD_8_BIT, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
	if (success != 0) {
		log("reading magnetorquer state failed. Is c card plugged in?\n", 2);
		return off;
	}

	// flags that indicate the state of their corresponding
	//   direction, where 1 = on and 0 = off
	uint8_t forwardOn = (value >> _forwardBits[mtNumber]) & 0x01;
	uint8_t reverseOn = (value >> _reverseBits[mtNumber]) & 0x01;

	// the state is actually a combination of the two bits, with the
	//   reverse bit being the high bit
	return (reverseOn << 1) | forwardOn;
}

// sets the state of magnetorquer <mtNumber> to the
//   desired state, after entering the transition state if
//   needed for a brief period of time
// returns 0 if successful and -1 if not successful
int setMagnetorquerState(unsigned char mtNumber, enum MTState desiredState) {
	// get the current state first to determine whether a transition state
	//   is needed to prevent large back emf, and to determine if a
	//   state change is even necessary
	enum MTState currentState = getMagnetorquerState(mtNumber);
	// determine if state change is needed
	if (currentState == desiredState) {
		log("unnecessary call to setMagnetorquerState. currentState already equals desiredState\n", 1);
		return 0;
	}

	// i2c operation values
	
	// mask used for setting the proper bits
	uint8_t mask = (0x01 << _forwardBits[mtNumber]) | (0x01 << _reverseBits[mtNumber]);
	uint8_t outputRegister = 0x01;
	uint32_t value = 0;
	// read the current values so that only the correct bits are changed when writing
	int success = i2cWordRead(_magnetorquerControllerAddress, &outputRegister, 1, &value, WORD_8_BIT, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);

	// enter transition state if needed
	if (currentState != off) {
		log("enabling both MT ouputs to allow the magnetic field to collapse and prevent back emf-induced damaged\n", 1);
		// modify the correct values with bitwise operations
		value |= mask;
		success |= i2cWrite(_magnetorquerControllerAddress, &outputRegister, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
		
		// give the magnetic field time to collapse
		usleep(100000);

		// if the desired state is transitioning, then this function
		//   has already done it's work
		if (desiredState == transitioning) {
			return (success == 0) ? 0 : -1;
		}
	}

	// write the desired state
	log("updating MT state\n", 1);
	// set the proper bits using bitwise operations
	uint8_t forwardValue = (desiredState == forward) ? 1 : 0;
	uint8_t reverseValue = (desiredState == reverse) ? 1 : 0;
	value &= forwardValue << _forwardBits[mtNumber];
	value &= reverseValue << _reverseBits[mtNumber];
	// modify the correct values with bitwise operations	
	success |= i2cWrite(_magnetorquerControllerAddress, &outputRegister, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
	if (success != 0) {
		log("failed to set magnetorquer state\n", 2);
		return -1;
	}

	return 0;
}






