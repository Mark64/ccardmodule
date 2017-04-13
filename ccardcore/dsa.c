// implementation for the DSA (deployable solar array) system calls
//
// using the TCA9554A GPIO expander
// http://www.ti.com/lit/ds/symlink/tca9554.pdf - datasheet
//
// by Mark Hill
#include<unistd.h>
#include<pthread.h>
#include<stdint.h>
#include<fcntl.h>
#include<stdlib.h>
#include<time.h>

#include "i2cctl.h"

#include "ccard.h"

#define DSA_COUNT 2

// defines the i2c address at which the GPIO expander is located
static const uint8_t _dsaControllerI2CAddress = 0x38;
// defines the pin location for each value, which corresponds to the
//   bit number on the device's registers
// the value at index 0 is the corresponding value for DSA 1, and
//   the value at each subsequent index 'n' is the corresponding value
//   for DSA 'n + 1'
// its a useless feature, because I don't think the cubesat will ever
//   have more than 1 DSA, but who knows what the EXA has planned
static const uint8_t _dsaReleaseActivateOutputs[] = {0, 2};
static const uint8_t _dsaDeployActivateOutputs[] = {1, 3};
static const uint8_t _dsaReleaseStatusInputs[] = {5, 7};
static const uint8_t _dsaDeployStatusInputs[] = {6, 8};

// flag indicating whether the DSA hardware has been properly
//   configured
// 0 = initialized
// -1 = uninitialzed
static int _isInitialized = -1;

// these store the user configured timeout values used by the system for the
//   DSA operations
// they are currently -1, and will be set to the proper value in initialization
static unsigned char _userReleaseTimeout = -1;
static unsigned char _userDeployTimeout = -1;

// stores the desired values for the DSAs
static enum DSAState _desiredDSAStates[DSA_COUNT]; 
// stores the current state of the DSAs as determined by reading
//   hardware state registers. see getDSAState(dsaNumber)
static enum DSAState _currentDSAStates[DSA_COUNT];

// this is the main update function which ensures that the hardware values
//   for the dsa state reflect the settings in desiredDSAStates
static void *updateDSAState(void *junkInput);
// this function is called by updateDSAState when a discrepancy
//   in the desired and current DSA states is found for
//   DSA <dsaNumber>
// this function determines the proper method of correcting the
//   discrepancy and passes that onto a separate thread if needed
// the array is an array of size DSA_COUNT * 2 and contains
//   unsigned integers indicating the number of attempts that
//   have been made on each operation
// returns 1 on success, indicating that it will (or has) attempted
//   to correct the discrepancy
// if the maximum number of tries has been reached, it will return 2
// if it fails for any other reason, it will return -1
static int correctDSA(unsigned char dsaNumber, uint8_t numTries[]);


// stores the thread used to update the DSA states
static pthread_t updateThread;

// -1 indicates the mutex hasn't been created, 0 indicates it has
static int _mutex_created = -1;
// stores the mutex lock for DSA operations
static pthread_mutex_t _lock;
// this function waits for the mutex lock and returns once the lock is
//   removed to allow access to the DSA state and configuration values
static void getLock() {
	if (!_mutex_created) {
		pthread_mutex_init(&_lock, NULL);
		_mutex_created = 1;
	}
	pthread_mutex_lock(&_lock);
}

// this frees the lock to allow another function to use the DSA
//   state and configuration values
static void releaseLock() {
	if (_mutex_created) {
		pthread_mutex_unlock(&_lock);
	}
}


// sets the 3V3 power supply to the C Card on (1) or off (0)
static void setPowerState(uint8_t state) {
	if (state == 1) {
		int export = open("sys/class/gpio/export", O_WRONLY);
		write(export, "102", 3);
		int dir = open("sys/class/gpio/gpio102/direction", O_WRONLY);
		write(dir, "out", 3);
		close(dir);
		int gpio = open("sys/class/gpio/gpio102/value", O_WRONLY);
		write(gpio, "1", 1);
		close(gpio);
	}
	else {	
		int export = open("sys/class/gpio/export", O_WRONLY);
		write(export, "102", 3);
		int dir = open("sys/class/gpio/gpio102/direction", O_WRONLY);
		write(dir, "out", 3);
		close(dir);
		int gpio = open("sys/class/gpio/gpio102/value", O_WRONLY);
		write(gpio, "0", 1);
		close(gpio);
	}
}


// sets the initial state of the GPIO expander and initializes configuration values
int initializeDSAHardware() {
	// skip initialization if it has already been done
	if (_isInitialized == 0) {
		return 0;
	}

	// this will be making modifications, so it should block
	//   other threads from accessing the DSA hardware and config values
	getLock();

	// make sure the 3V3 power supply is off
	setPowerState(0);

	// initializes the timeout values to the default included in the
	//   header file
	_userReleaseTimeout = releaseTimeoutMax;
	_userDeployTimeout = deployTimeoutMax;
	
	// initializes the pins as inputs or outputs, respectively
	// setting a pin as an input requires a write of 1, while setting
	//   it as an output requires a write of 0
	uint8_t configurationValue = 0xf0;
	uint8_t configurationRegister = 0x03;
	uint8_t outputValue = 0x00;
	uint8_t outputsRegister = 0x01;
	int failure = i2cWrite(_dsaControllerI2CAddress, &configurationRegister, 1, configurationValue, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
	failure |= i2cWrite(_dsaControllerI2CAddress, &outputsRegister, 1, outputValue, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
	if (failure) {
		log("failed to configure DSA GPIO expander. Is the C Card connected?\n", 2);
		return -1;
	}

	// spawns the update thread, which ensures that the hardware state
	//   reflects the values in desiredDSAStates
	pthread_create(&updateThread, NULL, &updateDSAState, NULL);
	
	_isInitialized = 0;
	// done changing and reading data
	releaseLock();

	return 0;
}

// cleans up and powers off the DSA hardware
void deinitializeDSAHardware() {
	// getting the lock makes sure that no state changes can take place
	getLock();
	// since the update threads will exit when they detect a
	//   change in the desired state, the desired state is set to to
	//   stowed so that they can be closed
	for (int i = 0; i < DSA_COUNT; i++) {
		_desiredDSAStates[i] = stowed;
	}
	// now wait for the threads to close
	usleep(1000000);

	// kill the main updating thread
	pthread_cancel(updateThread);

	// turn off the outputs
	uint8_t outputRegister = 0x01;
	uint8_t offValue = 0x00;
	i2cWrite(_dsaControllerI2CAddress, &outputRegister, 1, offValue, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);

	// turn off 3V3 power
	setPowerState(0);

	// set the isInitialized flag off
	_isInitialized = -1;

	// finish by releasing the lock
	releaseLock();
}


// this sets the user-configurable timeout value, resulting in the software using that
//   value rather than the value included in the header
void setReleaseTimeoutMax(unsigned char desiredTimeout) {
	// initialize if the user hasn't done that already
	initializeDSAHardware();
	// needs the lock to change values
	getLock();
	_userReleaseTimeout = desiredTimeout;
	releaseLock();
}

// this sets the user-configurable timeout value, resulting in the software using that
//   value rather than the value included in the header
void setDeployTimeoutMax(unsigned char desiredTimeout) {
	// initialize if the user hasn't done that already
	initializeDSAHardware();
	// needs the lock to change values
	getLock();
	_userDeployTimeout = desiredTimeout;
	releaseLock();
}


enum DSAState getDSAState(unsigned char dsaNumber) {
	// initialize if the user hasn't done that already
	initializeDSAHardware();
	// check to make sure the dsaNumber isn't out of bounds
	if (dsaNumber >= DSA_COUNT) {
		log("invalid DSA number in getDSAState\n", 2);
		return numError;
	}
	// reading may occur in the middle of a write, so needs the lock
	getLock();
	enum DSAState state = _currentDSAStates[dsaNumber];
	releaseLock();

	return state;
}

int setDSAState(unsigned char dsaNumber, enum DSAState desiredState) {
	// initialize if the user hasn't done that already
	initializeDSAHardware();
	// first, we need to check the input to ensure it makes sense
	//   and isn't going to put the system in an invalid state
	// lets get the current state
	// and if you're wondering why this isn't in the lock,
	//   its because the lock is handled by getDSAState
	if (dsaNumber >= DSA_COUNT) {
		log("invalid DSA number in setDSAState\n", 2);
		return -1;
	}

	enum DSAState currentState = getDSAState(dsaNumber);
	// stores the return value, which is determined by whether or
	//   not the input makes sense
	int returnValue = -1;
	if (desiredState != released || desiredState != deployed) {
		log("incompatible desired state in setDSAState\n", 2);
		return returnValue;
	}
	if (desiredState == deployed && currentState == stowed) {
		log("performing deploy operation while DSA is currently stored. Possible error\n", 2);
		returnValue = 3;
	}
	
	// writing definitely needs the lock in a multithreaded setup
	getLock();
	// write the desired state to the appropriate array index
	_desiredDSAStates[dsaNumber] = desiredState;
	
	releaseLock();
	return returnValue;	
}


// it monitors the 'desiredDSAStates' array for changes as compared
//   to the current value obtained reported by the hardware, and if it finds
//   a discrepancy, it spawns a thread to handle the change
// if all the DSA's have been deployed, this function exits to
//   prevent wasted resources
static void *updateDSAState(void *junkInput) {
	// this if statement is useless and should be removed, but
	//   in YouCompleteMe, the vim plugin I use to turn vim into
	//   a fully functional IDE, it flags the input as unused, so
	//   thats why this is here
	if (junkInput != NULL) {
		log("possible error in dsa updating thread", 2);
	}

	// this array contains values which indicate whether the
	// number of times an attempt was made to correct a discrepancy
	// the array is organized as follows:
	//   [releaseDSA0, deployDSA0,... releaseDSA<DSA_COUNT>, deployDSA<DSA_COUNT>]
	// 0 indicates no attempts has been made, 1 indicates an attempt has
	//   been made, and so on
	// the threads are passed a pointer to their respective
	//   flag, which they should update upon exit
	// if they were successful, they should still increment their flag,
	//   so if the first DSA release succeeds, the value in this array
	//   for that corresponding DSA should be 1
	// that way, this function can check how many times the
	//   operation has been attempted and give up if it fails too many
	//   times
	uint8_t attemptCount[DSA_COUNT * 2] = {0};

	// this array holds a flag for each DSA indicating
	//   whether the state for each is acceptable (0) or
	//   if it requires correction (1)
	uint8_t dsaDiscrepancies[DSA_COUNT] = {0};		

	// infinite loop designed to check values and ensure proper
	//   function and updating of the DSA state
	while (1) {
		// state update section
		//
		
		// needs the mutex lock
		getLock();

		// the TCA9554A represents the current state of its
		//   input registers as one byte, and the value of its
		//   output registers as a second byte
		// both are needed to determine the state of the software
		uint8_t inputRegister = 0x00;
		uint8_t outputRegister = 0x01;
		uint8_t registers[] = {inputRegister, outputRegister};
		// the first value in this array is the input value, second
		//   value is the output value
		uint32_t gpioState[2] = {0};
		
		int success = i2cWordRead(_dsaControllerI2CAddress, registers, 2, gpioState, WORD_8_BIT, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
		if (success != 0) {
			log("failed to read DSA state from i2c device. C Card unplugged?\n", 2); 
		}
		// no point in updating state values if the i2c read failed, the
		//   data would be useless
		else {
			for (int i = 0; i < DSA_COUNT; i++) {
				// uses the bit number (big endian format) to shift the bits
				//   and determine the corresponding value
				uint16_t inputReleaseValue = (gpioState[0] >> _dsaReleaseStatusInputs[i]) & 0b0001;
				uint16_t inputDeployValue = (gpioState[0] >> _dsaDeployStatusInputs[i]) & 0b0001;
				uint16_t outputReleaseValue = (gpioState[1] >> _dsaReleaseActivateOutputs[i]) & 0b0001;
				uint16_t outputDeployValue = (gpioState[1] >> _dsaDeployActivateOutputs[i]) & 0b0001;
				
				// because its technically possible to run a deploy operation without
				//   having first released the DSAs, the raw enumvalue for deploying 
				//   ignores the input release value, so when a deploy operation is 
				//   running, the input release value must be set to zero regardless 
				//   of its true value
				if (outputDeployValue == 1) {
					inputReleaseValue = 0;
				}
				// now, thanks to bitwise operations and a clever setup of the DSAState enum
				//   raw values, creating the proper state value is easy
				// to understand why the bits are shifted by the amount here, see
				//   the raw enum values in cleanccard.h
				enum DSAState state = (inputReleaseValue << 1) + (inputDeployValue << 1) + (inputDeployValue << 3) + outputReleaseValue + (outputDeployValue << 2);

				// now set the corresponding current state value
				_currentDSAStates[i] = state;
			}
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
			// check if the states differ
			if (_desiredDSAStates[i] != _currentDSAStates[i]) {
				// the states will differ while a correction is in progress,
				//   so this avoids a false flag (note: does not avoid 9/11)
				if ((!(_desiredDSAStates[i] == released && _currentDSAStates[i] == releasing)) || (!(_desiredDSAStates[i] == deployed && _currentDSAStates[i] == deployed))) {
					dsaDiscrepancies[i] = 1;
				}
			}
		}

		// fix discrepancy section
		//

		// if no discrepancies are found, and all DSA's have been deployed,
		//   then this flags that condition so that the infinite loop,
		//   and the current thread, can be exited
		// 0 = all DSAs deployed, 1 = not all DSAs deployed
		uint8_t allDeployedFlag = 0;
		// this indicates the result of correctDSA
		// if all attempts to correct the DSA return saying that
		//   the max attempts have been reached, then the loop is
		//   useless and should exit
		// 0 = max attempts reached, 1 = some operations still can be run
		uint8_t maxAttemptsReached = 0;
		for (int i = 0; i < DSA_COUNT; i++) {
			if (dsaDiscrepancies[i] == 1) {
				correctDSA(i, attemptCount);
			}
			if (_currentDSAStates[i] != deployed) {
				allDeployedFlag = 1;
			}
			if (attemptCount[i * 2] < releaseAttemptsMax || attemptCount[(i * 2) + 1] < deployAttemptsMax) {
				maxAttemptsReached = 1;
			}
		}

		// if all DSAs are deployed, break from the infinite loops
		//   and end the thread
		if (allDeployedFlag == 0 || maxAttemptsReached == 0) {
			log("exiting DSA update thread. Max attempts reached or all DSAs were deployed\n", 2);
			releaseLock();
			break;
		}

		releaseLock();

		// only runs every 0.1 seconds (plus execution time), otherwise it would
		//   be very bad for performance
		// plus its not like you need super high resolution here
		usleep(100000);
	}

	// turns off 3V3 power supply since it's no longer needed
	setPowerState(0);

	// deinitializes the DSA hardware when finished, so that if
	//   a subsequent query is run, the initialization sequence
	//   will be run again to set things up
	_isInitialized = -1;
	// returns only once all DSA's are deployed
	return NULL;
}


// this function is run in its own thread and handles releasing the specified DSA
// it will automatically exit upon success, timeout, or if the desiredState changes to something
//   other than released, which usually means a termination request
// it will always shut off power before exiting
// expects a pointer to an int containing the DSA number to release
// frees the memory associated with the pointer after recording the value
// returns NULL
static void *releaseDSA(void *dsaNumber) {
	// log that the thread was started
	log("release thread successfully created\n", 2);

	// store the dsaNumber and free the pointer
	uint8_t dsa = *(uint8_t *)dsaNumber;
	free(dsaNumber);
	
	// get the start time, which will be used to determine if the operation has timed out
	struct timespec start;
	clock_gettime(CLOCK_MONOTONIC, &start);
	
	// willl be writing to the c card
	getLock();
	// don't want to turn the 3V3 on if the operation has already been done
	if (_currentDSAStates[dsa] == released) {
		releaseLock();
		return NULL;
	}

	// turn on 3V3 supply
	setPowerState(1);

	// turn on the GPIO on the expander which will enable power to
	//   the proper switch for the operation
	uint8_t reg = 0x01;
	// read the existing value so that only the bit for this operation
	//   is changed
	uint32_t value = 0;
	int success = i2cWordRead(_dsaControllerI2CAddress, &reg, 1, &value, WORD_8_BIT, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
	// now change the proper bit to enable release
	uint8_t bit = _dsaReleaseActivateOutputs[dsa];
	uint8_t mask = (0x01 << bit);
	value |= mask;
	// write the changed value
	success |= i2cWrite(_dsaControllerI2CAddress, &reg, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
	if (success != 0) {
		log("releasing DSA failed. Thread exiting\n", 2);
		releaseLock();
		return NULL;
	}

	releaseLock();
	// loop that runs until the operaton has completed
	while (1) {
		// check the current time
		struct timespec currentTime;
		clock_gettime(CLOCK_MONOTONIC, &currentTime);
		if (currentTime.tv_sec - start.tv_sec > _userReleaseTimeout) {
			break;
		}
		// will be performing reads and writes
		getLock();
		// check the current state
		if (_currentDSAStates[dsa] == released) {
			log("release operation successful. thread exiting\n", 2);
			releaseLock();
			break;
		}
		// check if the user no longer wants a release operation to occur
		if (_desiredDSAStates[dsa] != released) {
			log("user requested release operation termination. thread exiting\n", 2);
			releaseLock();
			break;
		}
		
		releaseLock();
		// no need to hog resources
		usleep(200000);
	}

	// turn off power to the switch
	mask = 0xff ^ mask;
	value &= mask;
	// write the value to turn the switch off
	// very important that power is shut off, so this operation is run multiple times in case
	for (int i = 0; i < 4 || success == 0; i++) {
		success = i2cWrite(_dsaControllerI2CAddress, &reg, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
	}
	if (success != 0) {
		setPowerState(0);
		log("WARNING: UNABLE TO SWITCH OFF DSA RELEASE POWER. Shutting off 3V3 power supply. May interrupt operations\n", 6); 
	}

	return NULL;
}

// this function is run in its own thread and handles deploying the specified DSA
// it will automatically exit upon success, timeout, or if the desiredState changes to something
//   other than released, which usually means a termination request
// it will always shut off power before exiting
// expects a pointer to an int containing the DSA number to deploy
// frees the memory associated with the pointer after recording the value
// returns NULL
static void *deployDSA(void *dsaNumber) {
	// log that the thread was started
	log("deploy thread successfully created\n", 2);

	// store the dsaNumber and free the pointer
	uint8_t dsa = *(uint8_t *)dsaNumber;
	free(dsaNumber);
	
	// get the start time, which will be used to determine if the operation has timed out
	struct timespec start;
	clock_gettime(CLOCK_MONOTONIC, &start);
	
	// willl be writing to the c card
	getLock();
	// don't want to turn the 3V3 on if the operation has already been done
	if (_currentDSAStates[dsa] == deployed) {
		releaseLock();
		return NULL;
	}

	// turn on 3V3 supply
	setPowerState(1);

	// turn on the GPIO on the expander which will enable power to
	//   the proper switch for the operation
	uint8_t reg = 0x01;
	// read the existing value so that only the bit for this operation
	//   is changed
	uint32_t value = 0;
	int success = i2cWordRead(_dsaControllerI2CAddress, &reg, 1, &value, WORD_8_BIT, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
	// now change the proper bit to enable release
	uint8_t bit = _dsaDeployActivateOutputs[dsa];
	uint8_t mask = (0x01 << bit);
	value |= mask;
	// write the changed value
	success |= i2cWrite(_dsaControllerI2CAddress, &reg, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
	if (success != 0) {
		log("deploying DSA failed. Thread exiting\n", 2);
		releaseLock();
		return NULL;
	}

	releaseLock();
	// loop that runs until the operaton has completed
	while (1) {
		// check the current time
		struct timespec currentTime;
		clock_gettime(CLOCK_MONOTONIC, &currentTime);
		if (currentTime.tv_sec - start.tv_sec > _userReleaseTimeout) {
			break;
		}
		// will be performing reads and writes
		getLock();
		// check the current state
		if (_currentDSAStates[dsa] == deployed) {
			log("deploy operation successful. thread exiting\n", 2);
			releaseLock();
			break;
		}
	
		// check if the user no longer wants a deploy operation to occur
		if (_desiredDSAStates[dsa] != deployed) {
			log("user requested deploy operation termination. thread exiting\n", 2);
			releaseLock();
			break;
		}

		releaseLock();
		// no need to hog resources
		usleep(200000);
	}	

	// turn off power to the switch
	mask = 0xff ^ mask;
	value &= mask;
	// write the value to turn the switch off
	// very important that power is shut off, so this operation is run multiple times in case
	for (int i = 0; i < 4 || success == 0; i++) {
		success = i2cWrite(_dsaControllerI2CAddress, &reg, 1, value, HIGH_BYTE_FIRST, AUTO_INCREMENT_DISABLED);
	}
	if (success != 0) {
		setPowerState(0);
		log("WARNING: UNABLE TO SWITCH OFF DSA DEPLOY POWER. Shutting off 3V3 power supply. May interrupt operations\n", 6); 
	}

	return NULL;
}

// this is called by updateDSAState when a discrepancy is found
static int correctDSA(unsigned char dsaNumber, uint8_t numTries[]) {
	// check that dsaNumber is in bounds
	if (dsaNumber >= DSA_COUNT) {
		log("invalid dsaNumber in correctDSA\n", 2);
		return -1;
	}
	
	int returnValue = -1;
	// determines the discrepancy and schedules an operation if needed
	if (_currentDSAStates[dsaNumber] == releasing || _currentDSAStates[dsaNumber] == deploying) {
		log("possible duplicate call to correctDSA. exiting\n", 2);
		returnValue = 0;

	}
	else if (_desiredDSAStates[dsaNumber] == released && _currentDSAStates[dsaNumber] == stowed) {
		if (numTries[dsaNumber * 2] >= releaseAttemptsMax) {
			log("maximum release attempts reached. exiting without release attempt\n", 2);
			returnValue = 2;
		}
		else {
			log("scheduling release operation. watch for log message indicating release thread was started\n", 2);
			
			// creates a thread to release the specified DSA and manage timeouts
			pthread_t releaseThread;
			uint8_t *dsa = malloc(sizeof(uint8_t));
			*dsa = dsaNumber;
			int failure = pthread_create(&releaseThread, NULL, &releaseDSA, dsa);
			if (failure) {
				log("failed to create releaseDSA thread\n", 2);
				returnValue = -1;
			}
			else {
				returnValue = 0;
			}
		}
	}
	else if (_desiredDSAStates[dsaNumber] == deployed && (_currentDSAStates[dsaNumber] == released || _currentDSAStates[dsaNumber] == stowed)) {
		if (numTries[(dsaNumber * 2) + 1] >= deployAttemptsMax) {
			log("maximum deploy attempt limit reached. exiting without deploying\n", 2);
			returnValue = 2;
		}
		else {
			log("scheduling deploy operation. watch for log message indicating deploy thread was started\n", 2);

			// creates a thread to deploy the specified DSA and manage timeouts
			pthread_t deployThread;
			uint8_t *dsa = malloc(sizeof(uint8_t));
			*dsa = dsaNumber;
			int failure = pthread_create(&deployThread, NULL, &deployDSA, dsa);
			if (failure) {
				log("failed to create deployDSA thread\n", 2);
				returnValue = -1;
			}
			else {
				returnValue = 0;
			}
		}
	}
	else if (_desiredDSAStates[dsaNumber] == stowed && (_currentDSAStates[dsaNumber] != releasing || _currentDSAStates[dsaNumber] != deploying)) {
		log("unable to stow DSA. Either it is already deployed, release, or has an associated error\n", 2);
		returnValue = -1;
	}

	return returnValue;
}












