// implementation of the i2c controller
//
// at first, the plan was for this to be basically stolen/guided from sparkfun
// https://learn.sparkfun.com/tutorials/programming-the-pcduino/i2c-communications
// http://www.frank-buss.de/io-expander/linux-i2c-test.c
//
// However, turns out I hated the implementation the above sites gave, hence the
//   mess of code you see here
//
// by Mark Hill

#include<linux/types.h>
#include<linux/i2c.h>
#include<linux/kernel.h>
#include<linux/semaphore.h>

#include "i2cctl.h"


// sorry for the global. Forgive me for I know not what I do
// defaults to 1 because lets face it, thats normal
static u8 _bus = 1;
static int _i2cFile = -1000;

// this mutex is used to protect the i2c device from being used to do a read or write operations
//   simulateously on another thread
// if this weren't used, programs running on a separate thread trying to write to i2c devices
//   could potentially interfere with and corrupt other concurrent operations
// the other _mutex_created variable allows the mutex to be initialized without a discrete initialize function
// 1 = created, 0 = not created
static struct semaphore _i2c_lock;
static s8 _mutex_created = 0;

// this is a wait function and returns once the lock is removed to allow the i2c function to access the device
static int getLock(void) {
	if (!_mutex_created) {
		sema_init(&_i2c_lock, 1);
		_mutex_created = 1;
	}
	return down_interruptible(&_i2c_lock);
}

// this frees the lock to allow another function to use the i2c device
static void releaseLock(void) {
	if (_mutex_created)
		up(&_i2c_lock);
}


// initializes the _i2cFile variable which makes the i2c bus avaliable for reading and writing
// no need for this to be run outside this file, so its private.  User access can be provided by
//   running the i2c_set_bus function, which will call this function
// returns the i2c file descriptor
int i2c_init(void) {
	// this disrupts access to the i2c device, so obviously it sole access via the lock
	getLock();

	if (_i2cFile < 0) {
		u8 i2cBusName[12];
		snprintf(i2cBusName, 12, "/dev/i2c-%d", _bus);

		_i2cFile = open(i2cBusName, O_RDWR);
		
		// this means an error has occured
		if (_i2cFile < 0)
			printk(KERN_ERR "Error opening i2c file: %d\nIn function i2c_init in i2cctl.cpp\n", _i2cFile);
	}

	releaseLock();
	return _i2cFile;
}




// closes the i2c file
void i2cClose(void) {
	getLock();

	// closes the i2c file
	close(_i2cFile);
	_i2cFile = -1000;

	// releases the lock for the now useless i2c file
	releaseLock();
	_mutex_created = 0;
}




// sets the i2c device address and also configures the i2c device to take 10 bit or 8 bit addresses
// returns 0 for success and something else for error
static int i2c_set_address(u16 address) {
	// in case the bus was never set, this ensures the i2c device is always initialized
	i2c_init();
	
	// needs lock access because this will disrupt concurrent operations
	getLock();

	// set ten bit address mode
	u8 isTenBit = (address - 127 > 0) ? 1 : 0;
	int set10Bit = ioctl(_i2cFile, I2C_TENBIT, isTenBit);
	
	// this means an error has occured
	if (set10Bit < 0) {
		printk(KERN_ERR "Failed to set 10 bit mode with error code %d in i2cctl.cpp\n", set10Bit);

		releaseLock();
		return set10Bit;
	}

	// set the address of the slave to talk to 
	int setSlave = ioctl(_i2cFile, I2C_SLAVE, address);
	
	// this means an error has occured
	if (setSlave < 0) {
		printk(KERN_ERR "Failed to set slave address to %x with error %d in i2cctl.cpp\n", address, setSlave);

		releaseLock();
		return setSlave;
	}	

	releaseLock();
	return 0;
}




// set the bus used for i2c communication
int i2c_set_bus(u8 bus) {
	// prevent orphaned files
	i2cClose();

	// obviously needs the lock so it gets sole access to the i2c device
	getLock();

	_bus = bus;

	// finished with the lock before most functions, and i2c_init needs the lock, so it gets released
	releaseLock();
	
	// reinitialize the i2c when the desired bus changes
	i2c_init();

	// idc that this is unnecessary, I made it bool so itll return bool because I like bool
	return (_bus == bus && _i2cFile >= 0) ? 0 : -1;
}




// single or multiple byte read
u32 i2cRead(u16 address, u8 reg[], u8 num_reg) {
	// set address of i2c device and then check if it failed
	if (i2c_set_address(address) != 0) {
		printk(KERN_ERR "Failed to set device address %x in i2cctl.cpp\n", address); 
		return 0;
	}
	
	// loops through all the registers and sets the data returned from the read operation on each register
	//   to the correct position in the 'result' integer
	// also needs the lock during this loop
	getLock();
	u32 result = 0;
	for (int regIndex = 0; regIndex < num_reg; regIndex++) {
		// sets the curReg temporary variable
		u8 curReg = reg[regIndex];

		// write the address of the register to be read (the first step in the i2c operation is actually to 
		//   write to the slave the value of the i2c register you want to read from) and then check if it failed
		if (write(_i2cFile, &curReg, 1) < 0) {
			printk(KERN_ERR "Failed to set device register %x at address %x in i2cctl.cpp\n", curReg, address);

			releaseLock();
			return 0;
		}
	
		// if you got this far, its time to actually perform a read and then, as you can guess, check if it failed
		u32 data = 0;
		if (read(_i2cFile, &data, 1) < 0) {
			printk(KERN_ERR "Failed to read from register %x at address %x in i2cctl.cpp\n", curReg, address);

			releaseLock();
			return 0;
		}

		// offset the bits in the data variable so that it can be addedd to 'result'
		s8 offsets = num_reg - (regIndex + 1);
		data <<= offsets * 8;
		
		result += data;
	}

	releaseLock();
	return result;
}

// Slightly more efficient but highly specific version of i2cRead for dealing 
//   with sampling from the accelerometer and gyro or any other device with
//   multibyte words split across multiple registers
int i2c_read(u16 address, u8 reg[], u8 num_reg, u32 *readResults, u8 bytes_per_value, u8 high_byte_first, u8 auto_increment_enabled) {
	// set address of i2c device and then check if it failed
	if (i2c_set_address(address) != 0) {
		printk(KERN_ERR "Failed to set device address %x in i2cctl.cpp\n", address); 
		return -1;
	}
	
	// lock is needed for the rest of the function since reads and writes will
	//   take place
	getLock();
	
	// outer loop deals with the whole word being read
	// it loops 'num_reg/bytes_per_value' times, so reads bytes_per_value bytes per output value
	// result is placed into the index of 'readResults' that corresponds to the 'readIndex' variable
	for (int readIndex = 0; readIndex < num_reg/bytes_per_value; readIndex++) {
		
		// this variable will store the result of the read opration
		u32 result = 0;

		// now loop through twice to read from the 'bytes_per_value' number of registers that make up the 
		//   value to be placed into the variable 'result' above
		for (int regIndex = 0; regIndex < bytes_per_value; regIndex++) {
			
			// sets the curReg temporary variable based on the current readIndex and regIndex
			u8 curReg = reg[regIndex + (bytes_per_value*readIndex)];
	
			// if you got this far, its time to actually perform a read and then, as you can guess, check if it failed
			// in this version of i2cRead, we actually want to sample multiple times prior to determining and
			//   returning a value
			u32 readResult = 0;
				
			// write the address of the register to be read (the first step in the i2c operation is actually to 
			//   write to the slave the value of the i2c register you want to read from) 
			//   only if autoIncrement is not enabled or this is the first register to be read
			int writeSuccess = 0;
			if (regIndex == 0 || auto_increment_enabled == 0)
				 writeSuccess = write(_i2cFile, &curReg, 1);
			
			// if the write failed, print out the error
			if (writeSuccess < 0) {
				printk(KERN_ERR "Failed to set device register %x at address %x in i2cctl.cpp\n", curReg, address);
				if (auto_increment_enabled == 1) {
					printk(KERN_ERR "Read with auto_increment_enabled flag set to 1 failed, so read exited prematurely");
					releaseLock();
					return -1;
				}
			}
			// this means register address setting was successful
			// this is where the data is actually read and the success is checked
			else {
				u8 numBytes = auto_increment_enabled == 1 ? bytes_per_value : 1;
				u8 readArray[numBytes];
				// array stores the data, but readData is the actual values correct for MSB or LSB first
				u32 readData = 0;
				if (read(_i2cFile, readArray, numBytes) < 0) 
				// if the read was actually successful, correct the bit order (MSB first means the bytes
				//   are currently out of order), then add the result
				else {
					for (int i = 0; i < bytes_per_value; i++) {
						u32 temp = readArray[i];
						u8 numShifts = (high_byte_first == HIGH_BYTE_FIRST) ? (8 * (bytes_per_value - i - 1)) : (8 * i);
						readData += (temp << numShifts);
					}

					readResult += readData;
				}
			}
			
			// if auto increment is enabled, no shifting or subsequent reads are needed
			if (auto_increment_enabled == 1) {
				result += readResult;
				break;
			}
			else {
				// offset the bits in the data variable so that it can be addedd to 'result'
				// switches the offset order (0, 1, 2 vs 2, 1, 0 for example) depending on the high_byte_first flag
				int offsets = high_byte_first == HIGH_BYTE_FIRST ? bytes_per_value - regIndex - 1 : regIndex;
				readResult <<= offsets * 8;
			
				result += readResult;
			}
		}
		
		// add the read value into the array to be returned
		readResults[readIndex] = result;
	}

	// lock done
	releaseLock();
	return 0;
}



// single or multiple byte write
int i2c_write(u16 address, u8 reg[], u8 num_reg, u32 value, u8 high_byte_first, u8 auto_increment_enabled) {	
	// set address of i2c device and then check if it failed
	if (i2c_set_address(address) != 0) {
		printk(KERN_ERR "Failed to set device address %x in i2cctl.cpp\n", address); 
		return -1;
	}
	
	// needs the lock during this loop
	getLock();

	// flag that stores the success return value for the write function
	u8 writeSuccess = 0;

	// this mask removes all the unnecessary bits
	u32 mask = 0x000000ff;

	// with autoincrement enabled, the entire write must be done in a single transaction
	//   or else the register will be incremented before completion
	if (auto_increment_enabled == 1) {
		u8 writeData[num_reg + 1];
		writeData[0] = reg[0];
			
		// the 32 bit value must be split into single bytes
		for (int i = 0; i < num_reg; i++) {
			int8_t offsets = high_byte_first == HIGH_BYTE_FIRST ? (num_reg - i - 1) * 8 : (i * 8);
			writeData[i+1] = (value >> offsets) & mask;
		}

		writeSuccess = write(_i2cFile, &writeData, num_reg + 1);		
			
		if (writeSuccess < 0) {
			printk(KERN_ERR "Failed to write %x to device register %x at address %x in i2cctl.cpp\n", value, reg[0], address);
	
			releaseLock();
			return -1;
		}
		
		releaseLock();	
		return 0;
	}
	// else
	// loops through all the registers and sets them to the correct position in the 'result' integer
	for (int regIndex = 0; regIndex < num_reg; regIndex++) {
		// retrieves the byte to be written from within the value variable and puts it in a temporary variable
		mask = 0x000000ff;
		s8 offsets = high_byte_first == HIGH_BYTE_FIRST ? (num_reg - regIndex - 1) * 8 : (regIndex * 8);
		u8 writeValue = (value >> offsets) & mask;
		
		//printk(KERN_ERR "writing value %x obtained from a mask of %x on the original value %x\n", writeValue, mask, value);

		// sets the curReg temporary variable
		u8 curReg = reg[regIndex];

		// write the address of the register to be written (the first step in the i2c operation is actually to 
		//   write to the slave the value of the i2c register you want to write to), then write the byte from 
		//   the variable 'writeValue', then check if the whole operation failed
		u8 writeData[] = {curReg, writeValue};

		writeSuccess = write(_i2cFile, writeData, 1);

		// check for failure
		if (writeSuccess < 0) {
			printk(KERN_ERR "Failed to write %x to register %x at address %x in i2cctl.cpp\n", writeValue, curReg, address);
			releaseLock();
			return -1;
		}
	}

	releaseLock();
	return 0;
}


	







