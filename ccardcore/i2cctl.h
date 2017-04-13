// It became obvious that i2c is integral to more than just the sensor code, so it should be a separate file
//   with its own header
//
//   by Mark Hill

#ifndef _i2cctl
#define _i2cctl   

// macros for i2cWrite and i2cWordRead
// see the comments below for their use
#define AUTO_INCREMENT_ENABLED 1
#define AUTO_INCREMENT_DISABLED 0

#define WORD_8_BIT 1
#define WORD_16_BIT 2
#define WORD_24_BIT 3
#define WORD_32_BIT 4
#define WORD_64_BIT 8

#define HIGH_BYTE_FIRST 1
#define LOW_BYTE_FIRST 0


#include<stdint.h>


// IMPORTANT: This supports 10 bit addresses if you pass an address larger than 128 into the address argument
//   If your intention is not to access a 10 bit address, don't pass in a number > 128 or expect me to detect
//   your error

// theres really no use in having to specify the bus for each operation since in most cases i2c-1 will be 
//   used throughout, but this adds the ability to use mutiple busses with this code
// NOTE: limited to max 999 for the bus argument
// NOTE 2: if you run into issues with an improper i2c file or erros reading or writing, the i2c file
//   may be corrupted.  To reset the i2c device file, run this function, which will call the private 
//   function in i2cctl.cpp to reinitialize the i2c file
// returns 0 if successful (it will always be successful) and -1 on failure
int i2cSetBus(uint8_t bus);

// read from the registers in the reg[] array at i2c address 'address'
// can read multiple bytes at once (for instance, when there is an H and L register for 12 and
//   16 bit values) or just read one byte if only 1 register is provided
// Similar to 'i2cget -t bus address register'
// Returns the result of the read operation as a single integer containing the value with 
//   the most significant bit as the first bit of the first register in the reg array
// if 0, either there was an error or the value really is 0
uint32_t i2cRead(uint16_t address, uint8_t reg[], uint8_t numRegisters);

// Slightly more efficient but also more specific version of i2cRead for dealing
//   with sampling from the accelerometer and gyro or any device with support for
//   multibyte read
// Theoretically improves performance by a slight amount
// results are passed to the  array argument 'readResult'
// readResults must be the size of 'numberRegisters/bytesPerValue' 
//   and will contain return values for each group of 'bytesPerValue' registers
// wordSize is the number of bytes per value
//   use the macros defined at the top, or use integer values for number of bytes
// highByteFirst is used to indicate whether the first register for each word contains
//   the high byte or the low byte
//   use the macros HIGH_BYTE_FIRST and LOW_BYTE_FIRST
// autoIncrementEnabled is a device specific flag that only applies if the device
//   is configured to increment register number automatically after a read
//   0 = disabled, 1 = enabled, but please use the macros instead of explicit values
// returns -1 for failure and 0 for success 
int i2cWordRead(uint16_t address, uint8_t reg[], uint8_t numRegisters, uint32_t *readResults, uint8_t wordSize, uint8_t highByteFirst, uint8_t autoIncrementEnabled);

// obviously a write is needed
// similar to 'i2cset -y bus address register value'
// writes the lower byte to the first register and the higher byte
//   next, ending with the most significant byte
// registers are written in order they appear in the array
// highByteFirst is used to indicate whether the first register should be written
//   with the high byte or the low byte
//   use the macros HIGH_BYTE_FIRST and LOW_BYTE_FIRST
// autoIncrementEnabled is a device specific flag that only applies if the device
//   is configured to increment register number automatically after a read
//   0 = disabled, 1 = enabled
// returns a 0 on success and -1 on failure
int i2cWrite(uint16_t address, uint8_t reg[], uint8_t numRegisters, uint32_t value, uint8_t highByteFirst, uint8_t autoIncrementEnabled);




// closes out the i2c file
// I honestly can't anticipate a valid use for this since it's not like having the
//   file open is that big a strain, but someone else may have better use, and its
//   good practice to have this ability
void i2cClose();



#endif
