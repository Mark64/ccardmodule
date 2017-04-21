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
int i2c_set_bus(u8 bus);

// Slightly more efficient but also more specific version of i2cRead for dealing
//   with sampling from the accelerometer and gyro or any device with support for
//   multibyte read
// Theoretically improves performance by a slight amount
// results are passed to the  array argument 'readResult'
// readResults must be the size of 'numberRegisters/bytesPerValue' 
//   and will contain return values for each group of 'bytesPerValue' registers
// wordSize is the number of bytes per value
//   use the macros defined at the top, or use integer values for number of bytes
// high_byte_first is used to indicate whether the first register for each word contains
//   the high byte or the low byte
//   use the macros HIGH_BYTE_FIRST and LOW_BYTE_FIRST
// auto_increment_enabled is a device specific flag that only applies if the device
//   is configured to increment register number automatically after a read
//   0 = disabled, 1 = enabled, but please use the macros instead of explicit values
// returns -1 for failure and 0 for success 
int i2c_read(u16 address, u8 reg[], u8 num_reg, u32 *read_results, u8 bytes_per_value, u8 high_byte_first, u8 auto_increment_enabled);

// obviously a write is needed
// similar to 'i2cset -y bus address register value'
// writes the lower byte to the first register and the higher byte
//   next, ending with the most significant byte
// registers are written in order they appear in the array
// high_byte_first is used to indicate whether the first register should be written
//   with the high byte or the low byte
//   use the macros HIGH_BYTE_FIRST and LOW_BYTE_FIRST
// auto_increment_enabled is a device specific flag that only applies if the device
//   is configured to increment register number automatically after a read
//   0 = disabled, 1 = enabled
// returns a 0 on success and -1 on failure
int i2c_write(u16 address, u8 reg[], u8 num_reg, u32 value, u8 high_byte_first, u8 auto_increment_enabled);




// closes out the i2c file
// I honestly can't anticipate a valid use for this since it's not like having the
//   file open is that big a strain, but someone else may have better use, and its
//   good practice to have this ability
void i2c_close(void);



#endif
