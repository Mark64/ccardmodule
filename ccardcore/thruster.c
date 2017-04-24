// implementation for the thruster code
//
// by Mark Hill

#include<linux/kernel.h>
#include<linux/spi/spi.h>
#include<linux/types.h>
#include<linux/fs.h>
#include<linux/sysfs.h>

#include "ccard.h"


// creates and removes the thruster sysfs object
static inline s8 create_thruster_device(void);
static inline void remove_thruster_device(void);

// definitions for the thruster attributes sysfs callbacks



s8 init_thruster()
{
	return 0;
}


void cleanup_thruster()
{

}






//
// sysfs section
//

static inline s8 create_thruster_device()
{
	return 0;
}

static inline void remove_thruster_device()
{

}



