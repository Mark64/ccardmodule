// implementation for the log function and 
//   the init and deinit functions for the kernel
//   module build
// implements sysfs interface for exposing system
//   calls to scripting languages


//#include<linux/module.h>
#include<linux/kernel.h>
#include<stdio.h>

#include "ccard.h"

// implementation for the log function from cleanccard.h
void log(char message[], int logLevel) {
	printk(logLevel message);
}




