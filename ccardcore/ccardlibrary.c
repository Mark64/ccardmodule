// implementation for the log function and other library-specific
//   functions
//

#include<stdio.h>

#include "ccard.h"

// implementation for the log function from ccard.h for
//   the library build
void log(char message[], int logLevel) {
	printf("log level %d: %s", logLevel, message);
}

// the main function
// why does a library have a main function? well if you
//   choose to instead build this file as an executable, the
//   main function will run through tests of the library
//   functions
// this is useful for debugging
int main() {
	log("executable built succesfully\n", 2);

	return 0;
}




