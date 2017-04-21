# the Makefile wrapper so that I can do out of source builds without having to cd first
# based on Maurice Ling's Makefile in irvine-01-sw and this site
#   http://stackoverflow.com/questions/11143062/getting-cmake-to-build-out-of-source-without-wrapping-scripts

SHELL := /bin/bash
RM    := rm -rf
MKDIR := mkdir -p

TOOLCHAIN_GENERATOR_SCRIPT := installIntrepidToolchain
INSTALL_TEST := installTest

KBUILD_FILE_DIRECTORY := $(PWD)/ccardcore
KERNEL_SOURCE := /lib/modules/`uname -r`/build

all: toolchain
	@ $(MAKE) modules

toolchain:
	# installing the intrepid toolchain as defined by$(TOOLCHAIN_GENERATOR_SCRIPT)
	@ (sh $(TOOLCHAIN_GENERATOR_SCRIPT))

module: 
	$(MAKE) -C $(KERNEL_SOURCE) M=$(KBUILD_FILE_DIRECTORY)

install:
	# copying build to system
	@ (sh $(INSTALL_TEST))

clean:
	@  (cd ccardcore)
	@- $(RM) *.ko.cmd *.ko *.symvers *.mod.c modules.order




