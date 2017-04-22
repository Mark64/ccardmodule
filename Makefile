# the Makefile wrapper so that I can do out of source builds without having to cd first
# based on Maurice Ling's Makefile in irvine-01-sw and this site
#   http://stackoverflow.com/questions/11143062/getting-cmake-to-build-out-of-source-without-wrapping-scripts

SHELL := /bin/bash
RM    := rm -rf
MKDIR := mkdir -p

ENV_PREPARE_SCRIPT := preparelinux.sh
INSTALL_TEST := installTest

KBUILD_FILE_DIRECTORY := $(PWD)/ccardcore
KERNEL_SOURCE := $(PWD)/linux/linux/ #/lib/modules/`uname -r`/build
TOOLCHAIN_DIR := $(PWD)/linux/toolchain
CROSS_COMPILE := $(TOOLCHAIN_DIR)/bin/arm-linux-

BUILD_DIR := $(PWD)/build
#export KBUILD_OUTPUT=$(BUILD_DIR)

export MAKEARCH := $(MAKE) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILE)

all: toolchain
	@ $(MAKE) module

toolchain:
	# installing the intrepid toolchain as defined by$(TOOLCHAIN_GENERATOR_SCRIPT)
	@ (sh $(ENV_PREPARE_SCRIPT))

module: 
	$(MAKEARCH) -C $(KERNEL_SOURCE) M=$(KBUILD_FILE_DIRECTORY) modules

install:
	# copying build to system
	@ (sh $(INSTALL_TEST))

clean:
	@ (cd $(KBUILD_FILE_DIRECTORY) && mv *.o *.ko.cmd *.ko *.symvers *.mod.c modules.order .tmp_versions .*.o.cmd *.ko .*.ko.cmd $(BUILD_DIR))
	@ $(RM) $(BUILD_DIR)

reallyclean: clean
	@ (RM) linux/

