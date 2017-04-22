#!/bin/bash
# this allows the toolchain to be setup and placed in 
#   the project folder, no sudo required
# sourced from the cmake wiki, located at the following URL
#   as of this writing on 3/17/17
# https://cmake.org/Wiki/CMake_Cross_Compiling
# by Mark Hill

CUR_DIR=$(pwd)
# the location where the linux source code and compiler
#   are placed
#   which will also be used for the toolchain
#   since the toolchain can be removed and updated
#   by deleting the build directory
BUILD_DIR=$CUR_DIR/linux

# the location to which the repo with the toolchain
#   will be cloned
TOOLCHAIN_DEST_DIR=$BUILD_DIR/toolchain
# build tool location (for the gcc, g++, ld, etc. programs)
BUILD_TOOL_DIR=$TOOLCHAIN_DEST_DIR/bin
# the location of the repo containing the cross compile
#   toolchain
TOOLCHAIN_REPO=https://github.com/irvinecubesat/toolchain-arm-linux.git

# linux source code for the intrepid
LINUX_DEST_DIR=$BUILD_DIR/linux
LINUX_REPO=https://github.com/irvinecubesat/linux-2.6.30.2.git


if [ ! -e $TOOLCHAIN_DEST_DIR ]; then
	# make the parent directory
	mkdir -p $TOOLCHAIN_DEST_DIR 
	# clone the repo from the source into the destination
	#   specified by the variables above
	git clone $TOOLCHAIN_REPO $TOOLCHAIN_DEST_DIR
else
	cd $TOOLCHAIN_DEST_DIR
	git pull
	cd $CUR_DIR
fi

if [ ! -e $LINUX_DEST_DIR ]; then
	# make the parent directory
	mkdir -p $LINUX_DEST_DIR 
	# clone the repo from the source into the destination
	#   specified by the variables above
	git clone $LINUX_REPO $LINUX_DEST_DIR
fi

cd $LINUX_DEST_DIR
git pull
cp polysat.config .config
$MAKEARCH menuconfig
$MAKEARCH -j 8 modules_prepare
cd $CUR_DIR





