# the Makefile wrapper so that I can do out of source builds without having to cd first
# based on Maurice Ling's Makefile in irvine-01-sw and this site
#   http://stackoverflow.com/questions/11143062/getting-cmake-to-build-out-of-source-without-wrapping-scripts

SHELL := /bin/bash
RM    := rm -rf
MKDIR := mkdir -p

TOOLCHAIN_GENERATOR_SCRIPT := installIntrepidToolchain
CMAKE_TOOLCHAIN_FILE := toolchainIntrepidR8.cmake
INSTALL_TEST := installTest

KBUILD_FILE_DIRECTORY := $(PWD)/ccardcore
KERNEL_SOURCE := /lib/modules/`uname -r`/build

all: ./build/Makefile
	@ $(MAKE) -C build

allpc: ./buildpc/Makefile
	@ $(MAKE) -C buildpc

./build/Makefile: toolchain
	@ (cd build > /dev/null 2>&1 && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE=$(CMAKE_TOOLCHAIN_FILE) ..)

./buildpc/Makefile:
	@  ($(MKDIR) buildpc > /dev/null)
	@  (cd buildpc > /dev/null 2>&1 && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON ..)

toolchain:
	# installing the intrepid toolchain as defined by$(TOOLCHAIN_GENERATOR_SCRIPT)
	@ (sh $(TOOLCHAIN_GENERATOR_SCRIPT))

module:
	$(MAKE) -C $(KERNEL_SOURCE) M=$(KBUILD_FILE_DIRECTORY)

install: all
	# copying build to system
	@ (sh $(INSTALL_TEST))

distclean:
	@  ($(MKDIR) build > /dev/null)
	@  (cd build > /dev/null 2>&1 && cmake .. > /dev/null 2>&1)
	@- $(MAKE) --silent -C build clean || true
	@- $(RM) ./build/Makefile
	@- $(RM) ./build/src
	@- $(RM) ./build/test
	@- $(RM) ./build/CMake*
	@- $(RM) ./build/cmake.*
	@- $(RM) ./build/*.cmake
	@- $(RM) ./build/*.txt

clean:
	@ (cd build && $(MAKE) clean)

cleanpc:
	@ (cd buildpc && $(MAKE) clean)




