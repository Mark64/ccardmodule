# by Mark Hill
# using tutorial from https://www.johnlamp.net/cmake-tutorial-4-libraries-and-subdirectories.html

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


project("Clean CCard Userspace Builds")

# for building as a library
set(USERSPACE_LIBRARY_NAME ccard)
# this file has the userspace implementation of the
#   log function and other library-build-specific functions
set(USERSPACE_LIBRARY_MAIN_FILE ccardcore/ccardlibrary.c)

# for building as a userspace test executable
set(USERSPACE_TEST_NAME ccardtest)
set(USERSPACE_TEST_MAIN_FILE ccardcore/ccardlibrary.c)



# language settings
#   enable C99 mode
macro(use_c99)
	if (CMAKE_VERSION VERSION_LESS "3.1")
		if (CMAKE_C_COMPILER_ID STREQUAL "GNU")
			set(CMAKE_C_FLAGS "--std=gnu99 ${CMAKE_C_FLAGS}")
		endif()
	else()
		set (CMAKE_C_STANDARD 99)
	endif()
endmacro(use_c99)

use_c99()

# stop the stupid insource builds
if ( ${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR} )
	message(FATAL_ERROR "In-source builds not allowed, use the Makefile wrapper")
endif()



include_directories("${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/ccardcore")
# runs CMake on the subdirectory ccard core, where
#   the implementation is stored
add_subdirectory(ccardcore)


# create userspace dynamic library
add_library(${USERSPACE_LIBRARY_NAME} SHARED ${USERSPACE_LIBRARY_MAIN_FILE})
target_link_libraries(${USERSPACE_LIBRARY_NAME} ccardcore)

# create userspace test executable
add_executable(${USERSPACE_TEST_NAME} ${USERSPACE_TEST_MAIN_FILE})
target_link_libraries(${USERSPACE_TEST_NAME} ccardcore)



# threading section
find_package(Threads REQUIRED)
if(THEADS_HAVE_PTHREAD_ARG)
	target_compile_options(PUBLIC ${USERSPACE_LIBRARY_NAME} "-pthread")
	target_compile_options(PUBLIC ${USERSPACE_TEST_NAME} "-pthread")
endif()
if (CMAKE_THREAD_LIBS_INIT)
	target_link_libraries(${USERSPACE_LIBRARY_NAME} ${CMAKE_THREAD_LIBS_INIT})
	target_link_libraries(${USERSPACE_TEST_NAME} ${CMAKE_THREAD_LIBS_INIT})
endif()


set(CONFIGURED_ONCE TRUE CACHE INTERNAL "Apparently this shows CMake has been configured once ????")



