cmake_minimum_required(VERSION 3.0.0 FATAL_ERROR)

# Ref:
# https://gitlab.com/christophacham/cmake-kernel-module
# https://gitioc.upc.edu/open-source/xenomai-cmake/blob/master/cmake-modules/FindKernelHeaders.cmake
# https://stackoverflow.com/questions/50877135/cmake-specify-linux-kernel-module-output-build-directory

project(virt_serial VERSION 0.1.0 LANGUAGES C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Module info
add_definitions(-D__KERNEL__ -DMODULE)

# Find the kernel release
execute_process(
        COMMAND uname -r
        OUTPUT_VARIABLE KERNEL_RELEASE
        OUTPUT_STRIP_TRAILING_WHITESPACE
)

string(REGEX REPLACE "-amd64$" "-common" KERNEL_RELEASE_V "${KERNEL_RELEASE}")

# Find the headers
find_path(
        KERNELHEADERS_DIR
        include/linux/user.h
        PATHS /usr/src/linux-headers-${KERNEL_RELEASE_2}
)

set(KERNELHEADERS_DIR /lib/modules/${KERNEL_RELEASE}/build)

message(STATUS "Kernel release: ${KERNEL_RELEASE}")
message(STATUS "Kernel headers: ${KERNELHEADERS_DIR}")


# Add header files
include_directories(
        # ${KERNELHEADERS_DIR}/include
        /usr/src/linux-headers-${KERNEL_RELEASE}/include
        /usr/src/linux-headers-${KERNEL_RELEASE_V}/include
        /usr/src/linux-headers-${KERNEL_RELEASE_V}/arch/x86/include
)

set(DRIVER_FILE virt_serial.ko)
set(KBUILD_CMD make -C ${KERNELHEADERS_DIR} modules M=${CMAKE_CURRENT_BINARY_DIR} src=${CMAKE_CURRENT_SOURCE_DIR})

# Generate the Kbuild file through cmake.
FILE(WRITE ${CMAKE_CURRENT_SOURCE_DIR}/Kbuild "obj-m := virt_serial.o")

add_custom_command(OUTPUT ${DRIVER_FILE}
        COMMAND ${KBUILD_CMD}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        DEPENDS virt_serial.c VERBATIM)

add_custom_target(driver ALL DEPENDS ${DRIVER_FILE})
# Parse c files
add_library(dummy_target virt_serial.c)