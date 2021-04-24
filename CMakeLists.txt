#################################################################################
# CMake support for RTEMS lwIP
#
# Author: R.Mueller
#################################################################################
cmake_minimum_required(VERSION 3.13)

#################################################################################
# lwIP options
#################################################################################

option(LWIP_GENERATE_SECTIONS "Generate function and data sections" ON)
if(LWIP_GENERATE_SECTIONS)
    option(LWIP_REMOVE_UNUSED_CODE "Remove unused sections" ON)
endif()

option(LWIP_ADD_IPV6 "Compile IPv6 sourcces" ON)

#################################################################################
# Port select
#################################################################################

if(NOT RTEMS_BSP)
    message(
        FATAL_ERROR "RTEMS BSP variable not set. "
        "Please make sure to pass it with -DRTEMS_BSP=<arch/bsp>"
   )
endif()

set(LWIP_CMAKE_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include("${LWIP_CMAKE_CONFIG_DIR}/PortSelect.cmake")

rtems_lwip_port_select(${RTEMS_BSP})

set(LWIP_LIB_NAME lwip)
add_library(${LWIP_LIB_NAME})

add_subdirectory(lwip)

# The user needs to supply the lwipopts.h file. The user can do this by supplying the
# path of this file in the LWIP_CONFIG_PATH variable
if(NOT LWIP_CONFIG_PATH)
	message(WARNING "lwIP configuration include path LWIP_CONFIG_PATH not set!")
endif()

if(IS_ABSOLUTE ${LWIP_CONFIG_PATH})
	set(LWIP_CONFIG_PATH_ABS "${LWIP_CONFIG_PATH}")
else()
	get_filename_component(LWIP_CONFIG_PATH_ABS
		${LWIP_CONFIG_PATH} REALPATH BASE_DIR ${CMAKE_SOURCE_DIR}
	)
endif()

if(CMAKE_VERBOSE)
	message(STATUS "lwIP configuration path: ${LWIP_CONFIG_PATH}")
endif()

target_include_directories(${LWIP_LIB_NAME} PRIVATE
	${LWIP_CONFIG_PATH_ABS}
)

# Remove unused sections
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    if(LWIP_GENERATE_SECTIONS)
        target_compile_options(${LWIP_LIB_NAME} PRIVATE
            "-ffunction-sections"
            "-fdata-sections"
        )
    endif()

    if(LWIP_REMOVE_UNUSED_CODE)
        target_link_options(${LWIP_LIB_NAME} PRIVATE
            "Wl,--gc-sections"
        )
    endif()
endif()
