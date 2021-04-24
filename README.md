RTEMS lwIP Repository
======

This repository contains the RTEMS lwIP support and all available ports.
Currently, the following RTEMS BSPs are supported:

 - All `arm/tms*` BSPs
 - The `arm/stm32h7` and the `arm/nucleo-h743zi` BSP. An example application can be 
   found [here](https://github.com/rmspacefish/rtems-stm32-lwip)
 
# Using this repository

The repostiory contains the lwIP sources, the OS port for RTEMS and port files
belonging to a RTEMS BSP.

The user still needs to supply the `lwipopts.h` configuration file. The port folders contain
template option files to get started. The user can copy and rename this files into the application
and then pass the include path to the build system.

# Compiling with CMake

It is recommended to add this repository as a submodule. After that, you can use
`add_subdirectory` to add the repository and then link against the `lwip` target.

Following steps have to be taken by the application `CMakeLists.txt` for this to work:

 1. The CMake variable `RTEMS_BSP` has to be set to the used BSP
 2. The CMake variable `LWIP_CONFIG_PATH` needs to be set to the path including the `lwipopts.h`
    configuration file which is provided by the user.

# Compiling with waf

