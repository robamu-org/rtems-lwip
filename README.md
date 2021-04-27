RTEMS lwIP Repository
======

This repository contains the RTEMS lwIP support and all available ports.
Currently, the following RTEMS BSPs are supported:

 - All `arm/tms*` BSPs.
 - The `arm/stm32h7` and the `arm/nucleo-h743zi` BSP. An example application can be 
   found [here](https://github.com/rmspacefish/rtems-stm32-lwip).
 
# Using this repository

The repository contains the lwIP sources, the OS port for RTEMS and port files
belonging to a RTEMS BSP.

The user still needs to supply the `lwipopts.h` configuration file. The port folders contain
template option files to get started. The user can copy and rename this files into the application
and then pass the include path to the build system.

# Compiling with CMake

# Compiling with waf

