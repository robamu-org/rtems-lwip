RTEMS lwIP Repository
======

This repository contains the RTEMS lwIP support and all available ports.
Currently, the following RTEMS BSPs are supported:

 - All `arm/tms*` BSPs.
 - The `arm/stm32h7` and the `arm/nucleo-h743zi` BSP. An example application can be
   found [here](https://github.com/rmspacefish/rtems-stm32-lwip).

# Using this repository

Start by cloning the submodules

```sh
git submodule init
git submodule update
```

The repository contains the lwIP sources, the OS port for RTEMS and port files
belonging to a RTEMS BSP.

It is recommended that the user supplies the `lwipopts.h` configuration file. The port folders
contain template option files to get started. The user can copy and rename this files into the
application and then pass the include path to the build system using the `--lwip-opts` option.

# Building with waf

It is assumed that the path(s) containing the `lwipopts.h` file was stored in the environmental
variable `LWIP_OPTS_PATH`, the install prefix is stored in the environmental
variable `RTEMS_PREFIX` and the RTEMS BSP is stored inside `RTEMS_BSP`.
The lwIP support can now be built like this:

```sh
./waf configure --prefix="$RTEMS_PREFIX" --lwip-opts="$LWIP_OPTS_PATH" --rtems-bsp="$RTEMS_BSP"
./waf
./waf install
```
