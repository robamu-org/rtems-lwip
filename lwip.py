#
# RTEMS Project (https://www.rtems.org/)
#
# Copyright (c) 2021 Vijay Kumar Banerjee <vijay@rtems.org>.
# All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from rtems_waf import rtems
import yaml
import os

source_files = []
driver_source = []
include_files = {}

DEFAULT_OPTIONS_PATH = "./lwip/ports/os/rtems/defaults"
DEBUG_LWIP_WSCRIPT = False
BUILD_SAMPLE_FILE = False
RTEMS_LWIP_PORT_INC_FOLDER_NAME = "rtems_lwip"



def options(opt):
    opt.add_option(
        "--lwip-opts",
        default=DEFAULT_OPTIONS_PATH,
        dest="lwipopts_path",
        help="Path including the lwipopts.h file. If none is supplied, a default "
             " configuration file will be used"
    )


def configure(conf):
    conf.env.LWIPOPTS_PATH = conf.options.lwipopts_path
    if(conf.env.LWIPOPTS_PATH == DEFAULT_OPTIONS_PATH):
        print('No path for lwipopts.h file supplied, using default path %s' % DEFAULT_OPTIONS_PATH)
    else:
        print('Path(s) for lwIP options supplied: %s' % conf.env.LWIPOPTS_PATH)


def build(bld):
    arch_lib_path = rtems.arch_bsp_lib_path(
        bld.env.RTEMS_VERSION, bld.env.RTEMS_ARCH_BSP
    )
    arch_inc_path = rtems.arch_bsp_include_path(
        bld.env.RTEMS_VERSION, bld.env.RTEMS_ARCH_BSP
    )

    with open('file-import.yaml', 'r') as cf:
        files = yaml.full_load(cf.read())
        for f in files['files-to-import']:
            if f[-2:] == '.c':
                source_files.append(os.path.join('./lwip', f))

    # Determine port sources and include path, if there are any
    port_name = ""
    generic_port_path = './lwip/ports/drivers'
    if bld.env.RTEMS_BSP in ["nucleo-h743zi", "stm32h7"]:
        port_name = "stm32h7"
    elif "tms570" in bld.env.RTEMS_BSP:
        port_name = "tms570"
    driver_include_path = ""
    if port_name != "":
        port_path = './lwip/ports/drivers/%s' % port_name
        driver_include_path = os.path.join(
            port_path, 'include/%s' % RTEMS_LWIP_PORT_INC_FOLDER_NAME
        )
        for f in os.listdir(port_path):
            if f[-2:] == '.c':
                driver_source.append(os.path.join(port_path, f))
    else:
        print("Port does not exist. Only compiling lwIP source files")

    if DEBUG_LWIP_WSCRIPT:
        print('Port sources: ')
        print(driver_source)

    os_inc_path = './lwip/ports/os/rtems/arch'

    source_files.append('./lwip/ports/os/rtems/arch/sys_arch.c')
    source_files.append('./lwip/ports/drivers/rtems_lwip.c')
    source_files.append(driver_source)

    # Add the path containing the lwipopts.h file
    options_include_path = bld.env.LWIPOPTS_PATH

    if(options_include_path == ""):
      print("Invalid options path, setting default path")
      options_include_path = DEFAULT_OPTIONS_PATH

    lwip_core_include_path = './lwip/src/include'
    lwip_obj_includes = \
        './include %s ./lwip/src/include lwip/ports/port/include ./lwip/ports/os ' \
        './lwip/ports/drivers ./lwip/ports/os/rtems %s %s' \
        % (lwip_core_include_path, options_include_path, driver_include_path)

    if DEBUG_LWIP_WSCRIPT:
        print('Include paths for lwIP: ')
        print(lwip_obj_includes)

    bld(
        features='c',
        target='lwip_obj',
        includes=lwip_obj_includes,
        source=source_files
    )

    bld(
        features='c cstlib',
        target='lwip',
        use=['lwip_obj']
     )

    # Install static library
    bld.install_files(os.path.join('${PREFIX}', arch_lib_path), ["liblwip.a"])

    install_header_source = []
    install_header_target = []

    # Installation for the lwIP core header files
    for root, dirnames, filenames in os.walk(lwip_core_include_path):
        install_headers_source = [
            os.path.join(root, file)
            for file in filenames
            if (file[-2:] == '.h')
        ]
        install_headers_target = os.path.relpath(root, lwip_core_include_path)
        install_destination = os.path.join(bld.env.PREFIX, arch_inc_path, install_headers_target)
        if DEBUG_LWIP_WSCRIPT:
            print('Install Source: %s' % install_headers_source)
        if DEBUG_LWIP_WSCRIPT:
            print('Install Target: %s' % install_destination)
        bld.install_files(install_destination, install_headers_source)

    # Installation for the lwIP port header files
    generic_port_include_files = os.listdir(generic_port_path)
    install_headers_source = [
        os.path.join(generic_port_path, file)
        for file in generic_port_include_files
        if (file[-2:] == '.h')
    ]
    if driver_include_path != "":
        drv_include_files = os.listdir(driver_include_path)
        install_headers_source += [
            os.path.join(driver_include_path, file)
            for file in drv_include_files
            if (file[-2:] == '.h')
        ]
        install_destination = os.path.join(bld.env.PREFIX, arch_inc_path, 'rtems_lwip')
        bld.install_files(install_destination, install_headers_source)

    # Installation for OS abstraction layer
    os_include_files = os.listdir(os_inc_path)
    install_headers_source = [
        os.path.join(os_inc_path, file)
        for file in os_include_files
        if (file[-2:] == '.h')
    ]
    install_destination = os.path.join(bld.env.PREFIX, arch_inc_path, 'arch')
    bld.install_files(install_destination,  install_headers_source)

    if BUILD_SAMPLE_FILE:
        sample_inc = \
            './include ./lwip/ports/os/rtems ./lwip/ports/os ./lwip/src/include ' \
            './lwip/ports/os/rtems ./lwip/test/' \
            + os.path.relpath(os.path.join(arch_lib_path,'include'))
        bld.program(
            features='c',
            target='test01.exe',
            source='./lwip/test/sample_app.c',
            use='lwip',
            lib=['rtemscpu', 'rtemsbsp', 'rtemstest', 'lwip'],
            includes=sample_inc
        )

