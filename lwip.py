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

def build(bld):
    arch_lib_path = rtems.arch_bsp_lib_path(bld.env.RTEMS_VERSION,
                                            bld.env.RTEMS_ARCH_BSP)
    with open('file-import.yaml', 'r') as cf:
        files = yaml.full_load(cf.read())
        for f in files['files-to-import']:
            if f[-2:] == '.c':
                source_files.append(os.path.join('./lwip', f))
        
        for f in os.listdir('./lwip/ports/drivers'):
            if f[-2:] == '.c':
                driver_source.append(os.path.join('./lwip/ports/drivers', f))
    source_files.append('./lwip/ports/os/rtems/arch/sys_arch.c')
    
    #source_files.append('./lwip/ports/port/sys_arch.c')
    #source_files.append('./lwip/ports/port/perf.c')
    #source_files.append('./lwip/ports/port/netif/fifo.c')
    #source_files.append('./lwip/ports/port/netif/list.c')
    #source_files.append('./lwip/ports/port/netif/pcapif.c')
    #source_files.append('./lwip/ports/port/netif/sio.c')
    #source_files.append('./lwip/ports/port/netif/tapif.c')

    bld(features ='c',
        target='lwip_obj',
        includes='./include ./lwip/src/include lwip/ports/port/include ./lwip/ports/os ./lwip/ports/drivers ./lwip/ports/os/rtems',
        source=source_files,
        )
#    bld(features ='c',
#        target='lwip_drv_obj',
#        includes='./ ../ ../../src/include ./include ./lwip/src/include ./lwip/ports/os/rtems ./lwip/ports/os ./lwip/ports/drivers ' + os.path.relpath(os.path.join(arch_lib_path,'include')),
#        source=driver_source,
#        )
    bld(features='c cstlib',
        target = 'lwip',
        use=['lwip_obj'])

    bld.program(features='c',
                target='test01.exe',
                source='./lwip/test/sample_app.c',
                use='lwip',
                lib=['rtemscpu', 'rtemsbsp', 'rtemstest', 'lwip'],
                includes='./include ./lwip/ports/os/rtems ./lwip/ports/os ./lwip/src/include ./lwip/ports/os/rtems ./lwip/test/' + os.path.relpath(os.path.join(arch_lib_path,'include')))
