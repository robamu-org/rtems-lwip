#! /usr/bin/env python
#
#  Copyright (c) 2021 Vijay Kumar Banerjee <vijay@rtems.org>.
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

from __future__ import print_function

import os
import sys
import getopt
import yaml
from shutil import copyfile
from pathlib import Path

isForward = True
isEarlyExit = False
statsReport = False
LWIP_DIR = 'lwip'
LWIP_UPSTREAM_DIR = './lwip-upstream'

def usage():
    print("lwip-to-rtems.py [args]")
    print("  -?|-h|--help      print this and exit")
    print("  -R|--reverse      default origin -> lwip, reverse that")
    print("  -r|--rtems        lwip directory (default: '.')")
    print("  -l|--lwip         lwip upstream directory (default: 'lwip-upstream')")

# Parse the arguments
def parseArguments():
    global isForward, isEarlyExit, statsReport
    try:
        opts, args = getopt.getopt(sys.argv[1:],
                                   "?hdDembSRr:f:cv",
                                   [ "help",
                                     "help",
                                     "reverse"
                                     "stats"
                                     "rtems="
                                     "lwip="
                                     "verbose" ])
    except getopt.GetoptError as err:
        # print help information and exit:
        print(str(err)) # will print something like "option -a not recognized"
        usage()
        sys.exit(2)
    for o, a in opts:
        if o in ("-h", "--help", "-?"):
            usage()
            sys.exit()
        elif o in ("-S", "--stats"):
            statsReport = True
        elif o in ("-R", "--reverse"):
            isForward = False
        elif o in ("-r", "--rtems"):
            LWIP_DIR = a
        elif o in ("-l", "--lwip"):
            LWIP_UPSTREAM_DIR = a
        else:
            assert False, "unhandled option"

parseArguments()

print("lwip Directory:            %s" % (LWIP_DIR))
print("lwip upstream Directory:           %s" % (LWIP_UPSTREAM_DIR))
print("Direction:                   %s" % (("reverse", "forward")[isForward]))


def copyFiles(isforward):
    if (isforward):
        config_file = open('file-import.yaml', 'r').read()
        files = yaml.full_load(config_file)
        src_dir = os.path.abspath(LWIP_UPSTREAM_DIR)
        print("Files Imported:")
        for f in files['files-to-import']:
            dst_dir = os.path.join(os.path.abspath(LWIP_DIR), *f.split('/')[:-1])
            dst_file = os.path.join(dst_dir, f.split('/')[-1])
            if not os.path.exists(dst_dir):
                os.makedirs(dst_dir)
            if not os.path.exists(dst_file):
                Path(dst_file).touch()
                print(dst_file)
                copyfile(str(os.path.join(src_dir, f)),
                         str(dst_file))

copyFiles(isForward)
