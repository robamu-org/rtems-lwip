/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is system adaptation of the lwIP TCP/IP stack
 * by Adam Dunkels <adam@sics.se> for RTEMS system.
 *
 * Author: Premysl Houdek <houdepre@fel.cvut.cz>
 * Mentor: Pavel Pisa <pisa@cmp.felk.cvut.cz>
 * Industrial Informatics Group, FEE, Czech Technical University in Prague
 *
 */
/*
 * settings to adapt lwIP for compiler and machine architecture for RTEMS/GCC
 * DETAILS: ./lwip/doc/sys_arch.txt
 */
#ifndef __CC_H__
#define __CC_H__

/* Use timeval provided by RTEMS be defining this to 0 */
#define LWIP_TIMEVAL_PRIVATE 0

#include <stdio.h>
#include <rtems/malloc.h>  /*printk*/
#include <inttypes.h>
#include <malloc.h>
#include <sys/time.h>
#include <rtems.h>
#include <sys/errno.h>
#include <endian.h>
#include <stdlib.h>
#include <stdint.h>

#include <sys/uio.h> /*struct iovec*/

#ifndef iovec
#define iovec iovec
#endif

/* This file must either include a system-local <errno.h> which defines
   the standard *nix error codes, or it should #define LWIP_PROVIDE_ERRNO
   to make lwip/arch.h define the codes which are used throughout. */
#undef LWIP_PROVIDE_ERRNO

#if !defined(LWIP_NO_STDINT_H)
#define LWIP_NO_STDINT_H 0
#endif

#if LWIP_NO_STDINT_H
/* type definitions */
typedef uint8_t             u8_t;
typedef int8_t              s8_t;
typedef uint16_t            u16_t;
typedef int16_t             s16_t;
typedef uint32_t            u32_t;
typedef int32_t             s32_t;
typedef u32_t               mem_ptr_t;

#endif /*LWIP_NO_STDINT_H*/

#if !defined(LWIP_NO_INTTYPES_H)
#define LWIP_NO_INTTYPES_H 0
#endif

#if LWIP_NO_INTTYPES_H
/* Define (sn)printf formatters for these lwIP types */
#define U16_F PRIu16
#define S16_F PRId16
#define X16_F PRIx16
#define U32_F PRIu32
#define S32_F PRId32
#define X32_F PRIx32
#endif /*LWIP_NO_INTTYPES_H*/

#if defined(__arm__) && defined(__ARMCC_VERSION)
//
// Setup PACKing macros for KEIL/RVMDK Tools
//
    #define PACK_STRUCT_BEGIN __packed
    #define PACK_STRUCT_STRUCT
    #define PACK_STRUCT_END
    #define PACK_STRUCT_FIELD(x) x
#elif defined (__IAR_SYSTEMS_ICC__)
//
// Setup PACKing macros for IAR Tools
//
    #define PACK_STRUCT_BEGIN
    #define PACK_STRUCT_STRUCT
    #define PACK_STRUCT_END
    #define PACK_STRUCT_FIELD(x) x
    #define PACK_STRUCT_USE_INCLUDES
#elif defined (__TMS470__)
    #define PACK_STRUCT_BEGIN
    #define PACK_STRUCT_STRUCT
    #define PACK_STRUCT_END
    #define PACK_STRUCT_FIELD(x) x
#else
//
// Setup PACKing macros for GCC Tools
//
    #define PACK_STRUCT_BEGIN
    #define PACK_STRUCT_STRUCT __attribute__ ((__packed__))
    #define PACK_STRUCT_END
    #define PACK_STRUCT_FIELD(x) x
#endif

/*
 *     1 - load byte by byte, construct 16 bits word and add: not efficient for most platforms
 *     2 - load first byte if odd address, loop processing 16 bits words, add last byte.
 *     3 - load first byte and word if not 4 byte aligned, loop processing 32 bits words, add last word/byte.
 *
 *     see inet_chksum.c
 */
#ifndef LWIP_CHKSUM_ALGORITHM
#define LWIP_CHKSUM_ALGORITHM 2
#endif

/* this is used for 1) displaying statistics and 2) lwip debugging (set appropriate debugging level in lwipopts.h) */
//#ifdef LWIP_DEBUG


#define LWIP_PLATFORM_DIAG(expr)        printk expr

//#else
//#define LWIP_PLATFORM_DIAG(expr)
//#endif

//#define DEBUG
#ifdef DEBUG

/* for passing arguments to print function */
#define CC_ASSERT(message, assertion) do { if (!(assertion)) \
					     LWIP_PLATFORM_DIAG(message); } while (0)

//extern void __error__(char *pcFilename, unsigned long ulLine);
#define LWIP_PLATFORM_ASSERT(expr)      printk((const char *)expr)
/*
{                                       \
    if(!(expr))                         \
    {                                   \
        __error__(__FILE__, __LINE__);  \
    }                                   \
}
*/
#else
#define LWIP_PLATFORM_ASSERT(expr)
#define CC_ASSERT(message, assertion)
#endif /* DEBUG */

/* "lightweight" synchronization mechanisms */
/* #define SYS_ARCH_DECL_PROTECT(x) */ /* declare a protection state variable */
/* #define SYS_ARCH_PROTECT(x) */ /* enter protection mode */
/* #define SYS_ARCH_UNPROTECT(x) */ /* leave protection mode */

/* 32-bit random value used by igmp and others */
#define LWIP_RAND() ((uint32_t)random())

#endif /* __CC_H__ */
