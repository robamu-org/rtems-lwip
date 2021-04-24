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
 * mapping of lwIP system dependencies to RTEMS system services and types.
 * DETAILS: ./lwip/doc/sys_arch.txt
 */
#ifndef __ARCH_SYS_ARCH_H__
#define __ARCH_SYS_ARCH_H__

#include "lwip/opt.h"

#if (NO_SYS != 0)
#error "RTEMS SYS_ARCH cannot be compiled in NO_SYS variant"
#endif

#include <rtems/rtems/sem.h>
#include <rtems/rtems/intr.h>
#include <rtems/score/cpu.h>
#include <bsp/irq-generic.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Typedefs for the various port-specific types. */

#define sys_arch_printk printk

typedef struct {
  rtems_id mailbox;
  rtems_id sem;
} port_mailbox_t;

typedef struct {
  rtems_id semaphore;
} port_sem_t;

typedef struct {
  rtems_id mutex;
} port_mutex_t;

typedef port_mailbox_t sys_mbox_t;
typedef port_sem_t sys_sem_t;
typedef rtems_id sys_thread_t;
typedef port_mutex_t sys_mutex_t;
typedef rtems_interrupt_level sys_prot_t;

void
sys_arch_delay(unsigned int x);
void
sys_sem_signal_from_ISR(sys_sem_t *sem);

typedef void sys_irqreturn_t;
#define SYS_IRQ_NONE       ((void)0)
#define SYS_IRQ_HANDLED    ((void)1)
#define SYS_IRQ_RETVAL(x)  (IRQ_HANDLED)
typedef rtems_interrupt_handler sys_irq_handler_t;
#define SYS_IRQ_HANDLER_FNC(M_fnc_name)	\
  sys_irqreturn_t M_fnc_name(void *__irq_handler_context)
#define sys_irq_handler_get_context() (__irq_handler_context)

int
sys_request_irq(unsigned int irqnum, sys_irq_handler_t handler,
		unsigned long flags, const char *name, void *context);

static inline void
sys_arch_mask_interrupt_source(unsigned int x)
{
  bsp_interrupt_vector_disable(x);
}

static inline void
sys_arch_unmask_interrupt_source(unsigned int x)
{
  bsp_interrupt_vector_enable(x);
}

sys_prot_t sys_arch_protect();

void sys_arch_unprotect(sys_prot_t pval);

#ifdef __cplusplus
}
#endif

#endif /* __ARCH_SYS_ARCH_H__ */
