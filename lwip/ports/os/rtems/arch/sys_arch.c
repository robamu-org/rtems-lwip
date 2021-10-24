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

/* lwIP includes. */
#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/sys.h"
#include "lwip/mem.h"
#include "lwip/stats.h"

#if !NO_SYS

#include "sys_arch.h"
#include <stdint.h>
#include <arch/cc.h>
#include <rtems/rtems/clock.h>
#include <rtems/rtems/sem.h>
#include <rtems.h>

#define SYS_LWIP_MBOX_SIZE (sizeof(void *))

uint32_t
sys_now()
{
  uint64_t temp = rtems_clock_get_uptime_nanoseconds() / (1000 * 1000);

  return temp;
}

void
sys_init(void)
{
  //  Is called to initialize the sys_arch layer.
  return;
}

err_t
sys_sem_new(sys_sem_t *sem, u8_t count)
{
  rtems_status_code ret = rtems_semaphore_create(
    rtems_build_name('L', 'W', 'I', 'P'),
    count,
    RTEMS_COUNTING_SEMAPHORE,
    0,
    &sem->semaphore
    );

  if (ret != RTEMS_SUCCESSFUL) {
    sem->semaphore = RTEMS_ID_NONE;
    return ret;
  }
  return ERR_OK;
}


void
sys_sem_free(sys_sem_t *sem)
{
  rtems_semaphore_delete(
    sem->semaphore
    );
  sem->semaphore = RTEMS_ID_NONE;
}

void
sys_sem_signal(sys_sem_t *sem)
{
  rtems_semaphore_release(sem->semaphore);
}

void
sys_sem_signal_from_ISR(sys_sem_t *sem)
{
  rtems_semaphore_release(sem->semaphore);
}


u32_t
sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
  rtems_status_code status;
  rtems_interval tps = rtems_clock_get_ticks_per_second();
  rtems_interval tick_timeout;
  uint64_t       start_time;
  uint64_t       wait_time;

  start_time = rtems_clock_get_uptime_nanoseconds();
  if (timeout == 0) {
    tick_timeout = RTEMS_NO_TIMEOUT;
  } else {
    tick_timeout = (timeout * tps + 999) / 1000;
  }
  status = rtems_semaphore_obtain(sem->semaphore, RTEMS_WAIT, tick_timeout);
  if (status == RTEMS_TIMEOUT) {
    return SYS_ARCH_TIMEOUT;
  }
  if (status != RTEMS_SUCCESSFUL) {
    return SYS_ARCH_TIMEOUT;
  }
  wait_time = rtems_clock_get_uptime_nanoseconds() - start_time;
  return wait_time / (1000 * 1000);
}

int
sys_sem_valid(sys_sem_t *sem)
{
  return sem->semaphore == RTEMS_ID_NONE ? 0 : 1;
}

void
sys_sem_set_invalid(sys_sem_t *sem)
{
  sem->semaphore = RTEMS_ID_NONE;
}

err_t
sys_mbox_new(sys_mbox_t *mbox, int size)
{
  rtems_status_code ret;

  ret = rtems_message_queue_create(
    rtems_build_name('L', 'W', 'I', 'P'),
    size,
    SYS_LWIP_MBOX_SIZE,
    0,
    &mbox->mailbox
    );
  ret |= rtems_semaphore_create(
    rtems_build_name('L', 'W', 'I', 'P'),
    size,
    RTEMS_COUNTING_SEMAPHORE,
    0,
    &mbox->sem
    );
  if (ret != RTEMS_SUCCESSFUL) {
    mbox->mailbox = RTEMS_ID_NONE;
    mbox->sem = RTEMS_ID_NONE;
    return ret;
  }
  return ERR_OK;
}

void
sys_mbox_free(sys_mbox_t *mbox)
{
  rtems_message_queue_delete(mbox->mailbox);
  rtems_semaphore_delete(mbox->sem);
  sys_mbox_set_invalid(mbox);
}

void
sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
  rtems_semaphore_obtain(mbox->sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  rtems_message_queue_send(mbox->mailbox, &msg, SYS_LWIP_MBOX_SIZE);
}
err_t
sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
  rtems_status_code status = rtems_semaphore_obtain(mbox->sem,
						    RTEMS_NO_WAIT, 0);

  if (status != RTEMS_SUCCESSFUL) {
    return ERR_MEM;
  } else {
    rtems_message_queue_send(mbox->mailbox, &msg, SYS_LWIP_MBOX_SIZE);
    return ERR_OK;
  }
}

u32_t
sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
  rtems_status_code status;
  rtems_interval tps = rtems_clock_get_ticks_per_second();
  rtems_interval tick_timeout;
  uint64_t       start_time;
  uint64_t       wait_time;
  size_t         dummy;

  start_time = rtems_clock_get_uptime_nanoseconds();
  if (timeout == 0) {
    tick_timeout = RTEMS_NO_TIMEOUT;
  } else {
    tick_timeout = (timeout * tps + 999) / 1000;
  }
  status = rtems_message_queue_receive(mbox->mailbox,
				       msg,
				       &dummy,
				       RTEMS_WAIT,
				       tick_timeout
				       );
  if (status == RTEMS_TIMEOUT) {
    return SYS_ARCH_TIMEOUT;
  }
  if (status != RTEMS_SUCCESSFUL) {
    return SYS_ARCH_TIMEOUT;
  }
  wait_time = rtems_clock_get_uptime_nanoseconds() - start_time;
  rtems_semaphore_release(mbox->sem);
  return wait_time / (1000 * 1000);
}

u32_t
sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
  rtems_status_code status;
  size_t         dummy;

  status = rtems_message_queue_receive(mbox->mailbox, msg,
				       &dummy,
				       RTEMS_NO_WAIT,
				       0
				       );

  if (status != RTEMS_SUCCESSFUL) {
    return SYS_MBOX_EMPTY;
  } else {
    rtems_semaphore_release(mbox->sem);
    return 0;
  }
}
int
sys_mbox_valid(sys_mbox_t *mbox)
{
  return mbox->mailbox == RTEMS_ID_NONE ? 0 : 1;
}
void
sys_mbox_set_invalid(sys_mbox_t *mbox)
{
  mbox->sem = RTEMS_ID_NONE;
  mbox->mailbox = RTEMS_ID_NONE;
}

sys_thread_t
sys_thread_new(const char *name, lwip_thread_fn function, void *arg, int stack_size, int prio)
{
  rtems_id id;
  rtems_status_code res;

  rtems_name lwip_task_name = 0;
  if(name == NULL) {
    lwip_task_name = rtems_build_name('L', 'W', 'I', 'P');
  }
  else {
    if(name[0] == '\0') {
      lwip_task_name = rtems_build_name('L', 'W', 'I', 'P');
    }
    else {
      lwip_task_name = rtems_build_name(name[0], name[1], name[2], name[3]);
    }
  }

  res = rtems_task_create(
    lwip_task_name,
    prio,
    stack_size,
    RTEMS_PREEMPT,
    0,
    &id
    );

  if (res != RTEMS_SUCCESSFUL) {
    return 0;
  }

  res = rtems_task_start(id, (rtems_task_entry)function, (rtems_task_argument)arg);

  if (res != RTEMS_SUCCESSFUL) {
    rtems_task_delete(id);
    return 0;
  }
  return id;
}

err_t
sys_mutex_new(sys_mutex_t *mutex)
{
  rtems_status_code ret = rtems_semaphore_create(
    rtems_build_name('L', 'W', 'I', 'P'),
    1,
    RTEMS_PRIORITY|RTEMS_BINARY_SEMAPHORE|RTEMS_INHERIT_PRIORITY|RTEMS_LOCAL,
    0,
    &mutex->mutex
    );

  if (ret != RTEMS_SUCCESSFUL) {
    mutex->mutex = RTEMS_ID_NONE;
    return ret;
  }
  return ERR_OK;
}
/** Lock a mutex
 * @param mutex the mutex to lock */
void
sys_mutex_lock(sys_mutex_t *mutex)
{
  rtems_semaphore_obtain(mutex->mutex, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
}
/** Unlock a mutex
 * @param mutex the mutex to unlock */
void
sys_mutex_unlock(sys_mutex_t *mutex)
{
  rtems_semaphore_release(mutex->mutex);
}
/** Delete a semaphore
 * @param mutex the mutex to delete */
void
sys_mutex_free(sys_mutex_t *mutex)
{
  rtems_semaphore_delete(mutex->mutex);
}

void
sys_arch_delay(unsigned int timeout)
{
  rtems_interval tps = rtems_clock_get_ticks_per_second();
  rtems_interval tick_timeout = (timeout * tps + 999) / 1000;

  rtems_task_wake_after(tick_timeout);
}

/** Ticks/jiffies since power up. */
uint32_t
sys_jiffies(void)
{
  return rtems_clock_get_ticks_since_boot();
}

int
sys_request_irq(unsigned int irqnum, sys_irq_handler_t handler,
		unsigned long flags, const char *name, void *context)
{
  rtems_status_code res;

  res = rtems_interrupt_handler_install(irqnum,  name, flags,
					handler, context);
  return (res != RTEMS_SUCCESSFUL) ? -1 : 0;
}

sys_prot_t
sys_arch_protect()
{
  sys_prot_t pval;

  rtems_interrupt_disable(pval);
  return pval;
}

void
sys_arch_unprotect(sys_prot_t pval)
{
  rtems_interrupt_enable(pval);
}
err_t
sys_mbox_trypost_fromisr(sys_mbox_t *q, void *msg)
{
  return sys_mbox_trypost(q, msg);
}

#endif /* NO_SYS == 0 */

