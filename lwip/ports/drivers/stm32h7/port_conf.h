#ifndef RTEMS_LWIP_LWIP_PORTS_DRIVERS_STM32H7_RTEMS_CONF_H_
#define RTEMS_LWIP_LWIP_PORTS_DRIVERS_STM32H7_RTEMS_CONF_H_

#include "lwipopts.h"

/* RTEMS priorities range from 1 (highest) to 255 (lowest) */
#ifndef RTEMS_LWIP_INTERFACE_THREAD_PRIORITY
#define RTEMS_LWIP_INTERFACE_THREAD_PRIORITY    ( 50 )
#endif

/* Stack size of the interface thread */
#ifndef RTEMS_LWIP_INTERFACE_THREAD_STACK_SIZE
#define RTEMS_LWIP_INTERFACE_THREAD_STACK_SIZE  ( RTEMS_MINIMUM_STACK_SIZE )
#endif

#ifndef RTEMS_LWIP_ETH_DMA_TRANSMIT_TIMEOUT
#define RTEMS_LWIP_ETH_DMA_TRANSMIT_TIMEOUT     ( 20U )
#endif

/* The MPU protection is unreliable for the Socket API. The exact cause is not know yet.
It's safer to always clean data cache for now */
#ifndef RTEMS_LWIP_PROTECT_LWIP_HEAP_WITH_MPU
#define RTEMS_LWIP_PROTECT_LWIP_HEAP_WITH_MPU     1
#endif

#ifndef RTEMS_LWIP_DHCP_PRINTOUT
#define RTEMS_LWIP_DHCP_PRINTOUT                  1
#endif

#ifndef LWIP_RAM_HEAP_POINTER
#define LWIP_RAM_HEAP_POINTER                     (0x30040000)
#endif

#ifndef MEM_SIZE
#define MEM_SIZE                                  (8 * 1024)
#endif

#if NO_SYS == 1
#ifndef RTEMS_LWIP_DHCP_TASK_INTERVAL_MS
#define RTEMS_LWIP_DHCP_TASK_INTERVAL_MS 500
#endif
#endif /* NO_SYS == 1 */

#endif /* RTEMS_LWIP_LWIP_PORTS_DRIVERS_STM32H7_RTEMS_CONF_H_ */
