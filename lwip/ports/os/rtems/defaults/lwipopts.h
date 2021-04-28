#ifndef RTEMS_LWIP_LWIP_PORTS_OS_RTEMS_LWIPOPTS_H_
#define RTEMS_LWIP_LWIP_PORTS_OS_RTEMS_LWIPOPTS_H_

/**
 * @brief   This is the default lwipopts.h file which will be used if none is supplied by the
 *          user.
 * @details
 * This might not work on each port depending on which configuration options are used
 * and required by the port
 */

/**
 * NO_SYS==1: Provides VERY minimal functionality. Otherwise,
 * use lwIP facilities.
 */
#define NO_SYS                          0

/* DHCP will be enabled by default */
#define LWIP_DHCP                       1

/* LWIP_NETCONN==1: Enable Netconn API (require to use api_lib.c) */
#define LWIP_NETCONN                    1

/* LWIP_SOCKET==1: Enable Socket API (require to use sockets.c) */
#define LWIP_SOCKET                     1

/* OS specific options */
#define TCPIP_THREAD_NAME              "LWIP"
#define TCPIP_THREAD_STACKSIZE          RTEMS_MINIMUM_STACK_SIZE
#define TCPIP_MBOX_SIZE                 6
#define DEFAULT_UDP_RECVMBOX_SIZE       6
#define DEFAULT_TCP_RECVMBOX_SIZE       6
#define DEFAULT_ACCEPTMBOX_SIZE         6
#define DEFAULT_THREAD_STACKSIZE        500
#define TCPIP_THREAD_PRIO               60

#endif /* RTEMS_LWIP_LWIP_PORTS_OS_RTEMS_LWIPOPTS_H_ */
