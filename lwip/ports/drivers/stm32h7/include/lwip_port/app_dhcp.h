#ifndef LWIP_PORTS_DRIVERS_STM32H7_APP_DHCP_H_
#define LWIP_PORTS_DRIVERS_STM32H7_APP_DHCP_H_

#include <lwipopts.h>
#include <stdint.h>

#include <lwip/netif.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get the state of the internal DHCP state machine
 * @return
 */
uint8_t get_dhcp_state();

/* DHCP process states */
#define DHCP_OFF                   (uint8_t) 0
#define DHCP_START                 (uint8_t) 1
#define DHCP_WAIT_ADDRESS          (uint8_t) 2
#define DHCP_ADDRESS_ASSIGNED      (uint8_t) 3
#define DHCP_TIMEOUT               (uint8_t) 4
#define DHCP_LINK_DOWN             (uint8_t) 5

#if NO_SYS == 1
/**
 * This function should be called in mainloop or in a dedicated thread and takes care of
 * the DHCP handling.
 * @param netif
 */
void dhcp_periodic_handle(struct netif *netif);
#else

/**
 * Start a DHCP thread with the given parameters.
 * @param task_interval_ms
 * @param task_stack
 * @param task_priority
 */
void rtems_lwip_start_dhcp_thread(uint32_t task_interval_ms,
    size_t task_stack, uint8_t task_priority);

#endif

#ifdef __cplusplus
}
#endif

#endif /* LWIP_PORTS_DRIVERS_STM32H7_APP_DHCP_H_ */
