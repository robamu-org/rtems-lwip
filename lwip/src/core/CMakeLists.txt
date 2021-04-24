target_sources(${LWIP_LIB_NAME} PRIVATE
	altcp_alloc.c
	altcp_tcp.c
	altcp.c
	def.c
	dns.c
	inet_chksum.c
	init.c
	ip.c
	mem.c
	memp.c
	netif.c
	pbuf.c
	raw.c
	stats.c
	sys.c
	tcp_in.c
	tcp_out.c
	tcp.c
	timeouts.c
	udp.c
)

add_subdirectory(ipv4)

if(LWIP_ADD_IPV6)
    add_subdirectory(ipv6)
endif()
