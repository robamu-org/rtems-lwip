target_sources(${LIB_LWIP_NAME} PRIVATE
    lan8742.c
    app_dhcp.c
    app_ethernet.c
    ethernetif.c
)

if(RTEMS_LWIP_NUCLEO_H743ZI_VARIANT)
    target_compile_definitions(${LIB_LWIP_NAME} PRIVATE
        NUCLEO_H743ZI=1
    )
endif()

add_subdirectory(include)