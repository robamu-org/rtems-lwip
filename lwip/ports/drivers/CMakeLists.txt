target_sources(${LIB_LWIP_NAME} PRIVATE
    rtems_lwip.c
)

if(RTEMS_LWIP_ADD_TMS_570_DRV)
    add_subdirectory(tms570)
endif()

if(RTEMS_LWIP_ADD_STM32H7_DRV)
    option(RTEMS_LWIP_NUCLEO_H743ZI_VARIANT OFF "Can be set for Nucleo H743 ZI board variant")
    add_subdirectory(stm32h7)
endif()

target_include_directories(${LIB_LWIP_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_include_directories(${LIB_LWIP_NAME} INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
