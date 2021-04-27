target_include_directories(${LIB_LWIP_NAME} PRIVATE
    lwip_port
)

target_include_directories(${LIB_LWIP_NAME} INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
