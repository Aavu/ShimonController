set(DXL_SRC
        ${DXL_SRC}
        Src/dynamixel_sdk/group_bulk_read.cpp
        Src/dynamixel_sdk/group_bulk_write.cpp
        Src/dynamixel_sdk/group_sync_read.cpp
        Src/dynamixel_sdk/group_sync_write.cpp
        Src/dynamixel_sdk/packet_handler.cpp
        Src/dynamixel_sdk/port_handler.cpp
        Src/dynamixel_sdk/protocol1_packet_handler.cpp
        Src/dynamixel_sdk/protocol2_packet_handler.cpp
        )

if(APPLE OR __APPLE__)
    set(DXL_SRC
            ${DXL_SRC}
            Src/dynamixel_sdk/port_handler_mac.cpp
            )
elseif(UNIX OR __linux__)
    set(DXL_SRC
            ${DXL_SRC}
            Src/dynamixel_sdk/port_handler_linux.cpp
            )
endif()

add_library(${DXL_LIB} ${DXL_SRC})
target_include_directories(${DXL_LIB} PUBLIC Include/dynamixel_sdk)
