add_subdirectory(Src/${ARM_CONTROLLER_LIB})
add_subdirectory(Src/${HEAD_CONTROLLER_LIB})

add_library(${SHIMON_LIB} Src/Shimon.cpp)
target_include_directories(${SHIMON_LIB} PUBLIC ${CMAKE_SOURCE_DIR}/${SHIMON_LIB}/Include)
target_link_libraries(${SHIMON_LIB} PUBLIC ${ARM_CONTROLLER_LIB} ${HEAD_CONTROLLER_LIB} ${OSC_LIB} ${SERIAL_LIB})
