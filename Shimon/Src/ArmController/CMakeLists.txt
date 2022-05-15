add_library(${ARM_CONTROLLER_LIB}
        Src/ArmController.cpp)

target_include_directories(${ARM_CONTROLLER_LIB} PUBLIC Include ${CMAKE_SOURCE_DIR}/${SHIMON_LIB}/Include)
target_link_libraries(${ARM_CONTROLLER_LIB} PUBLIC ${OSC_LIB} ${SERIAL_LIB} -lpthread)