add_library(${HEAD_CONTROLLER_LIB} Src/HeadController.cpp)

target_include_directories(${HEAD_CONTROLLER_LIB} PUBLIC Include ${CMAKE_SOURCE_DIR}/${SHIMON_LIB}/Include)
target_link_libraries(${HEAD_CONTROLLER_LIB} PUBLIC ${SERIAL_LIB} ${DXL_LIB} ${YAML_LIB} ${OSC_LIB} -lpthread)
