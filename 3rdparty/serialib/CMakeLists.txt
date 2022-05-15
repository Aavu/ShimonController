add_library(serialib serialib.cpp)
target_include_directories(serialib PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/${SERIAL_LIB})