set(src-pattern "src/*.cpp")
set(contrib-pattern "src/contrib/*.cpp")

if (CMAKE_VERSION VERSION_GREATER 3.12)
    list(INSERT contrib-pattern 0 CONFIGURE_DEPENDS)
    list(INSERT src-pattern 0 CONFIGURE_DEPENDS)
endif()

file(GLOB ${YAML_LIB}-contrib-sources ${contrib-pattern})
file(GLOB ${YAML_LIB}-sources ${src-pattern})

add_library(${YAML_LIB} ${${YAML_LIB}-sources} ${${YAML_LIB}-contrib-sources})
#add_library(yaml-cpp::yaml-cpp ALIAS ${YAML_LIB})

set_property(TARGET ${YAML_LIB}
        PROPERTY
        CXX_STANDARD_REQUIRED ON)

set_property(TARGET yaml-cpp PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(${YAML_LIB} PUBLIC ${CMAKE_SOURCE_DIR}/3rdparty/${YAML_LIB}/include)