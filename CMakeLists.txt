cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(ModalityTracingPlugin)

find_package(gz-cmake3 REQUIRED)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-sim7 REQUIRED)

add_library(ModalityTracingPlugin SHARED ModalityTracingPlugin.cc)

set_property(TARGET ModalityTracingPlugin PROPERTY CXX_STANDARD 17)

target_link_libraries(ModalityTracingPlugin
    PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim7::gz-sim7
    modality)

add_custom_target(
    run-example
    DEPENDS ModalityTracingPlugin)

add_custom_command(
    TARGET run-example
    POST_BUILD
    COMMAND
    GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_BINARY_DIR} gz sim ${CMAKE_SOURCE_DIR}/examples/world.sdf --verbose 3
    COMMENT "Starting the example world")
