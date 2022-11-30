cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(ModalityTracingPlugin)

find_package(ignition-cmake2 REQUIRED)

find_package(ignition-plugin1 REQUIRED COMPONENTS register)
set(IGN_PLUGIN_VER ${ignition-plugin1_VERSION_MAJOR})

find_package(ignition-gazebo6 REQUIRED)

add_library(ModalityTracingPlugin SHARED ModalityTracingPlugin.cc)

set_property(TARGET ModalityTracingPlugin PROPERTY CXX_STANDARD 17)

target_link_libraries(ModalityTracingPlugin
    PRIVATE ignition-plugin${IGN_PLUGIN_VER}::ignition-plugin${IGN_PLUGIN_VER}
    PRIVATE ignition-gazebo6::ignition-gazebo6
    PRIVATE modality)

# TODO
target_link_directories(ModalityTracingPlugin
    PRIVATE /opt/modality-sdk/lib)

# TODO
target_include_directories(ModalityTracingPlugin
    PRIVATE /opt/modality-sdk/include)

add_custom_target(
    run-example
    DEPENDS ModalityTracingPlugin)

add_custom_command(
    TARGET run-example
    POST_BUILD
    COMMAND
    IGN_GAZEBO_SYSTEM_PLUGIN_PATH=${CMAKE_BINARY_DIR} ign gazebo ${CMAKE_SOURCE_DIR}/examples/world.sdf --verbose 3
    COMMENT "Starting the example world")
