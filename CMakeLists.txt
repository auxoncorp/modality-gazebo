cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(ModalityPlugins)

set(MODALITY_SDK_INCLUDE_DIR "" CACHE FILEPATH "Path to local Modality SDK include directory")
set(MODALITY_SDK_LIB_DIR "" CACHE FILEPATH "Path to local Modality SDK lib directory")

if(EXISTS ${MODALITY_SDK_INCLUDE_DIR})
    include_directories(SYSTEM ${MODALITY_SDK_INCLUDE_DIR})
endif()

if(EXISTS ${MODALITY_SDK_LIB_DIR})
    link_directories(${MODALITY_SDK_LIB_DIR})
endif()

find_package(gz-cmake3 REQUIRED)

find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-sim7 REQUIRED)

add_library(ModalityTracingPlugin SHARED ModalityTracingPlugin.cc)
set_property(TARGET ModalityTracingPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(
    ModalityTracingPlugin
    PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim7::gz-sim7
    modality)

add_library(ModalityMutatorsPlugin SHARED ModalityMutatorsPlugin.cc ForceMutator.cc)
set_property(TARGET ModalityMutatorsPlugin PROPERTY CXX_STANDARD 17)
target_link_libraries(
    ModalityMutatorsPlugin
    PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim7::gz-sim7
    modality)

add_custom_target(
    run-example
    DEPENDS
    ModalityTracingPlugin
    ModalityMutatorsPlugin)

add_custom_command(
    TARGET run-example
    POST_BUILD
    COMMAND
    GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_BINARY_DIR} gz sim -r ${CMAKE_SOURCE_DIR}/examples/world.sdf --verbose 3
    COMMENT "Starting the example world")
