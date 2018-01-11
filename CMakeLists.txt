cmake_minimum_required(VERSION 3.0)

project(osmoper)

# add CMakeModules to path
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_SOURCE_DIR}/CMakeModules")

# set output folder to /bin/
get_filename_component(CONFIG_NAME "${CMAKE_BINARY_DIR}" NAME)
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/${CONFIG_NAME}")
set(LIBRARY_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin/${CONFIG_NAME}")

# packages
find_package(Xo REQUIRED)
find_package(OpenSim 3.3 REQUIRED)

include_directories(${OPENSIMSIMBODY_INCLUDE_DIRS})
include_directories(${XO_INCLUDE_DIR})

# projects
add_subdirectory(submodules/xo)
add_subdirectory(src)
