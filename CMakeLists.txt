cmake_minimum_required(VERSION 3.31)
project(yandy_arm)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
option(YANDY_BUILD_EXAMPLES "" OFF)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_definitions(YANDY_CONFIG_PATH="${PROJECT_SOURCE_DIR}/config/")

include(FindModules)
add_subdirectory(src)

if (YANDY_BUILD_EXAMPLES)
    add_subdirectory(example)
endif ()