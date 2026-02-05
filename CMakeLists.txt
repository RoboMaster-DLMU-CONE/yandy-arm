cmake_minimum_required(VERSION 3.31)
project(yandy_arm)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_CXX_STANDARD 23)

include(FindModules)

add_executable(yandy_arm main.cpp)
