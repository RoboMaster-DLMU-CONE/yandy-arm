cmake_minimum_required(VERSION 3.31)

project(effector_test)

add_executable(effector_test main.cpp)

target_link_libraries(effector_test
    PRIVATE
    yandy_arm_effector
    yandy_arm_logger
    spdlog::spdlog
    tomlplusplus::tomlplusplus
)
