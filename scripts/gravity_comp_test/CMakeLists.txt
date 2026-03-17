cmake_minimum_required(VERSION 3.31)

project(gravity_comp_test)

add_executable(gravity_comp_test main.cpp)

target_link_libraries(gravity_comp_test
    PRIVATE
    yandy_arm_robot_system
    yandy_arm_types
    spdlog::spdlog
    Eigen3::Eigen
    pinocchio::pinocchio
)
