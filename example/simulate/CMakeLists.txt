add_executable(yandy_simulate main.cpp)

target_link_libraries(yandy_simulate PUBLIC
        rerun_sdk
        yandy_arm_robot_system
        pinocchio::pinocchio
)
