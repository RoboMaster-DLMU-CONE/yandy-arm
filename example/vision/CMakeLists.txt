add_executable(yandy_vision_example main.cpp)
target_link_libraries(yandy_vision_example
        PUBLIC
        yandy_arm_vision_system
        rerun_sdk
)