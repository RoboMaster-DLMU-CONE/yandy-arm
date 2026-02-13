add_executable(yandy_example_input main.cpp)

target_link_libraries(yandy_example_input
        PUBLIC
        yandy_arm_input_provider
        rerun_sdk
)