add_executable(yandy_vision_example_cv main_cv.cpp)
target_link_libraries(yandy_vision_example_cv
        PUBLIC
        yandy_arm_vision_system
)
