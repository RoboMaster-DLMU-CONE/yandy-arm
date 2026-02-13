add_executable(yandy_example_vision_cv main_cv.cpp)
target_link_libraries(yandy_example_vision_cv
        PUBLIC
        yandy_arm_vision_system
)
