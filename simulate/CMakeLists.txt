FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
        PATCH_COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/cmake/rerun_patch/ .
)

FetchContent_MakeAvailable(rerun_sdk)


add_executable(yandy_simulate main.cpp)

target_link_libraries(yandy_simulate PUBLIC
        rerun_sdk
        yandy_arm_robot_system
        pinocchio::pinocchio
)
