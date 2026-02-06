add_executable(yandy_simulate main.cpp)

include(FetchContent)
FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)

FetchContent_MakeAvailable(rerun_sdk)

find_package(SDL2 REQUIRED)

target_link_libraries(yandy_simulate PUBLIC
        SDL2
        rerun_sdk
        yandy_arm_arm_hw
        yandy_arm_solver
)