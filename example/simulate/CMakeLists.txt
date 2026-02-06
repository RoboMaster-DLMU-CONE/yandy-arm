add_executable(yandy_simulate main.cpp)

include(FetchContent)
FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)

FetchContent_MakeAvailable(rerun_sdk)

find_package(Curses REQUIRED)

target_link_libraries(yandy_simulate PUBLIC
        ${CURSES_LIBRARIES}
        rerun_sdk
        yandy_arm_arm_hw
        yandy_arm_solver
)
target_include_directories(yandy_simulate PUBLIC ${CURSES_INCLUDE_DIRS})
