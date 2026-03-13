include(FetchContent)
set(CMAKE_POLICY_VERSION_MINIMUM "3.5")
FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
)

FetchContent_MakeAvailable(rerun_sdk)

add_subdirectory(simulate)
add_subdirectory(vision)
add_subdirectory(input)
add_subdirectory(fsm)