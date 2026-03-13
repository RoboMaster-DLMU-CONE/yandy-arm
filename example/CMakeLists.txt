include(FetchContent)
set(CMAKE_POLICY_VERSION_MINIMUM "3.5")
message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")
FetchContent_Declare(
        rerun_sdk
        URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
        PATCH_COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/cmake/rerun_patch/ .
)

FetchContent_MakeAvailable(rerun_sdk)

add_subdirectory(simulate)
add_subdirectory(vision)
add_subdirectory(input)
add_subdirectory(fsm)