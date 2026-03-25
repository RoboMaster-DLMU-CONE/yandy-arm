include(FetchContent)
set(CMAKE_POLICY_VERSION_MINIMUM "3.5")
message(STATUS "PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}")

add_subdirectory(vision)
add_subdirectory(input)
add_subdirectory(fsm)
