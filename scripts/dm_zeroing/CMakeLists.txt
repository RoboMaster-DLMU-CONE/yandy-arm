add_executable(yandy_dm_zeroing main.cpp)
target_link_libraries(yandy_dm_zeroing
        PUBLIC
        OneMotor::OneMotor
        tomlplusplus::tomlplusplus
)