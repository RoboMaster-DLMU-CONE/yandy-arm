add_library(yandy_arm_logger STATIC core/Logger.cpp)
target_include_directories(yandy_arm_logger PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(yandy_arm_logger PUBLIC spdlog::spdlog tomlplusplus::tomlplusplus)

add_library(yandy_arm_arm STATIC modules/Arm.cpp)

target_include_directories(yandy_arm_arm PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(yandy_arm_arm
        PUBLIC
        yandy_arm_logger
        OneMotor::OneMotor
        tomlplusplus::tomlplusplus
)


add_executable(yandy_arm main.cpp)
target_link_libraries(yandy_arm PUBLIC yandy_arm_arm)