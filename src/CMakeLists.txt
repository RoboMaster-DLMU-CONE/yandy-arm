add_library(yandy_arm_types INTERFACE)
target_include_directories(yandy_arm_types INTERFACE ${PROJECT_SOURCE_DIR}/include)
#target_link_libraries(yandy_arm_types INTERFACE eigen)

add_library(yandy_arm_logger STATIC core/Logger.cpp)
target_include_directories(yandy_arm_logger PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(yandy_arm_logger PUBLIC spdlog::spdlog tomlplusplus::tomlplusplus)

add_library(yandy_arm_arm_hw STATIC modules/ArmHW.cpp)
target_include_directories(yandy_arm_arm_hw PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(yandy_arm_arm_hw
        PUBLIC
        yandy_arm_logger
        yandy_arm_types
        OneMotor::OneMotor
        tomlplusplus::tomlplusplus
)

add_library(yandy_arm_effector STATIC modules/Effector.cpp)
target_link_libraries(yandy_arm_effector
        PUBLIC
        yandy_arm_logger
        yandy_arm_types
        tomlplusplus::tomlplusplus
)

add_library(yandy_arm_solver STATIC modules/DynamicsSolver.cpp)
target_link_libraries(yandy_arm_solver
        PUBLIC
        yandy_arm_logger
        pinocchio::pinocchio
)

add_executable(yandy_arm main.cpp)
target_link_libraries(yandy_arm
        PUBLIC
        yandy_arm_arm_hw
        yandy_arm_effector
        yandy_arm_solver
)