add_library(yandy_arm_types INTERFACE)
target_include_directories(yandy_arm_types INTERFACE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(yandy_arm_types INTERFACE rpl::rpl)
target_compile_options(yandy_arm_types INTERFACE -Wno-interference-size)

add_library(yandy_arm_logger SHARED core/Logger.cpp)
target_include_directories(yandy_arm_logger PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(yandy_arm_logger PUBLIC spdlog::spdlog tomlplusplus::tomlplusplus)

add_library(yandy_arm_arm_hw SHARED modules/ArmHW.cpp)
target_include_directories(yandy_arm_arm_hw PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(yandy_arm_arm_hw
        PUBLIC
        yandy_arm_logger
        yandy_arm_types
        OneMotor::OneMotor
        pinocchio::pinocchio
        tomlplusplus::tomlplusplus
)

add_library(yandy_arm_effector SHARED modules/Effector.cpp)
target_link_libraries(yandy_arm_effector
        PUBLIC
        yandy_arm_logger
        yandy_arm_types
        tomlplusplus::tomlplusplus
)

add_library(yandy_arm_solver SHARED modules/DynamicsSolver.cpp)
target_link_libraries(yandy_arm_solver
        PUBLIC
        yandy_arm_logger
        pinocchio::pinocchio
)

add_library(yandy_arm_vision_system SHARED modules/VisionSystem.cpp)
target_link_directories(yandy_arm_vision_system
        PUBLIC
        /opt/MVS/lib/64/
)
target_include_directories(yandy_arm_vision_system
        PUBLIC
        /opt/MVS/include
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(yandy_arm_vision_system
        PUBLIC
        yandy_arm_types
        yandy_arm_logger
        ${OpenCV_LIBS}
        openvino::runtime
        MvCameraControl
)


add_library(yandy_arm_fsm SHARED modules/FSM.cpp)
include(GrepPumlFSM)

target_link_libraries(yandy_arm_fsm
        PUBLIC
        yandy_arm_logger
)

add_library(yandy_arm_input_provider SHARED
        modules/InputProvider.cpp
)

target_link_libraries(yandy_arm_input_provider
        PUBLIC
        yandy_arm_types
        yandy_arm_logger
        HySerial::HySerial
        tomlplusplus::tomlplusplus
)

if (YANDY_BUILD_TELEOP_CLIENT)
    add_subdirectory(teleop_client)
endif ()

add_library(yandy_arm_robot_system SHARED Robot.cpp)
target_link_libraries(yandy_arm_robot_system
        PUBLIC
        yandy_arm_arm_hw
        yandy_arm_effector
        yandy_arm_solver
        yandy_arm_vision_system
        yandy_arm_fsm
        yandy_arm_input_provider
)

add_executable(yandy_arm main.cpp)
target_link_libraries(yandy_arm
        PUBLIC
        yandy_arm_robot_system
)