# --- Yandy-Arm 安装与部署配置 ---

# 配置安装后的运行时库搜索路径 (RPATH)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# 安装可执行文件
install(TARGETS yandy_arm
        RUNTIME DESTINATION bin
)

# 安装所有动态库目标
install(TARGETS
        yandy_arm_logger
        yandy_arm_arm_hw
        yandy_arm_effector
        yandy_arm_solver
        yandy_arm_trajectory_planner
        yandy_arm_vision_system
        yandy_arm_fsm
        yandy_arm_input_provider
        yandy_arm_robot_system
        LIBRARY DESTINATION lib
)

# 处理 Systemd 服务自启动
set(SYSTEMD_SERVICE_FILE "${CMAKE_BINARY_DIR}/yandy-arm.service")

configure_file(
    "${PROJECT_SOURCE_DIR}/src/systemd/yandy-arm.service.in"
    "${SYSTEMD_SERVICE_FILE}"
    @ONLY
)

install(FILES "${SYSTEMD_SERVICE_FILE}"
        DESTINATION /lib/systemd/system/
)
