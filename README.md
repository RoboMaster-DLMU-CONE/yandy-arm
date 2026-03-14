# Yandy-Arm

[![C++23](https://img.shields.io/badge/C%2B%2B-23-blue.svg)](https://en.cppreference.com/w/cpp/23)
[![CMake](https://img.shields.io/badge/CMake-3.31+-green.svg)](https://cmake.org/)

**CONE 战队 2026 年 Yandy 工程机器人上位机控制系统**

Yandy-Arm 是一个面向 RoboMaster 工程机器人任务的高性能机械臂控制系统，集成了视觉识别、运动规划、动力学求解和状态机决策等核心功能，实现全自动的矿石抓取与存放操作。

## 核心特性

### 全自动作业流程
- **视觉引导抓取**: 基于 Hikrobot 工业相机 + OpenVINO 深度学习，实现矿石的实时识别与精确定位
- **智能存取决策**: 根据末端持有状态和仓库库存，自动规划最优存取路径
- **一键完成流程**: 从识别到存放全流程自动化，无需人工干预

### 高性能运动控制
- **250Hz 控制循环**: 基于 Ruckig 的实时轨迹规划，确保运动平滑且响应迅速
- **动力学前馈补偿**: 使用 Pinocchio 库进行精确的动力学计算，抵消重力和惯性影响
- **6 自由度控制**: 支持完整的空间位姿控制 (位置 + 姿态)
- **智能避障规划**: OMPL + RRTConnect 算法，自动生成无碰撞路径

### 状态机驱动
- **PlantUML 可视化**: 状态机逻辑使用 PlantUML 描述，代码与文档同步
- **Boost.MSM 实现**: 高性能有限状态机，支持复杂的模式切换和错误恢复
- **5 种运行模式**: 禁用、手动、抓取、存取、故障，覆盖全部操作场景

### 工业级可靠性
- **多线程架构**: 视觉处理与控制循环独立运行，通过无锁环形缓冲通信
- **急停保护**: 硬件级急停指令，确保异常情况下的系统安全
- **仿真模式**: 支持纯软件仿真调试，无需真实硬件即可验证逻辑

## 系统架构

```
┌─────────────────────────────────────────────────────────────────┐
│                        控制层 (250Hz)                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │ InputProvider│→ │   FSM (MSM)  │→ │ TrajectoryPlanner    │   │
│  │ (遥控器/键盘) │  │ (状态决策)   │  │ (Ruckig + OMPL)      │   │
│  └──────────────┘  └──────────────┘  └──────────────────────┘   │
│                              ↓                                   │
│                    ┌──────────────┐                             │
│                    │   ArmHW      │→ CAN → OneMotor 关节模组    │
│                    │ (硬件抽象层) │                             │
│                    └──────────────┘                             │
└─────────────────────────────────────────────────────────────────┘
                               ↕ NBuf (无锁通信)
┌─────────────────────────────────────────────────────────────────┐
│                        视觉层 (独立线程)                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │
│  │  HikDriver   │→ │EnergyDetector│→ │ EnergyPoseSolver     │   │
│  │ (MVS SDK)    │  │ (OpenVINO)   │  │ (坐标变换)           │   │
│  └──────────────┘  └──────────────┘  └──────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

## 技术栈

| 类别 | 技术选型 |
|------|----------|
| **语言标准** | C++23 |
| **构建系统** | CMake 3.31+ |
| **状态机** | Boost.MSM (PlantUML 驱动) |
| **动力学** | Pinocchio |
| **轨迹规划** | Ruckig |
| **运动规划** | OMPL |
| **视觉识别** | OpenCV + OpenVINO + Hikrobot MVS |
| **日志系统** | spdlog |
| **配置管理** | tomlplusplus |
| **数学库** | Eigen3 |
| **电机驱动** | OneMotor (CAN 总线) |

## 快速开始

### 环境要求

- **操作系统**: Linux (Ubuntu 22.04+)
- **编译器**: GCC 13+ (支持 C++23)
- **CMake**: 3.31+

### 依赖库

项目依赖以下外部库（通过 `cmake/Modules/` 自动查找）：

- Boost (MSM 状态机)
- HySerial (串口通信)
- OneMotor (CAN 电机驱动)
- Pinocchio (机器人动力学)
- OMPL (运动规划)
- Ruckig (轨迹规划)
- OpenCV
- OpenVINO
- spdlog
- tomlplusplus
- TL-expected

### 构建步骤

```bash
# Debug 模式 (开发调试)
cmake -B cmake-build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build cmake-build-debug

# Release 模式 (部署运行)
cmake -B cmake-build-release -DCMAKE_BUILD_TYPE=Release
cmake --build cmake-build-release
```

### 运行程序

```bash
# 运行主程序
./cmake-build-release/yandy_arm

# 运行示例程序 (需先开启选项)
cmake -B build -DYANDY_BUILD_EXAMPLES=ON
cmake --build build
./build/example/yandy_arm_example
```

### 配置选项

| CMake 选项 | 默认值 | 说明 |
|------------|--------|------|
| `YANDY_BUILD_EXAMPLES` | OFF | 是否编译示例程序 |
| `YANDY_BUILD_TELEOP_CLIENT` | OFF | 是否编译遥操作客户端 |

## 操作指南

### 键盘控制映射

**核心功能区**

| 按键 | 功能 | 说明 |
|:----:|:-----|:-----|
| **F** | 抓取模式 | 进入/退出视觉抓取流程 |
| **R** | 存取模式 | 进入/退出矿仓存取流程 |
| **G** | 启用/禁用 | 切换机械臂输出使能 |
| **C** | 手动夹爪 | 手动控制夹爪开合 |
| **Z** | 复位 | 清除错误/重置原点 |
| **B** | 急停 | 立即停止所有动作 |

**调试修正区** (组合键 `Ctrl` + `Key`)

| 组合键 | 功能 | 使用场景 |
|:------:|:-----|:---------|
| `Ctrl + F` | 切换持有状态 | 视觉误判时手动修正 |
| `Ctrl + E` | 库存计数 +1 | 强制增加仓库计数 |
| `Ctrl + Q` | 库存计数 -1 | 强制减少仓库计数 |

### 状态机说明

系统运行状态由 PlantUML 状态机管理，主要状态如下：

```
[*] → Disabled → ManualControl ↔ FetchingMode
                          ↓
                     StorageMode
                          ↓
                     ErrorMode → [*]
```

| 状态 | 说明 | 进入条件 |
|------|------|----------|
| `Disabled` | 禁用状态 | 系统初始状态 |
| `ManualControl` | 手动控制 | 启用输出后进入 |
| `FetchingMode` | 抓取模式 | 按下 F 键 |
| `StorageMode` | 存取模式 | 按下 R 键 |
| `ErrorMode` | 故障状态 | 急停或异常触发 |

详细状态机定义见 [`config/fsm.puml`](config/fsm.puml)，完整说明见 [`doc/fsm.md`](doc/fsm.md)。

## 项目结构

```
yandy-arm/
├── CMakeLists.txt          # 主构建配置
├── cmake/                  # CMake 模块
│   ├── FindModules.cmake   # 依赖查找入口
│   ├── GrepPumlFSM.cmake   # FSM 解析脚本
│   └── Modules/            # 各库的 Find 脚本
├── config/                 # 运行时配置
│   ├── urdf/               # URDF 机器人模型
│   ├── yolo/               # YOLO 检测模型
│   ├── robot.toml          # 机器人配置
│   ├── fsm.puml            # 状态机定义
│   ├── vision.toml         # 视觉参数
│   ├── effector.toml       # 夹爪 PID
│   └── joint.toml          # 关节参数
├── include/yandy/          # 公共头文件
│   ├── common/             # 通用类型 (Types.hpp, NBuf.hpp)
│   ├── core/               # 核心组件 (Logger)
│   ├── modules/            # 模块接口
│   └── Robot.hpp           # 主控制类
├── src/                    # 源代码
│   ├── main.cpp            # 程序入口
│   ├── Robot.cpp           # Robot 实现
│   ├── core/               # 核心实现
│   └── modules/            # 模块实现
├── example/                # 示例程序
└── doc/                    # 技术文档
```

## 核心模块

| 模块 | 功能 | 依赖 |
|------|------|------|
| **ArmHW** | CAN 总线硬件抽象 | OneMotor, Pinocchio |
| **DynamicsSolver** | 运动学/动力学求解 | Pinocchio, OMPL, Ruckig |
| **TrajectoryPlanner** | 实时轨迹生成 + 路径规划 | Ruckig, OMPL, Pinocchio |
| **VisionSystem** | 视觉识别与定位 | OpenCV, OpenVINO, MVS |
| **FSM** | 状态机决策 | Boost.MSM |
| **InputProvider** | 遥控器/键盘输入解析 | HySerial |
| **Effector** | 末端夹爪控制 | - |
| **Robot** | 系统集成与协调 | 全部模块 |

## 性能指标

| 指标 | 数值 |
|------|------|
| 控制频率 | 250 Hz (4ms 周期) |
| 视觉处理 | ~30 FPS |
| 轨迹规划 (Ruckig) | < 0.1ms |
| 路径规划 (OMPL) | < 1s (RRTConnect) |
| 状态机响应 | < 100μs |

## 开发指南

### 代码规范

- **命名空间**: `yandy::modules`, `yandy::common`, `yandy::core`
- **类名**: 大驼峰 (PascalCase)
- **成员变量**: `m_` 前缀
- **日志**: 使用 `spdlog`，格式 `logger->info("消息 {}", 参数)`

### 添加新状态

1. 在 `config/fsm.puml` 中修改 PlantUML 状态图
2. 在 `include/yandy/common/Types.hpp` 中添加状态枚举
3. 在 `src/modules/FSM.cpp` 中实现状态逻辑

### 调试技巧

```bash
# 开启仿真模式 (无需真实硬件)
# 修改 config/robot.toml: simulate = true

# 调整日志级别
# 修改 config/log.toml: level = "debug"
```

## 文档

- [FSM 详细说明](doc/fsm.md) - 状态机逻辑与键位映射
- [PlantUML 源文件](config/fsm.puml) - 状态机可视化定义

## 许可证
