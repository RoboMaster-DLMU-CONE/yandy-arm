## Packet内容

- x,y,z,r,p,y绝对值
- 代表不同状态的一个 uint8_t enum

## Packet来源

下位机C板USB通信

### 位置绝对值来源

- 连接到自定义控制器（在总线上检测到自定义控制器包）时：从自控获取
- 未连接到自定义控制器时：从遥控器获取xyzrpy（ls上+rs上->左摇杆xy,右摇杆z; ls上+rs中->
  左摇杆yr,右摇杆p，此时下位机程序里底盘应该给0的ref)

### 状态enum来源

遥控器+键盘

单遥控器：ls中/下时，拨动滚轮持续0.5s，根据rs的位置发出对应的状态。滚轮为0时状态发NULL

- ls中
    - rs上：SWITCH_ENABLE
    - rs中：RESET
    - rs下：FETCH
- ls下
    - rs上：SWITCH_STORE
    - rs中：SWITCH_GRIP

键盘：

## 状态

### NULL

正常通过xyzrpy进行运动规划并输出

### SWITCH_ENABLE

切换规划和输出启用状态

### RESET

机械臂强制回到初始位置，同时清空各种累积值

### SWITCH_FETCH

切换抓取状态
抓取状态：通过视觉检测，定位到合适的位置，打开夹爪，检测绝对值并进行微调。
再按一下进行夹取并返回原位。

### SWITCH_STORE

切换存取矿/正常模式
首先检测末端上是否有矿；
如果有矿，将矿带到空余的杆上，检测绝对值并进行微调。
如果没有矿，就从有矿的杆子上进行取矿，检测绝对值进行微调。
切回正常模式时，会松开末端执行器并回到存矿之前的位置

### SWITCH_GRIP

切换夹爪夹取状态

## 状态图

```mermaid
stateDiagram-v2
    [*] --> Disabled
    [*] --> ErrorMode: CMD_ERROR / emergency_stop
    Disabled: 禁用输出
    ErrorMode: 故障状态，禁止动作
    ErrorMode: 需要 CMD_RESET 才能清除
    ManualControl: [NULL状态] 正常规划
    ManualControl: 内部修正指令 (Debug/Fix)：
    ManualControl: - CMD_DEBUG_TOGGLE_HELD / toggle_held_flag
    ManualControl: - CMD_DEBUG_INC_STORE / inc_storage_count
    ManualControl: - CMD_DEBUG_DEC_STORE / dec_storage_count
    ManualControl: 内部逻辑检查：
    ManualControl: - CMD_SWITCH_STORE [store_logic_conflict] / log_store_conflict
    FetchingMode: [FETCH状态]
    FetchingMode: Entry 视觉定位 → 移动 → 开爪
    FetchingMode: Exit 关爪
    StorageMode: [STORE状态]
    StorageMode: Entry 移动到存/取点
    StorageMode: Exit 执行存/取动作 → 回位 → 更新计数
    ErrorMode --> ManualControl: CMD_RESET / clear_error_flags
    Disabled --> ManualControl: CMD_SWITCH_ENABLE / enable_output
    ManualControl --> Disabled: CMD_SWITCH_ENABLE / disable_output
    ManualControl --> FetchingMode: CMD_SWITCH_FETCH / action_vision_approach
    FetchingMode --> ManualControl: CMD_SWITCH_FETCH [grip_success] / action_grip_success
    FetchingMode --> ManualControl: CMD_SWITCH_FETCH [!grip_success] / action_grip_fail_log
    ManualControl --> StorageMode: CMD_SWITCH_STORE [has_mineral && can_deposit] / action_move_to_deposit
    ManualControl --> StorageMode: CMD_SWITCH_STORE [!has_mineral && can_retrieve] / action_move_to_retrieve
    StorageMode --> ManualControl: CMD_SWITCH_STORE / action_finish_store_procedure
    ManualControl --> ManualControl: -CMD_DEBUG_TOGGLE_HELD / toggle_held_flag
    ManualControl --> ManualControl: -CMD_DEBUG_INC_STORE / inc_storage_count
    ManualControl --> ManualControl: -CMD_DEBUG_DEC_STORE / dec_storage_count
    ManualControl --> ManualControl: -CMD_SWITCH_STORE [store_logic_conflict] / log_store_conflict
    ManualControl --> ManualControl: -CMD_RESET / action_clear_accumulators
    ManualControl --> ManualControl: -CMD_SWITCH_GRIP / action_toggle_gripper
```