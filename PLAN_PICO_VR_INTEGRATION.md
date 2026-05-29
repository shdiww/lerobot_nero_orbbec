# Pico VR 遥操作集成 Nero 机械臂 — 实施计划

## 目标

将 XRoboToolkit 的 Pico VR 控制器（右手）6D 位姿追踪集成到 lerobot 框架，
作为新的 teleoperator 设备 `pico_vr`，用于控制 Nero 机械臂进行数据采集。

## 架构概览

```
Pico VR 头显 + 右控制器
    │  (WiFi/局域网)
    ▼
XRoboToolkit PC Service (桌面后台服务, 必须先启动)
    │  (gRPC)
    ▼
xrobotoolkit_sdk (C++ pybind11 模块)
    │
    ▼
PicoVRController (新建, 封装SDK读取 + 增量计算)
    │  输出: (delta_x, delta_y, delta_z, delta_wx, delta_wy, delta_wz, delta_gripper)
    ▼
NeroPicoVR(Teleoperator) (新建, 同 NeroGamepad 的 action_features)
    │  solve_ik_from_deltas() → TracIK → joint_angles
    ▼
Nero(Robot).send_action() → 机械臂运动
```

## 交互设计

| Pico 右控制器 | 功能 |
|---|---|
| Grip 握住 (>0.9) | 激活追踪：捕捉参考位姿，开始计算6D增量 |
| Grip 松开 | 冻结机器人位置，重置参考位姿 |
| A 按钮 | 夹爪闭合（增量） |
| B 按钮 | 夹爪张开（增量） |
| X 按钮 | 回 Home |
| Y 按钮 | — (预留) |
| 右摇杆 | — (预留，可用于微调) |
| 右扳机 (trigger) | — (预留，可用于缩放系数) |

## 需安装的依赖

| 包 | 安装方式 | 用途 |
|---|---|---|
| `xrobotoolkit_sdk` | 见下方步骤 | Pico VR 数据读取核心 SDK |
| `pybind11`, `cmake` | 系统包或 pip | 编译 SDK 绑定 |
| XRoboToolkit PC Service 桌面应用 | `.deb` 安装 | gRPC 服务端，必须运行 |

**不需要安装**: placo, mujoco, meshcat, ur_rtde 等。
四元数运算用 `scipy`（已在 NeroGamepad IK 中使用）。

## 实施步骤

### Step 0: 环境准备与验证

1. 安装 SDK 依赖
2. 运行 `scripts/test_pico_sdk_import.py` 验证 SDK 可导入
3. 运行 `scripts/test_pico_data_read.py` 验证可读取 Pico 控制器数据

### Step 1: 创建 `pico_vr` teleoperator 模块

**文件**: `lerobot/src/lerobot/teleoperators/pico_vr/pico_vr_utils.py`

`PicoVRController` 类：
- 封装 `xrobotoolkit_sdk`，延迟 import
- `start()`: `xrt.init()`
- `stop()`: `xrt.close()`
- `update()`: 每帧读取位姿/按钮/摇杆
  - 首次 grip 激活时记录 `ref_controller_pose` 和 `ref_ee_pose`
  - 激活时计算 6D 增量（位置差 × scale_factor + 四元数差 → angle-axis）
  - grip 松开时重置参考位姿
- `get_deltas()`: 返回 `(dx, dy, dz, dwx, dwy, dwz)`
- `gripper_command()`: A=闭合, B=张开（增量）
- 事件方法: `is_home_requested()`, `is_start_pressed()`, `is_back_pressed()`

**坐标系变换**: `R_HEADSET_TO_WORLD = [[0,0,-1],[-1,0,0],[0,1,0]]`
**四元数计算**: 用 `scipy.spatial.transform.Rotation`，不引入 meshcat

**文件**: `lerobot/src/lerobot/teleoperators/pico_vr/configuration_pico_vr.py`

```python
@TeleoperatorConfig.register_subclass("pico_vr")
@dataclass
class NeroPicoVRConfig(TeleoperatorConfig):
    use_gripper: bool = True
    controller: str = "right"
    grip_threshold: float = 0.9
    scale_factor: float = 1.5
    gripper_step: float = 5
    max_joint_step: float = 0.1
    reject_threshold: float = 0.3
    ema_alpha: float = 0.2
    urdf_path: str = _DEFAULT_URDF
    base_link_name: str = "base_link"
    target_link_name: str = "gripper_base"
    home_joint_angles: list[float] = ...
```

**文件**: `lerobot/src/lerobot/teleoperators/pico_vr/teleop_pico_vr.py`

`NeroPicoVR(Teleoperator)` 类：
- 与 NeroGamepad 相同的 action_features、TracIKAdapter、solve_ik_from_deltas
- 区别仅在 get_action() 从 PicoVRController 读取 deltas

**文件**: `lerobot/src/lerobot/teleoperators/pico_vr/__init__.py`

### Step 2: 注册 teleoperator

- `teleoperators/utils.py`: 添加 `"pico_vr"` 分支
- `teleoperators/__init__.py`: 添加 `from . import pico_vr`

### Step 3: 创建录制脚本

**文件**: `lerobot/scripts/lerobot_record_nero_pico.py`
- 基于 `lerobot_record_nero.py`，默认 `teleop.type=pico_vr`
- 按键说明更新为 Pico 控制器

### Step 4: 测试与验证

1. SDK 连通性测试
2. 增量输出测试
3. IK 联调
4. 端到端测试

## 文件变更清单

| 操作 | 文件 | 说明 |
|---|---|---|
| 新建 | `scripts/test_pico_sdk_import.py` | SDK 导入测试 |
| 新建 | `scripts/test_pico_data_read.py` | Pico 数据读取测试 |
| 新建 | `teleoperators/pico_vr/__init__.py` | 模块导出 |
| 新建 | `teleoperators/pico_vr/configuration_pico_vr.py` | Config 注册 |
| 新建 | `teleoperators/pico_vr/pico_vr_utils.py` | PicoVRController 核心类 |
| 新建 | `teleoperators/pico_vr/teleop_pico_vr.py` | NeroPicoVR Teleoperator 类 |
| 新建 | `scripts/lerobot_record_nero_pico.py` | 录制脚本 |
| 修改 | `teleoperators/__init__.py` | 添加 pico_vr import |
| 修改 | `teleoperators/utils.py` | 添加 pico_vr 工厂分支 |

## 风险与注意事项

1. **SDK 安装**: 需要 cmake、g++、pybind11，与 uv 环境兼容性需验证
2. **延迟**: Pico → PC Service → gRPC → SDK → Python 链路，需实测是否满足 30fps
3. **坐标系**: R_HEADSET_TO_WORLD 为 UR5e 定义，Nero 方向可能需微调
4. **gRPC 断连**: 需优雅处理（返回零值或异常）
