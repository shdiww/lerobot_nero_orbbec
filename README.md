# lerobot_nero_orbbec

基于 [HuggingFace LeRobot](https://github.com/huggingface/lerobot) 的 Nero 机械臂 + Orbbec 深度相机集成项目。

## 项目结构

```
lerobot_nero_orbbec/
├── lerobot/            # LeRobot 子模块（fork 自 shdiww/lerobot_nero，含 nero/orbbec 注册代码）
├── pyAgxArm/           # Agilex Nero 机械臂 SDK 子模块
├── pyorbbecsdk/        # Orbbec 相机 SDK 子模块（需 C++ 编译）
├── scripts/            # Nero/Orbbec 相关测试与采集脚本
├── pyproject.toml      # 项目环境配置（基于 lerobot 的 pyproject.toml）
├── uv.lock             # 依赖锁文件
└── .venv/              # uv 管理的虚拟环境
```

### 子模块说明

| 子模块 | 来源 | 说明 |
|--------|------|------|
| `lerobot` | `shdiww/lerobot_nero` | 在原始 LeRobot 基础上注册了 nero 机器人、orbbec 相机、nero_gamepad 遥操作 |
| `pyAgxArm` | `agilexrobotics/pyAgxArm` | Agilex 机械臂 Python SDK，通过 CAN 总线控制 |
| `pyorbbecsdk` | `orbbec/pyorbbecsdk` | Orbbec 深度相机 SDK，需先 C++ 编译生成 .so 再安装 |

## 环境配置

### 前置条件

- Python >= 3.12
- [uv](https://docs.astral.sh/uv/) 包管理器
- CMake >= 3.15, C++14 编译器
- pybind11
- CAN 总线支持（Nero 机械臂需要）

### 1. 初始化子模块

```bash
git submodule update --init --recursive
```

### 2. 创建 uv 环境

```bash
uv lock
uv sync
```

### 3. 安装 Nero extra 依赖

```bash
uv sync --extra nero
```

这会安装 pygame（手柄）、python-can（CAN 总线）、scipy、pytracik（IK 求解）等依赖。

### 4. 安装 pyAgxArm（机械臂 SDK）

```bash
uv pip install -e ./pyAgxArm
```

### 5. 编译并安装 pyorbbecsdk（相机 SDK）

```bash
# 安装 pybind11（如果 uv sync 没有安装）
uv pip install pybind11

# C++ 编译
cd pyorbbecsdk
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -Dpybind11_DIR=$(../../.venv/bin/python -c "import pybind11; print(pybind11.get_cmake_dir())")
make -j$(nproc)
make install
cd ../..

# 安装 Python 包
uv pip install -e ./pyorbbecsdk
```

### 6. 验证安装

```bash
uv run python -c "import lerobot; import pyAgxArm; import pyorbbecsdk; print('All packages OK')"
```

## Nero 机器人在 LeRobot 中的注册

以下代码已集成在 `lerobot` 子模块（fork）中：

| 组件 | 注册类型 | 路径 |
|------|----------|------|
| Nero 机器人 | `RobotConfig.register_subclass("nero")` | `lerobot/src/lerobot/robots/nero/` |
| Orbbec 相机 | `CameraConfig.register_subclass("orbbec")` | `lerobot/src/lerobot/cameras/orbbec/` |
| Nero 手柄遥操作 | `TeleoperatorConfig.register_subclass("nero_gamepad")` | `lerobot/src/lerobot/teleoperators/nero_gamepad/` |

## 脚本说明

| 脚本 | 说明 |
|------|------|
| `scripts/lerobot_record_nero.py` | Nero 数据采集（手柄遥操作 + 录制） |
| `scripts/lerobot_teleoperate_nero.py` | Nero 手柄遥操作（不录制） |
| `scripts/test_nero_connect.py` | Nero 机械臂连接测试 |
| `scripts/test_nero_gamepad.py` | 手柄输入测试 |
| `scripts/test_orbbec_01_connect.py` | Orbbec 相机基础连接测试 |
| `scripts/test_orbbec_02_read_frame.py` | 单帧读取测试 |
| `scripts/test_orbbec_03_fps.py` | FPS 测试 |
| `scripts/test_orbbec_04_async_read.py` | 异步读取测试 |
| `scripts/test_orbbec_05_lerobot_integration.py` | LeRobot 相机接口集成测试 |
| `scripts/test_orbbec_06_nero_framework.py` | Nero 框架集成测试 |

## 注意事项

- `uv sync` 会覆盖通过 `uv pip install -e .` 安装的本地包（pyAgxArm、pyorbbecsdk2），每次 `uv sync` 后需重新安装
- CAN 总线使用前需配置：`sudo ip link set can0 up type can bitrate 1000000`
- pyorbbecsdk 的 `.so` 文件平台相关，当前编译配置为 `linux_x64`
