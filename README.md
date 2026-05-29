# lerobot_nero_orbbec

基于 [HuggingFace LeRobot](https://github.com/huggingface/lerobot) v0.5.2 的 Agilex Nero 7-DOF 机械臂 + Orbbec/RealSense 相机集成项目。

支持两种遥操作方式：**Xbox 手柄**（笛卡尔增量 + IK）和 **Pico VR**（6D 位姿追踪 + IK），覆盖数据采集、训练、推理全流程。

## 项目结构

```
lerobot_nero_orbbec/
├── lerobot/            # LeRobot 子模块（fork，含 nero/orbbec/pico_vr 注册代码）
├── pyAgxArm/           # Agilex Nero 机械臂 SDK 子模块（CAN 总线控制）
├── pyorbbecsdk/        # Orbbec 深度相机 SDK 子模块（C++ 编译 + pybind11）
├── scripts/            # 采集、遥操作、推理、测试脚本
├── data_push.py        # 数据集推送 HuggingFace Hub 工具
├── pyproject.toml      # 项目环境配置（基于 lerobot pyproject.toml 扩展）
├── uv.lock             # 依赖锁文件
└── .venv/              # uv 管理的虚拟环境
```

### 子模块说明

| 子模块 | 来源 | 说明 |
|--------|------|------|
| `lerobot` | `shdiww/lerobot_nero` | 在原始 LeRobot v0.5.2 基础上注册了 nero 机器人、orbbec 相机、nero_gamepad 和 pico_vr 遥操作 |
| `pyAgxArm` | `agilexrobotics/pyAgxArm` | Agilex 机械臂 Python SDK，通过 CAN 总线控制，支持 move_j(插补)/move_js(伺服) 两种运动模式 |
| `pyorbbecsdk` | `orbbec/pyorbbecsdk` | Orbbec 深度相机 SDK，需先 C++ 编译生成 .so 再安装，支持 RGB/Depth 对齐流 |

### 已注册的 LeRobot 组件

| 组件 | 注册类型 | 路径 |
|------|----------|------|
| Nero 机器人 | `RobotConfig.register_subclass("nero")` | `lerobot/src/lerobot/robots/nero/` |
| Orbbec 相机 | `CameraConfig.register_subclass("orbbec")` | `lerobot/src/lerobot/cameras/orbbec/` |
| Nero 手柄遥操作 | `TeleoperatorConfig.register_subclass("nero_gamepad")` | `lerobot/src/lerobot/teleoperators/nero_gamepad/` |
| Pico VR 遥操作 | `TeleoperatorConfig.register_subclass("pico_vr")` | `lerobot/src/lerobot/teleoperators/pico_vr/` |

### 数据流架构

```
遥操作输入                    机器人控制
┌──────────────┐             ┌──────────────┐
│  Xbox 手柄    │             │              │
│  或 Pico VR  │──get_action()──> IK 求解 ──> send_action()──> Nero 机械臂
│              │  (6D delta)   (TracIK)   (joint angles)    (CAN move_js)
└──────────────┘             └──────────────┘
                                    │
相机输入                     ┌──────┴──────┐
┌──────────────┐             │  get_observation()  │
│  Orbbec      │──read_latest()──> 7关节角 + gripper │
│  RealSense   │             └──────────────┘
└──────────────┘                    │
                                    ▼
                          ┌──────────────────┐
                          │  LeRobotDataset   │
                          │  (采集时写入)      │
                          └──────────────────┘
                                    │
                                    ▼
                          ┌──────────────────┐
                          │  训练 (lerobot-train) │
                          │  推理 (inference_nero) │
                          └──────────────────┘
```

## 环境配置（完整流程）

### 前置条件

- Python >= 3.12
- [uv](https://docs.astral.sh/uv/) 包管理器
- CMake >= 3.15, C++14 编译器
- CAN 总线支持（Nero 机械臂需要）
- Git（含 submodule 支持）
- XRoboToolkit PC Service（Pico VR 遥操作需要，详见 [Pico VR 测试](#pico-vr-测试脚本)）

### Step 1: 克隆仓库并初始化子模块

```bash
git clone --recurse-submodules git@github.com:shdiww/lerobot_nero_orbbec.git
# 如果忘记 --recurse-submodules，可以事后执行：
git submodule update --init --recursive
```

### Step 2: 创建 uv 环境

```bash
uv lock
uv sync
```

### Step 3: 安装项目依赖

```bash
# Nero extra（手柄、CAN、IK 等）
uv sync --extra nero

# 训练相关（如需训练模型）
uv sync --extra dataset --extra diffusion --extra training

# 仿真环境（如需 PushT 仿真验证）
uv sync --extra pusht

# RealSense 相机
uv sync --extra intelrealsense
```

> **注意**：`uv sync` 会覆盖通过 `uv pip install` 安装的手动包。每次 `uv sync` 后需重新执行 Step 4 和 Step 5。

### Step 4: 安装 pyAgxArm（机械臂 SDK）

```bash
uv pip install -e ./pyAgxArm
```

### Step 5: 编译并安装 pyorbbecsdk（Orbbec 相机 SDK）

```bash
# 安装 pybind11
uv pip install pybind11

# C++ 编译（必须用 uv 环境的 pybind11 和 Python，不能用系统级的）
cd pyorbbecsdk
rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -Dpybind11_DIR=$(/home/blzgz/lerobot_nero_orbbec/.venv/bin/python -c "import pybind11; print(pybind11.get_cmake_dir())") \
  -DPython3_EXECUTABLE=/home/blzgz/lerobot_nero_orbbec/.venv/bin/python
make -j$(nproc)
make install
cd ../..

# 安装 Python 包（必须用非 editable 模式，否则 .so 不会被正确加载）
uv pip install ./pyorbbecsdk
```

> **关键**：pyorbbecsdk **必须用非 editable 模式**安装（`uv pip install ./pyorbbecsdk`，不加 `-e`）。editable 模式会导致 Python 从源码目录加载而非 `install/lib/`，找不到 `.so` 文件。

### Step 6: 安装 XRoboToolkit SDK（Pico VR 遥操作）

Pico VR 遥操作依赖 `xrobotoolkit_sdk`，需手动编译安装：

```bash
cd /home/blzgz/XRoboToolkit-Teleop-Sample-Python/dependencies/XRoboToolkit-PC-Service-Pybind
bash setup_ubuntu.sh
```

安装后需确保 XRoboToolkit PC Service 在后台运行，Pico 头显与 PC 在同一 WiFi 下。

### Step 7: 验证安装

```bash
/home/blzgz/lerobot_nero_orbbec/.venv/bin/python -c "
import pyorbbecsdk; print('pyorbbecsdk OK')
import pyAgxArm; print('pyAgxArm OK')
import lerobot; print('lerobot OK')
import xrobotoolkit_sdk as xrt; xrt.init(); print('xrobotoolkit_sdk OK'); xrt.close()
"
```

## CAN 总线配置

```bash
sudo ip link set can0 up type can bitrate 1000000
```

建议写入开机自启：

```bash
echo 'sudo ip link set can0 up type can bitrate 1000000' >> ~/.bashrc
```

## 多相机配置

### 相机检测

```bash
# 检测所有 Orbbec 相机
uv run python -c "
from lerobot.cameras.orbbec import OrbbecCamera
cams = OrbbecCamera.find_cameras()
for c in cams:
    print(f'  {c[""name""]}  SN={c[""sn""]}')
"

# 检测 RealSense 相机
uv run python -c "
import pyrealsense2 as rs
ctx = rs.context()
for d in ctx.devices:
    print(f'  SN={d.get_info(rs.camera_info.serial_number)}  Name={d.get_info(rs.camera_info.name)}')
"
```

### Orbbec + RealSense 共存的 uvcvideo 驱动冲突

Orbbec SDK 和 RealSense 都依赖 Linux `uvcvideo` 内核驱动，同时使用时可能冲突。

**解决方案**：确保 `uvcvideo` 已加载，RealSense 优先使用 uvcvideo，Orbbec 使用其私有驱动（libusb）。

```bash
# 1. 确认 uvcvideo 已加载
sudo modprobe uvcvideo
lsmod | grep uvcvideo

# 2. 如果 Orbbec 占用了 uvcvideo 导致 RealSense 找不到设备
export ORBBEC_SDK_DISABLE_UVC=1

# 3. 重新拔插所有相机
```

**排查步骤**（如果 RealSense 还是 0 devices）：

```bash
lsusb | grep -i "Intel\|RealSense"

sudo apt install librealsense2-udev-rules 2>/dev/null || \
  curl -s https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules | \
  sudo tee /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**建议**：将 `ORBBEC_SDK_DISABLE_UVC=1` 写入 `~/.bashrc` 永久生效：

```bash
echo 'export ORBBEC_SDK_DISABLE_UVC=1' >> ~/.bashrc
source ~/.bashrc
```

## 遥操作与数据采集

### Xbox 手柄遥操作

**按键映射：**

| 按键 | 功能 |
|------|------|
| L-stick | 末端 X/Y 平移 |
| R-stick | 末端 Z 平移 / Z 旋转 |
| D-pad | 末端 X/Y 旋转 |
| A | 夹爪闭合 |
| B | 夹爪张开 |
| Y | 回 Home 位置 |
| Start | 开始录制 |
| Back | 结束录制 (自动 go home + 保存) |
| Home | 连接机械臂 |

**纯遥操作（不录制）：**

```bash
python scripts/lerobot_teleoperate_nero.py \
    --robot.type=nero --teleop.type=nero_gamepad \
    --display_data=true

# 带相机显示
python scripts/lerobot_teleoperate_nero.py \
    --robot.type=nero --teleop.type=nero_gamepad \
    --use_orbbec_camera=true --orbbec_sn=CL87B530020 \
    --orbbec_crop_width=640 --orbbec_crop_height=480 --orbbec_crop_y_offset=10 \
    --display_cameras=true
```

**数据采集：**

```bash
python scripts/lerobot_record_nero.py \
    --robot.type=nero --teleop.type=nero_gamepad \
    --use_orbbec_camera=true --orbbec_sn=CL87B530020 \
    --orbbec_crop_width=640 --orbbec_crop_height=480 \
    --orbbec_crop_y_offset=10 \
    --use_realsense_camera=true --realsense_sn=243622072771 \
    --display_cameras=true \
    --dataset.repo_id=nero_dual_cam_task \
    --dataset.single_task="pick up the cup" \
    --dataset.num_episodes=10
```

录制流程：启动 → 待命 → 按 Start 开始录制 → 操作 → 按 Back 结束 → 自动 go home → 下一个 episode

键盘快捷键：`Enter`=开始录制，`→`=结束录制，`←`=重录当前 episode，`Esc`=退出

### Pico VR 遥操作

**按键映射（右手控制器）：**

| 按键 | 功能 |
|------|------|
| Grip (>0.9) | 激活 6D 位姿追踪（握住移动，松开冻结） |
| A | 夹爪闭合（增量） |
| B | 夹爪张开（增量） |
| X | 回 Home 位置 |
| Y | 标记成功 + 回 Home |
| 右 Menu | 开始录制 |
| 左 Menu | 结束录制 (自动 go home + 保存) |

**纯遥操作：**

```bash
python scripts/lerobot_teleoperate_nero_pico.py \
    --robot.type=nero --teleop.type=pico_vr \
    --display_cameras=true
```

**数据采集：**

```bash
python scripts/lerobot_record_nero_pico.py \
    --robot.type=nero --teleop.type=pico_vr \
    --use_orbbec_camera=true --orbbec_sn=CL87B530020 \
    --orbbec_crop_width=640 --orbbec_crop_height=480 \
    --orbbec_crop_y_offset=10 \
    --use_realsense_camera=true --realsense_sn=243622072771 \
    --display_cameras=true \
    --dataset.repo_id=nero_pico_task \
    --dataset.single_task="pick up the cup" \
    --dataset.num_episodes=10
```

录制流程同 Xbox 手柄，区别在于用 Grip 键激活 6D 追踪，右/左 Menu 键控制录制。

### 多相机组合

**Orbbec + RealSense 混合：**

```bash
python scripts/lerobot_teleoperate_nero.py \
    --robot.type=nero --teleop.type=nero_gamepad \
    --use_orbbec_camera=true --orbbec_sn=CL87B530020 \
    --use_realsense_camera=true --realsense_sn=243622072771 \
    --display_cameras=true
```

数据采集同理，替换为 `lerobot_record_nero.py` 或 `lerobot_record_nero_pico.py` 并添加 `--dataset` 参数。

## 推理部署

加载训练好的策略模型在真实 Nero 机器人上执行：

```bash
python scripts/lerobot_inference_nero.py \
    --policy_path=outputs/two-cube-stack_act/checkpoints/100000/pretrained_model \
    --dataset_root=/home/blzgz/nero_data/dataset/20260524_113529/two-cube-stack_20260524_113529 \
    --dataset_repo_id=two-cube-stack_20260524_113529 \
    --n_episodes=10 \
    --display_cameras
```

**参数说明：**

| 参数 | 说明 |
|------|------|
| `--policy_path` | 训练好的模型 checkpoint 路径 |
| `--dataset_root` | 对应数据集的本地路径（用于加载 metadata） |
| `--dataset_repo_id` | 数据集 repo ID |
| `--n_episodes` | 执行 episode 数 |
| `--fps` | 控制循环频率（默认 30） |
| `--num_inference_steps` | Diffusion 去噪步数（越少越快，默认 10） |
| `--max_steps` | 每个 episode 最大步数（默认 800） |
| `--device` | 推理设备（默认 cuda） |
| `--display_cameras` | 显示相机画面 |

推理数据流：`get_observation()` → preprocess → `policy.select_action()` → postprocess → `send_action()`

## 数据集推送 HuggingFace Hub

```bash
python data_push.py
```

修改 `data_push.py` 中的 `REPO_ID` 和 `LOCAL_ROOT` 为实际值即可。

## 训练

### 仿真环境验证（PushT）

```bash
# 安装依赖
uv sync --extra pusht --extra diffusion --extra dataset --extra training
uv pip install -e ./pyAgxArm  # uv sync 会覆盖，需重装
uv pip install ./pyorbbecsdk  # 同上
uv pip install "httpx[socks]" # 代理支持
uv pip install "pymunk>=6.6.0,<6.7.0"  # gym-pusht 兼容

# 训练
lerobot-train \
  --policy.type=diffusion \
  --dataset.repo_id=lerobot/pusht \
  --policy.repo_id=local/pusht_diffusion \
  --policy.push_to_hub=false \
  --steps=100000 \
  --batch_size=8 \
  --wandb.enable=false \
  --output_dir=outputs/pusht_diffusion \
  --save_freq=20000 \
  --eval_freq=20000

# 评估（需用同步模式，避免 gym-pusht 多进程注册问题）
lerobot-eval \
  --policy.path=outputs/pusht_diffusion/checkpoints/100000/pretrained_model \
  --env.type=pusht \
  --eval.n_episodes=10 \
  --eval.use_async_envs=false \
  --policy.push_to_hub=false
```

### 训练参数说明

| 参数 | 说明 |
|------|------|
| `--steps=N` | 总训练步数（1 step = 1 个 batch 的梯度更新），非 epoch |
| `--batch_size=N` | 每步的 batch 大小 |
| `--policy.optimizer_lr` | 学习率 |
| `--save_freq=N` | 每 N 步保存一次 checkpoint |
| `--policy.push_to_hub=false` | 禁止推送到 HuggingFace Hub |
| `--wandb.enable=false` | 禁止 wandb 日志 |

### 关键代码位置

| 改什么 | 去哪改 |
|--------|--------|
| 超参（lr, steps, batch_size） | 命令行 `--steps`, `--batch_size`, `--policy.optimizer_lr` |
| Diffusion 专属参数 | `lerobot/src/lerobot/policies/diffusion/configuration_diffusion.py` |
| U-Net 网络结构 | `lerobot/src/lerobot/policies/diffusion/modeling_diffusion.py` |
| 数据预处理 | `lerobot/src/lerobot/policies/diffusion/processor_diffusion.py` |
| 训练循环 | `lerobot/src/lerobot/scripts/lerobot_train.py` |

## 脚本说明

### 主脚本

| 脚本 | 说明 |
|------|------|
| `scripts/lerobot_teleoperate_nero.py` | Xbox 手柄遥操作（不录制），支持 Orbbec/RealSense 相机显示 |
| `scripts/lerobot_teleoperate_nero_pico.py` | Pico VR 遥操作（不录制），6D 位姿追踪 + IK |
| `scripts/lerobot_record_nero.py` | Xbox 手柄数据采集（遥操作 + LeRobotDataset 录制） |
| `scripts/lerobot_record_nero_pico.py` | Pico VR 数据采集（遥操作 + LeRobotDataset 录制） |
| `scripts/lerobot_inference_nero.py` | 推理部署，加载策略模型在真实机器人上执行 |
| `data_push.py` | 数据集推送 HuggingFace Hub 工具 |

### Nero 机械臂测试脚本

| 脚本 | 说明 |
|------|------|
| `scripts/test_nero_connect.py` | Nero 连接/断开/重新连接生命周期测试 |
| `scripts/test_nero_gamepad.py` | Xbox 手柄连接、按键映射、增量动作读取测试 |

### Orbbec 相机测试脚本

| 脚本 | 说明 |
|------|------|
| `scripts/test_orbbec_01_connect.py` | Orbbec Pipeline 连接与断开基础测试 |
| `scripts/test_orbbec_02_read_frame.py` | 单帧读取 + 自动曝光/白平衡测试 |
| `scripts/test_orbbec_03_fps.py` | 连续读帧 + 帧率测量 |
| `scripts/test_orbbec_04_async_read.py` | 后台线程异步读帧 + 延迟测量 |
| `scripts/test_orbbec_05_lerobot_integration.py` | OrbbecCamera LeRobot Camera 接口集成测试 |
| `scripts/test_orbbec_06_nero_framework.py` | OrbbecCamera 在 Nero 框架下的工作测试 |
| `scripts/test_orbbec_camera.py` | Orbbec RGB + Depth 对齐流实时显示（SDK 直接调用） |

### Pico VR 测试脚本

| 脚本 | 说明 |
|------|------|
| `scripts/test_pico_sdk_import.py` | xrobotoolkit_sdk 导入与 API 完整性验证 |
| `scripts/test_pico_buttons.py` | Pico VR 原始按键/摇杆/位姿数据读取（SDK 直接调用） |
| `scripts/test_pico_data_read.py` | Pico VR 控制器数据完整读取测试（位姿+按键+摇杆） |
| `scripts/test_pico_vr_tracking.py` | Pico VR 6D 追踪 + IK 求解终端测试 |
| `scripts/test_pico_vr_plot.py` | Pico VR 6D 追踪曲线可视化（matplotlib） |
| `scripts/test_pico_buttons_teleop.py` | PicoVRController 类按键 + 6D delta 集成测试 |

## 常见问题

### 1. `uv sync` 后 pyAgxArm / pyorbbecsdk 丢失

`uv sync` 会覆盖手动 `uv pip install` 的包。每次 `uv sync` 后需重新安装：

```bash
uv pip install -e ./pyAgxArm
uv pip install ./pyorbbecsdk
```

### 2. pyorbbecsdk ImportError: cannot import name 'OBFormat'

**原因**：editable 安装（`-e`）导致 Python 从源码目录加载而非 `install/lib/`。

**解决**：用非 editable 模式安装：
```bash
uv pip install ./pyorbbecsdk  # 不要加 -e
```

### 3. pyorbbecsdk 编译失败：`invalid use of incomplete type 'PyFrameObject'`

**原因**：系统级 pybind11（2.9.x）太旧，不兼容 Python 3.13。

**解决**：cmake 时指定 uv 环境的 pybind11 和 Python：
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -Dpybind11_DIR=$(.venv/bin/python -c "import pybind11; print(pybind11.get_cmake_dir())") \
  -DPython3_EXECUTABLE=.venv/bin/python
```

### 4. opencv 报错 `cvDestroyAllWindows` 或 `cv2.ROTATE_90_CLOCKWISE` 不存在

**原因**：安装了 `opencv-python-headless`（无 GUI）或安装损坏。

**解决**：项目 pyproject.toml 已改为 `opencv-python`（非 headless）。如果 cv2 模块损坏，重装：
```bash
uv pip uninstall opencv-python && uv pip install opencv-python
```

### 5. RealSense 找不到设备（0 devices）

**排查**：
1. `lsusb | grep Intel` 确认 USB 识别
2. `sudo modprobe uvcvideo` 确认驱动加载
3. 安装 udev 规则：`sudo apt install librealsense2-udev-rules`
4. Orbbec 与 uvcvideo 冲突：`export ORBBEC_SDK_DISABLE_UVC=1`

### 6. gym-pusht 评估失败

**原因**：`gym-pusht` 注册在多进程中失效，或 `pymunk` 版本不兼容。

**解决**：
```bash
--eval.use_async_envs=false
uv pip install "pymunk>=6.6.0,<6.7.0"
```

### 7. HuggingFace Hub 401 Unauthorized

**原因**：训练完后自动推送模型到 Hub，但未登录。

**解决**：加 `--policy.push_to_hub=false` 禁止推送。

### 8. SOCKS 代理错误

**原因**：环境变量设置了 SOCKS 代理但缺少 `socksio` 包。

**解决**：`uv pip install "httpx[socks]"`

### 9. 子模块 URDF 文件丢失

**原因**：上游 lerobot 的 `.gitignore` 包含 `*.urdf`，pull 后被忽略。

**解决**：在 fork 中删除 `.gitignore` 里的 `*.urdf` 规则，然后 `git add -f` 强制添加：
```bash
cd lerobot
git add -f src/lerobot/robots/nero/nero_description/urdf/*.urdf
git commit -m "fix: track URDF files"
git push
```

### 10. Pico VR 连接失败

**排查**：
1. 确认 XRoboToolkit PC Service 在后台运行
2. 确认 Pico 头显与 PC 在同一 WiFi 下
3. 运行 `scripts/test_pico_sdk_import.py` 验证 SDK 可用
4. 运行 `scripts/test_pico_buttons.py` 验证原始数据可读
