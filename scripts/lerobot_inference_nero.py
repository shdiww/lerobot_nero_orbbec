"""Nero 机械臂推理脚本 — 加载训练好的策略模型，在真实机器人上执行闭环控制。

数据流:
    robot.get_observation()
        → robot_obs_to_policy_obs()  → 预处理 (preprocessor)
        → policy.select_action()     → 后处理 (postprocessor)
        → action_to_robot_dict()     → robot.send_action()

    关节数据集 (action=joint*.pos): 策略输出直接下发关节角.
    EEF 数据集 (action=eef.*, 由 lerobot_record_nero.py --eef_pose_state=true 采集):
        observation.state = FK(实测关节角) 换算的末端位姿
        action (末端位姿) → IK(seed=当前实测关节角) → 关节角下发
        IK 失败时保持上一帧关节目标, 且每帧相对当前关节限幅 (默认 0.15 rad) 保证安全.

    每个 episode: reset策略 → 循环观测→推理→执行 → go home

参数:
    --policy_path         模型 checkpoint 路径 (必需)
    --dataset_root        数据集本地路径 (用于加载 metadata, 决定 state/action 模式)
    --dataset_repo_id     数据集 repo ID
    --top_sn / --wrist_sn Orbbec 相机 SN (默认为当前硬件: Femto Bolt 全局 / Gemini 335 腕部)
    --n_episodes          执行 episode 数 (默认 5)
    --fps                 控制循环频率 (默认 30)
    --num_inference_steps Diffusion 去噪步数, 越少越快 (默认 10)
    --max_steps           每 episode 最大步数 (默认 800)
    --device              推理设备 (默认 cuda)
    --display_cameras     显示相机画面
    --max_joint_step      EEF 模式每帧关节变化限幅 (rad, 默认 0.15)

Usage:
    python scripts/lerobot_inference_nero.py \
        --policy_path=outputs/<task>_diffusion/checkpoints/<N>/pretrained_model \
        --dataset_root=<数据集路径> \
        --dataset_repo_id=<repo_id> \
        --n_episodes=1 \
        --display_cameras
"""

import os

os.environ.setdefault("ORBBEC_SDK_DISABLE_UVC", "1")

import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from nero_eef_utils import create_eef_ik_adapter, eef_dict_to_joints, joints_to_eef_dict

NERO_JOINT_NAMES = [
    "joint1", "joint2", "joint3",
    "joint4", "joint5", "joint6", "joint7",
]
JOINT_ACTION_NAMES = [f"{name}.pos" for name in NERO_JOINT_NAMES] + ["gripper.pos"]

# 当前硬件: 全局 Femto Bolt (1280x960 -> 裁剪 640x480), 腕部 Gemini 335 (原生 640x480)
DEFAULT_TOP_SN = "CL87B53009Z"
DEFAULT_WRIST_SN = "CP0F4630000K"


def load_policy(policy_path, device="cuda", dataset_root=None, dataset_repo_id=None, num_inference_steps=None):
    from lerobot.datasets import LeRobotDatasetMetadata
    from lerobot.policies import make_policy, make_pre_post_processors
    from lerobot.policies.pretrained import PreTrainedConfig

    cfg = PreTrainedConfig.from_pretrained(policy_path)
    cfg.device = device
    cfg.pretrained_path = Path(policy_path)
    if hasattr(cfg, "num_inference_steps") and num_inference_steps:
        cfg.num_inference_steps = num_inference_steps
        print(f"Override num_inference_steps={cfg.num_inference_steps}")

    ds_meta = None
    if dataset_root and dataset_repo_id:
        ds_meta = LeRobotDatasetMetadata(dataset_repo_id, root=dataset_root)

    policy = make_policy(cfg=cfg, ds_meta=ds_meta)
    policy.eval()

    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=cfg,
        pretrained_path=Path(policy_path),
    )

    return policy, preprocessor, postprocessor, ds_meta


def detect_eef_mode(ds_meta) -> bool:
    """根据数据集 metadata 的 action names 自动识别 EEF / 关节模式."""
    if ds_meta is None:
        return False
    names = ds_meta.features.get("action", {}).get("names", [])
    return "eef.x" in names


def make_robot(args):
    from lerobot.robots.nero import Nero, NeroRobotConfig
    from lerobot.cameras.orbbec.configuration_orbbec import OrbbecCameraConfig

    cameras = {}
    if args.top_sn:
        cameras["top"] = OrbbecCameraConfig(
            fps=30, width=1280, height=960, warmup_s=2,
            serial_number=args.top_sn,
            crop_width=args.top_crop_w, crop_height=args.top_crop_h, crop_y_offset=args.top_crop_y_offset,
        )
    if args.wrist_sn:
        cameras["wrist"] = OrbbecCameraConfig(
            fps=30, width=640, height=480, warmup_s=2,
            serial_number=args.wrist_sn,
        )
    robot_cfg = NeroRobotConfig(motion_mode="js", cameras=cameras)
    return Nero(robot_cfg)


def get_state_action_names(ds_meta):
    """state/action 维度名 (顺序即向量顺序, 必须与训练数据一致)."""
    if ds_meta is not None:
        action_names = list(ds_meta.features.get("action", {}).get("names", []))
        state_names = list(ds_meta.features.get("observation.state", {}).get("names", action_names))
        if action_names:
            return state_names, action_names
    return JOINT_ACTION_NAMES, JOINT_ACTION_NAMES


def build_state_vector(obs_dict, state_names, ik_adapter):
    """构造 observation.state 向量; EEF 模式用 FK(实测关节角) 换算."""
    if ik_adapter is not None and "eef.x" in state_names:
        joints = [obs_dict.get(f"{name}.pos") for name in NERO_JOINT_NAMES]
        if any(v is None for v in joints):
            return None
        eef = joints_to_eef_dict(ik_adapter, joints)
        eef["gripper.pos"] = obs_dict.get("gripper.pos", 0.0)
        return [eef[n] for n in state_names]
    return [float(obs_dict[n]) for n in state_names if n in obs_dict] or None


def robot_obs_to_policy_obs(obs_dict, state_vector, state_names, device="cuda"):
    observation = {}
    for key, val in obs_dict.items():
        if isinstance(val, np.ndarray) and val.ndim == 3:
            t = torch.from_numpy(val.copy()).to(torch.uint8)
            t = t.unsqueeze(0)
            _, h, w, c = t.shape
            t = t.permute(0, 3, 1, 2).contiguous().float() / 255.0
            observation[f"observation.images.{key}"] = t.to(device)

    if state_vector:
        observation["observation.state"] = torch.tensor([state_vector], dtype=torch.float32, device=device)
    return observation


def action_to_robot_dict(action_np, action_names, ik_adapter, seed_joints, last_joints, max_joint_step):
    """策略输出 -> robot.send_action 的 dict.

    EEF 模式: 末端位姿 -> IK(seed=实测关节角) -> 关节角; 无解时保持 last_joints.
    关节模式: 直接映射.
    """
    values = {name: float(v) for name, v in zip(action_names, action_np)}

    if "eef.x" in values:
        if ik_adapter is None or seed_joints is None:
            return None
        joints = eef_dict_to_joints(ik_adapter, values, seed_joints, max_joint_step)
        if joints is None:
            if last_joints is None:
                return None
            joints = last_joints
        action_dict = {f"{name}.pos": float(j) for name, j in zip(NERO_JOINT_NAMES, joints)}
        action_dict.update({k: v for k, v in values.items() if k == "gripper.pos"})
        return action_dict

    return {k: v for k, v in values.items() if k.endswith(".pos")}


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Nero inference")
    parser.add_argument("--policy_path", type=str, required=True)
    parser.add_argument("--dataset_root", type=str, default=None)
    parser.add_argument("--dataset_repo_id", type=str, default=None)
    parser.add_argument("--n_episodes", type=int, default=5)
    parser.add_argument("--display_cameras", action="store_true")
    parser.add_argument("--device", type=str, default="cuda")
    parser.add_argument("--fps", type=int, default=30, help="Control loop frequency")
    parser.add_argument("--num_inference_steps", type=int, default=10, help="Diffusion denoising steps (lower=faster, default=10)")
    parser.add_argument("--max_steps", type=int, default=800, help="Max steps per episode")
    parser.add_argument("--max_joint_step", type=float, default=0.15, help="EEF mode: per-step joint delta clip (rad)")
    parser.add_argument("--top_sn", type=str, default=DEFAULT_TOP_SN, help="Orbbec top camera SN (empty string to disable)")
    parser.add_argument("--wrist_sn", type=str, default=DEFAULT_WRIST_SN, help="Orbbec wrist camera SN (empty string to disable)")
    parser.add_argument("--top_crop_w", type=int, default=640)
    parser.add_argument("--top_crop_h", type=int, default=480)
    parser.add_argument("--top_crop_y_offset", type=int, default=10)
    args = parser.parse_args()

    policy, preprocessor, postprocessor, ds_meta = load_policy(
        args.policy_path, args.device, args.dataset_root, args.dataset_repo_id, args.num_inference_steps
    )

    state_names, action_names = get_state_action_names(ds_meta)
    eef_mode = "eef.x" in action_names

    ik_adapter = None
    if eef_mode:
        ik_adapter = create_eef_ik_adapter()
        if ik_adapter is None:
            raise RuntimeError("EEF dataset detected but TracIK init failed (check URDF)")
        print(f"EEF pose mode: state/action names = {state_names}")

    fps = args.fps
    dt = 1.0 / fps

    robot = make_robot(args)
    robot.connect()
    robot.move_to_home()

    print(f"Robot connected. Running {args.n_episodes} episodes at {fps}fps (mode={'eef' if eef_mode else 'joints'})...")

    for ep in range(args.n_episodes):
        print(f"\n--- Episode {ep + 1}/{args.n_episodes} ---")
        policy.reset()

        step = 0
        max_steps = args.max_steps
        last_joints = None

        while step < max_steps:
            loop_start = time.perf_counter()

            obs_dict = robot.get_observation()

            seed_joints = None
            if eef_mode:
                seed_joints = [obs_dict.get(f"{name}.pos") for name in NERO_JOINT_NAMES]
                if any(v is None for v in seed_joints):
                    seed_joints = None
                else:
                    seed_joints = [float(v) for v in seed_joints]
                    last_joints = list(seed_joints) if last_joints is None else last_joints

            state_vector = build_state_vector(obs_dict, state_names, ik_adapter)
            observation = robot_obs_to_policy_obs(obs_dict, state_vector, state_names, args.device)
            if observation.get("observation.state") is None:
                step += 1
                continue

            if preprocessor:
                observation = preprocessor(observation)

            infer_start = time.perf_counter()
            with torch.no_grad():
                action = policy.select_action(observation)
            infer_ms = (time.perf_counter() - infer_start) * 1000

            if postprocessor:
                action = postprocessor(action)

            action_np = action.squeeze(0).cpu().numpy()
            action_dict = action_to_robot_dict(
                action_np, action_names, ik_adapter, seed_joints, last_joints, args.max_joint_step
            )
            if action_dict:
                robot.send_action(action_dict)
                if eef_mode and all(f"{n}.pos" in action_dict for n in NERO_JOINT_NAMES):
                    last_joints = [action_dict[f"{n}.pos"] for n in NERO_JOINT_NAMES]

            if step % 30 == 0:
                print(f"  step {step}: infer={infer_ms:.1f}ms")

            if args.display_cameras:
                for cam_key, cam in robot.cameras.items():
                    frame = cam.read_latest()
                    if frame is not None:
                        if frame.ndim == 3 and frame.shape[2] == 3 and frame.dtype == np.uint8:
                            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        cv2.imshow(cam_key, frame)
                cv2.waitKey(1)

            elapsed = time.perf_counter() - loop_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

            step += 1

        print(f"Episode {ep + 1} done ({step} steps)")
        robot.move_to_home()
        time.sleep(2.0)

    robot.disconnect()
    print("All episodes completed.")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
