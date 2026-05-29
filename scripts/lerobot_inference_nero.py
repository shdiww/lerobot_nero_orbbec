"""Nero 机械臂推理脚本 — 加载训练好的策略模型，在真实机器人上执行闭环控制。

数据流:
    robot.get_observation()
        → robot_obs_to_policy_obs()  → 预处理 (preprocessor)
        → policy.select_action()     → 后处理 (postprocessor)
        → action_tensor_to_dict()    → robot.send_action()

    每个episode: reset策略 → 循环观测→推理→执行 → go home

参数:
    --policy_path         模型 checkpoint 路径 (必需)
    --dataset_root        数据集本地路径 (用于加载 metadata)
    --dataset_repo_id     数据集 repo ID
    --n_episodes          执行 episode 数 (默认 5)
    --fps                 控制循环频率 (默认 30)
    --num_inference_steps Diffusion 去噪步数, 越少越快 (默认 10)
    --max_steps           每 episode 最大步数 (默认 800)
    --device              推理设备 (默认 cuda)
    --display_cameras     显示相机画面

Usage:
    python scripts/lerobot_inference_nero.py \
        --policy_path=outputs/two-cube-stack_act/checkpoints/100000/pretrained_model \
        --dataset_root=/home/blzgz/nero_data/dataset/20260524_113529/two-cube-stack_20260524_113529 \
        --dataset_repo_id=two-cube-stack_20260524_113529 \
        --n_episodes=10 \
        --display_cameras
"""

import os

os.environ.setdefault("ORBBEC_SDK_DISABLE_UVC", "1")

import time
from pathlib import Path

import cv2
import numpy as np
import torch

CAMERA_CONFIG = {
    "top": {
        "type": "orbbec",
        "fps": 30,
        "width": 1280,
        "height": 960,
        "serial_number": "CL87B530020",
        "crop_width": 640,
        "crop_height": 480,
    },
    "realsense": {
        "type": "intelrealsense",
        "fps": 30,
        "width": 640,
        "height": 480,
        "serial_number_or_name": "243622072771",
    },
}

JOINT_NAMES = [
    "joint1.pos", "joint2.pos", "joint3.pos",
    "joint4.pos", "joint5.pos", "joint6.pos", "joint7.pos",
    "gripper.pos",
]


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


def make_robot():
    from lerobot.robots.nero import Nero, NeroRobotConfig
    from lerobot.cameras.orbbec.configuration_orbbec import OrbbecCameraConfig
    from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig

    cameras = {}
    for key, cam_cfg in CAMERA_CONFIG.items():
        cam_type = cam_cfg.pop("type")
        if cam_type == "orbbec":
            cameras[key] = OrbbecCameraConfig(**cam_cfg)
        elif cam_type == "intelrealsense":
            cameras[key] = RealSenseCameraConfig(**cam_cfg)
        cam_cfg["type"] = cam_type

    robot_cfg = NeroRobotConfig(motion_mode="js", cameras=cameras)
    robot = Nero(robot_cfg)
    return robot


def robot_obs_to_policy_obs(obs_dict, device="cuda"):
    observation = {}
    for key, val in obs_dict.items():
        if isinstance(val, np.ndarray) and val.ndim == 3:
            t = torch.from_numpy(val.copy()).to(torch.uint8)
            t = t.unsqueeze(0)
            _, h, w, c = t.shape
            t = t.permute(0, 3, 1, 2).contiguous().float() / 255.0
            observation[f"observation.images.{key}"] = t.to(device)
        elif isinstance(val, (int, float)):
            observation[f"observation.state.{key}"] = torch.tensor([[val]], dtype=torch.float32, device=device)

    state_vals = []
    for name in JOINT_NAMES:
        if name in obs_dict:
            state_vals.append(float(obs_dict[name]))
    if state_vals:
        observation["observation.state"] = torch.tensor([state_vals], dtype=torch.float32, device=device)

    return observation


def action_tensor_to_dict(action):
    action_np = action.squeeze(0).cpu().numpy()
    action_dict = {}
    for i, name in enumerate(JOINT_NAMES):
        if i < len(action_np):
            action_dict[name] = float(action_np[i])
    return action_dict


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
    args = parser.parse_args()

    policy, preprocessor, postprocessor, ds_meta = load_policy(
        args.policy_path, args.device, args.dataset_root, args.dataset_repo_id, args.num_inference_steps
    )

    fps = args.fps
    dt = 1.0 / fps

    robot = make_robot()
    robot.connect()
    robot.move_to_home()

    print(f"Robot connected. Running {args.n_episodes} episodes at {fps}fps...")

    for ep in range(args.n_episodes):
        print(f"\n--- Episode {ep + 1}/{args.n_episodes} ---")
        policy.reset()

        step = 0
        max_steps = args.max_steps

        while step < max_steps:
            loop_start = time.perf_counter()

            obs_dict = robot.get_observation()
            observation = robot_obs_to_policy_obs(obs_dict, args.device)

            if preprocessor:
                observation = preprocessor(observation)

            infer_start = time.perf_counter()
            with torch.no_grad():
                action = policy.select_action(observation)
            infer_ms = (time.perf_counter() - infer_start) * 1000

            if postprocessor:
                action = postprocessor(action)

            action_dict = action_tensor_to_dict(action)
            robot.send_action(action_dict)

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
