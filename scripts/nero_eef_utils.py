"""Nero EEF (末端) 位姿工具 — 关节角 <-> 末端位姿 互转, 供采集/推理/回放共用.

EEF pose 约定 (旋转用 6D 连续表示, 即旋转矩阵前两行):
    [x, y, z, r00, r01, r02, r10, r11, r12, gripper]  共 10 维
    - 位置: base_link 坐标系, 米
    - 姿态: R[:2, :].flatten() (Zhou et al. 6D 表示, 处处连续;
      Nero 工具朝向在所有 euler 序下都贴近 ±π 卷绕边界, 故不用 euler/四元数)
    - gripper: 米制开合宽度 0 ~ NERO_GRIPPER_MAX_WIDTH_M

数据流:
    采集:  observation.state = FK(实测关节角) + gripper
           action            = FK(下发目标关节角) + gripper
    部署:  policy 输出 action(EEF) → 6D→R 正交化 → IK(seed=当前实测关节角) → 关节角
"""

import logging

import numpy as np

logger = logging.getLogger(__name__)

EEF_POSE_KEYS = [
    "eef.x", "eef.y", "eef.z",
    "eef.r00", "eef.r01", "eef.r02",
    "eef.r10", "eef.r11", "eef.r12",
]
EEF_KEYS = [*EEF_POSE_KEYS, "gripper.pos"]


def make_eef_action_features(use_gripper: bool = True) -> dict[str, type]:
    features = {key: float for key in EEF_POSE_KEYS}
    if use_gripper:
        features["gripper.pos"] = float
    return features


def create_eef_ik_adapter(
    urdf_path: str | None = None,
    base_link_name: str = "base_link",
    target_link_name: str = "gripper_base",
    timeout: float = 0.005,
):
    """创建 TracIKAdapter (复用 nero_gamepad 的求解器), 失败返回 None."""
    from lerobot.teleoperators.nero_gamepad.teleop_nero_gamepad import (
        TracIKAdapter,
        _try_create_tracik_solver,
    )

    if urdf_path is None:
        from lerobot.robots.nero.config_nero import NERO_URDF_PATH

        urdf_path = str(NERO_URDF_PATH)

    solver = _try_create_tracik_solver(
        urdf_path,
        base_link_name=base_link_name,
        target_link_name=target_link_name,
        timeout=timeout,
    )
    if solver is None:
        return None
    return TracIKAdapter(solver)


def joints_to_eef_dict(ik_adapter, joints) -> dict[str, float]:
    """FK: 7 关节角 -> {eef.x/y/z, eef.r00..r12} (无 gripper)."""
    pos, rot = ik_adapter.fk(np.asarray(joints, dtype=np.float64))
    r6 = np.asarray(rot, dtype=np.float64)[:2, :].flatten()
    return {
        "eef.x": float(pos[0]),
        "eef.y": float(pos[1]),
        "eef.z": float(pos[2]),
        "eef.r00": float(r6[0]), "eef.r01": float(r6[1]), "eef.r02": float(r6[2]),
        "eef.r10": float(r6[3]), "eef.r11": float(r6[4]), "eef.r12": float(r6[5]),
    }


def _rot6d_to_matrix(r6: np.ndarray) -> np.ndarray:
    """6D 表示 -> 旋转矩阵 (Gram-Schmidt 正交化, Zhou et al. 2018)."""
    a1 = np.asarray(r6[0:3], dtype=np.float64)
    a2 = np.asarray(r6[3:6], dtype=np.float64)
    b1 = a1 / (np.linalg.norm(a1) + 1e-9)
    b2 = a2 - np.dot(b1, a2) * b1
    b2 = b2 / (np.linalg.norm(b2) + 1e-9)
    b3 = np.cross(b1, b2)
    return np.stack([b1, b2, b3], axis=0)


def eef_dict_to_joints(ik_adapter, eef: dict[str, float], seed_joints, max_joint_step: float = 0.15):
    """IK: {eef.*} -> 7 关节角. 无解返回 None; 有解则相对 seed 限幅 max_joint_step 保证平滑.

    eef dict 只需包含 9 个位姿键 (xyz + 6D 旋转); gripper 由调用方单独处理.
    """
    pos = np.array([eef["eef.x"], eef["eef.y"], eef["eef.z"]], dtype=np.float64)
    r6 = np.array([eef[k] for k in EEF_POSE_KEYS[3:]], dtype=np.float64)
    rot = _rot6d_to_matrix(r6)

    seed = np.asarray(seed_joints, dtype=np.float64)
    result = ik_adapter.ik(pos, rot, seed=seed)
    if result is None:
        return None
    result = np.asarray(result, dtype=np.float64)

    if max_joint_step and max_joint_step > 0:
        result = seed + np.clip(result - seed, -max_joint_step, max_joint_step)
    return result
