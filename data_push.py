"""数据集推送 HuggingFace Hub 工具.

将本地 LeRobotDataset 推送到 HuggingFace Hub 私有仓库。
修改下方 REPO_ID 和 LOCAL_ROOT 为实际值后运行。

Usage:
    python data_push.py
"""
from huggingface_hub import HfApi
from lerobot.datasets import LeRobotDataset
REPO_ID = "blzgz/two-cube-stack"
LOCAL_ROOT = "dataset/20260524_113529/two-cube-stack_20260524_113529"
api = HfApi()
api.create_repo(repo_id=REPO_ID, repo_type="dataset", private=True, exist_ok=True)
ds = LeRobotDataset(REPO_ID, root=LOCAL_ROOT)
ds.push_to_hub(private=True)