from huggingface_hub import HfApi
from lerobot.datasets import LeRobotDataset
REPO_ID = "blzgz/two-cube-stack"
LOCAL_ROOT = "dataset/20260524_113529/two-cube-stack_20260524_113529"
api = HfApi()
api.create_repo(repo_id=REPO_ID, repo_type="dataset", private=True, exist_ok=True)
ds = LeRobotDataset(REPO_ID, root=LOCAL_ROOT)
ds.push_to_hub(private=True)