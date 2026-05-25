## can连接
```bash
sudo ip link set can0 up type can bitrate 1000000
```

## 机械臂遥操-两摄像头
```bash
python scripts/lerobot_teleoperate_nero.py \
    --robot.type=nero \
    --teleop.type=nero_gamepad \
    --use_orbbec_camera=true \
    --orbbec_crop_width=640 \
    --orbbec_crop_height=480 \
    --orbbec_sn=CL87B530020 \
    --use_realsense_camera=true \
    --realsense_sn=243622072771 --display_cameras=true \
    --robot.motion_mode=js
```
## 数据集录制
```bash
python scripts/lerobot_record_nero.py  \
    --robot.type=nero --teleop.type=nero_gamepad  \
    --use_orbbec_camera=true --orbbec_sn=CL87B530020  \
    --orbbec_crop_width=640 --orbbec_crop_height=480    \
    --orbbec_crop_y_offset=10   --use_realsense_camera=true \
    --realsense_sn=243622072771 --display_cameras=true \
    --dataset.repo_id=cube-stack  \
    --dataset.single_task="cube-stack"   \
    --dataset.num_episodes=10 
```
## 实机回放
```bash
lerobot-replay --robot.type=nero \
    --robot.motion_mode=js \
    --dataset.repo_id=nero_dual_cam_task_20260520_104659  \
    --dataset.root=/home/blzgz/nero_data/dataset/20260520_104659/nero_dual_cam_task_20260520_104659  \
    --dataset.episode=0   \
    --dataset.fps=30 
```


## 可视化采集的数据
```bash
lerobot-dataset-viz --repo-id nero_dual_cam_task_20260520_124142 \
    --root ../nero_data/dataset/20260520_124142/nero_dual_cam_task_20260520_124142/ \
    --episode-index 2 --mode local
```

## 训练
```bash
lerobot-train     
    --dataset.repo_id=two-cube-stack     \
    --dataset.root=/home/blzgz/nero_data/dataset/20260524_113529/two-cube-stack_20260524_113529   \
    --policy.type=diffusion     \
    --wandb.enable=true     \
    --wandb.project=nero-two-cube-stack    \
    --policy.n_obs_steps=3     \
    --batch_size=4     \
    --steps=100000    \
    --save_freq=20000     \
    --eval_freq=20000     \
    --policy.push_to_hub=false    \
    --output_dir=outputs/two-cube-stack_diffusion    \
    --job_name=two-cube-stack-diffusion
```

## 实机推理

```bash
python scripts/lerobot_inference_nero.py     \
    --policy_path=outputs/two-cube-stack_diffusion/checkpoints/100000/pretrained_model     \
    --dataset_root=/home/blzgz/lerobot_nero_orbbec/dataset/20260524_113529/two-cube-stack_20260524_113529     \
    --dataset_repo_id=two-cube-stack_20260524_113529  \
    --n_episodes=1     \
    --display_cameras     \
    --num_inference_steps=15
```