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
    --orbbec_crop_y_offset=10 \
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