'''
Description: Do not Edit
Author: hao.lin (voyah perception)
Date: 2025-07-07 16:43:42
LastEditors: Do not Edit
LastEditTime: 2025-07-08 16:51:21
'''
import os
import yaml
import subprocess
import shutil
import time

# 模板路径
LIO_YAML_TEMPLATE = "/Users/linhao/eggy_slam/config/lio.yaml"
MDET_YAML_TEMPLATE = "/Users/linhao/eggy_slam/config/m-detector.yaml"

# 你的 eggy 和 toRosbag 可执行文件路径
EGGY_BIN = "/Users/linhao/eggy_slam/build/eggy"
TOROSBAG_BIN = "/Users/linhao/eggy_slam/build/ToRosbag"

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def save_yaml(obj, path):
    with open(path, 'w') as f:
        yaml.dump(obj, f, default_flow_style=False)

def process_clip(clip_dir, work_dir):
    print(f"Processing {clip_dir} ...")
    t0 = time.time()
    # 路径准备
    pcd_folder = os.path.join(clip_dir, "pcd/middle")
    imu_txt = os.path.join(clip_dir, "rtk_imu.txt")
    bag_path = os.path.join(clip_dir, "clip.bag")
    lio_yaml = os.path.join(clip_dir, "lio.yaml")
    mdet_yaml = os.path.join(clip_dir, "m-detector.yaml")
    torosbag_yaml = os.path.join(clip_dir, "torosbag.yaml")

    # 统计帧数
    frame_count = len([f for f in os.listdir(pcd_folder) if f.endswith('.pcd')])

    # 1. 生成 toRosbag 的 config
    t1 = time.time()
    torosbag_cfg = {
        "pcd_folder": pcd_folder,
        "imu_txt": imu_txt,
        "bag_path": bag_path,
        # 其他参数可根据 ToRosbag 默认模板补充
        "point_topic": "/RS/lidar",
        "imu_topic": "/RS/imu"
    }
    save_yaml(torosbag_cfg, torosbag_yaml)
    t2 = time.time()

    # 2. 执行 ToRosbag
    print(f"  Running ToRosbag for {clip_dir} ...")
    subprocess.run([TOROSBAG_BIN, torosbag_yaml], check=True)
    t3 = time.time()

    # 3. 生成 lio.yaml
    lio_cfg = load_yaml(LIO_YAML_TEMPLATE)
    lio_cfg['rosbag'] = bag_path
    save_yaml(lio_cfg, lio_yaml)
    t4 = time.time()

    # 4. 生成 m-detector.yaml
    mdet_cfg = load_yaml(MDET_YAML_TEMPLATE)
    mdet_cfg['root_dir'] = clip_dir
    mdet_cfg['output_fusion_dir'] = os.path.join(clip_dir, "postprocessing")
    save_yaml(mdet_cfg, mdet_yaml)
    t5 = time.time()

    # 5. 执行 eggy
    print(f"  Running eggy for {clip_dir} ...")
    subprocess.run([
        EGGY_BIN,
        "--lio", lio_yaml,
        "--dyn", mdet_yaml,
        "--mode_fusion"
    ], check=True)
    t6 = time.time()

    print(f"Finished {clip_dir}: {frame_count} frames")
    print(f"  1. gen torosbag.yaml: {t2-t1:.2f}s")
    print(f"  2. ToRosbag:         {t3-t2:.2f}s")
    print(f"  3. gen lio.yaml:     {t4-t3:.2f}s")
    print(f"  4. gen m-detector:   {t5-t4:.2f}s")
    print(f"  5. eggy:             {t6-t5:.2f}s")
    print(f"  Total:               {t6-t0:.2f}s")
    # 可选：写入日志
    with open(os.path.join(clip_dir, "process_log.txt"), "w") as logf:
        logf.write(f"frames: {frame_count}\n")
        logf.write(f"gen_torosbag_yaml: {t2-t1:.2f} s\n")
        logf.write(f"torosbag: {t3-t2:.2f} s\n")
        logf.write(f"gen_lio_yaml: {t4-t3:.2f} s\n")
        logf.write(f"gen_mdet_yaml: {t5-t4:.2f} s\n")
        logf.write(f"eggy: {t6-t5:.2f} s\n")
        logf.write(f"total: {t6-t0:.2f} s\n")

def main():
    # 你的所有clip文件夹的父目录
    clips_root = "/Users/linhao/Desktop/clips"
    for name in os.listdir(clips_root):
        clip_dir = os.path.join(clips_root, name)
        if not os.path.isdir(clip_dir):
            continue
        # 判断是否为clip目录（可加更严格判断）
        if not os.path.exists(os.path.join(clip_dir, "pcd/middle")):
            continue
        try:
            process_clip(clip_dir, clips_root)
        except Exception as e:
            print(f"Error processing {clip_dir}: {e}")

if __name__ == "__main__":
    main()