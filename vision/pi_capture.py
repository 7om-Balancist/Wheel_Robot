#!/usr/bin/env python3
"""树莓派相机采集脚本: 每 1 秒存一帧, 共 500 张, 零标注。

硬件参数参考 vision.py (已在该文件验证):
  - 摄像头节点依次尝试 [11, 21, 31, 41, 0, 1, 2]
  - V4L2 后端, 640x480

用法 (在树莓派上运行):
    python3 pi_capture.py                   # 存到脚本同目录 data_buff_debuff/raw_data/images/
    python3 pi_capture.py /path/to/dir      # 或指定保存目录

采集建议: 每 1 秒一张的空档里, 手动变换角度/距离/光照/背景, 让 500 张尽量多样;
固定机位连拍 500 张重复画面, 训练效果会差很多。
**一次采集要同时拍到两类方块** —— 两类分开采集会让"类别"和"场景/时段"绑死, 那正是
2026-09-29 实车误检的根因 (见 train.py 文件头)。宁可画面乱一点, 也要混着拍。

⚠ 2026-09-29 默认目录从 `data/images/` 改成 `data_buff_debuff/raw_data/images/`:
    以前默认往 `data/images/` 里丢, 而那个目录正是**旧擂台数据集**的图片目录 ——
    新采的图会和旧数据混在一起, 下次谁跑一遍 split_data.py 就一起进了训练集。
    默认目录**非空时直接拒绝写入**(见 main()), 因为新图叫 img_0001.jpg, 而标注导出
    叫 `img_ 0001 .jpg`, build_dataset3.py 抠帧号时两者都算 1 号帧, 会互相覆盖。
"""
import sys
import time
from pathlib import Path

import cv2

CAMERA_NODES = [11, 21, 31, 41, 0, 1, 2]  # 与 vision.py 一致
WIDTH, HEIGHT = 640, 480
TOTAL = 500
INTERVAL = 1.0


def open_camera():
    """按 vision.py 同款顺序尝试打开摄像头, 返回可用的 VideoCapture。"""
    for node in CAMERA_NODES:
        cap = cv2.VideoCapture(node, cv2.CAP_V4L2)
        if not cap.isOpened():
            continue
        ok, _ = cap.read()
        if not ok:
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        print(f"[capture] 使用摄像头 /dev/video{node} ({WIDTH}x{HEIGHT})")
        return cap
    return None


def main() -> None:
    explicit = len(sys.argv) > 1
    if explicit:
        out_dir = Path(sys.argv[1])
    else:
        out_dir = Path(__file__).resolve().parent / "data_buff_debuff" / "raw_data" / "images"
    # 默认目录非空就停下: 往已有的标注数据里追加 img_0001.jpg 会和 `img_ 0001 .jpg`
    # 撞帧号(build_dataset3.py 都用原帧号命名), 后写的那张会把先写的覆盖掉, 而且
    # **一声不响**。明确在命令行给了目录就按用户的意思来。
    if not explicit and out_dir.is_dir() and any(out_dir.iterdir()):
        print(f"[capture] ✗ 默认目录 {out_dir} 里已经有东西了, 不再往里写。\n"
              f"          继续往这儿采集会和已有的 img_XXXX 撞帧号并静默覆盖。二选一:\n"
              f"            换一个空目录:  python3 pi_capture.py /path/to/new_batch\n"
              f"            先挪走已有内容, 再重跑。")
        sys.exit(1)
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = open_camera()
    if cap is None:
        print("[capture] 所有候选摄像头节点都打不开, 请检查 USB 连接是否松动")
        sys.exit(1)

    for _ in range(10):  # 丢前几帧, 等曝光/自动增益稳定
        cap.read()

    print(f"[capture] 开始采集: 每 {INTERVAL:.0f}s 一张, 共 {TOTAL} 张 -> {out_dir}")
    count = 0
    while count < TOTAL:
        ok, frame = cap.read()
        if not ok:
            print("[capture] 读取失败, 1s 后重试")
            time.sleep(INTERVAL)
            continue
        path = out_dir / f"img_{count:04d}.jpg"
        cv2.imwrite(str(path), frame)
        print(f"[capture] {count + 1}/{TOTAL}  {path}")
        count += 1
        time.sleep(INTERVAL)

    cap.release()
    print(f"[capture] 完成, 共 500 张 -> {out_dir}\n"
          f"[capture] 把这个目录拷回电脑的 buff_debuff_yolo/data_buff_debuff/raw_data/images/,\n"
          f"[capture] 标注结果放同级的 labels/ (每张图一个同名 .txt, YOLO 格式),\n"
          f"[capture] 然后跑 python build_dataset3.py 重新划分 —— ⚠ 类别号必须是 0=debuff 1=buff。")


if __name__ == "__main__":
    main()
