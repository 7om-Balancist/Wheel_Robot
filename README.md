# 轮式格斗机器人 · 树莓派视觉节点

中国机器人大赛轮式格斗赛项的视觉端。**只负责一件事：从摄像头画面里认出能量块，把"看到了什么"通过串口告诉控制器。** 所有决策权（要不要躲、要不要追）都在控制器，视觉不参与运动控制。

- 目标检测：**YOLO11n**（Ultralytics 训练，导出 ONNX / ncnn），两类目标
  - `0: debuff` 减益块 —— 撞到自己扣分，必须躲
  - `1: buff` 增益块 —— 撞下擂台己方加分，空闲时去追
- 推理后端：**ONNX Runtime** 或 **ncnn**（自动优先 ncnn，树莓派上更快）
- 输出：**12 字节定长帧，20 Hz**，通过串口发给控制器（`/dev/ttyACM0`，115200 8N1）

对应控制器端固件见分支 [`Car_Controller`](https://github.com/7om-Balancist/Wheel_Robot/tree/Car_Controller)。

---

## 目录

```
vision/
  pi_detect.py            主程序：采集 + 检测 + 发送（三线程）
  pi_capture.py           训练数据采集脚本（定时抓图，默认 500 张 / 1 秒一张）
  best_ncnn_model/
    metadata.yaml         模型元数据：imgsz 640、stride 32、类别名 debuff/buff
    model.ncnn.param      ncnn 网络结构（权重 .bin 未入库，见下）
    model_ncnn.py         pnnx 导出的参考实现（仅供对照，运行时不用）
```

---

## 模型文件（**未包含在仓库内**）

模型权重体积较大（onnx 约 10 MB、ncnn bin 约 5 MB、训练权重 pt 约 5 MB），因此**不入库**。运行前需要自行放置：

| 后端 | 需要放置的文件 | 位置 |
|---|---|---|
| ncnn（推荐，默认优先） | `model.ncnn.bin` | 与 `model.ncnn.param` 同目录，即 `vision/best_ncnn_model/` |
| ONNX Runtime | `best.onnx` | 与 `pi_detect.py` 同目录，即 `vision/` |

> ⚠ 仓库里保留了 `model.ncnn.param`（网络结构，22 KB）与 `metadata.yaml`，**但 ncnn 的权重 `model.ncnn.bin` 不在**。
> 所以克隆后直接跑会**优先尝试 ncnn 并失败**，此时请改用 `--runtime onnx`（需要 `best.onnx`）。

**从训练权重自行导出**（在有 ultralytics 的机器上）：

```bash
# ONNX
yolo export model=best.pt format=onnx imgsz=640 opset=12

# ncnn（ultralytics 会生成 best_ncnn_model/ 目录，含 param + bin）
yolo export model=best.pt format=ncnn imgsz=640 half=False
```

模型规格（`best_ncnn_model/metadata.yaml`）：YOLO11n、`imgsz 640×640`、`task=detect`、`stride 32`、`nc=2`（debuff / buff）。

模型路径的解析顺序：`--model` 参数 → 环境变量 `DEBUFF_MODEL` → 在当前目录/脚本目录下自动发现
（`best.onnx`、`best_384.onnx`、`best_512.onnx`、`best_320.onnx`；ncnn 目录 `best_ncnn_model/`、
`best_ncnn/`、`best_ncnn_384/`），发现多个时会按内容挑选并告警。

---

## 环境

树莓派 5 + Raspberry Pi OS（Bookworm），Python 3.9+。

```bash
# 基础依赖
pip install opencv-python numpy pyserial

# 推理后端二选一（或都装）
pip install onnxruntime          # ONNX 后端
pip install ncnn                 # ncnn 后端（也可用 pip 的 ncnn-python 轮子）

# 仅在导出/核验模型时需要
pip install onnx ultralytics pyyaml
```

> Python 包都是**懒加载**的：没装 `onnxruntime` 只是用不了 onnx 后端，不会导致脚本起不来。

---

## 运行

```bash
cd vision

python3 pi_detect.py                      # 正常跑：自动选后端（ncnn → onnx）、自动找模型
DISPLAY=1 python3 pi_detect.py            # 带画面调试（需显示器），ESC 退出
python3 pi_detect.py --runtime onnx       # 强制 ONNX 后端
python3 pi_detect.py --runtime ncnn       # 强制 ncnn 后端
python3 pi_detect.py --model /path/to/best.onnx
python3 pi_detect.py --source test.jpg --save out.jpg   # 单张图自测：只推理，不连摄像头/串口
```

命令行参数只有四个：

| 参数 | 说明 |
|---|---|
| `--model` | 模型路径。**文件**当 onnx，**目录**（含 `model.ncnn.param`）当 ncnn；留空则自动查找 |
| `--runtime` | `auto`（默认，优先 ncnn）/ `onnx` / `ncnn` |
| `--source` | 单张图自测模式，不连摄像头也不连串口 |
| `--save` | 自测模式下把标注结果另存为图片 |

**换模型后务必先自测**：`--source` 或 `DISPLAY=1` 看一眼画的框和类别对不对。类别映射错了的后果是
"车去撞减益块"，那是给对方加分。

---

## 串口协议（与控制器端 `pi_comm.c` 严格一致）

12 字节定长，大端：

```
偏移  0     1      2    3     4     5     6     7     8     9     10    11
     0xAA | TYPE | ID | X_H | X_L | Y_H | Y_L | W_H | W_L | H_H | H_L | XOR
```

| 字段 | 含义 |
|---|---|
| `0xAA` | 帧头 |
| `TYPE` | `0x00` NONE（心跳）/ `0x01` BOMB（减益块）/ `0x04` BOOST（增益块） |
| `ID` | 固定 0（控制器当前不消费） |
| `X` / `Y` | 目标框中心相对画面中心的偏移，**int16 大端**，限幅 ±1000，单位 0.1% 画面（负=左/上） |
| `W` / `H` | 框宽/高占画面比例，限幅 0~1000 |
| `XOR` | `TYPE..H_L` 共 **10 个数据字节**的异或（不含 0xAA） |

约定：

- **发帧节奏 20 Hz（50 ms）**。控制器侧的帧超时是 500 ms，所以 50 ms 有 10 倍余量。
- **BOMB 帧坐标全 0**：控制器的躲避逻辑只认"类型的边沿"，不用坐标。
- **BOOST 帧带坐标**：控制器据此对准并直进撞击。
- **没有目标时发 NONE 心跳帧**，而不是停发 —— 让控制器在**当拍**就知道目标没了，
  不必等 500 ms 超时；同时保持链路新鲜（心跳帧也会刷新时间戳）。
- **采集/推理失效时真停发**（fail-safe）：摄像头掉线或推理抛异常 → 立即不再发帧 →
  控制器 500 ms 后判离线 → 回到纯传感器模式继续跑。**宁可判离线，也不发假数据。**

---

## 参数（全部通过环境变量，不改代码）

| 变量 | 默认 | 说明 |
|---|---|---|
| `SERIAL_PORT` | `/dev/ttyACM0` | 串口设备（接控制器的 USB-TTL） |
| `SERIAL_BAUD` | `115200` | 波特率，必须与控制器 UART5 一致 |
| `DEBUFF_CONF` | `0.35` | 置信度阈值 |
| `DEBUFF_MIN_W` | `0.12` | 目标框最小**相对宽度**，用作距离门控（过滤远处小框） |
| `BUFF_MIN_W` | 同 `DEBUFF_MIN_W` | 增益块单独的距离门控 |
| `DEBUFF_STOP_DELAY` | `0.5` | 减益块消失后继续发 BOMB 的延迟（秒） |
| `DEBUFF_BUFF_STOP_DELAY` | `0.0` | 增益块消失后继续发 BOOST 的延迟（秒） |
| `DEBUFF_CLS` / `BUFF_CLS` | `0` / `1` | 类别编号（元数据缺失时才用） |
| `DEBUFF_FRAME_TYPE` | `0x01` | BOMB 帧类型字节 |
| `BOOST_FRAME_TYPE` | `0x04` | BOOST 帧类型字节 |
| `DEBUFF_MODEL` | 空 | 默认模型路径（等价 `--model`） |
| `DEBUFF_NCNN_DIR` | 空 | 指定 ncnn 模型目录 |
| `DEBUFF_MODELS` | 空 | 指定多个候选模型路径（按优先级） |
| `DEBUFF_IMG` | 空 | 自测用默认图片路径 |
| `DEBUFF_EXPECT_NC` | 空 | 强制要求模型 `nc` 等于该值，不符则退出（防换错模型） |
| `DEBUFF_ALLOW_NO_SERIAL` | 空 | 置 1 时串口打不开降级为"只检测不发送"（默认直接退出） |
| `ORT_THREADS` | `0` | ONNX Runtime 线程数（0 = 库自己决定） |
| `NCNN_THREADS` | `4` | ncnn 线程数 |
| `CAMERA_NODES` | `11,21,31,41,0,1,2` | 摄像头设备节点尝试顺序 |
| `DISPLAY` | 未设置 | 设置则开画面窗口 |

两个停发延迟**故意分开**：减益块是危险，晚一拍改发 NONE 只是让车多躲半秒（安全方向）；
增益块在**自己正前方**、车正全速冲过去，多保持 0.5 秒就是朝一个已经消失的块继续冲 ——
撞空、冲出台边都可能。想回到"两类共用 0.5 秒"的老行为：`DEBUFF_BUFF_STOP_DELAY=0.5`。

---

## 设计要点

**1. 三线程：采集 / 检测 / 发送，各自独立**

- 采集线程（`_capture_run`）只管取帧，写进一个受锁保护的"最新帧"槽；
- 检测线程（`_detect_run`）取最新帧推理，把"该发什么"通过 `set_frame()` 写进发送器的帧槽；
- 发送线程（`_send_loop`）每 50 ms 把当前帧写一次串口。

`set_frame()` 只是替换一个 Python 引用（CPython 下引用赋值原子），所以换帧**即时生效**，
不需要重启线程。发送线程持有自己的锁，避免与 `serial.close()` 撞车（`stop_and_join()` 先停再关）。

**2. 健康门控：只有"采集成功 + 帧新鲜 + 检测成功"才允许发帧**

任一环节失效 → 立即停发。这是 fail-safe 的方向选择：**视觉宁可让控制器判离线，也不发假数据**。

**3. NONE 心跳帧，而不是"停发"**

停发要靠控制器 500 ms 超时才生效，叠上停发延迟会让车朝已消失的目标多冲一段。
发 NONE 帧则让控制器的躲避/追踪条件**当拍**就不成立。

**4. 类别映射优先从模型元数据自动识别**

`resolve_class_ids()` 会读 ONNX/ncnn 导出时写进模型的类别名（`debuff` / `buff`），
识别不到才退回 `DEBUFF_CLS`/`BUFF_CLS` 环境变量。而"这两个值是默认猜的"还是"人显式指定的"
用 `CLS_FROM_ENV` 区分 —— **默认值不被当成授权**，因为猜错的后果是去撞减益块。
多类模型读不到类别名时**报错退出**，而不是继续猜。

**5. 决策优先级：减益块 > 增益块**

`decide_frame()` 是唯一一份"检测结果 → 该发什么帧"的实现，规则顺序即优先级：
有够大的减益框 → BOMB（坐标全 0）；否则有够大的增益框 → BOOST（取面积最大的那个带坐标）；
都没有 → None。躲开减益块永远比得分优先。

**6. 距离门控（`MIN_W`）而不是"只要有框就报"**

用框的**相对宽度**判断远近，过滤掉远处的小框，避免为了地平线上的一个小点去冲。

**7. 自己实现 letterbox + YOLO11 解码 + NMS**

`letterbox()` / `decode_yolo11()` 是手写的，不依赖 ultralytics 在设备上运行 ——
树莓派上只需要 `onnxruntime` 或 `ncnn`，省掉一大堆依赖和启动时间。

**8. 单张图自测模式**

`--source` 走一条完全独立的路径：只读图、只推理、打印/保存结果，不碰摄像头和串口。
这是"验证模型和类别映射对不对"的标准入口，也是导出一致性核对的入口。

---

## 数据采集

`pi_capture.py` 用于采训练数据：定时抓图存盘（默认 **500 张、每 1 秒一张**），
摄像头节点顺序与 `pi_detect.py` 一致（默认 `11,21,31,41,0,1,2`）。
产出的图片按 `.gitignore` 规则不入库。

---

## 已知限制

1. **权重未入库**（见「模型文件」），克隆后必须自行放置 one 份权重；只有 `.param` 时 ncnn 后端起不来，
   请改用 `--runtime onnx`。
2. **无跨帧跟踪**：每帧独立判断，没有目标 ID 和轨迹，所以 BOOST 帧是"当拍快照"，不带任何记忆。
   控制器的增益块追踪目前也因此被注释掉（开环 + 与攻敌抢触发，效果不好）。
3. **`MIN_W` 距离门控偏保守**：太远的块不会被上报，需要按场地实测调。
4. **类别映射强依赖模型元数据**：自制/二次导出的模型若丢了类别名，必须显式给 `DEBUFF_CLS`/`BUFF_CLS`。
5. **帧率受 CPU 推理限制**：树莓派 5 上建议走 ncnn + `NCNN_THREADS=4`；onnx 后端更慢。
6. **坐标分辨率 0.1%**：`W/H` 上限 1000，框的精细尺寸信息被量化，不适合做精密测距。

---

## 许可与致谢

- 模型基于 Ultralytics YOLO11n 训练/导出，遵循 **AGPL-3.0**（见 `best_ncnn_model/metadata.yaml`）。
  如需商用或闭源分发，请自行确认许可要求。
- ncnn 部分由 pnnx 导出，遵循 ncnn 的 BSD-3-Clause。
- `pi_detect.py` / `pi_capture.py` 的业务逻辑为本项目原创。
