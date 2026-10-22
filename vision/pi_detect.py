#!/usr/bin/env python3
"""树莓派端 YOLO11n 能量块检测 → 串口发帧 → 小车躲避减益块/追踪增益块。

视觉方案: 用 ONNX Runtime 或 NCNN 跑 YOLO11n, 替代旧的 AprilTag/模板匹配方案。
支持双类模型(buff+debuff), 按检测结果分流:
    * 检测到 debuff(减益块) → 持续发 BOMB 帧 → 小车紧急躲避(撞减益块给对方加分)
    * 仅检测到 buff(增益块) → 发 BOOST 帧(带目标坐标) → 小车空闲时追踪并撞下擂台得分
    * 两者同时可见 → 优先发 BOMB 帧(躲避永远比得分优先)
    * 单类模型(nc=1)完全兼容旧行为: 所有目标都按 debuff 处理
目标消失超过停发延迟 → 改发 NONE(0x00) 心跳帧, 显式清掉 STM32 上的标签;
这个延迟**按上一次发的帧类型分开**: debuff 用 STOP_DELAY_SEC(默认 0.5s, 多躲一会儿
是安全方向), buff 用 BUFF_STOP_DELAY_SEC(默认 0, 车正朝块冲过去, 多保持只会撞空);
采集/检测失效(摄像头掉线、推理抛异常) → 立即停发(fail-safe)。

安全语义(重要):
    * fail-safe 方向 = 停发。视觉采集/检测一旦失效, 立即停发 → STM32 帧超时(500ms)
      清除标签 → 小车回到"纯传感器模式继续跑"。绝不会因为视觉卡死在旧帧上而
      无限发帧。
    * 类别映射优先从模型元数据(onnx/ncnn 导出时写入的类别名)自动识别,
      识别不到用 DEBUFF_CLS/BUFF_CLS 环境变量(默认 0/1)。
      ⚠ 若把减益块错标成增益类, 小车会去撞减益块(给对方加分)! 换模型后
        务必用 DISPLAY=1 或 --source 先确认类别映射正确。

串口协议(与 car9.22 的 pi_comm.h 权威一致, 12 字节定长大端):
    0xAA | TYPE | ID | X_H X_L | Y_H Y_L | W_H W_L | H_H H_L | CHECKSUM
    CHECKSUM = TYPE..H_L 共 10 个数据字节的 XOR(不含 0xAA)
    X: 目标框中心水平偏移 -1000~+1000, 单位 0.1% 画面宽(负=左 正=右)
    Y: 垂直偏移同单位(负=上 正=下); W/H: 框宽/高占画面比例 0~1000
    BOMB 帧坐标全 0(STM32 躲避逻辑只看类型边沿, 不用坐标)。

用法(在树莓派上):
    python3 pi_detect.py                            # 正常跑, 无画面 (自动选 ncnn→onnx)
    DISPLAY=1 python3 pi_detect.py                  # 带画面调试(需显示器), ESC 退出
    python3 pi_detect.py --source x.jpg             # 单张图自测(不连摄像头/串口)
    python3 pi_detect.py --runtime ncnn              # 强制 ncnn 后端(需已装 ncnn + 模型)
    python3 pi_detect.py --runtime onnx              # 强制 onnx 后端
    python3 pi_detect.py --model /path/to/model      # 指定模型: 文件=onnx, 目录=ncnn
    SERIAL_PORT=/dev/ttyUSB0 python3 pi_detect.py   # 换串口
    DEBUFF_CLS=0 BUFF_CLS=1 python3 pi_detect.py    # 手动指定类别编号(元数据缺失时)
    CAMERA_NODES="0,1" python3 pi_detect.py         # 换摄像头节点
    DEBUFF_STOP_DELAY=1.0 python3 pi_detect.py      # 调 debuff 停发延迟(默认 0.5)
    DEBUFF_BUFF_STOP_DELAY=0.5 python3 pi_detect.py # 调 buff 停发延迟(默认 0, 回到旧行为)
    DEBUFF_EXPECT_NC=2 python3 pi_detect.py         # 强制要求双类模型(nc 或 buff 类对不上就退出)
    DEBUFF_ALLOW_NO_SERIAL=1 python3 pi_detect.py   # 串口打不开时降级成"只检测不发送"(默认直接退出)
"""
from __future__ import annotations

import argparse
import os
import re
import sys
import threading
import time
import traceback
from collections.abc import Mapping
from pathlib import Path

import cv2
import numpy as np

try:
    import onnxruntime as ort
except ImportError:
    ort = None

try:
    import serial
except ImportError:
    serial = None

try:
    import ncnn
except ImportError:
    ncnn = None

# ==================== 配置 ====================
# Windows 控制台默认 GBK, 一旦 print 里出现 GBK 编不出的字符(如 ⚠/✓)就会抛
# UnicodeEncodeError —— 而且可能正好发生在 __init__ 里, 直接把程序打挂。
# 树莓派是 UTF-8 碰不到, 但自测/调试经常在 Windows 上跑, 这里统一兜底成"打不出来就替换"。
# 注意: 打印内容本身也尽量用 GBK 能编的字符(源码里的 ⚠ 写在注释里没问题, 别写进 print)。
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(errors="replace")
    except (AttributeError, ValueError):
        pass

SERIAL_PORT = os.environ.get("SERIAL_PORT", "/dev/ttyACM0")
SERIAL_BAUD = int(os.environ.get("SERIAL_BAUD", "115200"))
CAMERA_NODES = [int(x) for x in os.environ.get("CAMERA_NODES", "11,21,31,41,0,1,2")
                .split(",") if x.strip()]
FRAME_WIDTH, FRAME_HEIGHT = 640, 480

IMGSZ = 640                                      # 模型输入尺寸(训练/导出一致)
CONF_THRES = float(os.environ.get("DEBUFF_CONF", "0.35"))  # 置信度阈值
IOU_THRES = 0.45                                 # NMS 阈值
MIN_W = float(os.environ.get("DEBUFF_MIN_W", "0.12"))      # 框最小相对宽度 触发距离), 过滤远处小框
# 增益块单独一个触发距离: 追上去撞需要贴近(可以更大), 躲减益块要更早发现(通常更小)。
# 默认 == MIN_W, 不设就是原来的行为。
BUFF_MIN_W = float(os.environ.get("BUFF_MIN_W", str(MIN_W)))
MAX_DET = 300                                    # NMS 最多保留框数(与 ultralytics 一致)

SEND_INTERVAL = 0.05                             # 50ms 一帧(与 run_new.py 一致)
STOP_DELAY_SEC = float(os.environ.get("DEBUFF_STOP_DELAY", "0.5"))  # debuff 消失后停发延迟(秒)
# 增益块(buff)消失后**单独**的停发延迟, 默认 0(块一消失当拍就停追)。
# 为什么不跟 debuff 共用一个值: 两个方向完全相反 ——
#   * 减益块是**危险**, 车在躲它。晚一拍改发 NONE 只会让车多躲 0.5s, 是安全方向。
#   * 增益块是**奖励, 而且在自己前方**, 车正全速朝它冲过去(BOOST_CHASE_SPD=560)。
#     沿用 0.5s 就是让车朝一个已经消失的块继续冲满 0.5s —— 撞空、冲出台边都可能。
#     何况 BOOST 帧还带坐标, STM32 的 P3.5 会一直按最后那个坐标追。
# 想回到"两类共用一个延迟"的老行为: DEBUFF_BUFF_STOP_DELAY=0.5
BUFF_STOP_DELAY_SEC = float(os.environ.get("DEBUFF_BUFF_STOP_DELAY", "0.0"))
CAPTURE_TIMEOUT = 2.0                            # 采集心跳超时(秒), 超时判定摄像头失效

# 类别编号: debuff=减益块(发 BOMB 帧躲避), buff=增益块(发 BOOST 帧追踪)。
# 优先用模型元数据里的类别名自动识别(resolve_class_ids), 识别不到才用这里的值。
try:
    DEBUFF_CLS = int(os.environ.get("DEBUFF_CLS", "0"))
    BUFF_CLS = int(os.environ.get("BUFF_CLS", "1"))
except (ValueError, TypeError):
    print("警告: DEBUFF_CLS/BUFF_CLS 无法解析, 用默认 0/1")
    DEBUFF_CLS, BUFF_CLS = 0, 1

# 这两个值是"人显式指定"还是"默认猜的"? resolve_class_ids 靠它决定
# "读不到类别名时能不能猜" —— 猜错了小车会去撞减益块, 所以默认值不该被当成授权。
CLS_FROM_ENV = ("DEBUFF_CLS" in os.environ) or ("BUFF_CLS" in os.environ)

# 帧类型(与 pi_comm.h 的 PI_TAG_* 对应): 0x00=NONE 0x01=BOMB 0x04=BOOST
def _env_type(name: str, default: int) -> int:
    try:
        v = int(os.environ.get(name, hex(default)), 0)
        return v if 0 <= v <= 255 else default
    except (ValueError, TypeError):
        print(f"警告: {name} 无法解析, 已用默认 {hex(default)}")
        return default

FRAME_TYPE_NONE = 0x00                           # PI_TAG_NONE: 无标签(心跳帧)
FRAME_TYPE_BOMB = _env_type("DEBUFF_FRAME_TYPE", 0x01)
FRAME_TYPE_BOOST = _env_type("BOOST_FRAME_TYPE", 0x04)
FRAME_ID = 0


def build_frame(tag_type: int, x: int = 0, y: int = 0, w: int = 0, h: int = 0) -> bytes:
    """构建 12 字节帧: 0xAA|TYPE|ID|X_H X_L|Y_H Y_L|W_H W_L|H_H H_L|CK。

    X/Y 限幅 -1000~+1000(int16 大端), W/H 限幅 0~1000;
    CHECKSUM = TYPE..H_L 共 10 个数据字节的 XOR(与 STM32 pi_comm.c 一致)。
    """
    def s16(v: int) -> int:
        return max(-1000, min(1000, int(v))) & 0xFFFF
    def u10(v: int) -> int:
        return max(0, min(1000, int(v))) & 0xFFFF
    payload = bytes([tag_type & 0xFF, FRAME_ID,
                     (s16(x) >> 8) & 0xFF, s16(x) & 0xFF,
                     (s16(y) >> 8) & 0xFF, s16(y) & 0xFF,
                     (u10(w) >> 8) & 0xFF, u10(w) & 0xFF,
                     (u10(h) >> 8) & 0xFF, u10(h) & 0xFF])
    ck = 0
    for b in payload:
        ck ^= b
    return bytes([0xAA]) + payload + bytes([ck])

def decide_frame(dets: list, debuff_cls, buff_cls, img_w: float, img_h: float,
                 min_w: float = None, buff_min_w: float = None):
    """把一帧的检测结果归类成"该给 STM32 发什么" —— 返回 (active, payload)。

    active ∈ {"BOMB", "BOOST", None}; payload = (type, x, y, w, h) 或 None。
    规则(顺序即优先级):
      1. 有"够大"的 debuff 框 → BOMB(坐标全 0; STM32 只看类型边沿)
      2. 否则有"够大"的 buff 框 → BOOST(取面积最大的那个, 带相对坐标/尺寸)
      3. 都没有 → None(由调用方决定发 NONE 心跳还是停发)

    ⚠ 这是**唯一**一份"检测结果 → 发什么帧"的实现, 必须被两处共用:
        * pi_detect.DebuffDetector._detect_run —— 小车上真正跑的路径
        * scripts/export_ncnn.py 的 ncnn/onnx 一致性校验 —— 判断导出有没有跑偏
      以前校验脚本里手抄了一份同样的逻辑, 已经和真身漂移了两处(img_w 取法、
      buff_cls==debuff_cls 的兜底), 于是"校验通过"并不真的等于"小车行为一致"。
      改这里就等于同时改了两边。

    img_w/img_h 由调用方给, 两边都要传**实际帧尺寸**(image.shape[1]/[0])。
    别再传摄像头 cap.get(CAP_PROP_FRAME_WIDTH) 的上报值: 驱动谎报时(实测有报 640
    实际出 1280 的), 门控 MIN_W 的分母凭空翻倍, "多远算够大"会静默变掉 ——
    _detect_run 现在只把上报值留着做不一致告警, 不再参与计算。
    """
    min_w = MIN_W if min_w is None else min_w
    buff_min_w = BUFF_MIN_W if buff_min_w is None else buff_min_w
    if not dets:
        return None, None

    def wide(d) -> float:
        return (d["xyxy"][2] - d["xyxy"][0]) / img_w

    debuffs = [d for d in dets if debuff_cls is not None
               and d["cls"] == debuff_cls and wide(d) >= min_w]
    buffs = [d for d in dets if buff_cls is not None and buff_cls != debuff_cls
             and d["cls"] == buff_cls and wide(d) >= buff_min_w]

    if debuffs:
        # 减益块优先: 躲避永远比得分重要
        return "BOMB", (FRAME_TYPE_BOMB, 0, 0, 0, 0)
    if buffs:
        d = max(buffs, key=lambda d: (d["xyxy"][2] - d["xyxy"][0])
                * (d["xyxy"][3] - d["xyxy"][1]))
        x1, y1, x2, y2 = d["xyxy"]
        cx = (x1 + x2) / 2 / img_w
        cy = (y1 + y2) / 2 / img_h
        return "BOOST", (FRAME_TYPE_BOOST,
                         int(round((cx - 0.5) * 2000)),   # -1000(最左)~+1000(最右)
                         int(round((cy - 0.5) * 2000)),   # -1000(最上)~+1000(最下)
                         int(round((x2 - x1) / img_w * 1000)),
                         int(round((y2 - y1) / img_h * 1000)))
    return None, None


# 发送间隔必须明显小于 STM32 端 500ms 帧超时, 否则 bomb_warn 会闪烁、小车反复进退躲避
if SEND_INTERVAL >= 0.45:
    print(f"警告: SEND_INTERVAL={SEND_INTERVAL}s 接近/超过 STM32 500ms 帧超时, "
          f"会让 bomb_warn 闪烁! 应远小于 0.5s")

DEBUG_DISPLAY = "DISPLAY" in os.environ          # 与 car7.15 vision.py 一致


_ONNX_FILES = ("best.onnx", "best_384.onnx", "best_512.onnx", "best_320.onnx")
# 640 原尺寸的正式导出名。同内容档位里要**优先**选它: 384/512/320 是
# scripts/export_variants.py 产出的降分辨率变体(mAP50 低 0.4~2.2%), 只是"够用"的备份。
# 只有它们和 best.onnx 在同一目录时, 排在后面才不会让 ONNX 路径静默换成小尺寸模型。
_ONNX_MAIN = "best.onnx"
_MODEL_FILES = _ONNX_FILES + ("model.ncnn.param",)
# ncnn 目录的候选名。ultralytics 8.4 导出的是 best_ncnn_model; best_ncnn / best_ncnn_384
# 是旧版本/旧部署留下的名字(往往是**单类旧模型**)。名单只在这里写一次, 别处一律引用它 ——
# _peek_class_names 曾手抄过一份**少了三个 onnx 变体**的名单, 于是"目录里只放
# best_384.onnx"这种 README 推荐的变体目录读不出类别名、被降档, 而真正的病根
# 就是这份重复名单(两套规则打架)。
_NCNN_DIR_NAMES = ("best_ncnn_model", "best_ncnn", "best_ncnn_384")


def _norm_name(s) -> str:
    """类别名规范化: 转小写 + 去掉分隔符, 用来做包含匹配。

    为什么需要: 'de_buff' / 'de-buff' / 'De Buff' 这类写法**含 buff 但不含 'debuff'**,
      按原样匹配会把减益类判成增益类(_class_roles 里落到 elif 分支) → 一个真双类模型
      被算成"确定没有增益类", 排到最低档, 静默输给过期模型。
      去掉 _ - 空格 之后再匹配, 'de_buff' 就是 'debuff' 了。
    ⚠ 判定顺序(C 处)依然必须是"先 debuff 再 buff": 规范化不改变 'debuff' 含 'buff' 这个事实。
    """
    return re.sub(r"[\s_\-]+", "", str(s).lower())


def _peek_class_names(p: Path):
    """**便宜地**读出某个候选模型的类别名集合(小写), 读不到返回 None。

    ⚠ 这个结果**只用来给候选排序**(判断哪个是"双类的新模型"), 不参与真正的类别映射 ——
      真正的映射始终由 resolve_class_ids 从**已加载的模型对象**上读。所以这里允许用
      "直接在文件字节里找类别名"这类近似手段, 但**找法必须躲开 'debuff' 含 'buff'
      子串**这个坑(见下面读字节那段), 否则排序会反过来把旧单类模型顶到第一。

    顺序: 目录的 metadata.yaml → 目录里的 best*.onnx / best_ncnn* → onnx 的
    metadata_props(要 onnx 包) → onnx 文件明文搜字节(ultralytics 把 names 明文写在
    metadata_props 里)。
    """
    try:
        if p.is_dir():
            meta = p / "metadata.yaml"
            if meta.is_file():
                import yaml
                # 必须显式 utf-8: ultralytics 写的是 utf-8, 而 Windows(cp936)/LANG=C 下
                # read_text() 用 locale 默认编码 —— 中文类别名会变乱码(判不出增/减),
                # 或直接抛 UnicodeDecodeError 被外层吞掉 → names=None → 目录被降档。
                names = yaml.safe_load(meta.read_text(encoding="utf-8")).get("names")
                if isinstance(names, dict):
                    return {str(v).lower() for v in names.values()}
                if isinstance(names, list):
                    return {str(v).lower() for v in names}
            # weights/ 这种目录: 名字在里面的模型文件里。**逐个看、取最好的那个**,
            # 不能固定顺序取第一个命中的 —— 同一个目录里既有旧单类 best.onnx 又有新双类
            # best_384.onnx 时, 先命中旧 best.onnx 会把整个目录判成"确定没有增益类"
            # (tier=0, 比"读不出名字"的 tier=1 还低), 这些变体目录就永远选不上了。
            found = []
            for inner in _MODEL_FILES + _NCNN_DIR_NAMES:
                sub = p / inner
                try:
                    if not sub.exists():
                        continue
                except OSError:
                    continue
                n = _peek_class_names(sub)
                if n:
                    found.append(n)
            for n in found:
                if _has_both_classes(n):
                    return n          # 有双类的就算数, 不管它是第几个文件
            return found[0] if found else None
        if p.suffix.lower() != ".onnx" or not p.is_file():
            return None
        try:
            import ast
            import onnx
            m = onnx.load(str(p), load_external_data=False)
            raw = {kv.key: kv.value for kv in m.metadata_props}.get("names")
            if raw:
                return {str(v).lower() for v in ast.literal_eval(raw).values()}
        except Exception:
            pass
        data = p.read_bytes()     # 退路: names 以明文存在 onnx metadata_props 里
        # ⚠ 这里**不能**直接搜 b"buff": 'debuff' 里就含 'buff' 这个子串, 单类的旧减益
        #    模型会被误判成"双类", 排序时反而压过真正的新双类模型 —— 增益块照样静默失效。
        #    只认**带引号**的 token: "'buff'" 不是 "'debuff'" 的子串, 干净可辨;
        #    中文名(减益/增益)没有这个包含关系, 直接搜字即可。
        def tok(s: str) -> bool:
            t = s.encode()        # 必须两边都是 bytes: str in bytes 会抛 TypeError
            return any(q + t + q in data for q in (b"'", b'"'))
        found = set()
        if tok("debuff") or "减".encode() in data:
            found.add("debuff")
        if tok("buff") or "增".encode() in data:
            found.add("buff")
        return found or None
    except Exception:
        return None


def _class_roles(names) -> tuple:
    """从类别名集合里分辨 (有没有 debuff, 有没有 buff)。

    ⚠ 必须先判 'debuff' 再看 'buff': **'debuff' 里就含 'buff' 这个子串**, 直接判断会把
      减益类同时算成增益类(本文件别处也反复强调这个坑; 校验脚本就栽在它上面过)。
      写成 if/elif: 一个名字同时含两者时按 debuff 处理 —— 那是"不撞减益块"的安全方向。
    ⚠ names 还允许是 **dict** —— 模型 metadata.yaml 里 names 的**原生形态**就是
      {0: 'debuff', 1: 'buff'}。直接 `for n in names` 迭代出来的是**键**(整数 0/1),
      _norm_name 之后变成 '0'/'1', 于是一个**真双类模型会被判成"两个类都没有"**,
      而且不报错、不抛异常。本文件内部调用点都先过 _peek_class_names(统一成名字集合),
      所以踩不到; scripts/export_ncnn.py 直接把 read_meta 的原始 dict 传进来就踩了 ——
      它把 v3 打成"单类旧模型, 排序里必然输给本次导出的", 而 v3 是双类、修好之前
      mtime 还更新: 那句提示正好把"部署时可能静默跑旧模型"的警告**反着说**了。
      所以这里统一取 values()。
    """
    if isinstance(names, Mapping):
        names = names.values()
    has_d = has_b = False
    for n in names or ():
        n = _norm_name(n)          # 'de_buff'/'de-buff' 也要能认出减益(见 _norm_name)
        if "debuff" in n or "减" in n:
            has_d = True
        elif "buff" in n or "增" in n:
            has_b = True
    return has_d, has_b


def _has_both_classes(names) -> bool:
    """类别名里是否**同时**有 debuff 和 buff —— 区分"双类新模型/单类旧模型"的内容特征。"""
    has_d, has_b = _class_roles(names)
    return has_d and has_b


def _is_main_artifact(p: Path) -> int:
    """是否"正式名"(best.onnx / best_ncnn_model 目录) —— 同内容档内的排序用。

    384/512/320 是 scripts/export_variants.py 产出的**降分辨率变体**(mAP50 低 0.4~2.2%),
    best_ncnn / best_ncnn_384 是旧命名。它们和正式名是同一份权重导出的, 内容档位相同,
    于是只能靠 mtime 决胜 —— 跑一次 export_variants.py 就把这些文件的 mtime 刷成"现在",
    ONNX 路径会**静默**从 best.onnx 变成 best_320.onnx。正式名排在前面就断了这条路。
    (变体仍然会被选中: 桌面上只拷了 best_384.onnx 时它是唯一候选, 见 find_weights_dir。)
    """
    return 1 if p.name in (_ONNX_MAIN, "best_ncnn_model") else 0


def _rank_by_content(paths: list, verbose: bool = False) -> list:
    """把候选模型按**内容**排序(不是按 mtime), 返回 [(path, names), ...] 从优到劣。

    为什么不能拿 mtime 当主排序键: 拷贝(`scp -r`/`cp -r`)、解压、以及往目录里新写一个
    文件(比如导出时产生的 __pycache__)都会把 mtime 重置成"现在", 于是"哪个模型更新"
    变成了"哪个文件最后被碰过" —— 旧模型被拷一次就能盖过新模型, 而且完全静默。

    排序键(从重要到次要):
      1. 类别名同时含 debuff+buff 的排最前 —— 这是"能追增益块"的必要条件
      2. 读不出类别名的排中间: 可能是好模型只是元数据被裁过, 比"确定没有增益类"强
      3. 确定是单类(没有增益类)的排最后
      4. 同档内正式名(best.onnx / best_ncnn_model)优先于降分辨率变体/旧命名(见 _is_main_artifact)
      5. 同档内 mtime 新的优先
      6. 再平手按路径名字符串倒序, 结果确定(且 '..._v2' 会排在 '...-2' 前面)

    ⚠ 第 4/5 条只在**同内容档**内起作用。两个都是双类、且都叫正式名时(比如先后两次
      重训的 runs/buff_debuff_v2 与 runs/buff_debuff_v3), 唯一判据就只剩 mtime 了 ——
      这一档是**没有内容依据可分辨**的, 所以这里会额外打印一条醒目告警(见下面 [选模型]),
      而不是假装自己选对了。
    """
    def mtime(p: Path) -> float:
        try:
            return p.stat().st_mtime
        except OSError:
            return 0.0

    ranked = []
    for p in paths:
        names = _peek_class_names(p)
        tier = 2 if _has_both_classes(names) else (1 if names is None else 0)
        ranked.append((tier, _is_main_artifact(p), mtime(p), str(p), names, p))
    ranked.sort(key=lambda t: (t[0], t[1], t[2], t[3]), reverse=True)

    if verbose and ranked:
        best = ranked[0][5]
        for _tier, _m, mt, s, names, p in ranked:
            mark = "->" if p is best else "  "
            print(f"  [选模型] {mark} {s}  类别名={names or '读不到'}  "
                  f"mtime={time.strftime('%Y-%m-%d %H:%M', time.localtime(mt))}")
        # 同为双类的候选超过一个 → 说明"谁是更新"只能靠 mtime 猜, 而 mtime 会被
        # cp -r / scp / 解压刷新。这种情况必须让人看见, 不能静默挑一个 (建议删掉旧的,
        # 或用 --model / DEBUFF_MODEL 钉死)。
        duals = [p for t, _m, _mt, _s, _n, p in ranked if t == 2]
        if len(duals) > 1:
            print(f"  [选模型] ⚠ 有 {len(duals)} 个候选**都是双类模型**, 无法按内容分辨, "
                  f"只能按 mtime 挑(拷贝/解压会刷新 mtime!) —— 已选中最上面那个:")
            for p in duals:
                print(f"  [选模型]     {p}")
            print(f"  [选模型] ⚠ 如果不是你要的那个, 删掉多余的, 或用 --model/DEBUFF_MODEL 指定")
    return [(p, names) for _t, _m, _mt, _s, names, p in ranked]


def _all_weights_dirs() -> list:
    """所有候选根下的 `runs/*/weights/` 目录(仅枚举, 不在这里决定用哪个)。

    为什么不写死 `runs/buff_debuff-2`: 每重训一次目录名就换(buff_debuff-2 → buff_debuff_v2),
    写死会让树莓派一直默默加载**旧模型** —— 换了模型却没有任何报错, 是最难查的一类问题。
    """
    out, seen = [], set()
    for root in _candidate_roots():
        runs = root / "runs"
        try:
            if not runs.is_dir():
                continue
            entries = sorted(runs.iterdir())
        except OSError:
            continue
        for run in entries:
            w = run / "weights"
            try:
                if w.is_dir() and w not in seen:
                    seen.add(w)
                    out.append(w)
            except OSError:
                continue
    return out


def _find_in_weights(name_ok) -> list:
    """`runs/*/weights/` 下文件名满足 name_ok 的条目, 按内容优劣排序(见 _rank_by_content)。"""
    found = []
    for w in _all_weights_dirs():
        try:
            entries = sorted(w.iterdir())
        except OSError:
            continue
        for p in entries:
            try:
                if name_ok(p.name):
                    found.append(p)
            except OSError:
                continue
    return [p for p, _n in _rank_by_content(found)]


def _warn_if_foreign(p: Path, here: Path) -> None:
    """选中的模型不在"脚本所在目录 / 它的上一级"这棵树里时告警。

    搜索根含 cwd / 家目录 / 桌面 —— 那是为了"文件随便放都能跑", 但它们会把**无关工程**
    的 runs/*/weights 也扫进来。别的 YOLO 工程只要类别名恰好也叫 debuff/buff(或读不出
    名字), 就可能靠更新的 mtime 压过本项目的模型: 小车对着完全无关的目标追/躲, 不报错。

    正常的两种部署形态都不会触发本告警:
      * 整目录: <脚本目录>/runs/<run>/weights/...        → 在 here 之下
      * 平铺  : 模型和脚本同放 ~/Desktop (脚本在 ~/Desktop/buff_debuff_yolo/) → 在 here.parent 之下
    """
    try:
        rp = p.resolve()
        for base in (here, here.parent):
            try:
                rp.relative_to(base.resolve())
                return
            except ValueError:
                continue
    except (OSError, RuntimeError):
        return
    print(f"  [选模型] ⚠ 选中的模型在脚本目录之外: {p}")
    print(f"  [选模型]   搜索根含 cwd/桌面/家目录, 可能扫到了**无关工程**的 runs/*/weights。")
    print(f"  [选模型]   确认这是你要的模型, 否则用 --model / DEBUFF_MODEL 钉死。")


def default_model_path() -> str:
    """找 ONNX 模型: 所有候选(脚本同目录平铺 + runs/*/weights/)里**内容最优**的一个。

    与 find_weights_dir() 认同一套文件名(_ONNX_FILES), 不再出现"平铺 best_384.onnx
    find_weights_dir 看得见、default_model_path 却找不到"这种两套规则打架的情况。

    ⚠ 不能给"脚本同目录的 best.onnx"无条件优先: 按 README 的部署方式, 桌面那份可能是
      很久以前的**单类**模型(scp 过来的), 无条件优先会让新训好的双类模型永远加载不到
      → 增益块静默失效(小车不追块, 且没有任何报错)。
    """
    here = Path(__file__).resolve().parent
    # 平铺模型要在**所有**候选根里找, 不能只看脚本自己所在的那一层:
    # README 的部署方式是"pi_detect.py 和模型都拷到 ~/Desktop", 但脚本也可能被放在
    # ~/Desktop/buff_debuff_yolo/ 里(整个工程拷过去), 此时新模型平铺在 ~/Desktop
    # (= here.parent) —— 只看 here 会漏掉它, 于是候选里只剩 runs/*/weights 里的**旧模型**,
    # 静默加载旧单类模型、增益块整条失效。这里与 find_weights_dir 认同一套搜索根。
    flat = []
    for root in _candidate_roots():
        for f in _ONNX_FILES:
            p = root / f
            try:
                if p.is_file():
                    flat.append(p)
            except OSError:
                continue
    ranked = _rank_by_content(flat + _find_in_weights(lambda n: n in _ONNX_FILES),
                              verbose=True)
    if ranked:
        _warn_if_foreign(ranked[0][0], here)
        return str(ranked[0][0])
    print("找不到任何 ONNX 模型。已搜索这些根目录:")
    for r in _candidate_roots():
        print(f"  {r}")
    # 交给 YOLO11Onnx.__init__ 报出我们自己的友好错误(而不是裸 FileNotFoundError)
    return str(here / "best.onnx")


def _candidate_roots():
    """布局无关的搜索根: cwd / 脚本同目录 / 项目根 / 家目录 / 桌面(含中文"桌面")。

    树莓派上文件不管整目录拷到桌面还是平铺拷到桌面, 都能被找到。
    """
    here = Path(__file__).resolve().parent
    # here / here.parent 放最前: 同分时优先脚本自己所在的那棵树(平铺部署最常见的形态)
    roots = [here, here.parent]
    try:
        roots.append(Path.cwd())
    except OSError:          # cwd 被删掉时会抛
        pass
    try:
        home = Path.home()
        # ⚠ Path.home() 在 HOME 未设置且 uid 没有 passwd 项时(容器 / 某些 systemd
        #   场景)抛的是 RuntimeError, 不是 OSError —— 只 catch OSError 会让它冒到
        #   上层被报成"找不到模型", 极难查。
        roots += [home / "Desktop", home / "桌面", home]
    except (OSError, RuntimeError):
        pass
    out, seen = [], set()
    for r in roots:
        try:
            r = r.resolve()
        except (OSError, RuntimeError):
            continue
        if r not in seen:
            seen.add(r)
            out.append(r)
            out.append(r / "buff_debuff_yolo")   # 项目文件夹名, 兼容不同拷贝深度
    return out


def find_weights_dir():
    """找包含任意模型文件(_MODEL_FILES)的目录(布局无关), 找不到返回 None。

    优先级: 环境变量 DEBUFF_MODELS → runs/*/weights/ 和各搜索根平铺目录里**内容最优**的。
    只要目录里有任一加速变体就认, 这样桌面只拷 best_384.onnx 也能被找到。
    """
    env = os.environ.get("DEBUFF_MODELS")
    if env:
        p = Path(env).expanduser()
        if p.is_dir() and any((p / f).exists() for f in _MODEL_FILES):
            print(f"[选模型] DEBUFF_MODELS 指定: {p}")
            return p
        print(f"警告: DEBUFF_MODELS={env} 里没有任何模型文件, 忽略")

    cands = []
    for w in _all_weights_dirs():
        try:
            if any((w / f).exists() for f in _MODEL_FILES):
                cands.append(w)
        except OSError:
            continue
    for root in _candidate_roots():
        for sub in (root / "weights", root):
            try:
                if sub.is_dir() and any((sub / f).exists() for f in _MODEL_FILES):
                    if sub not in cands:
                        cands.append(sub)
            except OSError:
                continue
    ranked = _rank_by_content(cands, verbose=True)
    return ranked[0][0] if ranked else None


def find_ncnn_dir() -> str:
    """找 ncnn 模型目录(含 model.ncnn.param), 布局无关, 找不到返回 ''。

    候选 = 环境变量 DEBUFF_NCNN_DIR → runs/*/weights/best_ncnn* → 平铺在各搜索根下的
    best_ncnn / best_ncnn_model / best_ncnn_384 以及搜索根本身, 然后按**内容**排
    (见 _rank_by_content)。

    ⚠ 不能用固定顺序(`root/best_ncnn` 排在 `root/best_ncnn_model` 前面)第一个命中就返回:
      ultralytics 8.4 导出的目录名带 `_model` 后缀, 而旧部署留在同一个平铺目录里的
      `best_ncnn` 往往是**单类旧模型** —— 固定顺序会让旧模型永远赢, 增益块静默失效。
    """
    env = os.environ.get("DEBUFF_NCNN_DIR")
    if env:
        p = Path(env).expanduser()
        if p.is_dir() and (p / "model.ncnn.param").is_file():
            print(f"[选模型] DEBUFF_NCNN_DIR 指定: {p}")
            return str(p)
        print(f"警告: DEBUFF_NCNN_DIR={env} 不是有效的 ncnn 目录(缺 model.ncnn.param), 忽略")

    def ok_dir(d: Path) -> bool:
        try:
            return d.is_dir() and (d / "model.ncnn.param").is_file()
        except OSError:
            return False

    cands = [d for d in _find_in_weights(lambda n: n.startswith("best_ncnn"))
             if ok_dir(d)]
    for root in _candidate_roots():
        for sub in (root / "best_ncnn", root / "best_ncnn_model",
                    root / "best_ncnn_384", root):
            if ok_dir(sub) and sub not in cands:
                cands.append(sub)
    ranked = _rank_by_content(cands, verbose=True)
    if ranked:
        _warn_if_foreign(ranked[0][0], Path(__file__).resolve().parent)
        return str(ranked[0][0])
    return ""


def find_test_image(preferred: str = "") -> "Path | None":
    """布局无关地找一张可推理的图片(自测/测速用), 找不到返回 None。

    查找顺序:
      1) 显式传入路径(--image/--source): 原样 / 展开 ~ / 相对当前目录 都试
      2) 环境变量 DEBUFF_IMG
      3) 所有候选根下的 data/images/val、data/images、根目录本身,
         取第一个 jpg/jpeg/png
    """
    def ok(p: Path) -> bool:
        try:
            # >10KB 过滤掉临时文件/图标之类的噪声, 避免建议一张没意义的图
            return p.is_file() and p.stat().st_size > 10240
        except OSError:
            return False

    if preferred:
        for p in (Path(preferred), Path(preferred).expanduser(),
                  Path.cwd() / preferred):
            if ok(p):
                return p.resolve()
    env = os.environ.get("DEBUFF_IMG")
    if env:
        p = Path(env).expanduser()
        if ok(p):
            return p.resolve()
    # 只在脚本同目录/cwd 里扫裸目录(避免扫到家目录/上级目录时误拾无关图片)
    bare_allowed = {Path(__file__).resolve().parent, Path.cwd().resolve()}
    for root in _candidate_roots():
        subs = ["data/images/val", "data/images", ""]
        if root not in bare_allowed:
            subs = ["data/images/val", "data/images"]   # 其他根只找 data 下的图
        for sub in subs:
            d = root / sub if sub else root
            if not d.is_dir():
                continue
            try:
                for f in sorted(d.iterdir()):
                    if f.suffix.lower() in (".jpg", ".jpeg", ".png") and ok(f):
                        return f.resolve()
            except OSError:
                continue
    return None


def letterbox(im, new_shape=(IMGSZ, IMGSZ), color=(114, 114, 114)):
    """缩放+居中填充到精确的 new_shape (与 ultralytics 训练 auto=False 一致)。

    注意: 静态输入 ONNX 要求输入精确等于 640x640, 所以这里不能用
    ultralytics 默认的 auto=True (按 stride 对齐, 640x480 会被判定为无需填充)。
    Returns:
        im:   填充后的图 (new_shape x new_shape)
        ratio: 缩放比例
        pad:   (dw, dh) 左/上填充像素数, 反变换用
    """
    shape = im.shape[:2]                                  # h, w
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))  # w, h
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw, dh = dw / 2, dh / 2
    if shape[::-1] != new_unpad:
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return im, r, (dw, dh)


class YOLO11Onnx:
    """YOLO11n 检测 ONNX 推理 + 后处理(解码 + NMS), 不依赖 ultralytics/torch。

    支持任意类别数(nc>=1): 输出布局 [1, 4+nc, N], 类别分数已在图内做 sigmoid
    (已用 onnx 官方跑同一模型逐框验证一致, 见 verify_onnx.py)。
    尝试从 onnx 元数据读类别名(ultralytics 导出自带), 供 debuff/buff 语义映射。
    """

    def __init__(self, model_path: str, imgsz: int = 0, conf: float = CONF_THRES,
                 iou: float = IOU_THRES, threads: int = 0):
        if ort is None:
            raise RuntimeError(
                "未安装 onnxruntime。\n"
                "树莓派 64 位 OS: pip3 install onnxruntime\n"
                "树莓派 32 位 OS: 最新 onnxruntime 已不发 armv7 wheel, 需装旧版, 例如 "
                "pip3 install onnxruntime==1.9.1 (或换 64 位系统)")
        p = Path(model_path)
        # is_file() 而不是 exists(): 传进来一个目录(比如 ncnn 模型目录)时 exists() 为真,
        # 错误会推迟到 onnxruntime 内部, 变成一个完全看不懂的报错。
        if not p.is_file():
            raise FileNotFoundError(
                f"找不到 ONNX 模型文件: {model_path}\n"
                f"请把 best.onnx 放到脚本同目录, 或用 --model 指定路径。")
        so = ort.SessionOptions()
        if threads and threads > 0:          # 显式线程数(默认用 ORT 自动)
            so.intra_op_num_threads = threads
            so.inter_op_num_threads = threads
        self.session = ort.InferenceSession(str(p), sess_options=so,
                                            providers=["CPUExecutionProvider"])
        self.input_name = self.session.get_inputs()[0].name
        inp = self.session.get_inputs()[0].shape
        if imgsz <= 0:                       # 自动读取模型输入尺寸 → 384/512 小模型即插即用
            h, w = inp[2], inp[3]
            imgsz = h if (isinstance(h, int) and isinstance(w, int) and h == w) else IMGSZ
        out = self.session.get_outputs()[0]
        # 输出 [1, 4+nc, N] 或 [1, N, 4+nc]; 通道维远小于 anchor 维
        shape = out.shape
        self.nc = (shape[1] - 4) if shape[1] < shape[2] else (shape[2] - 4)
        self.names = self._read_names()
        self.imgsz = imgsz
        self.conf = conf
        self.iou = iou
        print(f"模型: {p.name}  输入 {self.imgsz}x{self.imgsz}  nc={self.nc}  "
              f"线程 {'auto' if threads <= 0 else threads}")

    def _read_names(self):
        """读 ultralytics 导出时写进 onnx 元数据的类别名(形如 {0:'buff',1:'debuff'})。"""
        try:
            import ast
            raw = self.session.get_modelmeta().custom_metadata_map.get("names")
            if raw:
                d = ast.literal_eval(raw)
                return {int(k): str(v) for k, v in dict(d).items()}
        except Exception:
            pass
        return None

    def _preprocess(self, bgr: np.ndarray):
        img, ratio, pad = letterbox(bgr, self.imgsz)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)        # BGR → RGB
        img = img.astype(np.float32) / 255.0              # 归一化
        blob = np.transpose(img, (2, 0, 1))[None]          # HWC → BCHW, 加 batch
        return np.ascontiguousarray(blob, dtype=np.float32), ratio, pad

    @staticmethod
    def _nms(xyxy: np.ndarray, scores: np.ndarray, iou_thres: float,
             max_det: int = MAX_DET):
        """标准 NMS: 按分数降序(稳定排序), 抑制 IoU > 阈值 的低分框, 最多 max_det 个。"""
        order = np.argsort(-scores, kind="stable")
        keep = []
        while order.size > 0 and len(keep) < max_det:
            i = order[0]
            keep.append(i)
            if order.size == 1:
                break
            rest = order[1:]
            xx1 = np.maximum(xyxy[i, 0], xyxy[rest, 0])
            yy1 = np.maximum(xyxy[i, 1], xyxy[rest, 1])
            xx2 = np.minimum(xyxy[i, 2], xyxy[rest, 2])
            yy2 = np.minimum(xyxy[i, 3], xyxy[rest, 3])
            inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
            area_i = (xyxy[i, 2] - xyxy[i, 0]) * (xyxy[i, 3] - xyxy[i, 1])
            area_r = (xyxy[rest, 2] - xyxy[rest, 0]) * (xyxy[rest, 3] - xyxy[rest, 1])
            ious = inter / (area_i + area_r - inter + 1e-9)
            order = rest[ious <= iou_thres]
        return keep

    def _postprocess(self, output: np.ndarray, orig_shape, ratio, pad) -> list:
        """ONNX 原始输出 → 原图坐标框(复用模块级 decode_yolo11 同一份已验证数学)。"""
        return decode_yolo11(output[0], self.nc, orig_shape, ratio, pad,
                             self.conf, self.iou)

    def detect(self, bgr: np.ndarray) -> list:
        """检测一帧 BGR 图, 返回 [{xyxy:[x1,y1,x2,y2], conf}], 已过滤低分框。"""
        blob, ratio, pad = self._preprocess(bgr)
        out = self.session.run(None, {self.input_name: blob})[0]
        return self._postprocess(out, bgr.shape[:2], ratio, pad)


def decode_yolo11(pred: np.ndarray, nc: int, orig_shape, ratio, pad,
                  conf: float, iou: float, max_det: int = MAX_DET) -> list:
    """把推理原始输出(pred: (4+nc, N) 或 (N, 4+nc))解码成原图坐标框列表。

    与 YOLO11Onnx._postprocess 共用, 也可给 NCNN 等其他运行时复用(已验证同一份数学)。
    nc<=0 时按 pred 形状自动推断(4+nc 是较小的那一维)。返回项含 'cls' 类别编号;
    nc=1 时 cls 恒为 0, 行为与旧单类版完全一致。
    """
    if pred.ndim == 2 and nc <= 0:
        nc = min(pred.shape) - 4
    if pred.ndim == 2 and pred.shape[0] == 4 + nc:
        pred = pred.transpose(1, 0)                       # → (N, 4+nc)
    boxes = pred[:, :4]                                   # cx, cy, w, h (输入像素坐标)
    cls_scores = pred[:, 4:4 + nc]                        # (N, nc) 各类别分数
    # 防御: 个别 ultralytics 版本导出的类别分数未 sigmoid(可为 >1 的 logit)。
    # 仅当分数确定不是概率(出现 >1.0)才补 sigmoid; 本项目 best.onnx 已内置 sigmoid(已验证)。
    if cls_scores.max() > 1.0:
        cls_scores = 1.0 / (1.0 + np.exp(-cls_scores))
    cls_ids = cls_scores.argmax(axis=1)                   # 每框取分数最高的类别
    scores = cls_scores.max(axis=1)
    mask = scores > conf
    boxes, scores, cls_ids = boxes[mask], scores[mask], cls_ids[mask]
    if boxes.shape[0] == 0:
        return []
    x1 = boxes[:, 0] - boxes[:, 2] / 2
    y1 = boxes[:, 1] - boxes[:, 3] / 2
    x2 = boxes[:, 0] + boxes[:, 2] / 2
    y2 = boxes[:, 1] + boxes[:, 3] / 2
    xyxy = np.stack([x1, y1, x2, y2], axis=1)
    # 按类别分别 NMS(不同类别互不抑制)
    keep_all = []
    for c in np.unique(cls_ids):
        idx = np.where(cls_ids == c)[0]
        keep = YOLO11Onnx._nms(xyxy[idx], scores[idx], iou, max_det=max_det)
        keep_all.extend(idx[keep].tolist())
    keep_all.sort()
    xyxy, scores, cls_ids = xyxy[keep_all], scores[keep_all], cls_ids[keep_all]
    # 反变换回原图坐标
    dw, dh = pad
    xyxy[:, [0, 2]] = (xyxy[:, [0, 2]] - dw) / ratio
    xyxy[:, [1, 3]] = (xyxy[:, [1, 3]] - dh) / ratio
    oh, ow = orig_shape
    xyxy[:, [0, 2]] = np.clip(xyxy[:, [0, 2]], 0, ow)
    xyxy[:, [1, 3]] = np.clip(xyxy[:, [1, 3]], 0, oh)
    return [{"xyxy": r.tolist(), "conf": float(c), "cls": int(k)}
            for r, c, k in zip(xyxy, scores, cls_ids)]


class YOLO11Ncnn:
    """YOLO11n ncnn 推理(树莓派 ARM 上比 onnxruntime 快), 与 YOLO11Onnx 同接口。

    权重/激活 fp16(use_fp16_storage/packed/arithmetic), Cortex-A72 无 dotprod 也受益。
    输出直接喂 decode_yolo11(与 onnx 完全同一份已验证数学)。
    """

    def __init__(self, model_dir: str, conf: float = CONF_THRES,
                 iou: float = IOU_THRES, threads: int = 4):
        if ncnn is None:
            raise RuntimeError(
                "未安装 ncnn。树莓派上请: pip3 install ncnn\n"
                "或用 --runtime onnx 退回 onnxruntime。")
        d = Path(model_dir)
        param = d / "model.ncnn.param"
        bin_ = d / "model.ncnn.bin"
        if not param.exists() or not bin_.exists():
            raise FileNotFoundError(
                f"找不到 ncnn 模型: {d}\n"
                f"请把含 model.ncnn.param 和 model.ncnn.bin 的文件夹(如 best_ncnn)"
                f"放到脚本同目录/桌面。")
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False
        self.net.opt.num_threads = max(1, int(threads))
        self.net.opt.use_fp16_storage = True
        self.net.opt.use_fp16_packed = True
        self.net.opt.use_fp16_arithmetic = True
        self.net.load_param(str(param))
        self.net.load_model(str(bin_))
        self.imgsz = self._read_imgsz(d)
        self.names = self._read_names(d)
        # ⚠ nc 必须在这里算出来: resolve_class_ids() 靠它区分单类/双类。
        #   原来 __init__ 里没有 self.nc, getattr(model,'nc',1) 会拿到 1 →
        #   双类模型被当成单类 → 增益块**永远不会**被追踪(整条 buff 功能静默失效)。
        self.nc = self._infer_nc()
        if isinstance(self.names, dict) and len(self.names) != self.nc:
            # metadata.yaml 与模型输出对不上 → **不能再用这份类别名**。
            # 危险场景: 模型是旧的单类(nc=1), 但目录里的 metadata.yaml 是新的双类 ——
            # 只打一句警告的话, resolve_class_ids 会照 metadata 返回 (0,1), 于是
            # decide_frame 永远等一个模型根本输不出的 cls1(buff): 增益块整条功能静默失效,
            # 只剩一行谁也不会注意的警告。留空 names 让下游走"以模型为准"的安全分支
            # (nc=1 → 只有 debuff; nc>1 且无名字 → 拒绝猜, 除非人显式指定 DEBUFF_CLS)。
            print(f"警告: metadata.yaml 里写了 {len(self.names)} 个类别名, 但模型输出 nc={self.nc} —— "
                  f"两者对不上, **忽略 metadata 的类别名**(以模型输出为准)")
            print(f"      该目录里的 metadata.yaml 与模型文件可能来自不同的导出, 请重新导出。")
            self.names = None
        self.conf = conf
        self.iou = iou
        print(f"模型: {d.name}(ncnn)  输入 {self.imgsz}x{self.imgsz}  nc={self.nc}  "
              f"线程 {threads}")

    @staticmethod
    def _squeeze_out(out_mat) -> np.ndarray:
        """ncnn 输出 → 2D (4+nc, N) 或 (N, 4+nc), 兼容 (1,C,N)/(C,N,1) 等包装。"""
        arr = np.asarray(out_mat)
        while arr.ndim > 2:
            if arr.shape[0] == 1:
                arr = arr[0]
            elif arr.shape[-1] == 1:
                arr = arr[..., 0]
            else:
                break
        return arr

    def _infer_nc(self) -> int:
        """跑一次全零假推理, 由输出通道数推 nc = 最小维 - 4; 顺带当模型自检。

        ⚠ 不要用 ncnn.Mat(np.zeros((H,W,3))) 构造输入: Python 绑定会把 (640,640,3)
          误读成 w=3/h=640/c=640, 送进 extract 直接**段错误**(已实测)。
          必须走 detect() 里同样用过的 from_pixels 路径。
        构造/推理失败 → 直接抛异常, 让 create_model 在打开串口之前就报错退出。
        """
        blank = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
        mat = ncnn.Mat.from_pixels(blank, ncnn.Mat.PixelType.PIXEL_BGR2RGB,
                                   self.imgsz, self.imgsz)
        mat.substract_mean_normalize([], [1 / 255.0] * 3)
        with self.net.create_extractor() as ex:
            ex.input("in0", mat)
            _, out = ex.extract("out0")
        arr = self._squeeze_out(out)
        if arr.ndim != 2:
            raise RuntimeError(f"ncnn 输出形状无法解析: {np.asarray(out).shape}")
        n_ch = min(arr.shape)
        if n_ch <= 4:
            raise RuntimeError(f"ncnn 输出通道数 {n_ch} 推不出类别数(nc = 通道数 - 4)")
        return n_ch - 4

    @staticmethod
    def _read_names(model_dir: Path):
        """从 metadata.yaml 读类别名(ultralytics 导出时写入), 供语义映射。"""
        meta = model_dir / "metadata.yaml"
        if meta.exists():
            try:
                import yaml
                names = yaml.safe_load(meta.read_text(encoding="utf-8")).get("names")
                if isinstance(names, dict):
                    return {int(k): str(v) for k, v in names.items()}
                if isinstance(names, list):
                    return {i: str(n) for i, n in enumerate(names)}
            except Exception:
                pass
        return None

    @staticmethod
    def _read_imgsz(model_dir: Path) -> int:
        """从 metadata.yaml 读 imgsz(ultralytics 导出时写入), 兼容 640/384。

        ⚠ 必须容忍**标量**写法: ultralytics 导出的正常形态是 list(`imgsz: [640, 640]`),
          但 metadata 被手改/别的工具生成时可能是 `imgsz: 384`(int) 或 `imgsz: '384'`(str)。
          原来无条件 `int(sz[0])`: 字符串会取到第一个**字符**('384'[0]='3' → imgsz=3,
          把画面缩到 3x3 去推理, 基本检不到任何框且完全不报错), 整数会抛 TypeError 被
          except 吞掉 → 静默退回 640(一个 384 训练的模型被喂 640, 精度悄悄掉)。
        """
        meta = model_dir / "metadata.yaml"
        if meta.exists():
            try:
                import yaml
                sz = yaml.safe_load(meta.read_text(encoding="utf-8")).get("imgsz", IMGSZ)
                if isinstance(sz, (list, tuple)):
                    sz = sz[0] if sz else IMGSZ
                sz = int(sz)
                # 离谱尺寸(<=64 或非 32 倍数)一定是解析错了/元数据坏了 —— 宁可报错,
                # 也别静默用一个会把推理彻底废掉的尺寸(见上面 '384' → 3 那个坑)。
                if sz <= 64 or sz % 32:
                    raise ValueError(f"imgsz={sz} 不合理(应 >64 且为 32 的倍数)")
                return sz
            except Exception as e:
                print(f"警告: 读 metadata.yaml 的 imgsz 失败({e}), 用默认 {IMGSZ}")
        return IMGSZ

    def detect(self, bgr: np.ndarray) -> list:
        img, ratio, pad = letterbox(bgr, self.imgsz)
        mat = ncnn.Mat.from_pixels(img, ncnn.Mat.PixelType.PIXEL_BGR2RGB,
                                   self.imgsz, self.imgsz)
        mat.substract_mean_normalize([], [1 / 255.0] * 3)
        with self.net.create_extractor() as ex:
            ex.input("in0", mat)
            _, out_mat = ex.extract("out0")
        arr = self._squeeze_out(out_mat)
        return decode_yolo11(arr, self.nc, bgr.shape[:2], ratio, pad,
                             self.conf, self.iou)


def create_model(runtime: str = "auto", model: str = ""):
    """按 --runtime/--model 解析推理后端, 返回 (backend, model_object)。

    runtime: auto(默认, 优先 ncnn) | onnx | ncnn
    model: 文件路径 = onnx, 目录(含 model.ncnn.param) = ncnn; 空 = 自动搜索。
    模型建不出来直接抛异常, 调用方 catch → 退出码 1(不残留已开串口)。
    """
    def is_ncnn_path(p: str) -> bool:
        try:
            d = Path(p)
            return d.is_dir() and (d / "model.ncnn.param").is_file()
        except OSError:
            return False

    # DEBUFF_MODEL: 把模型钉死在指定路径上(部署时最省心的做法 —— 自动搜索再好,
    # 也不如"就用这个"确定)。--model 优先于它。
    model = model or os.environ.get("DEBUFF_MODEL", "").strip()

    if runtime == "onnx":
        path = model or default_model_path()
        t = int(os.environ.get("ORT_THREADS", "0") or 0)
        return "onnx", YOLO11Onnx(path, threads=t)

    if runtime == "ncnn":
        if ncnn is None:
            raise RuntimeError(
                "未安装 ncnn: 树莓派上 pip3 install ncnn, 或改用 --runtime onnx")
        if model:
            # ⚠ 显式给了 --model 就必须用它: 以前这里写的是
            #   `model if is_ncnn_path(model) else find_ncnn_dir()`, 路径打错/少拷了
            #   model.ncnn.param 时会**静默换成自动搜到的另一个模型**(可能就是旧单类),
            #   操作者完全不知道自己的参数被忽略了。
            if not is_ncnn_path(model):
                raise RuntimeError(
                    f"--model 指定的 {model} 不是有效的 ncnn 模型目录"
                    f"(需要里面有 model.ncnn.param)。")
            d = model
        else:
            d = find_ncnn_dir()
        if not d:
            raise RuntimeError(
                "找不到 ncnn 模型目录(model.ncnn.param).\n"
                "请用 --model 指向 best_ncnn_model 目录, 或设 DEBUFF_NCNN_DIR=目录,\n"
                "或把 best_ncnn_model 文件夹放到脚本同目录/桌面。")
        t = int(os.environ.get("NCNN_THREADS", "4"))
        return "ncnn", YOLO11Ncnn(d, threads=t)

    # auto: 显式 --model 优先; 否则装了 ncnn 且有模型就用 ncnn, 退回 onnx
    if model:
        if is_ncnn_path(model):
            t = int(os.environ.get("NCNN_THREADS", "4"))
            return "ncnn", YOLO11Ncnn(model, threads=t)
        # 不是 ncnn 目录就一律按 onnx 文件处理; 路径不对时由 YOLO11Onnx 抛出明确错误
        # (而不是偷偷换成别的模型)。
        t = int(os.environ.get("ORT_THREADS", "0") or 0)
        return "onnx", YOLO11Onnx(model, threads=t)
    if ncnn is not None:
        d = find_ncnn_dir()
        if d:
            t = int(os.environ.get("NCNN_THREADS", "4"))
            return "ncnn", YOLO11Ncnn(d, threads=t)
    return "onnx", YOLO11Onnx(default_model_path())


def resolve_class_ids(model) -> tuple:
    """决定 debuff/buff 分别对应模型的哪个类别编号。

    优先级:
      1. 模型元数据类别名(onnx metadata / ncnn metadata.yaml): 名字含 'debuff'/'减' → debuff;
         含 'buff'/'增' 且非 debuff → buff
         (必须先判 'debuff' 再判 'buff', 因为 'debuff' 包含 'buff' 子串)
      2. nc>1 但读不到类别名 → 只能按 DEBUFF_CLS/BUFF_CLS 猜, 大声警告(可能反!)
      3. nc<=1 单类模型 → (DEBUFF_CLS, None), 所有目标按 debuff 处理(兼容旧行为)

    ⚠ 先看名字再看 nc: 名字是唯一真正带语义的信息。只凭 nc 判断会踩到
      "ncnn 后端没算出 nc → 双类模型被当单类 → 增益块永远不被追踪" 这个坑。
    返回 (debuff_cls, buff_cls, 来源说明); buff_cls=None 表示该模型没有增益类。
    """
    names = getattr(model, "names", None)
    if isinstance(names, dict) and names:
        d = b = None
        for k, v in names.items():
            low = _norm_name(v)     # 与 _class_roles 用同一种规范化, 别搞两套
            if ("debuff" in low or "减" in low) and d is None:
                d = k
            elif ("buff" in low or "增" in low) and b is None:
                b = k
        if d is not None:
            return d, b, f"模型元数据类别名 {names}"
        print(f"警告: 模型有类别名 {names} 但解析不出 debuff/buff, 退回 DEBUFF_CLS/BUFF_CLS")
    nc = getattr(model, "nc", 1)
    if nc > 1:
        # 多类模型读不到类别名 = 无法知道哪一类是减益块。猜错的方向是"去撞减益块",
        # 那是直接把分送给对方的致命错误, 所以**默认拒绝猜**; 只有人显式设了
        # DEBUFF_CLS/BUFF_CLS(自担责任的显式选择)才按他说的来。
        if not CLS_FROM_ENV:
            raise RuntimeError(
                f"模型是 {nc} 类, 但读不到类别名(metadata), 无法确定哪一类是 debuff。\n"
                f"      !! 类别映射一旦反了, 小车会去撞减益块(给对方加分), 所以这里不猜。\n"
                f"      请二选一:\n"
                f"        1) 用带 metadata.yaml 的 ncnn 模型, 或带 names 元数据的 onnx;\n"
                f"        2) 确认类别顺序后显式指定: DEBUFF_CLS=0 BUFF_CLS=1 python3 pi_detect.py")
        print(f"警告: 模型是 {nc} 类且读不到类别名, 按你显式设置的 "
              f"DEBUFF_CLS={DEBUFF_CLS} / BUFF_CLS={BUFF_CLS} 处理。\n"
              f"      !! 映射反了小车会去撞减益块(给对方加分) —— 请确认顺序无误。")
        return DEBUFF_CLS, BUFF_CLS, f"环境变量显式指定(nc={nc} 但无类别名)"
    return DEBUFF_CLS, None, "单类模型: 所有目标按 debuff 处理"


class DebuffFrameSender:
    """持续发送"当前帧": 有目标发 BOMB/BOOST 帧, **没目标发 NONE(0x00) 心跳帧**。

    当前帧由检测线程通过 set_frame() 更新(BOMB 帧或带坐标的 BOOST 帧),
    发送线程每 interval 秒发一帧最新内容 — 换帧即时生效, 不用重启线程。

    为什么没目标也要发: 只停发的话 STM32 靠 PI_TIMEOUT_MS(500ms) 才判定失效,
    叠上调用方 STOP_DELAY_SEC 会让小车朝已消失的目标多冲一段; 发 NONE 帧则
    **当拍**就让躲避/追踪条件不成立(STM32 只按帧 TYPE 判条件), 同时保持链路新鲜。

    注意分工: 目标消失 → 心跳帧(不停发); 采集/推理失效(摄像头掉线、异常) →
    stop() 真停发(fail-safe, 宁可让 STM32 判离线也不发假数据)。
    """

    def __init__(self, ser, interval: float = SEND_INTERVAL):
        self.serial_port = ser
        self.frame = None
        self.interval = interval
        self._send_frames = False
        self._send_thread = None
        self.lock = threading.Lock()

    def set_frame(self, frame: bytes):
        self.frame = frame        # CPython 引用赋值原子, 发送线程读到的总是完整帧

    def start(self):
        self._send_frames = True
        if self._send_thread is None or not self._send_thread.is_alive():
            self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
            self._send_thread.start()

    def stop(self):
        self._send_frames = False

    def stop_and_join(self, timeout: float = 2.0):
        """停发并等发送线程退出(避免与 serial.close() 撞车)。"""
        self.stop()
        if self._send_thread is not None and self._send_thread.is_alive():
            self._send_thread.join(timeout=timeout)

    def _send_loop(self):
        failures = 0
        while self._send_frames:
            if self.serial_port is not None and self.serial_port.is_open:
                frame = self.frame
                if frame is not None:
                    with self.lock:
                        try:
                            self.serial_port.write(frame)
                            self.serial_port.flush()
                            failures = 0
                        except Exception as e:
                            failures += 1
                            if failures == 1 or failures % 100 == 0:
                                print(f"串口写入失败({failures} 次): {e}")
            time.sleep(self.interval)


class DebuffDetector:
    """采集线程 + 检测线程 + 发送线程, 结构对齐 car7.15 vision.py。

    健康门控: 采集成功且有新鲜帧 + 检测成功, 才允许发帧;
    任一环节失效 → 立即停发(fail-safe, 见模块 docstring)。
    """

    def __init__(self, model):
        self.frame_lock = threading.Lock()
        self.capture_image = None
        self.capture_success = False
        self.capture_width = self.capture_height = 0.0
        self._last_capture = 0.0
        self._last_seen = 0.0
        self._last_type = FRAME_TYPE_NONE     # 上一次发出去的帧类型 → 决定用哪个停发延迟
        self._warn_ts = 0.0                  # 限流: "上报分辨率与实际帧宽不一致"告警
        self.display_frame = None

        # 模型对象已由 create_model 建好(onnx 或 ncnn, 同 .detect() 接口);
        # 建失败会直接抛异常 → 不会残留已开串口。
        self.yolo = model

        self.detect_error = False            # 检测线程意外死亡的标记 → run() 退出码 1
        self._err_log_ts = 0.0               # 检测异常打印限流(10s 一次)

        # ---- 类别解析必须在**开串口之前**做完 ----
        # 映射不对 / 没有增益类时应当直接失败退出, 而不是先占住串口再报错。
        self.class_names = getattr(model, "names", None)
        self.debuff_cls, self.buff_cls, cls_src = resolve_class_ids(model)
        if self.buff_cls is not None and self.buff_cls == self.debuff_cls:
            print("警告: BUFF_CLS 与 DEBUFF_CLS 相同, 忽略增益类(全部按减益处理)")
            self.buff_cls = None
        print(f"类别映射: debuff=cls{self.debuff_cls}"
              + (f", buff=cls{self.buff_cls}" if self.buff_cls is not None else "(无增益类)")
              + f"  [{cls_src}]")

        # 期望类别数: 设 DEBUFF_EXPECT_NC=2 就要求**必须是双类模型**, 否则直接失败。
        # 小车要追增益块就必须加载到双类模型; 加载成单类时整条 buff 功能会静默消失
        # (不报错、不追块), 属于现场最难发现的问题。实车部署建议设成 2。
        nc = int(getattr(model, "nc", 1) or 1)
        expect = os.environ.get("DEBUFF_EXPECT_NC", "").strip()
        if expect:
            try:
                want = int(expect)
            except ValueError:
                raise RuntimeError(f"DEBUFF_EXPECT_NC={expect} 不是整数")
            if want != nc:
                raise RuntimeError(
                    f"DEBUFF_EXPECT_NC={want}, 但当前模型是 nc={nc} 类"
                    f"({getattr(model, 'names', None)})。\n"
                    f"      很可能选到了旧的单类模型 —— 看上面 [选模型] 那几行日志, "
                    f"或用 --model / DEBUFF_MODEL 显式指定。")
            if self.buff_cls is None:
                # 只比 nc 是不够的: nc=2 但第二类不叫 buff/增(比如 'penalty')时,
                # 类别数对得上、增益类却根本解析不出来 —— 增益块照样静默失效, 而这
                # 正是把 DEBUFF_EXPECT_NC 推荐成"实车部署建议设 2"想防的事。
                raise RuntimeError(
                    f"DEBUFF_EXPECT_NC={want} 数目上满足(模型 nc={nc}), 但**没能解析出增益类**: "
                    f"类别名 {getattr(model, 'names', None)} 里找不到 buff/增。\n"
                    f"      这样部署上去增益块会静默失效(车不追块, 也不报错)。\n"
                    f"      请确认导出模型的类别名是 debuff/buff, 或用 DEBUFF_CLS/BUFF_CLS 显式指定。")
        elif self.buff_cls is None:
            print("警告: 当前模型没有增益类(buff) —— **增益块不会被追踪**。\n"
                  "      如果是部署到小车的模型, 基本可以断定选错模型了(应为双类 nc=2);\n"
                  "      若确实只想躲减益块, 忽略本行; 想把它变成硬错误请设 "
                  "DEBUFF_EXPECT_NC=2。")

        self.serial_port = None
        self._init_serial()
        self.sender = DebuffFrameSender(self.serial_port)

        self._stop = threading.Event()
        self.capture_thread = threading.Thread(target=self._capture_run, daemon=True)
        self.detect_thread = threading.Thread(target=self._detect_run, daemon=True)

    def _init_serial(self):
        if serial is None:
            self._serial_fail("未安装 pyserial")
            return
        try:
            self.serial_port = serial.Serial(
                SERIAL_PORT, SERIAL_BAUD,
                serial.EIGHTBITS, serial.PARITY_NONE,
                serial.STOPBITS_ONE, timeout=None, write_timeout=0.5,
            )
            print(f"串口 {SERIAL_PORT} 已打开")
        except Exception as e:
            self._serial_fail(f"{SERIAL_PORT} 打开失败({e})")

    def _serial_fail(self, why: str) -> None:
        """串口不可用时的唯一出口: 默认**直接失败退出**, 只有显式放行才降级。

        ⚠ 为什么不能只打一句警告就继续: 串口打不开时(树莓派上 /dev/ttyACM0 需要用户在
          dialout 组、USB 没插好、没装 pyserial)脚本会打印"检测正常"并以**退出码 0**
          结束运行, 而实际**一帧都发不出去** —— 车完全不躲减益块、不追增益块。现场看到的
          是"视觉看起来明明是好的", 极难查; systemd 还会把它当成健康服务。
          起不来(退出码 1)远比"假装在工作"好: 前者一眼能看见, 后者要丢一整场。
          确实只想跑检测(比如在电脑上试模型), 显式设 DEBUFF_ALLOW_NO_SERIAL=1。
        """
        msg = (f"串口不可用: {why}\n"
               f"      部署到小车时这等于一帧都发不出去(车不会躲减益块/不追增益块)。\n"
               f"      常见原因与处理:\n"
               f"        * 用户不在 dialout 组  -> sudo usermod -aG dialout $USER  (然后重新登录)\n"
               f"        * 设备节点不对          -> ls /dev/ttyACM* /dev/ttyUSB*  或 SERIAL_PORT=/dev/ttyUSB0\n"
               f"        * 没装 pyserial         -> pip3 install pyserial")
        allow = os.environ.get("DEBUFF_ALLOW_NO_SERIAL", "").strip().lower()
        if allow in ("1", "true", "yes", "on"):
            print(f"警告: {msg}\n"
                  f"      DEBUFF_ALLOW_NO_SERIAL={allow} 已显式放行 -> 继续运行, 但只检测不发送。")
            return
        raise RuntimeError(msg + f"\n      确实只想跑检测不发帧, 请显式设 DEBUFF_ALLOW_NO_SERIAL=1。")

    def class_label(self, cls: int) -> str:
        """类别编号 → 显示名(模型元数据有名字用名字, 否则 clsN)。"""
        if self.class_names and cls in self.class_names:
            return str(self.class_names[cls])
        return f"cls{cls}"

    # ---- 采集 ----
    def _apply_resolution(self, cap) -> None:
        """请求目标分辨率并读回实际值。**首开和重连都走这里**, 别只在一处 set。"""
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        self.capture_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        self.capture_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"分辨率: {int(self.capture_width)}x{int(self.capture_height)}")

    def _probe(self):
        for node in CAMERA_NODES:
            cap = cv2.VideoCapture(node, cv2.CAP_V4L2)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    return cap, node
                cap.release()
        return None, None

    def _capture_run(self):
        cap, node = self._probe()
        if cap is None:
            print("未找到摄像头! 检测线程将判定采集失效并保持停发。")
            return
        print(f"摄像头 /dev/video{node}")
        self._apply_resolution(cap)

        try:
            while not self._stop.is_set():
                try:
                    ret, img = cap.read()
                except Exception as e:
                    print(f"摄像头读取异常: {e}")
                    ret = False
                if ret and img is not None:
                    with self.frame_lock:
                        self.capture_image = img
                        self.capture_success = True
                        self._last_capture = time.time()
                else:
                    # 掉线/读失败: 明确置失效(检测线程据此停发), 然后尝试重连
                    with self.frame_lock:
                        self.capture_image = None
                        self.capture_success = False
                    self._last_capture = 0.0
                    if cap is not None:
                        cap.release()
                    print("摄像头掉线, 每 2s 尝试重连...")
                    cap, node = None, None
                    while not self._stop.is_set():
                        time.sleep(2.0)
                        cap, node = self._probe()
                        if cap is not None:
                            print(f"摄像头重连成功 /dev/video{node}")
                            self._apply_resolution(cap)
                            # ↑ 必须是同一个方法: 之前重连只 set(setting) 不读回,
                            #   self.capture_* 还是第一次打开时的旧值 —— 重连后若分辨率
                            #   变了(换了节点/驱动重新协商), 上报值和实际帧宽就长期脱节。
                            break
        finally:
            if cap is not None:
                cap.release()
            print("采集线程退出")

    # ---- 检测 + 发送控制 ----
    def _detect_run(self):
        try:
            while not self._stop.is_set():
                now = time.time()
                with self.frame_lock:
                    image = self.capture_image.copy() if self.capture_image is not None else None
                    capture_ok = self.capture_success and (now - self._last_capture) < CAPTURE_TIMEOUT
                if not capture_ok or image is None:
                    self.sender.stop()          # 采集失效 → 停发(fail-safe)
                    time.sleep(0.5)
                    continue

                try:
                    dets = self.yolo.detect(image)
                except Exception as e:
                    now = time.time()
                    if now - self._err_log_ts > 10.0:    # 限流, 避免刷屏
                        print(f"[检测异常] {e}")
                        traceback.print_exc()
                        self._err_log_ts = now
                    self.sender.stop()          # 检测异常 → 停发(fail-safe)
                    time.sleep(1.0)
                    continue

                # ---- 归类成"发什么帧": 规则与离线校验共用同一份 decide_frame ----
                # 分母用**实际帧宽**, 不用摄像头 cap.get() 的上报值: 上报值可能被驱动
                # 谎报(实测有报 640 实际出 1280), 那样 MIN_W 的分母会凭空翻倍, "多远
                # 算够大"静默变掉。上报值只用来发现不一致 → 告警(限流 30s), 不参与计算。
                img_w = float(image.shape[1])
                img_h = float(image.shape[0])
                if self.capture_width and abs(self.capture_width - img_w) > 1:
                    if time.time() - self._warn_ts > 30.0:
                        self._warn_ts = time.time()
                        print(f"警告: 摄像头上报宽度 {int(self.capture_width)} 与实际帧宽 "
                              f"{int(img_w)} 不一致, 距离门控按实际帧宽算")

                active, payload = decide_frame(dets, self.debuff_cls, self.buff_cls,
                                               img_w, img_h)
                if payload is not None:
                    self.sender.set_frame(build_frame(*payload))
                    self._last_seen = time.time()
                    self._last_type = payload[0]     # payload=(type, x, y, w, h)
                    self.sender.start()
                else:
                    # 停发延迟按**上一次发出去的帧类型**取, 两类不同(见 BUFF_STOP_DELAY_SEC):
                    # 躲减益块时多保持一会儿是安全方向; 追增益块时多保持就是朝已消失的
                    # 块继续冲, 所以增益块默认 0。
                    hold = (BUFF_STOP_DELAY_SEC if self._last_type == FRAME_TYPE_BOOST
                            else STOP_DELAY_SEC)
                    if time.time() - self._last_seen > hold:
                        # 目标消失 → 发 NONE 心跳帧, **不要只是停发**。
                        # 只停发的话, STM32 的 last_update_ms 还被最后一帧"钉"着,
                        # 要再等满 PI_TIMEOUT_MS(500ms) 才失效 → 叠上这里的 hold,
                        # 小车会朝一个已经消失的目标多冲最多 ~1s。
                        # NONE 帧(type=0x00)让躲避/追踪条件当拍就不成立, 又保持链路新鲜。
                        # 语义分工(STM32 侧 pi_comm.c): valid 只代表"链路活着"(心跳也算),
                        # 有没有目标看 type != PI_TAG_NONE / target_det —— 别再用 valid 判目标。
                        self.sender.set_frame(build_frame(FRAME_TYPE_NONE))
                        self._last_type = FRAME_TYPE_NONE
                        self.sender.start()
                    # else: 目标刚消失, 还在 hold 内 → 保持上一帧不动

                if DEBUG_DISPLAY:
                    frame = image.copy()
                    for d in dets:
                        x1, y1, x2, y2 = (int(v) for v in d["xyxy"])
                        color = (0, 0, 255) if d["cls"] == self.debuff_cls else (0, 255, 0)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame, f"{self.class_label(d['cls'])} {d['conf']:.2f}",
                                    (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                    if active == "BOMB":
                        status, color = "[BOMB!]", (0, 0, 255)
                    elif active == "BOOST":
                        status, color = "[BOOST]", (0, 255, 0)
                    else:
                        status, color = "SAFE", (0, 255, 0)
                    cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
                    with self.frame_lock:
                        self.display_frame = frame   # 只在主线程 imshow(highgui 非线程安全)
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        except Exception:
            print("检测线程异常退出")
            traceback.print_exc()
            self.detect_error = True
        finally:
            self.sender.stop()
            print("检测线程退出")

    def run(self):
        print(f"BOMB帧: {build_frame(FRAME_TYPE_BOMB).hex(' ')}  "
              f"BOOST帧样例: {build_frame(FRAME_TYPE_BOOST, 100, 0, 300, 300).hex(' ')}")
        print(f"conf>{CONF_THRES}  min_w: debuff>{MIN_W} buff>{BUFF_MIN_W}(分母=实际帧宽)  "
              f"停发延迟: debuff {STOP_DELAY_SEC}s / buff {BUFF_STOP_DELAY_SEC}s"
              f"(目标消失后还会朝旧目标冲这么久)")
        self.capture_thread.start()
        self.detect_thread.start()
        try:
            if DEBUG_DISPLAY:
                # 所有 highgui(namedWindow/imshow/waitKey)都在主线程, 避免 GTK 非线程安全
                cv2.namedWindow("debuff", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("debuff", 960, 540)
                while not self._stop.is_set() and self.detect_thread.is_alive():
                    with self.frame_lock:
                        frame = self.display_frame
                    if frame is not None:
                        cv2.imshow("debuff", frame)
                        if cv2.waitKey(1) & 0xFF == 27:   # ESC 退出
                            self._stop.set()
                            break
            else:
                while self.detect_thread.is_alive():
                    time.sleep(0.2)
        except KeyboardInterrupt:
            print("退出")
        finally:
            self._stop.set()
            self.sender.stop_and_join()
            self.capture_thread.join(timeout=3)
            self.detect_thread.join(timeout=3)
            if self.serial_port is not None:
                try:
                    self.serial_port.close()
                except Exception:
                    pass
            cv2.destroyAllWindows()
        if self.detect_error:
            print("检测线程意外退出, 进程以退出码 1 结束(便于 systemd 监控发现)")
            raise SystemExit(1)


def run_source(args, model) -> int:
    """单张图自测: 只推理, 不连摄像头/串口。"""
    img = cv2.imread(args.source)
    if img is None:
        print(f"无法读取图片: {args.source}")
        auto = find_test_image()
        if auto:
            print(f"提示: 自动找到了 {auto}, 可改跑:")
            print(f"  python3 {Path(sys.argv[0]).name} --source {auto}")
        else:
            print("请给绝对路径(相对路径以当前目录为准), 例如:")
            print(f"  python3 {Path(sys.argv[0]).name} --source ~/Desktop/照片.jpg")
            print("或者先 cd 到图片所在目录再跑: cd ~/Desktop && python3 ~/pi_detect.py --source 照片.jpg")
        return 1
    dets = model.detect(img)
    names = getattr(model, "names", None)
    print(f"检测到 {len(dets)} 个目标:")
    for d in dets:
        label = str(names[d["cls"]]) if names and d["cls"] in names else f"cls{d['cls']}"
        print(f"  [{label}] {d['xyxy']}  conf={d['conf']:.3f}")
    if args.save:
        # 颜色要按**这个模型解析出来的** debuff 编号, 不能用模块级默认 DEBUFF_CLS:
        # 自测图正是用来肉眼核对"发的帧对不对"的, 画错颜色等于把结论带偏。
        try:
            d_cls = resolve_class_ids(model)[0]
        except Exception:
            d_cls = DEBUFF_CLS
        for d in dets:
            x1, y1, x2, y2 = (int(v) for v in d["xyxy"])
            color = (0, 0, 255) if d["cls"] == d_cls else (0, 255, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            label = str(names[d["cls"]]) if names and d["cls"] in names else f"cls{d['cls']}"
            cv2.putText(img, f"{label} {d['conf']:.2f}", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.imwrite(args.save, img)
        print(f"已保存: {args.save}")
    return 0


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="YOLO11n 能量块检测 → 串口发帧(减益块躲避/增益块追踪)")
    p.add_argument("--model", default="",
                   help="模型路径: 文件=onnx, 目录(含 model.ncnn.param)=ncnn; 默认自动找")
    p.add_argument("--runtime", default="auto", choices=["auto", "onnx", "ncnn"],
                   help="推理后端: auto(优先 ncnn)/onnx/ncnn")
    p.add_argument("--source", help="单张图自测模式(不连摄像头/串口)")
    p.add_argument("--save", help="自测模式下保存标注图")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    try:
        backend, model = create_model(args.runtime, args.model)
    except (RuntimeError, FileNotFoundError) as e:
        print(f"[错误] {e}")
        raise SystemExit(1)
    print(f"后端: {backend}")
    if args.source:
        raise SystemExit(run_source(args, model))
    try:
        node = DebuffDetector(model)
    except (RuntimeError, FileNotFoundError) as e:
        # 类别映射解析失败(例如多类模型读不到类别名)也要以"错误+退出码1"结束,
        # 而不是把 traceback 甩给操作者 —— 这类失败的提示信息本身就是给他看的。
        print(f"[错误] {e}")
        raise SystemExit(1)
    node.run()


if __name__ == "__main__":
    main()
