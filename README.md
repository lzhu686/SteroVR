# StereoVR - USB 双目立体视觉 VR 流媒体库

<div align="center">

**实时双目视觉 → VR 头显**

[![Python 3.8+](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

</div>

---

## 概述

StereoVR 是一个 USB 双目相机流媒体库，支持将双目视频实时传输到 VR 头显，适用于:

- 🤖 **机器人遥操作** - 远程控制时的第一人称视角
- 🎮 **VR 透视** - 将真实世界投射到 VR 中
- 🔬 **立体视觉研究** - 双目视觉算法开发

### 支持的传输模式

| 特性 | WebSocket 模式 | XRoboToolkit 模式 |
|------|----------------|-------------------|
| 协议 | WSS (Base64 JPEG) | UDP RTP H.264 |
| 延迟 | ~80-150ms | **~30-50ms** |
| 客户端 | 任意 WebXR 浏览器 | XRoboToolkit Unity |
| 设备支持 | Quest/PICO/Vision Pro | PICO |
| 适用场景 | 演示/调试 | **遥操作(推荐)** |

---

## 快速开始

### 安装依赖

```bash
pip install opencv-python numpy websockets

# XRoboToolkit 模式还需要 FFmpeg:
# Ubuntu: sudo apt install ffmpeg
# Windows: 下载 ffmpeg.org 并添加到 PATH
```

### 方式一: WebSocket 模式 (WebXR)

```bash
python start.py
```

在 VR 浏览器访问 `https://你的IP:8445`

### 方式二: XRoboToolkit 模式 (低延迟遥操作)

```bash
python start_xrobo_compat.py
```

在 PICO 的 XRoboToolkit Client 中:
1. 选择视频源 `USB_STEREO`
2. 输入 PC IP 地址
3. 点击 Listen

---

## 协议原理对比

### 为什么 UDP RTP H.264 延迟更低？

#### WebSocket 模式的延迟分析

```
┌─────────────────────────────────────────────────────────────────────────┐
│  WebSocket 模式 (总延迟 ~80-150ms)                                       │
│                                                                          │
│  发送端:                                                                  │
│  相机采集 → JPEG 编码 → Base64 → JSON 封装 → TCP/WebSocket               │
│    16ms      5-10ms     3-5ms      1ms        20-40ms                    │
│                                                                          │
│  接收端:                                                                  │
│  WebSocket → JSON 解析 → Base64 解码 → JPEG 解码 → Canvas 渲染           │
│    1-5ms       1ms         2ms         5-10ms       16ms                 │
└─────────────────────────────────────────────────────────────────────────┘
```

**延迟来源:**
1. **JPEG 逐帧编码** - 每帧独立压缩，无法利用帧间冗余，编码慢
2. **Base64 开销** - 数据量增加 33%，传输时间增加
3. **TCP 重传** - 丢包需要重传，增加延迟抖动
4. **JS 解码** - 浏览器 JavaScript 解码效率较低

#### UDP RTP H.264 模式的延迟分析

```
┌─────────────────────────────────────────────────────────────────────────┐
│  UDP RTP H.264 模式 (总延迟 ~30-50ms)                                    │
│                                                                          │
│  发送端:                                                                  │
│  相机采集 → H.264 硬编码 → RTP 封装 → UDP 发送                            │
│    16ms       1-3ms        <1ms       <1ms                               │
│                                                                          │
│  接收端:                                                                  │
│  UDP 接收 → RTP 解封装 → H.264 硬解码 → GPU 纹理                          │
│    <1ms        <1ms         1-2ms       <1ms                             │
└─────────────────────────────────────────────────────────────────────────┘
```

**低延迟原因:**
1. **帧间压缩** - H.264 利用帧间冗余，P帧只存差异，编码快 5-10 倍
2. **硬件加速** - GPU/专用芯片编解码，延迟极低
3. **UDP 无重传** - 丢包不等待，牺牲可靠性换取低延迟
4. **零拷贝** - 解码直接输出到 GPU 纹理，无内存拷贝

### H.264 关键参数

```
┌────────────────────────────────────────────────────────────────────────┐
│  GOP 结构 (关键帧间隔 = 15)                                              │
│                                                                         │
│   I ─── P ─── P ─── P ─── ... ─── P ─── I ─── P ─── ...                │
│   │                               │     │                               │
│   └────── 15 帧 (0.25秒@60fps) ───┘     │                               │
│                                          │                               │
│  I帧: 完整图像，可独立解码，体积大 (~50KB)                               │
│  P帧: 只存储差异，体积小 (~5KB)                                          │
│                                                                         │
│  遥操作推荐:                                                             │
│  - GOP = 15-30 (丢包后 0.25-0.5 秒恢复)                                  │
│  - zerolatency (禁用 B 帧)                                               │
│  - ultrafast (最快编码)                                                  │
└────────────────────────────────────────────────────────────────────────┘
```

### 延迟测量对比

| 环节 | WebSocket | UDP RTP H.264 |
|------|-----------|---------------|
| 相机采集 | 16ms | 16ms |
| 编码 | 5-10ms | 1-3ms |
| 传输封装 | 4-6ms | <1ms |
| 网络传输 | 20-40ms | 5-15ms |
| 接收解码 | 5-10ms | 1-2ms |
| 渲染 | 16ms | 8ms |
| **总计** | **66-98ms** | **32-45ms** |

---

## 项目结构

```
StereoVR/
│
├── 📦 sterovr/                    # 核心 Python 包
│   ├── __init__.py                # 库入口，导出公共 API
│   ├── config.py                  # 配置管理 (dataclass)
│   ├── camera.py                  # 相机采集模块
│   ├── server.py                  # WebSocket 服务器
│   ├── h264_sender.py             # H.264 编码器 (FFmpeg)
│   └── xrobo_compat_server.py     # XRoboToolkit 兼容服务器
│
├── 🌐 web/                        # Web 前端
│   ├── index.html                 # 主页
│   ├── dual_infrared_viewer.html  # 2D 双目查看器
│   └── dual_infrared_vr_viewer.html  # VR 立体查看器
│
├── 📖 docs/                       # 文档
│   ├── QUICK_START.md             # 快速开始
│   ├── INTEGRATION_GUIDE.md       # XRoboToolkit 集成指南
│   ├── BODY_TRACKING_DESIGN.md    # 身体追踪设计
│   └── UNIFIED_TELEOP_SYSTEM.md   # 统一遥操作系统
│
├── 🚀 start.py                    # WebSocket 模式启动
├── 🚀 start_xrobo_compat.py       # XRoboToolkit 模式启动
└── README.md                      # 项目说明
```

---

## API 使用

### 作为 Python 库

```python
# 遥操作场景 (低延迟)
from sterovr import start_xrobo_server
start_xrobo_server(device_id=0)

# WebXR 场景
from sterovr import start_websocket_server
start_websocket_server()
```

### 自定义配置

```python
from sterovr import CameraConfig, EncoderConfig, StereoVRConfig

config = StereoVRConfig(
    camera=CameraConfig(
        stereo_width=2560,
        stereo_height=720,
        fps=60
    ),
    encoder=EncoderConfig(
        bitrate=8_000_000,
        keyframe_interval=15  # 短 GOP，快速恢复
    )
)
```

### 直接使用相机模块

```python
from sterovr import create_camera

camera = create_camera()
camera.start()

while True:
    frame = camera.get_frame()
    if frame:
        # frame.left_eye  - 左眼图像
        # frame.right_eye - 右眼图像
        cv2.imshow('Left', frame.left_eye)
```

---

## 支持的相机

| 型号 | 传感器 | 分辨率 | 帧率 | 基线 | 视场角 |
|------|--------|--------|------|------|--------|
| HBVCAM-F2439GS-2 V11 | AR0234 | 2560x720 | 60fps | 60mm | 125° |

支持任何 UVC 双目相机 (左右并排输出)。

---

## 遥操作优化建议

### 1. 降低分辨率

```bash
python start_xrobo_compat.py --width 1920 --height 540
```

### 2. 使用 USB 有线

```bash
adb reverse tcp:63901 tcp:63901
adb reverse tcp:12345 tcp:12345
```

### 3. 使用 5GHz WiFi

2.4GHz 延迟高且不稳定。

---

## 常见问题

### Q: 为什么用 UDP 而不是 TCP？

TCP 丢包会重传，阻塞后续帧。实时视频场景下，**丢一帧比等一帧更好**。

### Q: zerolatency 是什么？

禁用 B 帧和前瞻分析，减少编码缓冲，降低延迟。

### Q: 如何测量延迟？

在相机前放置毫秒计时器，拍摄 VR 画面，对比时间差。

---

## 作者

**Liang ZHU** - lzhu686@connect.hkust-gz.edu.cn

香港科技大学 (广州)

## 许可证

MIT License
