# RGB125 双目立体视觉 VR 系统

基于 USB 双目相机的实时立体视觉 VR 透视系统，支持 WebXR 标准，兼容多种 VR 设备。

## ✨ 特性

- 🎯 **一键启动**: 简化的启动流程，只需一个命令
- 🔒 **安全加密**: 自动配置 HTTPS + WSS 加密传输
- 📱 **跨设备支持**: 支持 Quest 3、PICO 4、Vision Pro 等主流 VR 设备
- ⚡ **高性能**: 60fps 实时传输，低延迟优化
- 🔌 **USB有线模式**: 自动ADB端口转发，延迟更低更稳定
- 🎨 **双目立体**: 125° 视场角，60mm 基线距离

## 🎮 支持的 VR 设备

| 设备 | 状态 |
|------|------|
| Meta Quest 3 / Quest Pro | ✅ 完全支持 |
| Meta Quest 2 | ✅ 支持 |
| PICO 4 / PICO Neo3 | ✅ 支持 |
| Apple Vision Pro | ✅ 支持 |
| HTC Vive / Vive Pro | ✅ 支持 |
| Valve Index | ✅ 支持 |

## 🚀 快速开始

### 1. 安装依赖

```bash
pip install opencv-python websockets numpy
```

### 2. 启动服务器

```bash
python start.py
```

服务器会自动：
- ✅ 检测USB连接的VR设备并设置ADB端口转发
- ✅ 生成 SSL 证书（如果不存在）
- ✅ 启动 HTTPS 文件服务器（端口 8445）
- ✅ 启动 WSS WebSocket 服务器（端口 8765）

### 3. 在 VR 设备中访问

**USB有线模式（推荐，低延迟）**：
```
https://127.0.0.1:8445
```

**WiFi无线模式**：
```
https://你的电脑IP:8445
```

## 🔌 USB有线模式

USB有线连接比WiFi延迟更低、更稳定。

### 前置条件

1. VR设备开启**开发者模式**和**USB调试**
2. 安装 ADB 工具
3. USB线连接VR设备到电脑

### 验证连接

```bash
adb devices
# 应显示类似: PA8A10MGJ3060002D    device
```

### 工作原理

```
VR浏览器 → 127.0.0.1:8445 → USB线 → PC服务器
         (ADB reverse)
```

启动脚本会自动执行 `adb reverse` 命令，无需手动配置。

## 📁 项目结构

```
SteroVR/
├── start.py              # 🚀 主启动脚本（只需运行这个！）
├── server.py             # WebSocket 服务器核心代码
├── index.html            # 🏠 主页导航（VR设备访问入口）
├── README.md             # 📖 完整文档
├── QUICK_START.md        # ⚡ 快速上手指南
│
├── web/                  # 前端应用文件
│   ├── dual_infrared_viewer.html      # 普通2D查看器
│   └── dual_infrared_vr_viewer.html   # VR 立体查看器
│
├── server.crt            # SSL 证书（自动生成）
└── server.key            # SSL 私钥（自动生成）
```

## ⚙️ 配置参数

如需调整相机参数，编辑 `server.py` 文件顶部：

```python
STEREO_WIDTH = 2560     # 双目拼接图像宽度
STEREO_HEIGHT = 720     # 双目拼接图像高度
CAMERA_WIDTH = 1280     # 单目图像宽度
CAMERA_HEIGHT = 720     # 单目图像高度
TARGET_FPS = 60         # 目标帧率
JPEG_QUALITY = 100      # JPEG压缩质量 (1-100)
```

## 🔧 故障排除

### USB设备未检测到

**现象**: 启动时显示"未检测到USB设备"

**解决方法**:
1. 确认VR设备已开启开发者模式和USB调试
2. 运行 `adb devices` 检查设备状态
3. 若显示 `unauthorized`，在VR中确认USB调试授权

### WebSocket 连接失败

**现象**: 页面显示 "WebSocket: 未连接"

**解决方法**:
1. 确保 Python 服务器已启动
2. 检查防火墙是否允许 8445 和 8765 端口

### 证书警告

**现象**: 浏览器提示"您的连接不是私密连接"

**解决方法**: 这是正常现象（自签名证书），点击"高级"→"继续前往"

## 🌐 端口说明

| 端口 | 用途 | 协议 |
|------|------|------|
| 8445 | 文件服务器 | HTTPS |
| 8765 | 视频流传输 | WSS (WebSocket over SSL) |

## 📖 使用提示

1. **推荐USB有线**: 延迟更低，带宽更稳定
2. **远程访问**: 必须通过 HTTPS 访问才能使用 WebXR
3. **多客户端**: 支持多个设备同时连接

## 🔐 系统架构

```
USB有线模式:
┌─────────────┐  ADB reverse  ┌────────────────────┐
│ VR浏览器     │ ──────────── │ PC服务器            │
│ 127.0.0.1   │    USB线      │ HTTPS:8445         │
└─────────────┘               │ WSS:8765           │
                              └────────────────────┘

WiFi无线模式:
┌─────────────┐    WiFi      ┌────────────────────┐
│ VR浏览器     │ ──────────── │ PC服务器            │
│ 192.168.x.x │   局域网      │ HTTPS:8445         │
└─────────────┘               │ WSS:8765           │
                              └────────────────────┘
```

## 👨‍💻 作者

**Liang ZHU** - lzhu686@connect.hkust-gz.edu.cn

## 📄 许可证

MIT License
