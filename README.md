# RGB125 双目立体视觉 VR 系统

基于 USB 双目相机的实时立体视觉 VR 透视系统，支持 WebXR 和 Quest 3 设备。

## 系统架构

```
┌──────────────┐      ┌────────────────────┐      ┌─────────────┐
│ USB双目相机   │  →   │ Python WebSocket   │  →   │ VR浏览器    │
│ (2560x720)   │      │ 服务器 (WSS:8765)  │      │ (WebXR)     │
└──────────────┘      └────────────────────┘      └─────────────┘
       │                       │                        │
  OpenCV采集             Base64编码              Three.js渲染
  分割左右图像            JSON传输               WebXR立体显示
```

## 文件说明

| 文件 | 说明 |
|------|------|
| `dual_infrared_vr_viewer.html` | VR 前端页面，支持 WebXR 立体显示 |
| `usb_stereo_websocket_server.py` | WebSocket 服务器 (本地 ws://) |
| `usb_stereo_websocket_server_ssl.py` | WebSocket 服务器 (远程 wss://) |
| `start_stereo_server.py` | 统一启动脚本 (HTTPS + WSS) |

## 快速开始

### 依赖安装

```bash
pip install opencv-python websockets numpy
```

### 本地使用

```bash
# 启动 WebSocket 服务器
python usb_stereo_websocket_server.py

# 双击打开 dual_infrared_vr_viewer.html
```

### 远程设备访问 (Quest 3 / 手机)

```bash
# 启动统一服务器 (HTTPS:8445 + WSS:8765)
python start_stereo_server.py
```

然后在 VR 设备浏览器中访问：
```
https://你的电脑IP:8445/RGB125/dual_infrared_vr_viewer.html
```

## 配置参数

在 `usb_stereo_websocket_server.py` 或 `usb_stereo_websocket_server_ssl.py` 文件顶部修改：

```python
STEREO_WIDTH = 2560     # 双目拼接图像宽度
STEREO_HEIGHT = 720     # 双目拼接图像高度
CAMERA_WIDTH = 1280     # 单目图像宽度
CAMERA_HEIGHT = 720     # 单目图像高度
TARGET_FPS = 60         # 目标帧率
JPEG_QUALITY = 100      # JPEG压缩质量 (1-100)
```

## 网络访问说明

| 访问方式 | 协议 | 说明 |
|----------|------|------|
| 本地双击HTML | `file://` + `ws://` | 直接可用 |
| 远程设备访问 | `https://` + `wss://` | 需要 SSL 证书 |

远程访问时，浏览器会提示证书不安全（自签名证书），选择"继续前往"即可。

## 端口说明

| 端口 | 用途 |
|------|------|
| 8445 | HTTPS 文件服务器 |
| 8765 | WSS WebSocket 服务器 |

## 技术特性

- 125° 对角线视场角 (DFOV)
- 60mm 基线距离
- 实时 JPEG 压缩传输
- 自适应质量调整
- WebXR 立体渲染
- 支持多客户端连接

## 许可证

MIT License
