# RGB125 双目立体视觉 VR 系统

基于 USB 双目相机的实时立体视觉 VR 透视系统，支持 WebXR 标准，兼容多种 VR 设备。

## 支持的 VR 设备

| 设备 | 状态 |
|------|------|
| Meta Quest 3 / Quest Pro | ✅ 完全支持 |
| Meta Quest 2 | ✅ 支持 |
| PICO 4 / PICO Neo3 | ✅ 支持 |
| Apple Vision Pro | ✅ 支持 |
| HTC Vive / Vive Pro | ✅ 支持 |
| Valve Index | ✅ 支持 |
| 其他 WebXR 兼容设备 | ✅ 支持 |

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
| `usb_stereo_websocket_server_ssl.py` | WebSocket 服务器 (WSS 加密) |
| `start_stereo_server.py` | 统一启动脚本 (HTTPS + WSS) |

## 快速开始

### 依赖安装

```bash
pip install opencv-python websockets numpy
```

### 本地使用

```bash
# 启动服务器
python start_stereo_server.py

# 或单独启动 WebSocket 服务器
python usb_stereo_websocket_server_ssl.py
```

### 远程设备访问 (VR 设备 / 手机)

```bash
# 启动统一服务器 (HTTPS:8445 + WSS:8765)
python start_stereo_server.py
```

然后在 VR 设备浏览器中访问：
```
https://你的电脑IP:8445/dual_infrared_vr_viewer.html
```

## 故障排除

### VR 模式下只显示单目画面

**问题描述**: 进入 VR 模式后，左右眼显示相同的画面，没有立体效果。

**解决方法**:
1. 退出 VR 模式（按下 VR 头显上的菜单键）
2. 刷新浏览器页面
3. 重新点击"进入 VR"按钮

### WebSocket 连接失败

**问题描述**: 页面显示 "WebSocket: 未连接"

**解决方法**:
1. 确保 Python 服务器已启动
2. 检查防火墙是否允许 8765 端口
3. 确保 VR 设备与电脑在同一局域网

### 证书不受信任

**问题描述**: 浏览器提示"您的连接不是私密连接"

**解决方法**:
1. 点击"高级"或"显示详情"
2. 选择"继续前往"或"仍然访问"
3. 这是正常现象，自签名证书会触发此警告

## 配置参数

在 `usb_stereo_websocket_server_ssl.py` 文件顶部修改：

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
- SSL/TLS 加密传输

## 作者

**Liang ZHU**
Email: lzhu686@connect.hkust-gz.edu.cn

## 许可证

MIT License
