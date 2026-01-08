"""
StereoVR - USB 双目立体视觉 VR 流媒体库

支持两种传输模式:
1. WebSocket (WebXR) - 通过浏览器访问，兼容 Quest/PICO/Vision Pro
2. UDP RTP H.264 - 兼容 XRoboToolkit Unity Client，低延迟

快速开始:

    # WebSocket 模式 (WebXR)
    from sterovr import WebSocketServer
    server = WebSocketServer()
    server.start()

    # XRoboToolkit 兼容模式 (低延迟)
    from sterovr import XRoboServer
    server = XRoboServer()
    server.start()

作者: Liang ZHU
邮箱: lzhu686@connect.hkust-gz.edu.cn
"""

__version__ = "2.0.0"
__author__ = "Liang ZHU"

# 导出核心类
from .config import (
    CameraConfig,
    EncoderConfig,
    NetworkConfig,
    StereoVRConfig,
    StreamProtocol,
    DEFAULT_CONFIG,
    TELEOP_CONFIG,
    WEBXR_CONFIG
)

from .camera import StereoCamera, FrameData, create_camera

# 版本信息
def get_version() -> str:
    return __version__

# 便捷启动函数
def start_websocket_server(**kwargs):
    """启动 WebSocket 服务器 (WebXR 模式)"""
    from .server import USBStereoWebSocketServerSSL
    config = kwargs.get('config', WEBXR_CONFIG)
    server = USBStereoWebSocketServerSSL(
        port=config.network.ws_port,
        use_ssl=config.network.use_ssl
    )
    import asyncio
    asyncio.run(server.start_server())

def start_xrobo_server(**kwargs):
    """启动 XRoboToolkit 兼容服务器 (低延迟模式)"""
    from .xrobo_compat_server import XRoboCompatServer
    device_id = kwargs.get('device_id', 0)
    server = XRoboCompatServer(device_id=device_id)
    server.start()
