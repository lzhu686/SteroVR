#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
立体视觉服务器启动脚本
同时启动 HTTPS 文件服务器 + WSS WebSocket 服务器

使用方法:
    python start_stereo_server.py

其他设备访问:
    1. 浏览器打开: https://你的IP:8445/RGB125/dual_infrared_vr_viewer.html
    2. 信任自签名证书
    3. 页面会自动连接 wss://你的IP:8765
"""

import asyncio
import ssl
import os
import sys
import socket
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler


def get_local_ip():
    """获取本机局域网IP地址"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()
        return local_ip
    except:
        return "localhost"


def start_https_server(port=8445):
    """启动 HTTPS 文件服务器"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    stereo_vision_dir = os.path.dirname(script_dir)  # StereoVision 目录
    project_root = os.path.dirname(stereo_vision_dir)  # 项目根目录

    # 查找证书
    cert_file = os.path.join(project_root, "webxr_cert.pem")
    key_file = os.path.join(project_root, "webxr_key.pem")

    if not os.path.exists(cert_file) or not os.path.exists(key_file):
        print(f"❌ 找不到SSL证书: {cert_file}")
        print("请先运行 start.py 或手动生成证书")
        return

    # 切换到 StereoVision 目录
    os.chdir(stereo_vision_dir)

    # 创建SSL上下文
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(cert_file, key_file)

    # 创建HTTPS服务器
    server = HTTPServer(('0.0.0.0', port), SimpleHTTPRequestHandler)
    server.socket = ssl_context.wrap_socket(server.socket, server_side=True)

    local_ip = get_local_ip()

    print(f"🌐 HTTPS文件服务器启动在端口 {port}")
    print(f"📂 服务目录: {stereo_vision_dir}")
    print(f"🔗 本地访问: https://localhost:{port}/RGB125/dual_infrared_vr_viewer.html")
    print(f"🔗 局域网访问: https://{local_ip}:{port}/RGB125/dual_infrared_vr_viewer.html")

    server.serve_forever()


async def start_websocket_server():
    """启动 WSS WebSocket 服务器"""
    # 导入 SSL 版本的服务器
    from usb_stereo_websocket_server_ssl import USBStereoWebSocketServerSSL

    server = USBStereoWebSocketServerSSL(
        host="0.0.0.0",
        port=8765,
        use_ssl=True
    )

    try:
        await server.start_server()
    except KeyboardInterrupt:
        pass
    finally:
        server.cleanup()


def main():
    """主函数 - 同时启动两个服务器"""
    local_ip = get_local_ip()

    print("\n" + "=" * 70)
    print("🚀 立体视觉服务器启动中...")
    print("=" * 70)

    # 在后台线程中启动 HTTPS 服务器
    https_thread = threading.Thread(target=start_https_server, args=(8445,), daemon=True)
    https_thread.start()

    print("\n" + "=" * 70)
    print("📱 其他设备访问方法:")
    print("=" * 70)
    print(f"1. 在VR设备或手机浏览器打开:")
    print(f"   https://{local_ip}:8445/RGB125/dual_infrared_vr_viewer.html")
    print()
    print(f"2. 浏览器会提示证书不安全，选择'继续前往'或'信任此证书'")
    print()
    print(f"3. 页面会自动连接到 WSS 服务器: wss://{local_ip}:8765")
    print("=" * 70 + "\n")

    # 启动 WebSocket 服务器 (这会阻塞)
    asyncio.run(start_websocket_server())


if __name__ == "__main__":
    main()
