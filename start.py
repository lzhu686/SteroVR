#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
立体视觉服务器启动脚本
同时启动 HTTPS 文件服务器 + WSS WebSocket 服务器

使用方法:
    python start.py

其他设备访问:
    1. 浏览器打开: https://你的IP:8445
    2. 信任自签名证书
    3. 选择查看模式（2D或VR）

作者: Liang ZHU
邮箱: lzhu686@connect.hkust-gz.edu.cn
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

    # 查找证书 (当前目录)
    cert_file = os.path.join(script_dir, "server.crt")
    key_file = os.path.join(script_dir, "server.key")

    if not os.path.exists(cert_file) or not os.path.exists(key_file):
        print(f"❌ 找不到SSL证书: {cert_file}")
        print("正在自动生成证书...")
        import subprocess
        cmd = [
            'openssl', 'req', '-x509', '-newkey', 'rsa:4096',
            '-keyout', key_file, '-out', cert_file,
            '-days', '365', '-nodes',
            '-subj', '/C=CN/ST=Guangdong/L=Guangzhou/O=HKUST-GZ/CN=localhost'
        ]
        subprocess.run(cmd, capture_output=True)

    # 切换到当前目录
    os.chdir(script_dir)

    # 创建SSL上下文
    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(cert_file, key_file)

    # 创建HTTPS服务器
    server = HTTPServer(('0.0.0.0', port), SimpleHTTPRequestHandler)
    server.socket = ssl_context.wrap_socket(server.socket, server_side=True)

    local_ip = get_local_ip()

    print(f"🌐 HTTPS文件服务器启动在端口 {port}")
    print(f"📂 服务目录: {script_dir}")
    print(f"🔗 本地访问: https://localhost:{port}")
    print(f"🔗 局域网访问: https://{local_ip}:{port}")

    server.serve_forever()


async def start_websocket_server():
    """启动 WSS WebSocket 服务器"""
    # 导入服务器模块
    from server import USBStereoWebSocketServerSSL

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
    print(f"   https://{local_ip}:8445")
    print()
    print(f"2. 浏览器会提示证书不安全，选择'继续前往'或'信任此证书'")
    print()
    print(f"3. 进入导航页面，选择'进入VR模式'或'打开2D查看器'")
    print("=" * 70 + "\n")

    # 启动 WebSocket 服务器 (这会阻塞)
    asyncio.run(start_websocket_server())


if __name__ == "__main__":
    main()
