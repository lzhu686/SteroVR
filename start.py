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
import subprocess
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler

# 项目根目录
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ROOT_DIR)

HTTPS_PORT = 8445
WSS_PORT = 8765


def get_local_ip():
    """获取本机局域网IP地址"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "localhost"


def setup_adb_reverse():
    """检测ADB设备并设置端口转发，返回(成功, 设备ID)"""
    try:
        result = subprocess.run(['adb', 'devices'], capture_output=True, text=True, timeout=5)
        lines = result.stdout.strip().split('\n')[1:]  # 跳过标题行
        devices = [l.split('\t')[0] for l in lines if '\tdevice' in l]

        if not devices:
            return False, None

        # 为第一个设备设置端口转发
        device = devices[0]
        for port in [HTTPS_PORT, WSS_PORT]:
            subprocess.run(['adb', '-s', device, 'reverse', f'tcp:{port}', f'tcp:{port}'],
                          capture_output=True, timeout=5)
        return True, device
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False, None


def start_https_server():
    """启动 HTTPS 文件服务器"""
    web_dir = os.path.join(ROOT_DIR, "web")
    cert_file = os.path.join(ROOT_DIR, "server.crt")
    key_file = os.path.join(ROOT_DIR, "server.key")

    # 生成证书
    if not os.path.exists(cert_file) or not os.path.exists(key_file):
        print("正在生成SSL证书...")
        subprocess.run([
            'openssl', 'req', '-x509', '-newkey', 'rsa:2048',
            '-keyout', key_file, '-out', cert_file,
            '-days', '365', '-nodes',
            '-subj', '/CN=localhost'
        ], capture_output=True)

    # 自定义 Handler，指定 web 目录为根目录（无需 chdir）
    Handler = lambda *args, **kwargs: SimpleHTTPRequestHandler(
        *args, directory=web_dir, **kwargs
    )

    ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ssl_context.load_cert_chain(cert_file, key_file)

    server = HTTPServer(('0.0.0.0', HTTPS_PORT), Handler)
    server.socket = ssl_context.wrap_socket(server.socket, server_side=True)
    server.serve_forever()


async def start_websocket_server():
    """启动 WSS WebSocket 服务器"""
    from teleopVision.server import USBStereoWebSocketServerSSL
    server = USBStereoWebSocketServerSSL(host="0.0.0.0", port=WSS_PORT, use_ssl=True)
    try:
        await server.start_server()
    except KeyboardInterrupt:
        pass
    finally:
        server.cleanup()


def main():
    """主函数 - 同时启动两个服务器"""
    print("\n" + "=" * 60)
    print("🚀 立体视觉服务器启动中...")
    print("=" * 60)

    # 检测并设置ADB端口转发
    adb_ok, device_id = setup_adb_reverse()

    # 启动HTTPS服务器
    threading.Thread(target=start_https_server, daemon=True).start()

    # 显示访问信息
    local_ip = get_local_ip()
    print(f"\n📡 服务端口: HTTPS:{HTTPS_PORT} | WSS:{WSS_PORT}")

    if adb_ok:
        print(f"\n✅ USB设备已连接: {device_id}")
        print(f"   🔌 有线访问: https://127.0.0.1:{HTTPS_PORT}")
    else:
        print("\nℹ️  未检测到USB设备，使用WiFi连接")

    print(f"   📶 WiFi访问: https://{local_ip}:{HTTPS_PORT}")

    print(f"""
⚠️  首次使用请信任两个端口的证书:
   1. 主页面: https://127.0.0.1:{HTTPS_PORT} (或WiFi IP)
   2. 视频流: https://127.0.0.1:{WSS_PORT} (或WiFi IP)

   每个端口都需要点击"高级"→"继续前往"
{"=" * 60}
""")

    asyncio.run(start_websocket_server())


if __name__ == "__main__":
    main()
