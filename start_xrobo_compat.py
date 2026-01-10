#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StereoVR 统一启动脚本
一键启动 XRoboToolkit 兼容服务器

功能:
1. 检测系统依赖 (FFmpeg, OpenCV)
2. 检测相机设备
3. 启动 XRoboToolkit 兼容服务器
4. 显示连接信息

使用方法:
    # 基本启动
    python start_xrobo_compat.py

    # 指定相机设备
    python start_xrobo_compat.py --device 1

    # 自定义分辨率
    python start_xrobo_compat.py --width 1920 --height 540 --fps 30

作者: Liang ZHU
邮箱: lzhu686@connect.hkust-gz.edu.cn
日期: 2025
"""

import sys
import os
import socket
import subprocess
import argparse
import time

# 添加当前目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def get_local_ip() -> str:
    """获取本机局域网 IP"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"


def check_ffmpeg() -> bool:
    """检查 FFmpeg 是否安装"""
    try:
        result = subprocess.run(
            ['ffmpeg', '-version'],
            capture_output=True,
            timeout=5
        )
        return result.returncode == 0
    except:
        return False


def check_opencv() -> bool:
    """检查 OpenCV 是否安装"""
    try:
        import cv2
        return True
    except ImportError:
        return False


def list_cameras() -> list:
    """列出可用的相机设备"""
    import cv2
    cameras = []
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                cameras.append({
                    'id': i,
                    'resolution': f'{w}x{h}'
                })
            cap.release()
    return cameras


def print_banner():
    """打印横幅"""
    banner = """
╔═══════════════════════════════════════════════════════════════════╗
║                                                                   ║
║   ███████╗████████╗███████╗██████╗  ██████╗ ██╗   ██╗██████╗      ║
║   ██╔════╝╚══██╔══╝██╔════╝██╔══██╗██╔═══██╗██║   ██║██╔══██╗     ║
║   ███████╗   ██║   █████╗  ██████╔╝██║   ██║██║   ██║██████╔╝     ║
║   ╚════██║   ██║   ██╔══╝  ██╔══██╗██║   ██║╚██╗ ██╔╝██╔══██╗     ║
║   ███████║   ██║   ███████╗██║  ██║╚██████╔╝ ╚████╔╝ ██║  ██║     ║
║   ╚══════╝   ╚═╝   ╚══════╝╚═╝  ╚═╝ ╚═════╝   ╚═══╝  ╚═╝  ╚═╝     ║
║                                                                   ║
║         XRoboToolkit Compatible Video Streaming Server            ║
║                                                                   ║
╚═══════════════════════════════════════════════════════════════════╝
"""
    print(banner)


def print_status(text: str, ok: bool):
    """打印状态"""
    status = "✓" if ok else "✗"
    color = "\033[92m" if ok else "\033[91m"
    reset = "\033[0m"
    print(f"  {color}[{status}]{reset} {text}")


def main():
    parser = argparse.ArgumentParser(
        description='StereoVR XRoboToolkit 兼容服务器启动脚本'
    )
    parser.add_argument('--device', '-d', type=int, default=0,
                        help='相机设备 ID (默认: 0)')
    parser.add_argument('--width', '-W', type=int, default=2560,
                        help='视频宽度 (默认: 2560)')
    parser.add_argument('--height', '-H', type=int, default=720,
                        help='视频高度 (默认: 720)')
    parser.add_argument('--fps', '-f', type=int, default=60,
                        help='帧率 (默认: 60)')
    parser.add_argument('--bitrate', '-b', type=int, default=8000000,
                        help='码率 (默认: 8000000)')
    parser.add_argument('--check', action='store_true',
                        help='仅检查依赖，不启动服务器')
    parser.add_argument('--list-cameras', action='store_true',
                        help='列出可用相机')

    args = parser.parse_args()

    print_banner()

    # 检查依赖
    print("\n📋 检查系统依赖...\n")

    ffmpeg_ok = check_ffmpeg()
    print_status("FFmpeg", ffmpeg_ok)

    opencv_ok = check_opencv()
    print_status("OpenCV (cv2)", opencv_ok)

    if not ffmpeg_ok:
        print("\n⚠️  FFmpeg 未安装!")
        print("   安装方法:")
        print("   - Ubuntu: sudo apt-get install ffmpeg")
        print("   - Windows: https://ffmpeg.org/download.html")
        print("   - macOS: brew install ffmpeg")

    if not opencv_ok:
        print("\n⚠️  OpenCV 未安装!")
        print("   安装方法: pip install opencv-python")

    if not (ffmpeg_ok and opencv_ok):
        print("\n❌ 依赖检查失败，请先安装缺失的依赖")
        sys.exit(1)

    print("\n✅ 所有依赖已满足\n")

    # 列出相机
    if args.list_cameras:
        print("📷 检测可用相机...\n")
        cameras = list_cameras()
        if cameras:
            for cam in cameras:
                print(f"   设备 {cam['id']}: {cam['resolution']}")
        else:
            print("   未检测到相机设备")
        print()
        sys.exit(0)

    if args.check:
        sys.exit(0)

    # 获取网络信息
    local_ip = get_local_ip()

    print("=" * 60)
    print("Network Info")
    print("=" * 60)
    print(f"   Local IP: {local_ip}")
    print(f"   TCP Port: 13579 (control)")
    print(f"   Camera Device: {args.device}")
    print(f"   Resolution: {args.width}x{args.height}")
    print(f"   FPS: {args.fps}")
    print(f"   Bitrate: {args.bitrate // 1000000} Mbps")
    print("=" * 60)
    print()

    print("PICO Headset Steps:")
    print("-" * 40)
    print("   1. Open XRoboToolkit Unity Client")
    print("   2. Select video source: USB_STEREO")
    print(f"   3. Enter PC IP: {local_ip}")
    print("   4. Click Listen button")
    print("-" * 40)
    print()

    # Start server
    print("Starting server...")
    print()

    try:
        from teleopVision.xrobo_compat_server import XRoboCompatServer
        server = XRoboCompatServer(device_id=args.device)
        server.start()
    except KeyboardInterrupt:
        print("\n\n👋 收到中断信号，正在停止...")
    except Exception as e:
        print(f"\n❌ 服务器错误: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
