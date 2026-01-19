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


def check_v4l2loopback(device_path: str) -> bool:
    """检查 V4L2 Loopback 设备是否可用"""
    if sys.platform == 'win32':
        return False  # Windows 不支持 v4l2loopback

    import os
    # 检查设备文件是否存在
    if not os.path.exists(device_path):
        return False

    # 尝试读取设备信息
    try:
        result = subprocess.run(
            ['v4l2-ctl', '--device', device_path, '--all'],
            capture_output=True,
            timeout=5
        )
        # 检查是否包含 loopback 相关信息
        output = result.stdout.decode('utf-8', errors='ignore')
        return 'loopback' in output.lower() or result.returncode == 0
    except:
        # v4l2-ctl 未安装或执行失败，仍然允许继续
        return os.path.exists(device_path)


def list_cameras_legacy() -> list:
    """列出可用的相机设备 (旧版兼容)"""
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


def select_camera_interactive():
    """交互式选择相机"""
    from teleopVision.camera_utils import list_cameras, select_camera_interactive as _select
    return _select()


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
        description='StereoVR XRoboToolkit 兼容服务器启动脚本',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python start_xrobo_compat.py                      # 使用默认设备 0
  python start_xrobo_compat.py --select             # 交互式选择相机
  python start_xrobo_compat.py --device 2           # 使用设备索引 2
  python start_xrobo_compat.py --device /dev/stereo_camera  # 使用 udev 符号链接 (推荐)
  python start_xrobo_compat.py --list-cameras       # 列出所有相机

V4L2 Loopback 双进程架构 (推荐用于 ROS2 发布):
  # 1. 启动主服务器 (输出到 loopback)
  python start_xrobo_compat.py --device /dev/stereo_camera --loopback /dev/video99 --loopback-fps 30

  # 2. 在另一个终端启动 ROS2 发布
  python -m teleopVision.ros2_loopback_publisher --device /dev/video99 --fps 30

架构优势:
  - PICO 视频流 (60fps 高优先级) 与 ROS2 发布 (30fps) 完全隔离
  - ROS2 进程崩溃不影响 PICO 实时视频流
  - 独立资源分配，互不影响延迟

UDEV 规则设置 (创建固定设备链接):
  sudo tee /etc/udev/rules.d/99-stereo-camera.rules << 'EOF'
  SUBSYSTEM=="video4linux", ATTRS{idVendor}=="1bcf", ATTRS{idProduct}=="2d4f", ATTR{index}=="0", SYMLINK+="stereo_camera", MODE="0666"
  EOF
  sudo udevadm control --reload-rules && sudo udevadm trigger
"""
    )
    parser.add_argument('--device', '-d', type=str, default='0',
                        help='相机设备 (索引如 0, 2 或路径如 /dev/stereo_camera)')
    parser.add_argument('--select', '-s', action='store_true',
                        help='交互式选择相机')
    parser.add_argument('--width', '-W', type=int, default=2560,
                        help='视频宽度 (默认: 2560)')
    parser.add_argument('--height', '-H', type=int, default=720,
                        help='视频高度 (默认: 720)')
    parser.add_argument('--fps', '-f', type=int, default=60,
                        help='帧率 (默认: 60)')
    parser.add_argument('--bitrate', '-b', type=int, default=8000000,
                        help='码率 (默认: 8000000)')
    parser.add_argument('--loopback', type=str, default=None,
                        help='V4L2 Loopback 设备路径 (如 /dev/video99)，用于独立 ROS2 发布')
    parser.add_argument('--loopback-fps', type=int, default=30,
                        help='Loopback 输出帧率 (默认: 30)')
    parser.add_argument('--check', action='store_true',
                        help='仅检查依赖，不启动服务器')
    parser.add_argument('--list-cameras', '-l', action='store_true',
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

    # V4L2 Loopback 检查 (仅当使用 loopback 模式时)
    if args.loopback and sys.platform != 'win32':
        print("\n🔧 检查 V4L2 Loopback...")
        loopback_ok = check_v4l2loopback(args.loopback)
        print_status(f"V4L2 Loopback ({args.loopback})", loopback_ok)

        if not loopback_ok:
            print(f"\n⚠️  V4L2 Loopback 设备 {args.loopback} 不可用!")
            print("   请先加载 v4l2loopback 模块:")
            print(f"   sudo modprobe v4l2loopback video_nr=99 card_label=\"StereoVR_ROS2\"")
            print()
            # 不退出，允许用户继续尝试

    print("\n✅ 所有依赖已满足\n")

    # 列出相机
    if args.list_cameras:
        print("📷 检测可用相机...\n")
        from teleopVision.camera_utils import list_cameras
        cameras = list_cameras()
        if cameras:
            for cam in cameras:
                print(f"   {cam}")
        else:
            print("   未检测到相机设备")
        print()
        sys.exit(0)

    if args.check:
        sys.exit(0)

    # 交互式选择相机
    selected_device = args.device
    if args.select:
        cam = select_camera_interactive()
        if cam is None:
            print("\n❌ 未选择相机，退出")
            sys.exit(1)
        selected_device = cam.device_path
        print(f"\n✅ 已选择: {cam.name}")
        print(f"   设备路径: {cam.device_path}\n")
    else:
        # 解析设备参数 (支持数字或路径)
        if selected_device.isdigit():
            selected_device = int(selected_device)

    # 获取网络信息
    local_ip = get_local_ip()

    print("=" * 60)
    print("Network Info")
    print("=" * 60)
    print(f"   Local IP: {local_ip}")
    print(f"   TCP Port: 13579 (control)")
    print(f"   Camera Device: {selected_device}")
    print(f"   Resolution: {args.width}x{args.height}")
    print(f"   FPS: {args.fps}")
    print(f"   Bitrate: {args.bitrate // 1000000} Mbps")
    if args.loopback:
        print(f"\n   V4L2 Loopback: {args.loopback}")
        print(f"   Loopback FPS: {args.loopback_fps}")
        print(f"   架构: 双进程模式 (PICO + ROS2 独立)")
    print("=" * 60)
    print()

    print("PICO Headset Steps:")
    print("-" * 40)
    print("   1. Open XRoboToolkit Unity Client")
    print("   2. Select video source: USB_STEREO")
    print(f"   3. Enter PC IP: {local_ip}")
    print("   4. Click Listen button")
    print("-" * 40)

    if args.loopback:
        print()
        print("ROS2 Publishing Steps (独立进程):")
        print("-" * 40)
        print("   在另一个终端运行:")
        print(f"   python -m teleopVision.ros2_loopback_publisher --device {args.loopback} --fps {args.loopback_fps}")
        print()
        print("   验证 ROS2 话题:")
        print("   ros2 topic list | grep stereo")
        print("   ros2 topic hz /stereo/left/compressed")
        print("-" * 40)
    print()

    # Start server
    print("Starting server...")
    print()

    try:
        from teleopVision.xrobo_compat_server import XRoboCompatServer

        # 创建服务器配置
        # 注意: XRoboCompatServer 只接受 device_id 和 loopback 参数
        # width, height, fps, bitrate 由 PICO 客户端命令传递
        server_config = {
            'device_id': selected_device,
        }

        # 添加 loopback 配置 (V4L2 Loopback 双进程架构)
        if args.loopback:
            server_config['loopback_device'] = args.loopback
            server_config['loopback_fps'] = args.loopback_fps
            print(f"🔄 V4L2 Loopback 双进程架构已启用:")
            print(f"   Loopback 设备: {args.loopback}")
            print(f"   Loopback 帧率: {args.loopback_fps} fps")
            print(f"   数据流向:")
            print(f"     Camera → FFmpeg → H.264 → PICO ({args.fps}fps)")
            print(f"                     → MJPEG → {args.loopback} ({args.loopback_fps}fps)")
            print()

        server = XRoboCompatServer(**server_config)
        server.start()
    except KeyboardInterrupt:
        print("\n\n👋 收到中断信号，正在停止...")
    except Exception as e:
        print(f"\n❌ 服务器错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == '__main__':
    main()
