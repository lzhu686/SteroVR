#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
相机设备扫描与选择工具

支持:
- Linux: 扫描 /dev/video* 设备，获取 V4L2 设备信息
- Windows: 使用 FFmpeg 列出 DirectShow 设备

作者: Liang ZHU
邮箱: lzhu686@connect.hkust-gz.edu.cn
"""

import os
import sys
import subprocess
import re
import logging
from typing import List, Optional, Tuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass
class CameraDevice:
    """相机设备信息"""
    index: int              # 设备索引 (用于选择)
    device_path: str        # 设备路径 (Linux: /dev/video0, Windows: 设备名)
    name: str               # 设备名称
    is_capture: bool = True # 是否为采集设备 (排除 metadata 设备)

    def __str__(self):
        return f"[{self.index}] {self.device_path} - {self.name}"


def list_cameras_linux() -> List[CameraDevice]:
    """
    Linux: 扫描 /dev/video* 设备

    使用 v4l2-ctl 或直接读取 /sys/class/video4linux/ 获取设备信息
    """
    devices = []
    video_devices = []

    # 查找所有 /dev/video* 设备
    for i in range(20):  # 检查 video0 到 video19
        path = f"/dev/video{i}"
        if os.path.exists(path):
            video_devices.append((i, path))

    # 获取设备名称
    for idx, (dev_num, path) in enumerate(video_devices):
        name = "Unknown Camera"
        is_capture = True

        # 方法1: 从 /sys/class/video4linux/ 读取
        sys_path = f"/sys/class/video4linux/video{dev_num}/name"
        if os.path.exists(sys_path):
            try:
                with open(sys_path, 'r') as f:
                    name = f.read().strip()
            except:
                pass

        # 方法2: 使用 v4l2-ctl (如果可用)
        if name == "Unknown Camera":
            try:
                result = subprocess.run(
                    ['v4l2-ctl', '-d', path, '--info'],
                    capture_output=True, text=True, timeout=2
                )
                for line in result.stdout.split('\n'):
                    if 'Card type' in line:
                        name = line.split(':')[1].strip()
                        break
            except:
                pass

        # 检查是否为采集设备 (排除 metadata 设备)
        # metadata 设备名称通常包含 "Metadata" 或设备号为奇数
        if 'metadata' in name.lower():
            is_capture = False

        # 只添加采集设备
        if is_capture:
            devices.append(CameraDevice(
                index=len(devices),
                device_path=path,
                name=name,
                is_capture=is_capture
            ))

    return devices


def list_cameras_windows() -> List[CameraDevice]:
    """
    Windows: 使用 FFmpeg 列出 DirectShow 设备
    """
    devices = []

    try:
        result = subprocess.run(
            ['ffmpeg', '-hide_banner', '-list_devices', 'true', '-f', 'dshow', '-i', 'dummy'],
            capture_output=True, text=True, timeout=10
        )

        lines = result.stderr.split('\n')
        in_video_section = False

        for line in lines:
            if 'DirectShow video devices' in line:
                in_video_section = True
                continue
            if 'DirectShow audio devices' in line:
                break
            if in_video_section and '"' in line and 'Alternative name' not in line:
                match = re.search(r'"([^"]+)"', line)
                if match:
                    name = match.group(1)
                    devices.append(CameraDevice(
                        index=len(devices),
                        device_path=f"video={name}",
                        name=name,
                        is_capture=True
                    ))

    except Exception as e:
        logger.warning(f"获取 Windows 相机列表失败: {e}")

    return devices


def list_cameras() -> List[CameraDevice]:
    """
    跨平台相机扫描

    返回: 相机设备列表
    """
    if sys.platform == 'win32':
        return list_cameras_windows()
    else:
        return list_cameras_linux()


def select_camera_interactive(devices: Optional[List[CameraDevice]] = None) -> Optional[CameraDevice]:
    """
    交互式选择相机

    参数:
        devices: 相机列表 (如果为 None，会自动扫描)

    返回:
        选中的相机设备，或 None (用户取消)
    """
    if devices is None:
        devices = list_cameras()

    if not devices:
        print("\n未检测到任何相机设备!")
        print("请检查:")
        print("  1. 相机是否已连接")
        print("  2. 相机驱动是否已安装")
        if sys.platform != 'win32':
            print("  3. 用户是否有 /dev/video* 访问权限 (尝试: sudo usermod -aG video $USER)")
        return None

    print("\n" + "=" * 50)
    print("检测到以下相机设备:")
    print("=" * 50)

    for dev in devices:
        print(f"  {dev}")

    print("=" * 50)

    # 如果只有一个设备，询问是否直接使用
    if len(devices) == 1:
        choice = input(f"\n只检测到一个相机，是否使用 [{devices[0].name}]? (Y/n): ").strip().lower()
        if choice in ['', 'y', 'yes']:
            return devices[0]
        return None

    # 多个设备，让用户选择
    while True:
        try:
            choice = input(f"\n请选择相机 [0-{len(devices)-1}] (q 退出): ").strip()

            if choice.lower() == 'q':
                return None

            idx = int(choice)
            if 0 <= idx < len(devices):
                return devices[idx]
            else:
                print(f"无效选择，请输入 0-{len(devices)-1}")

        except ValueError:
            print("请输入数字")
        except KeyboardInterrupt:
            print("\n已取消")
            return None


def find_camera_by_name(name_pattern: str, devices: Optional[List[CameraDevice]] = None) -> Optional[CameraDevice]:
    """
    根据名称查找相机

    参数:
        name_pattern: 相机名称的部分匹配
        devices: 相机列表

    返回:
        匹配的相机设备
    """
    if devices is None:
        devices = list_cameras()

    name_lower = name_pattern.lower()
    for dev in devices:
        if name_lower in dev.name.lower():
            return dev

    return None


def get_device_path_or_index(device: CameraDevice) -> Tuple[str, int]:
    """
    获取设备路径和索引

    返回:
        (device_path, device_index)
        - Linux: ("/dev/video0", 0)
        - Windows: ("video=Camera Name", 0)
    """
    if sys.platform == 'win32':
        return device.device_path, device.index
    else:
        # 从 /dev/video{N} 提取数字
        match = re.search(r'/dev/video(\d+)', device.device_path)
        if match:
            return device.device_path, int(match.group(1))
        return device.device_path, device.index


# === 命令行工具 ===

def main():
    """命令行入口: 列出所有相机"""
    import argparse

    parser = argparse.ArgumentParser(description='相机设备扫描工具')
    parser.add_argument('--select', '-s', action='store_true', help='交互式选择相机')
    parser.add_argument('--find', '-f', type=str, help='根据名称查找相机')
    args = parser.parse_args()

    devices = list_cameras()

    if args.find:
        dev = find_camera_by_name(args.find, devices)
        if dev:
            print(f"找到相机: {dev}")
            print(f"设备路径: {dev.device_path}")
        else:
            print(f"未找到包含 '{args.find}' 的相机")
        return

    if args.select:
        dev = select_camera_interactive(devices)
        if dev:
            print(f"\n已选择: {dev}")
            print(f"设备路径: {dev.device_path}")
        return

    # 默认: 列出所有相机
    if not devices:
        print("未检测到任何相机设备")
        return

    print(f"\n检测到 {len(devices)} 个相机设备:\n")
    for dev in devices:
        print(f"  {dev}")


if __name__ == '__main__':
    main()
