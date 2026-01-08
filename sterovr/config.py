#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StereoVR 核心配置模块

集中管理所有配置参数，便于不同模块复用。

支持的相机型号:
- HBVCAM-F2439GS-2 V11 (AR0234 CMOS, 60mm 基线)
- 其他 USB 双目相机

作者: Liang ZHU
邮箱: lzhu686@connect.hkust-gz.edu.cn
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Dict, Any
import json


class CameraModel(Enum):
    """支持的相机型号"""
    HBVCAM_F2439GS = "HBVCAM-F2439GS-2"    # AR0234 传感器
    GENERIC_STEREO = "GENERIC"              # 通用双目相机


class StreamProtocol(Enum):
    """流传输协议"""
    WEBSOCKET = "websocket"      # WebSocket (Base64 JPEG) - WebXR 兼容
    UDP_RTP = "udp_rtp"          # UDP RTP H.264 - XRoboToolkit 兼容


@dataclass
class CameraConfig:
    """
    相机配置

    根据 HBVCAM-F2439GS-2 V11 规格优化:
    - 传感器: AR0234 (1/2.6" CMOS)
    - 单目 200万像素，双目 400万像素
    - 基线: 60mm
    - 视场角: 85°(无畸变) 或 125°(无畸变)
    """

    # === 分辨率配置 ===
    # 推荐使用 MJPG 格式以获得最佳性能
    # MJPG 2560x720 @ 60FPS 是最佳平衡点

    stereo_width: int = 2560        # 双目拼接宽度 (左+右)
    stereo_height: int = 720        # 高度
    fps: int = 60                   # 帧率

    # === 物理参数 ===
    baseline_mm: float = 60.0       # 基线距离 (毫米)
    fov_degrees: float = 125.0      # 视场角 (度)
    focal_length_mm: float = 2.96   # 焦距 (毫米, 125°镜头)

    # === 设备配置 ===
    device_id: int = 0              # 相机设备 ID
    use_mjpg: bool = True           # 使用 MJPG 格式 (推荐)
    buffer_size: int = 1            # 缓冲区大小 (最小化延迟)

    # === 自动调整 ===
    auto_exposure: bool = False     # 自动曝光 (关闭可减少延迟)
    auto_white_balance: bool = True # 自动白平衡

    @property
    def single_eye_width(self) -> int:
        """单眼宽度"""
        return self.stereo_width // 2

    @property
    def aspect_ratio(self) -> float:
        """宽高比"""
        return self.single_eye_width / self.stereo_height

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典"""
        return {
            'stereo_width': self.stereo_width,
            'stereo_height': self.stereo_height,
            'fps': self.fps,
            'baseline_mm': self.baseline_mm,
            'fov_degrees': self.fov_degrees,
            'single_eye_width': self.single_eye_width,
            'aspect_ratio': round(self.aspect_ratio, 4)
        }

    @classmethod
    def preset_low_latency(cls) -> 'CameraConfig':
        """
        低延迟预设 (遥操作推荐)

        - 较低分辨率减少编码/传输时间
        - 60fps 保持流畅
        """
        return cls(
            stereo_width=1920,
            stereo_height=540,
            fps=60,
            buffer_size=1,
            auto_exposure=False
        )

    @classmethod
    def preset_high_quality(cls) -> 'CameraConfig':
        """
        高画质预设

        - 最高分辨率
        - 适合带宽充足的场景
        """
        return cls(
            stereo_width=2560,
            stereo_height=720,
            fps=60
        )

    @classmethod
    def preset_balanced(cls) -> 'CameraConfig':
        """
        平衡预设 (默认)

        - 2560x720 @ 60fps
        - 兼顾画质和延迟
        """
        return cls()


@dataclass
class EncoderConfig:
    """
    编码器配置

    针对遥操作场景优化:
    - 超低延迟编码预设
    - 较高码率保证画质
    - 短 GOP 便于快速恢复
    """

    # === 编码参数 ===
    codec: str = "h264"             # 编码器 (h264/h265)
    bitrate: int = 8_000_000        # 码率 (8 Mbps)
    preset: str = "ultrafast"       # 编码预设 (ultrafast 最低延迟)
    tune: str = "zerolatency"       # 调优模式 (zerolatency 禁用 B 帧)
    keyframe_interval: int = 30     # 关键帧间隔 (GOP 大小)

    # === 延迟优化 ===
    # 关键帧间隔越小，丢包恢复越快，但码率开销越大
    # 遥操作场景建议 15-30

    @property
    def bitrate_mbps(self) -> float:
        """码率 (Mbps)"""
        return self.bitrate / 1_000_000

    @classmethod
    def preset_low_latency(cls) -> 'EncoderConfig':
        """
        低延迟预设 (遥操作推荐)

        - 较短 GOP (15帧)
        - 较高码率保证画质
        """
        return cls(
            bitrate=10_000_000,       # 10 Mbps
            keyframe_interval=15,     # 每 0.25 秒一个关键帧
            preset="ultrafast",
            tune="zerolatency"
        )

    @classmethod
    def preset_bandwidth_limited(cls) -> 'EncoderConfig':
        """
        带宽受限预设

        - 较低码率
        - 较长 GOP 节省带宽
        """
        return cls(
            bitrate=4_000_000,        # 4 Mbps
            keyframe_interval=60,     # 每秒一个关键帧
            preset="ultrafast",
            tune="zerolatency"
        )


@dataclass
class NetworkConfig:
    """网络配置"""

    # === WebSocket 模式 (WebXR) ===
    ws_port: int = 8765             # WebSocket 端口
    https_port: int = 8445          # HTTPS 端口
    use_ssl: bool = True            # 使用 SSL

    # === UDP RTP 模式 (XRoboToolkit) ===
    tcp_control_port: int = 63901   # TCP 控制端口
    udp_video_port: int = 12345     # UDP 视频端口

    # === JPEG 质量 (WebSocket 模式) ===
    jpeg_quality: int = 85          # JPEG 压缩质量 (1-100)

    @classmethod
    def preset_low_latency(cls) -> 'NetworkConfig':
        """低延迟预设"""
        return cls(
            jpeg_quality=75  # 较低质量，更快编码
        )


@dataclass
class StereoVRConfig:
    """
    StereoVR 完整配置

    使用示例:
        # 默认配置
        config = StereoVRConfig()

        # 低延迟配置 (遥操作)
        config = StereoVRConfig.for_teleoperation()

        # 自定义配置
        config = StereoVRConfig(
            camera=CameraConfig(stereo_width=1920, stereo_height=540),
            encoder=EncoderConfig(bitrate=6_000_000)
        )
    """

    camera: CameraConfig = field(default_factory=CameraConfig)
    encoder: EncoderConfig = field(default_factory=EncoderConfig)
    network: NetworkConfig = field(default_factory=NetworkConfig)
    protocol: StreamProtocol = StreamProtocol.UDP_RTP

    @classmethod
    def for_teleoperation(cls) -> 'StereoVRConfig':
        """
        遥操作场景配置

        优化目标: 最低延迟
        - 低延迟相机配置
        - 低延迟编码配置
        - UDP RTP 协议
        """
        return cls(
            camera=CameraConfig.preset_low_latency(),
            encoder=EncoderConfig.preset_low_latency(),
            network=NetworkConfig.preset_low_latency(),
            protocol=StreamProtocol.UDP_RTP
        )

    @classmethod
    def for_webxr(cls) -> 'StereoVRConfig':
        """
        WebXR 场景配置

        用于通过浏览器访问
        """
        return cls(
            camera=CameraConfig.preset_balanced(),
            encoder=EncoderConfig(),
            network=NetworkConfig(),
            protocol=StreamProtocol.WEBSOCKET
        )

    def to_json(self) -> str:
        """导出为 JSON"""
        return json.dumps({
            'camera': self.camera.to_dict(),
            'encoder': {
                'codec': self.encoder.codec,
                'bitrate': self.encoder.bitrate,
                'preset': self.encoder.preset,
                'keyframe_interval': self.encoder.keyframe_interval
            },
            'network': {
                'ws_port': self.network.ws_port,
                'tcp_control_port': self.network.tcp_control_port,
                'jpeg_quality': self.network.jpeg_quality
            },
            'protocol': self.protocol.value
        }, indent=2)


# === 预定义配置 ===

# 默认配置 (平衡)
DEFAULT_CONFIG = StereoVRConfig()

# 遥操作配置 (低延迟)
TELEOP_CONFIG = StereoVRConfig.for_teleoperation()

# WebXR 配置
WEBXR_CONFIG = StereoVRConfig.for_webxr()


if __name__ == '__main__':
    # 测试配置
    print("=== StereoVR 配置测试 ===\n")

    print("默认配置:")
    print(DEFAULT_CONFIG.to_json())

    print("\n遥操作配置 (低延迟):")
    print(TELEOP_CONFIG.to_json())

    print("\nWebXR 配置:")
    print(WEBXR_CONFIG.to_json())
