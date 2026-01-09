#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
H.264 视频编码发送器模块
基于 StereoVR 的 USB 双目相机模块，输出 H.264 编码的 UDP 视频流
兼容 XRoboToolkit Unity Client 的 MediaDecoder

功能特性:
1. 复用 StereoVR 的相机采集逻辑
2. 使用 GStreamer 进行硬件加速 H.264 编码
3. 通过 UDP 发送 RTP 封装的视频流
4. 完全兼容 XRoboToolkit-Orin-Video-Sender 协议

依赖安装:
    # Ubuntu/Debian
    sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
    pip install opencv-python numpy

    # Windows (需要安装 GStreamer)
    # 下载: https://gstreamer.freedesktop.org/download/

作者: Liang ZHU
邮箱: lzhu686@connect.hkust-gz.edu.cn
日期: 2025
"""

import cv2
import numpy as np
import time
import threading
import logging
import subprocess
import socket
import struct
import os
import sys
from typing import Optional, Tuple, Callable
from dataclasses import dataclass
from enum import Enum

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class EncoderType(Enum):
    """编码器类型"""
    SOFTWARE = "software"      # x264 软件编码
    NVIDIA = "nvidia"          # NVENC 硬件编码
    VAAPI = "vaapi"            # Intel/AMD VAAPI
    AUTO = "auto"              # 自动检测


@dataclass
class VideoConfig:
    """
    视频配置 - 双臂灵巧手遥操作 (高帧率优先)

    相机: HBVCAM-F2439GS-2 V11 (AR0234 全局快门)
    场景: 双臂灵巧手精细操作，快速运动清晰度优先

    设计原则:
    1. 2560x720: 相机原生 MJPEG 60fps 分辨率
    2. 60fps: 手指快速移动时保持清晰，运动流畅
    3. 20Mbps: 高码率保证画质，每帧333Kbit
    4. GOP=1: 每帧独立，训练数据可用
    5. 低延迟: 编码负载小，CPU 跟得上
    """
    width: int = 2560            # 双目总宽度 (相机原生 MJPEG 2560x720@60fps)
    height: int = 720            # 720p - 相机原生60fps分辨率
    fps: int = 60                # 60fps - 灵巧手快速运动需要高帧率
    bitrate: int = 20971520      # 20Mbps - 高质量，每帧333Kbit (20 x 1024 x 1024)
    keyframe_interval: int = 1   # GOP=1 - 每帧独立

    @property
    def single_eye_width(self) -> int:
        """单眼宽度"""
        return self.width // 2


class StereoCamera:
    """
    立体相机采集类
    复用 StereoVR server.py 的相机采集逻辑
    """

    def __init__(self, config: VideoConfig, device_id: int = 0):
        self.config = config
        self.device_id = device_id
        self.cap: Optional[cv2.VideoCapture] = None
        self.is_running = False
        self.frame_lock = threading.Lock()
        self.latest_frame: Optional[np.ndarray] = None
        self.frame_id = 0
        self.capture_thread: Optional[threading.Thread] = None

        # 统计
        self.stats = {
            'frames_captured': 0,
            'fps_actual': 0.0,
            'last_frame_time': 0.0
        }

    def initialize(self) -> bool:
        """初始化相机"""
        logger.info(f"初始化 USB 双目相机 (设备: {self.device_id})...")

        try:
            self.cap = cv2.VideoCapture(self.device_id)

            # 设置 MJPG 格式 (减少 USB 带宽)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.config.fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 最小缓冲

            if not self.cap.isOpened():
                logger.error("无法打开相机设备")
                return False

            # 验证实际参数
            actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

            logger.info(f"相机配置成功: {actual_w}x{actual_h} @ {actual_fps:.1f}fps")

            # 测试读取
            ret, frame = self.cap.read()
            if not ret:
                logger.error("相机读取测试失败")
                return False

            logger.info("相机初始化成功")
            return True

        except Exception as e:
            logger.error(f"相机初始化异常: {e}")
            return False

    def start_capture(self):
        """启动采集线程"""
        if self.is_running:
            return

        self.is_running = True
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        logger.info("相机采集线程已启动")

    def _capture_loop(self):
        """采集循环"""
        frame_interval = 1.0 / self.config.fps
        frame_count = 0
        fps_start_time = time.time()

        while self.is_running and self.cap and self.cap.isOpened():
            loop_start = time.time()

            ret, frame = self.cap.read()
            if ret and frame is not None:
                with self.frame_lock:
                    self.latest_frame = frame
                    self.frame_id += 1

                self.stats['frames_captured'] += 1
                self.stats['last_frame_time'] = time.time()
                frame_count += 1

                # 每秒更新 FPS
                if frame_count >= 60:
                    elapsed = time.time() - fps_start_time
                    self.stats['fps_actual'] = frame_count / elapsed
                    frame_count = 0
                    fps_start_time = time.time()

            # 控制帧率
            elapsed = time.time() - loop_start
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)

    def get_frame(self) -> Tuple[Optional[np.ndarray], int]:
        """获取最新帧"""
        with self.frame_lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy(), self.frame_id
            return None, -1

    def stop(self):
        """停止采集"""
        self.is_running = False
        if self.capture_thread:
            self.capture_thread.join(timeout=2)
        if self.cap:
            self.cap.release()
        logger.info("相机已停止")


class H264Encoder:
    """
    H.264 编码器
    使用 GStreamer 进行编码
    """

    def __init__(self, config: VideoConfig, encoder_type: EncoderType = EncoderType.AUTO):
        self.config = config
        self.encoder_type = encoder_type
        self.process: Optional[subprocess.Popen] = None
        self.is_running = False

    def _detect_encoder(self) -> str:
        """检测可用的编码器"""
        if self.encoder_type == EncoderType.NVIDIA:
            return "nvh264enc"
        elif self.encoder_type == EncoderType.VAAPI:
            return "vaapih264enc"
        elif self.encoder_type == EncoderType.SOFTWARE:
            return "x264enc"
        else:
            # 自动检测
            try:
                # 检查 NVENC
                result = subprocess.run(['gst-inspect-1.0', 'nvh264enc'],
                                       capture_output=True, timeout=5)
                if result.returncode == 0:
                    logger.info("检测到 NVIDIA 硬件编码器")
                    return "nvh264enc"
            except:
                pass

            try:
                # 检查 VAAPI
                result = subprocess.run(['gst-inspect-1.0', 'vaapih264enc'],
                                       capture_output=True, timeout=5)
                if result.returncode == 0:
                    logger.info("检测到 VAAPI 硬件编码器")
                    return "vaapih264enc"
            except:
                pass

            logger.info("使用软件编码器 x264")
            return "x264enc"

    def _build_pipeline(self, target_ip: str, target_port: int) -> str:
        """构建 GStreamer pipeline"""
        encoder = self._detect_encoder()

        # 基础输入
        pipeline = (
            f"appsrc name=src format=time is-live=true do-timestamp=true "
            f"caps=video/x-raw,format=BGR,width={self.config.width},"
            f"height={self.config.height},framerate={self.config.fps}/1 ! "
            f"videoconvert ! video/x-raw,format=I420 ! "
        )

        # 编码器配置
        bitrate_kbps = self.config.bitrate // 1000

        if encoder == "nvh264enc":
            pipeline += (
                f"nvh264enc preset=low-latency-hq bitrate={bitrate_kbps} "
                f"gop-size={self.config.keyframe_interval} ! "
            )
        elif encoder == "vaapih264enc":
            pipeline += (
                f"vaapih264enc rate-control=cbr bitrate={bitrate_kbps} "
                f"keyframe-period={self.config.keyframe_interval} ! "
            )
        else:
            pipeline += (
                f"x264enc tune=zerolatency speed-preset=ultrafast "
                f"bitrate={bitrate_kbps} key-int-max={self.config.keyframe_interval} ! "
            )

        # RTP 封装和 UDP 输出
        pipeline += (
            f"h264parse config-interval=1 ! "
            f"rtph264pay pt=96 config-interval=1 ! "
            f"udpsink host={target_ip} port={target_port} sync=false"
        )

        return pipeline

    def start(self, target_ip: str, target_port: int) -> bool:
        """启动编码器"""
        try:
            pipeline = self._build_pipeline(target_ip, target_port)
            logger.info(f"GStreamer Pipeline: {pipeline}")

            # 启动 GStreamer 进程
            self.process = subprocess.Popen(
                ['gst-launch-1.0', '-e'] + pipeline.split(),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            self.is_running = True
            logger.info(f"编码器已启动，输出到 {target_ip}:{target_port}")
            return True

        except Exception as e:
            logger.error(f"启动编码器失败: {e}")
            return False

    def encode_frame(self, frame: np.ndarray) -> bool:
        """编码一帧"""
        if not self.is_running or not self.process:
            return False

        try:
            # 写入帧数据到 stdin
            self.process.stdin.write(frame.tobytes())
            self.process.stdin.flush()
            return True
        except Exception as e:
            logger.error(f"编码帧失败: {e}")
            return False

    def stop(self):
        """停止编码器"""
        self.is_running = False
        if self.process:
            self.process.stdin.close()
            self.process.terminate()
            self.process.wait(timeout=5)
        logger.info("编码器已停止")


class H264VideoSender:
    """
    H.264 视频发送器 (主类)
    整合相机采集和 H.264 编码
    """

    def __init__(self, config: Optional[VideoConfig] = None):
        self.config = config or VideoConfig()
        self.camera: Optional[StereoCamera] = None
        self.encoder: Optional[H264Encoder] = None
        self.is_streaming = False
        self.send_thread: Optional[threading.Thread] = None

        # 回调
        self.on_frame_sent: Optional[Callable[[int], None]] = None

        # 统计
        self.stats = {
            'frames_sent': 0,
            'bytes_sent': 0,
            'start_time': 0.0
        }

    def initialize(self, device_id: int = 0) -> bool:
        """初始化发送器"""
        self.camera = StereoCamera(self.config, device_id)
        return self.camera.initialize()

    def start_streaming(self, target_ip: str, target_port: int) -> bool:
        """开始流式传输"""
        if self.is_streaming:
            logger.warning("已经在传输中")
            return False

        if not self.camera:
            logger.error("相机未初始化")
            return False

        # 启动相机采集
        self.camera.start_capture()

        # 创建编码器
        self.encoder = H264Encoder(self.config)
        if not self.encoder.start(target_ip, target_port):
            return False

        # 启动发送线程
        self.is_streaming = True
        self.stats['start_time'] = time.time()
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()

        logger.info(f"开始向 {target_ip}:{target_port} 发送 H.264 视频流")
        return True

    def _send_loop(self):
        """发送循环"""
        last_frame_id = -1

        while self.is_streaming:
            frame, frame_id = self.camera.get_frame()

            if frame is not None and frame_id != last_frame_id:
                if self.encoder.encode_frame(frame):
                    self.stats['frames_sent'] += 1
                    self.stats['bytes_sent'] += frame.nbytes
                    last_frame_id = frame_id

                    if self.on_frame_sent:
                        self.on_frame_sent(frame_id)
            else:
                time.sleep(0.001)

    def stop_streaming(self):
        """停止传输"""
        self.is_streaming = False

        if self.send_thread:
            self.send_thread.join(timeout=2)

        if self.encoder:
            self.encoder.stop()

        if self.camera:
            self.camera.stop()

        logger.info("视频流已停止")

    def get_stats(self) -> dict:
        """获取统计信息"""
        runtime = time.time() - self.stats['start_time'] if self.stats['start_time'] > 0 else 0
        return {
            'frames_sent': self.stats['frames_sent'],
            'bytes_sent': self.stats['bytes_sent'],
            'runtime_seconds': runtime,
            'fps_camera': self.camera.stats['fps_actual'] if self.camera else 0,
            'avg_fps': self.stats['frames_sent'] / runtime if runtime > 0 else 0
        }


# ============== TCP H.264 发送器 (兼容 OrinVideoSender) ==============

class SimpleH264Sender:
    """
    TCP H.264 发送器
    完全兼容 OrinVideoSender 的协议格式:
    - 使用 TCP 连接 (不是 UDP)
    - 帧格式: [length:4 bytes Big-Endian][H.264 NAL data]
    - 无 RTP 封装

    参考: XRoboToolkit-Orin-Video-Sender/main_zed_asio.cpp
    """

    def __init__(self, config: Optional[VideoConfig] = None):
        self.config = config or VideoConfig()
        self.cap: Optional[cv2.VideoCapture] = None
        self.ffmpeg_process: Optional[subprocess.Popen] = None
        self.tcp_socket: Optional[socket.socket] = None
        self.is_running = False
        self.send_thread: Optional[threading.Thread] = None
        self.target_ip = ""
        self.target_port = 0

        # 统计信息
        self.stats = {
            'frames_sent': 0,
            'bytes_sent': 0,
            'start_time': 0.0
        }

    def _check_nvenc_available(self) -> bool:
        """检测是否有NVIDIA硬件编码器可用"""
        try:
            result = subprocess.run(
                ['ffmpeg', '-hide_banner', '-encoders'],
                capture_output=True, text=True, timeout=5
            )
            if 'h264_nvenc' in result.stdout:
                logger.info("检测到 NVIDIA NVENC 硬件编码器")
                return True
        except Exception as e:
            logger.debug(f"检测NVENC失败: {e}")
        return False

    def _get_encoder_args(self, bitrate_k: int, use_nvenc: bool) -> list:
        """
        获取编码器参数

        NVENC (硬件编码): 更快，CPU占用低
        libx264 (软件编码): 通用，但CPU占用高
        """
        if use_nvenc:
            logger.info(f"使用 NVENC 硬件编码器, 码率: {bitrate_k}kbps")
            # FFmpeg 4.4 的 NVENC 参数格式
            # 注意: 需要强制 yuv420p 像素格式以确保兼容性
            return [
                '-pix_fmt', 'yuv420p',       # 强制 YUV420P，确保解码器兼容
                '-c:v', 'h264_nvenc',
                '-preset', 'p1',              # 最低延迟 (FFmpeg 4.x: p1-p7)
                '-tune', 'll',                # low latency
                '-profile:v', 'baseline',     # 基线配置，最大兼容性
                '-level', '5.1',
                '-rc', 'cbr',                 # 恒定码率
                '-b:v', f'{bitrate_k}k',
                '-maxrate', f'{bitrate_k}k',
                '-bufsize', f'{bitrate_k // 4}k',  # 增大缓冲区提升清晰度
                '-g', '1',                    # GOP=1，每帧都是关键帧
                '-keyint_min', '1',
            ]
        else:
            logger.info(f"使用 libx264 软件编码器, 码率: {bitrate_k}kbps")
            # 清晰度优化说明:
            # - preset: ultrafast(最快/最差) < superfast < veryfast < faster < fast < medium(默认/最好)
            #   提升到 superfast 可改善清晰度，但会增加 CPU 负载
            # - profile: baseline(最简单) < main < high(压缩效率最好)
            #   baseline 兼容性最好，但压缩效率低约 15%
            # - bufsize: 越大允许码率波动越大，复杂场景更清晰
            #   当前 bitrate/4 (约 5Mbit)，可增大到 bitrate/2 获得更好质量
            return [
                '-c:v', 'libx264',
                '-preset', 'superfast',      # 从 ultrafast 提升到 superfast，清晰度更好
                '-tune', 'zerolatency',
                '-profile:v', 'main',        # 从 baseline 提升到 main，压缩效率更高
                '-level', '5.1',
                '-b:v', f'{bitrate_k}k',
                '-maxrate', f'{int(bitrate_k * 1.2)}k',  # 允许峰值超出 20%
                '-minrate', f'{int(bitrate_k * 0.8)}k',  # 允许低谷到 80%
                '-bufsize', f'{bitrate_k // 4}k',        # 增大缓冲区，从 /8 改为 /4
                '-g', '1',
                '-keyint_min', '1',
                '-x264-params', 'repeat-headers=1',
            ]

    def initialize(self, device_id: int = 0) -> bool:
        """初始化相机"""
        logger.info(f"初始化相机 (设备: {device_id})...")

        # 保存设备ID供后续FFmpeg直接读取
        self.device_id = device_id

        # 先用OpenCV测试相机是否可用并获取实际参数
        test_cap = cv2.VideoCapture(device_id)
        test_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        test_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        test_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        test_cap.set(cv2.CAP_PROP_FPS, self.config.fps)

        if not test_cap.isOpened():
            logger.error("无法打开相机")
            return False

        # 验证实际参数
        actual_w = int(test_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(test_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = test_cap.get(cv2.CAP_PROP_FPS)
        logger.info(f"相机配置: {actual_w}x{actual_h} @ {actual_fps:.1f}fps")

        # 保存实际帧率
        self.actual_fps = actual_fps

        # 关闭测试连接，让FFmpeg独占相机
        test_cap.release()

        # Windows需要获取相机名称用于DirectShow
        if sys.platform == 'win32':
            self.camera_name = self._get_windows_camera_name(device_id)
            if not self.camera_name:
                logger.warning(f"无法获取相机名称，使用默认: video={device_id}")
                self.camera_name = f"video={device_id}"

        # 不再使用 self.cap，改用 FFmpeg 直接读取相机
        self.cap = None

        logger.info("相机初始化成功 (将使用FFmpeg直接采集)")
        return True

    def _get_windows_camera_name(self, device_id: int) -> Optional[str]:
        """
        获取Windows上的相机名称用于DirectShow
        使用ffmpeg -list_devices来获取设备列表
        """
        try:
            result = subprocess.run(
                ['ffmpeg', '-hide_banner', '-list_devices', 'true', '-f', 'dshow', '-i', 'dummy'],
                capture_output=True, text=True, timeout=10
            )
            # 解析输出找到视频设备
            lines = result.stderr.split('\n')
            video_devices = []
            in_video_section = False

            for line in lines:
                if 'DirectShow video devices' in line:
                    in_video_section = True
                    continue
                if 'DirectShow audio devices' in line:
                    in_video_section = False
                    continue
                if in_video_section and '"' in line:
                    # 提取设备名称
                    import re
                    match = re.search(r'"([^"]+)"', line)
                    if match and 'Alternative name' not in line:
                        video_devices.append(match.group(1))

            if device_id < len(video_devices):
                camera_name = f'video={video_devices[device_id]}'
                logger.info(f"找到相机: {camera_name}")
                return camera_name
            elif video_devices:
                camera_name = f'video={video_devices[0]}'
                logger.warning(f"设备ID {device_id} 超出范围，使用第一个相机: {camera_name}")
                return camera_name

        except Exception as e:
            logger.warning(f"获取Windows相机名称失败: {e}")

        return None

    def start_streaming(self, target_ip: str, target_port: int) -> bool:
        """
        开始流式传输 (TCP H.264)

        工作流程:
        1. 连接到 Unity Client 的 MediaDecoder TCP 服务
        2. 使用 FFmpeg 直接从相机读取 MJPEG 并转码为 H.264
        3. 读取 FFmpeg 输出的 H.264 NAL 单元
        4. 封装为 [length:4][data] 格式发送
        """
        if self.is_running:
            logger.warning("已经在传输中")
            return False

        self.target_ip = target_ip
        self.target_port = target_port

        # 连接到 Unity Client 的 MediaDecoder (带重试)
        logger.info(f"连接到 {target_ip}:{target_port}...")
        max_retries = 5
        retry_delay = 0.5
        connected = False

        for attempt in range(max_retries):
            try:
                self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.tcp_socket.settimeout(10)
                self.tcp_socket.connect((target_ip, target_port))
                logger.info(f"TCP 连接成功: {target_ip}:{target_port}")
                connected = True
                break
            except Exception as e:
                if attempt < max_retries - 1:
                    logger.warning(f"TCP 连接失败 (尝试 {attempt+1}/{max_retries}): {e}, {retry_delay}秒后重试...")
                    time.sleep(retry_delay)
                    retry_delay *= 1.5  # 指数退避
                else:
                    logger.error(f"TCP 连接失败，已达最大重试次数: {e}")

        if not connected:
            return False

        # FFmpeg 命令 - 直接从相机读取 MJPEG 并转码为 H.264
        # 优化: 跳过 OpenCV 解码，减少 CPU 开销
        bitrate_k = self.config.bitrate // 1000

        # 使用实际帧率（相机可能不支持60fps）
        actual_fps = int(getattr(self, 'actual_fps', self.config.fps))
        if actual_fps < self.config.fps:
            logger.warning(f"相机实际帧率 {actual_fps}fps < 请求的 {self.config.fps}fps，使用实际帧率")

        # 检测是否有NVIDIA硬件编码器
        use_nvenc = self._check_nvenc_available()
        encoder_args = self._get_encoder_args(bitrate_k, use_nvenc)

        # 根据操作系统构建输入参数
        if sys.platform == 'win32':
            # Windows: 使用 DirectShow
            # 需要先获取相机名称
            camera_name = getattr(self, 'camera_name', f'video={self.device_id}')
            input_args = [
                '-f', 'dshow',
                '-video_size', f'{self.config.width}x{self.config.height}',
                '-framerate', str(actual_fps),
                '-vcodec', 'mjpeg',  # 请求MJPEG格式
                '-i', camera_name,
            ]
        else:
            # Linux: 使用 V4L2
            input_args = [
                '-f', 'v4l2',
                '-input_format', 'mjpeg',
                '-video_size', f'{self.config.width}x{self.config.height}',
                '-framerate', str(actual_fps),
                '-i', f'/dev/video{self.device_id}',
            ]

        ffmpeg_cmd = [
            'ffmpeg',
            '-y',
        ] + input_args + encoder_args + [
            '-f', 'h264',
            '-'
        ]

        try:
            logger.info(f"启动 FFmpeg 编码器 (直接读取相机)...")
            logger.info(f"FFmpeg 命令: {' '.join(ffmpeg_cmd)}")
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.DEVNULL,  # 不需要stdin，FFmpeg直接读取相机
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            # 启动一个线程来读取 stderr
            def log_stderr():
                for line in self.ffmpeg_process.stderr:
                    logger.warning(f"[FFmpeg] {line.decode('utf-8', errors='ignore').strip()}")

            import threading
            stderr_thread = threading.Thread(target=log_stderr, daemon=True)
            stderr_thread.start()
        except Exception as e:
            logger.error(f"启动 FFmpeg 失败: {e}")
            self.tcp_socket.close()
            return False

        # 启动发送线程
        self.is_running = True
        self.stats['start_time'] = time.time()
        self.stats['frames_sent'] = 0
        self.stats['bytes_sent'] = 0

        # 只需要发送线程 (FFmpeg 自己从相机读取)
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()

        logger.info(f"开始向 {target_ip}:{target_port} 发送 TCP H.264 流")
        return True

    def _capture_loop(self):
        """采集循环 - 读取相机帧并写入 FFmpeg"""
        frame_interval = 1.0 / self.config.fps
        frame_count = 0

        while self.is_running and self.cap and self.cap.isOpened():
            loop_start = time.time()

            ret, frame = self.cap.read()
            if ret and frame is not None:
                try:
                    self.ffmpeg_process.stdin.write(frame.tobytes())
                    frame_count += 1

                    if frame_count % 60 == 0:
                        logger.debug(f"已采集 {frame_count} 帧")
                except Exception as e:
                    logger.error(f"写入 FFmpeg 失败: {e}")
                    break

            elapsed = time.time() - loop_start
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)

        # 关闭 FFmpeg stdin
        try:
            self.ffmpeg_process.stdin.close()
        except:
            pass

    def _send_loop(self):
        """
        发送循环 - 从 FFmpeg 读取 H.264 并通过 TCP 发送

        由于设置了 GOP=1 和 repeat-headers=1，每帧都以 SPS NAL 开头。
        我们按 SPS 分割数据流，每个 SPS 到下一个 SPS 之间的数据就是一帧。

        与 OrinVideoSender 一致的格式:
        [length: 4 bytes, Big-Endian][H.264 frame data]
        """
        buffer = b''
        NAL_STARTCODE_4 = b'\x00\x00\x00\x01'

        frame_count = 0
        last_log_time = time.time()
        first_sps_found = False

        logger.info("开始发送 H.264 数据流 (按帧发送, GOP=1)...")

        while self.is_running:
            try:
                # 从 FFmpeg stdout 读取数据
                chunk = self.ffmpeg_process.stdout.read(32768)
                if not chunk:
                    if self.ffmpeg_process.poll() is not None:
                        logger.info("FFmpeg 进程已结束")
                        break
                    time.sleep(0.001)
                    continue

                buffer += chunk

                # 按 SPS NAL (type 7) 分割帧
                while len(buffer) > 5:
                    # 首先确保我们从 SPS 开始（对于第一帧）
                    if not first_sps_found:
                        # 找到第一个 SPS
                        sps_idx = self._find_nal_type(buffer, 7, 0)
                        if sps_idx < 0:
                            break  # 还没有找到 SPS
                        if sps_idx > 0:
                            buffer = buffer[sps_idx:]  # 丢弃 SPS 之前的数据
                        first_sps_found = True
                        continue

                    # 找下一个 SPS（从当前位置 +4 开始搜索，跳过当前 SPS）
                    next_sps_idx = self._find_nal_type(buffer, 7, 5)

                    if next_sps_idx > 0:
                        # 找到下一个 SPS，提取当前帧
                        frame_data = buffer[:next_sps_idx]
                        buffer = buffer[next_sps_idx:]

                        self._send_frame(frame_data)
                        frame_count += 1

                        # 每秒输出一次日志
                        now = time.time()
                        if now - last_log_time >= 1.0:
                            elapsed = now - self.stats['start_time']
                            fps = frame_count / elapsed if elapsed > 0 else 0
                            mbps = (self.stats['bytes_sent'] * 8) / (elapsed * 1000000) if elapsed > 0 else 0
                            logger.info(f"已发送 {frame_count} 帧, {mbps:.1f} Mbps, {fps:.1f} fps")
                            last_log_time = now
                    else:
                        # 没有找到下一个 SPS，等待更多数据
                        # 但如果缓冲区太大，强制发送
                        if len(buffer) > 512 * 1024:  # 512KB
                            frame_data = buffer
                            buffer = b''
                            first_sps_found = False  # 需要重新找 SPS
                            self._send_frame(frame_data)
                            frame_count += 1
                            logger.warning(f"缓冲区溢出，强制发送 {len(frame_data)} bytes")
                        break

            except Exception as e:
                logger.error(f"发送循环错误: {e}")
                import traceback
                traceback.print_exc()
                break

        # 发送剩余数据
        if buffer and len(buffer) > 0:
            self._send_frame(buffer)
            frame_count += 1

        logger.info(f"发送循环结束, 共发送 {frame_count} 帧")

    def _find_nal_type(self, data: bytes, nal_type: int, start_pos: int = 0) -> int:
        """
        在数据中查找特定类型的 NAL 单元
        返回找到的位置，如果没找到返回 -1
        """
        NAL_STARTCODE_4 = b'\x00\x00\x00\x01'
        NAL_STARTCODE_3 = b'\x00\x00\x01'

        pos = start_pos
        while pos < len(data) - 4:
            # 检查 4 字节 startcode
            if data[pos:pos+4] == NAL_STARTCODE_4:
                if (data[pos+4] & 0x1F) == nal_type:
                    return pos
                pos += 4
            # 检查 3 字节 startcode
            elif data[pos:pos+3] == NAL_STARTCODE_3:
                if (data[pos+3] & 0x1F) == nal_type:
                    return pos
                pos += 3
            else:
                pos += 1

        return -1

    def _send_frame(self, frame_data: bytes):
        """
        发送一个完整的 H.264 帧

        格式与 XRoboToolkit-Orin-Video-Sender 一致:
        [4字节Big-Endian长度][完整H.264 Annexb数据(含startcode)]

        参考: https://github.com/XR-Robotics/XRoboToolkit-Orin-Video-Sender
        """
        if not self.tcp_socket or not frame_data:
            return

        try:
            size = len(frame_data)
            # 官方格式: 4字节Big-Endian长度 + 完整Annexb数据(含00 00 00 01)
            packet = struct.pack('>I', size) + frame_data

            self.tcp_socket.sendall(packet)

            self.stats['frames_sent'] += 1
            self.stats['bytes_sent'] += len(packet)

            # 前几帧输出详细调试信息
            if self.stats['frames_sent'] <= 3:
                hex_preview = frame_data[:min(20, len(frame_data))].hex()
                logger.info(f"[Frame {self.stats['frames_sent']}] 大小: {size} bytes, 数据头: {hex_preview}...")

        except Exception as e:
            logger.error(f"发送帧失败: {e}")
            self.is_running = False

    def stop_streaming(self):
        """停止传输"""
        logger.info("正在停止视频流...")
        self.is_running = False

        if hasattr(self, 'capture_thread') and self.capture_thread:
            self.capture_thread.join(timeout=2)

        if self.send_thread:
            self.send_thread.join(timeout=2)

        if self.ffmpeg_process:
            try:
                self.ffmpeg_process.stdin.close()
            except:
                pass
            self.ffmpeg_process.terminate()
            try:
                self.ffmpeg_process.wait(timeout=2)
            except:
                self.ffmpeg_process.kill()

        if self.tcp_socket:
            try:
                self.tcp_socket.close()
            except:
                pass

        if self.cap:
            self.cap.release()

        # 打印统计
        elapsed = time.time() - self.stats['start_time'] if self.stats['start_time'] > 0 else 0
        logger.info(f"视频流已停止. 统计: {self.stats['frames_sent']} NAL, "
                   f"{self.stats['bytes_sent'] / 1024 / 1024:.1f} MB, {elapsed:.1f}s")


# ============== 测试代码 ==============

def test_simple_sender():
    """测试 TCP H.264 发送器"""
    print("=" * 60)
    print("TCP H.264 视频发送器测试")
    print("兼容 OrinVideoSender / XRoboToolkit-Unity-Client")
    print("=" * 60)

    config = VideoConfig(width=2560, height=720, fps=60, bitrate=8000000)
    sender = SimpleH264Sender(config)

    if not sender.initialize(device_id=0):
        print("相机初始化失败")
        return

    print("\n请在 PICO 头显上:")
    print("1. 启动 Unity Client")
    print("2. 选择 USB_STEREO 相机")
    print("3. 点击 Listen 按钮")
    print("4. 记录显示的 IP 和端口\n")

    target_ip = input("请输入 PICO 头显 IP: ").strip() or "192.168.1.100"
    target_port = int(input("请输入目标端口 (默认 12345): ").strip() or "12345")

    if sender.start_streaming(target_ip, target_port):
        print(f"\n正在向 {target_ip}:{target_port} 发送 TCP H.264 视频流...")
        print("按 Ctrl+C 停止\n")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass

    sender.stop_streaming()
    print("测试完成")


if __name__ == '__main__':
    test_simple_sender()
