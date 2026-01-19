#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
H.264 视频编码发送器模块
用于 USB 双目相机的低延迟视频流传输，支持 VR 遥操作场景

功能特性:
1. FFmpeg 直接采集相机 MJPEG，跳过 OpenCV 解码开销
2. 自动检测 NVENC 硬件编码，回退到 libx264 软件编码
3. TCP 流式传输，兼容 XRoboToolkit Unity Client 的 MediaDecoder
4. 低延迟优化：小 bufsize、GOP=1、zerolatency

架构说明:
┌─────────────┐    MJPEG     ┌─────────────┐    H.264    ┌─────────────┐
│  USB Camera │ ──────────→  │   FFmpeg    │ ──────────→ │  TCP Send   │
│  (V4L2/DShow)│   (USB)     │ NVENC/x264  │   (pipe)    │  to PICO    │
└─────────────┘              └─────────────┘              └─────────────┘

为什么跳过 OpenCV？
- 传统方案: Camera → OpenCV(MJPEG解码) → rawvideo pipe → FFmpeg(编码) → TCP
- 优化方案: Camera → FFmpeg(直接读取MJPEG并编码) → TCP
- 优势: 减少一次完整的 MJPEG 解码 + BGR 转换，CPU 占用降低 30-50%

协议格式 (兼容 XRoboToolkit-Orin-Video-Sender):
[4字节 Big-Endian 长度][H.264 Annexb 数据(含 00 00 00 01 startcode)]

作者: Liang ZHU
邮箱: lzhu686@connect.hkust-gz.edu.cn
日期: 2025
"""

import cv2
import time
import threading
import logging
import subprocess
import socket
import struct
import sys
from typing import Optional
from dataclasses import dataclass
from collections import deque

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


# ============== 常量配置 ==============

class StreamingConstants:
    """流媒体常量"""
    BUFFER_READ_SIZE = 32768        # FFmpeg stdout 读取缓冲 (32KB)
    MAX_BUFFER_SIZE = 512 * 1024    # 帧缓冲区溢出阈值 (512KB)
    TCP_RETRY_COUNT = 5             # TCP 连接重试次数
    TCP_RETRY_DELAY = 0.5           # TCP 重试延迟 (秒)

    # 连接参数
    SEND_TIMEOUT_SECONDS = 5        # TCP 发送超时 (秒)
    CONNECTION_LOST_THRESHOLD = 2   # 连续发送失败次数，触发连接断开回调


@dataclass
class VideoConfig:
    """
    视频配置 - 双臂灵巧手遥操作场景

    相机: HBVCAM-F2439GS-2 V11 (AR0234 全局快门)
    场景: 双臂灵巧手精细操作，快速运动清晰度优先

    设计原则:
    1. 2560x720: 相机原生 MJPEG 分辨率 (左右眼各 1280x720)
    2. 30fps: 相机在此分辨率下的实际帧率
    3. 20Mbps: 高码率保证画质，每帧约 83KB
    4. GOP=1: 每帧独立解码，适合网络丢包恢复
    """
    width: int = 2560            # 双目总宽度
    height: int = 720            # 720p
    fps: int = 60                # 请求帧率 (实际由相机决定)
    bitrate: int = 20971520      # 20 Mbps (20 * 1024 * 1024)
    keyframe_interval: int = 1   # GOP=1

    # V4L2 Loopback 双进程架构配置
    loopback_device: Optional[str] = None  # V4L2 Loopback 设备路径 (如 /dev/stereo_camera)
    loopback_fps: int = 30                  # Loopback 输出帧率 (ROS2 发布用)

    @property
    def single_eye_width(self) -> int:
        """单眼宽度"""
        return self.width // 2

    @property
    def bitrate_kbps(self) -> int:
        """码率 (kbps)"""
        return self.bitrate // 1000


# ============== TCP H.264 发送器 ==============

class SimpleH264Sender:
    """
    TCP H.264 发送器

    工作原理:
    1. OpenCV 仅用于测试相机可用性和获取实际参数
    2. FFmpeg 通过 V4L2 (Linux) 或 DirectShow (Windows) 直接读取相机 MJPEG
    3. FFmpeg 内部解码 MJPEG 并编码为 H.264 (NVENC 或 libx264)
    4. Python 读取 FFmpeg 的 stdout，按 SPS NAL 分割帧
    5. 封装为 [长度][数据] 格式通过 TCP 发送

    为什么这样更快？
    - MJPEG 是压缩格式，USB 带宽需求低 (约 50-100 Mbps)
    - FFmpeg 的 MJPEG 解码器比 OpenCV 更优化
    - 避免了 Python 层面的大数组 (BGR frame) 拷贝
    - 减少了进程间通信的数据量 (MJPEG vs rawvideo)
    """

    def __init__(self, config: Optional[VideoConfig] = None):
        self.config = config or VideoConfig()
        self.ffmpeg_process: Optional[subprocess.Popen] = None
        self.tcp_socket: Optional[socket.socket] = None
        self.is_running = False
        self.send_thread: Optional[threading.Thread] = None
        self.device_id = 0
        self.device_path = ""   # 设备路径 (Linux: /dev/video0, Windows: video=Name)
        self.actual_fps = 30.0
        self.camera_name = ""   # Windows DirectShow 设备名 (兼容旧代码)

        # 连接信息 (用于重连)
        self._target_ip = ""
        self._target_port = 0
        self._consecutive_send_failures = 0

        # 回调函数
        self.on_connection_lost: Optional[callable] = None  # 连接断开回调
        self.on_connection_restored: Optional[callable] = None  # 连接恢复回调

        # 统计信息
        self.stats = {
            'frames_sent': 0,
            'bytes_sent': 0,
            'start_time': 0.0,
            'last_frame_time': 0.0,
            'connection_lost_count': 0,
        }

        # 网络状态监控 (仅用于统计日志)
        self._send_times = deque(maxlen=30)  # 最近 30 帧的发送耗时

    # ============== 编码器检测与配置 ==============

    def _check_nvenc_available(self) -> bool:
        """
        检测 NVIDIA NVENC 硬件编码器是否可用

        NVENC 优势:
        - GPU 编码，CPU 占用 < 5%
        - 延迟低于软件编码
        - 支持 2560x720@30fps 毫无压力
        """
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
        获取编码器参数 (低延迟优先)

        遥操作核心原则:
        - 延迟优先于画质
        - 画质通过提高码率保证，而不是编码器参数
        - preset p1 + baseline = 最低编码延迟 (~5ms)

        参数说明:
        - preset p1: NVENC 最快预设，编码延迟最低
        - profile baseline: 无 CABAC，解码延迟最低
        - tune ll + zerolatency: 禁用帧缓冲和重排
        - bufsize bitrate/20: 小缓冲，减少编码器内部延迟

        为什么不用 p4/high/spatial-aq？
        - p4 vs p1: +10-30ms 编码延迟
        - high vs baseline: +5-15ms 解码延迟 (CABAC)
        - spatial-aq: +5-10ms 分析延迟
        - 对遥操作是致命的，宁可画质差也要延迟低
        """
        if use_nvenc:
            logger.info(f"使用 NVENC 硬件编码器 (低延迟), 码率: {bitrate_k} kbps")
            return [
                '-pix_fmt', 'yuv420p',
                '-c:v', 'h264_nvenc',
                '-preset', 'p1',               # 最快预设，最低延迟
                '-tune', 'll',                 # low latency 调优
                '-profile:v', 'baseline',      # 无 CABAC，解码最快
                '-level', '4.2',               # 足够 2560x720@60fps
                '-rc', 'cbr',
                '-b:v', f'{bitrate_k}k',
                '-maxrate', f'{bitrate_k}k',
                '-bufsize', f'{bitrate_k // 20}k',  # 小缓冲，低延迟
                '-g', '1',                     # GOP=1，每帧独立
                '-keyint_min', '1',
                '-delay', '0',
                '-zerolatency', '1',
            ]
        else:
            logger.info(f"使用 libx264 软件编码器 (低延迟), 码率: {bitrate_k} kbps")
            return [
                '-pix_fmt', 'yuv420p',
                '-c:v', 'libx264',
                '-preset', 'ultrafast',        # 最快预设
                '-tune', 'zerolatency',        # 零延迟调优
                '-profile:v', 'baseline',      # 无 CABAC
                '-level', '4.2',
                '-b:v', f'{bitrate_k}k',
                '-maxrate', f'{bitrate_k}k',
                '-bufsize', f'{bitrate_k // 20}k',
                '-g', '1',
                '-keyint_min', '1',
            ]

    # ============== 相机初始化 ==============

    def initialize(self, device: int | str = 0) -> bool:
        """
        初始化相机

        参数:
            device: 设备标识，支持以下格式:
                - int: 设备索引 (0, 1, 2...)
                - str (Linux): 设备路径 ("/dev/video0", "/dev/video13")
                - str (Windows): DirectShow 设备名 ("video=Camera Name")

        步骤:
        1. 解析设备标识
        2. 用 OpenCV 测试相机是否可用
        3. 获取相机实际支持的分辨率和帧率
        4. 关闭 OpenCV，让 FFmpeg 独占相机
        """
        # 解析设备标识
        if isinstance(device, str):
            if sys.platform == 'win32':
                # Windows: 直接使用设备名
                self.device_path = device if device.startswith('video=') else f"video={device}"
                self.camera_name = self.device_path
                # 尝试从设备名提取索引用于 OpenCV
                self.device_id = self._find_device_index_windows(device)
            else:
                # Linux: 使用设备路径
                self.device_path = device
                # 从路径提取数字用于 OpenCV
                import re
                match = re.search(r'/dev/video(\d+)', device)
                self.device_id = int(match.group(1)) if match else 0
        else:
            # 数字索引
            self.device_id = device
            if sys.platform == 'win32':
                self.device_path = ""  # 稍后获取
            else:
                self.device_path = f"/dev/video{device}"

        logger.info(f"初始化相机 (设备: {device}, 路径: {self.device_path or '待获取'})...")

        # 用 OpenCV 测试相机
        test_cap = cv2.VideoCapture(self.device_id)
        test_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        test_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        test_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        test_cap.set(cv2.CAP_PROP_FPS, self.config.fps)

        if not test_cap.isOpened():
            logger.error(f"无法打开相机: {device}")
            return False

        # 获取实际参数 (相机可能不支持请求的值)
        actual_w = int(test_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(test_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.actual_fps = test_cap.get(cv2.CAP_PROP_FPS)
        logger.info(f"相机配置: {actual_w}x{actual_h} @ {self.actual_fps:.1f}fps")

        # 关闭 OpenCV，让 FFmpeg 独占
        test_cap.release()

        # Windows: 获取 DirectShow 设备名称 (如果还没有)
        if sys.platform == 'win32' and not self.device_path:
            self.camera_name = self._get_windows_camera_name(self.device_id)
            if self.camera_name:
                self.device_path = self.camera_name
            else:
                self.device_path = f"video={self.device_id}"
                self.camera_name = self.device_path
                logger.warning(f"无法获取相机名称，使用默认: {self.camera_name}")

        logger.info(f"相机初始化成功 (设备路径: {self.device_path})")
        return True

    def _find_device_index_windows(self, device_name: str) -> int:
        """Windows: 根据设备名查找设备索引"""
        try:
            result = subprocess.run(
                ['ffmpeg', '-hide_banner', '-list_devices', 'true', '-f', 'dshow', '-i', 'dummy'],
                capture_output=True, text=True, timeout=10
            )
            lines = result.stderr.split('\n')
            video_devices = []
            in_video_section = False

            for line in lines:
                if 'DirectShow video devices' in line:
                    in_video_section = True
                    continue
                if 'DirectShow audio devices' in line:
                    break
                if in_video_section and '"' in line and 'Alternative name' not in line:
                    import re
                    match = re.search(r'"([^"]+)"', line)
                    if match:
                        video_devices.append(match.group(1))

            # 查找匹配的设备
            search_name = device_name.replace('video=', '')
            for idx, name in enumerate(video_devices):
                if name == search_name or search_name in name:
                    return idx

        except Exception as e:
            logger.warning(f"查找Windows设备索引失败: {e}")
        return 0

    def _get_windows_camera_name(self, device_id: int) -> Optional[str]:
        """
        获取 Windows DirectShow 设备名称

        FFmpeg 在 Windows 上需要设备名而不是数字 ID
        格式: video=设备名称
        """
        try:
            result = subprocess.run(
                ['ffmpeg', '-hide_banner', '-list_devices', 'true', '-f', 'dshow', '-i', 'dummy'],
                capture_output=True, text=True, timeout=10
            )
            lines = result.stderr.split('\n')
            video_devices = []
            in_video_section = False

            for line in lines:
                if 'DirectShow video devices' in line:
                    in_video_section = True
                    continue
                if 'DirectShow audio devices' in line:
                    break
                if in_video_section and '"' in line and 'Alternative name' not in line:
                    import re
                    match = re.search(r'"([^"]+)"', line)
                    if match:
                        video_devices.append(match.group(1))

            if device_id < len(video_devices):
                name = f'video={video_devices[device_id]}'
                logger.info(f"找到相机: {name}")
                return name

        except Exception as e:
            logger.warning(f"获取Windows相机名称失败: {e}")
        return None

    # ============== 流媒体传输 ==============

    def start_streaming(self, target_ip: str, target_port: int) -> bool:
        """
        开始流式传输

        流程:
        1. TCP 连接到 PICO 的 MediaDecoder
        2. 启动 FFmpeg 进程 (直接读取相机)
        3. 启动发送线程 (读取 FFmpeg 输出并发送)
        """
        if self.is_running:
            logger.warning("已经在传输中")
            return False

        # 保存连接信息 (用于重连)
        self._target_ip = target_ip
        self._target_port = target_port

        # 1. TCP 连接 (带重试)
        if not self._connect_tcp(target_ip, target_port):
            return False

        # 2. 启动 FFmpeg
        if not self._start_ffmpeg():
            self.tcp_socket.close()
            return False

        # 3. 启动发送线程
        self.is_running = True
        self.stats = {
            'frames_sent': 0,
            'bytes_sent': 0,
            'start_time': time.time(),
            'last_frame_time': 0.0,
            'connection_lost_count': 0,
        }
        self._consecutive_send_failures = 0
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()

        logger.info(f"开始向 {target_ip}:{target_port} 发送 TCP H.264 流")
        return True

    def _connect_tcp(self, target_ip: str, target_port: int) -> bool:
        """TCP 连接 (带指数退避重试)"""
        logger.info(f"连接到 {target_ip}:{target_port}...")
        retry_delay = StreamingConstants.TCP_RETRY_DELAY

        for attempt in range(StreamingConstants.TCP_RETRY_COUNT):
            try:
                self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

                # 设置连接超时
                self.tcp_socket.settimeout(5.0)

                self.tcp_socket.connect((target_ip, target_port))

                # 连接成功后设置发送超时
                self.tcp_socket.settimeout(StreamingConstants.SEND_TIMEOUT_SECONDS)

                logger.info(f"TCP 连接成功: {target_ip}:{target_port}")
                return True
            except socket.timeout:
                logger.warning(f"TCP 连接超时 ({attempt+1}/{StreamingConstants.TCP_RETRY_COUNT})")
                logger.info(f"  -> 请确认 PICO 端 MediaDecoder 已启动监听 {target_port} 端口")
            except ConnectionRefusedError:
                logger.warning(f"TCP 连接被拒绝 ({attempt+1}/{StreamingConstants.TCP_RETRY_COUNT})")
                logger.info(f"  -> PICO 端的 MediaDecoder.startServer() 可能尚未执行")
                logger.info(f"  -> 或者端口 {target_port} 被其他进程占用")
            except Exception as e:
                logger.warning(f"TCP 连接失败 ({attempt+1}/{StreamingConstants.TCP_RETRY_COUNT}): {e}")

            if attempt < StreamingConstants.TCP_RETRY_COUNT - 1:
                logger.info(f"等待 {retry_delay:.1f} 秒后重试...")
                time.sleep(retry_delay)
                retry_delay *= 1.5

        logger.error(f"TCP 连接失败，已达最大重试次数 ({StreamingConstants.TCP_RETRY_COUNT})")
        logger.error(f"请检查:")
        logger.error(f"  1. PICO 端 Unity Client 是否已点击 Listen 按钮")
        logger.error(f"  2. PICO IP 是否正确 (当前: {target_ip})")
        logger.error(f"  3. 端口 {target_port} 是否正确")
        logger.error(f"  4. 防火墙是否允许此端口")
        return False

    def _start_ffmpeg(self) -> bool:
        """
        启动 FFmpeg 进程

        为什么直接读取相机比 OpenCV + pipe 更好？

        方案 A (旧): OpenCV → rawvideo pipe → FFmpeg
        ┌─────────┐   MJPEG    ┌─────────┐   BGR (5.5MB/帧)   ┌─────────┐
        │ Camera  │ ────────→  │ OpenCV  │ ─────────────────→ │ FFmpeg  │
        └─────────┘            └─────────┘                    └─────────┘
        - OpenCV 解码 MJPEG → BGR (CPU 密集)
        - BGR 帧通过 pipe 传输 (2560x720x3 = 5.5MB/帧)
        - FFmpeg 再次转换 BGR → YUV420P

        方案 B (新): FFmpeg 直接读取
        ┌─────────┐   MJPEG (~100KB/帧)   ┌─────────┐
        │ Camera  │ ─────────────────────→ │ FFmpeg  │
        └─────────┘                        └─────────┘
        - FFmpeg 直接从 V4L2/DirectShow 读取 MJPEG
        - FFmpeg 内部解码 MJPEG → YUV (更优化)
        - 没有 Python 层面的大数组拷贝
        - 进程间通信数据量减少 50 倍
        """
        bitrate_k = self.config.bitrate_kbps
        actual_fps = int(self.actual_fps)

        if actual_fps < self.config.fps:
            logger.warning(f"相机实际帧率 {actual_fps}fps < 请求的 {self.config.fps}fps")

        # 检测编码器
        use_nvenc = self._check_nvenc_available()
        encoder_args = self._get_encoder_args(bitrate_k, use_nvenc)

        # 构建输入参数 (跨平台)
        if sys.platform == 'win32':
            input_args = [
                '-f', 'dshow',
                '-video_size', f'{self.config.width}x{self.config.height}',
                '-framerate', str(actual_fps),
                '-vcodec', 'mjpeg',
                '-i', self.device_path,  # 使用设备路径
            ]
        else:
            input_args = [
                '-f', 'v4l2',
                '-input_format', 'mjpeg',
                '-video_size', f'{self.config.width}x{self.config.height}',
                '-framerate', str(actual_fps),
                '-i', self.device_path,  # 使用设备路径 (如 /dev/video13)
            ]

        # ============== 构建 FFmpeg 命令 ==============
        # 基础命令: 输入参数 + 编码器参数 + H.264 输出到 stdout
        ffmpeg_cmd = ['ffmpeg', '-y'] + input_args + encoder_args + ['-f', 'h264', '-']

        # V4L2 Loopback 双输出模式 (仅 Linux)
        # 架构:
        # Camera → FFmpeg → [H.264 → stdout → PICO (60fps)]
        #                 → [MJPEG → /dev/stereo_camera (30fps) → ROS2独立进程]
        if self.config.loopback_device and sys.platform != 'win32':
            loopback_fps = self.config.loopback_fps
            loopback_device = self.config.loopback_device

            logger.info(f"启用 V4L2 Loopback 双输出模式")
            logger.info(f"  主输出: H.264 → TCP → PICO ({actual_fps}fps)")
            logger.info(f"  副输出: MJPEG → {loopback_device} ({loopback_fps}fps)")

            # 重新构建命令，使用 tee 分发多输出
            # FFmpeg 多输出语法: -map 0:v 指定输入流，多个输出分别配置
            ffmpeg_cmd = [
                'ffmpeg', '-y'
            ] + input_args + [
                # 输出 1: H.264 到 stdout (PICO 高优先级)
                '-map', '0:v',
            ] + encoder_args + [
                '-f', 'h264', 'pipe:1',

                # 输出 2: MJPEG 到 V4L2 Loopback (ROS2 发布)
                '-map', '0:v',
                '-r', str(loopback_fps),       # 降帧率
                '-c:v', 'mjpeg',               # MJPEG 编码 (ROS2 易处理)
                '-q:v', '3',                   # MJPEG 质量 (1-31, 越小越好)
                '-f', 'v4l2',
                loopback_device
            ]

        try:
            logger.info(f"FFmpeg 命令: {' '.join(ffmpeg_cmd)}")
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )

            # 后台线程读取 FFmpeg 日志
            threading.Thread(target=self._log_ffmpeg_stderr, daemon=True).start()
            return True

        except Exception as e:
            logger.error(f"启动 FFmpeg 失败: {e}")
            return False

    def _log_ffmpeg_stderr(self):
        """后台读取 FFmpeg stderr 并输出日志"""
        for line in self.ffmpeg_process.stderr:
            logger.warning(f"[FFmpeg] {line.decode('utf-8', errors='ignore').strip()}")

    # ============== 帧发送循环 ==============

    def _send_loop(self):
        """
        发送循环 - 从 FFmpeg 读取 H.264 并通过 TCP 发送

        H.264 NAL 单元分割:
        - GOP=1 时，每帧都以 SPS NAL (type=7) 开头
        - 格式: [SPS][PPS][IDR slice] 或 [SPS][PPS][P slice]
        - 我们按 SPS 分割，每个 SPS 到下一个 SPS 之间就是一个完整帧

        发送格式 (兼容 XRoboToolkit-Orin-Video-Sender):
        [4字节 Big-Endian 长度][H.264 Annexb 数据]
        """
        buffer = b''
        frame_count = 0
        last_log_time = time.time()
        first_sps_found = False

        logger.info("开始发送 H.264 数据流 (GOP=1, 按帧发送)...")

        while self.is_running:
            try:
                # 从 FFmpeg 读取数据
                chunk = self.ffmpeg_process.stdout.read(StreamingConstants.BUFFER_READ_SIZE)
                if not chunk:
                    if self.ffmpeg_process.poll() is not None:
                        logger.info("FFmpeg 进程已结束")
                        break
                    time.sleep(0.001)
                    continue

                buffer += chunk

                # 按 SPS NAL 分割帧
                while len(buffer) > 5:
                    if not first_sps_found:
                        sps_idx = self._find_nal_type(buffer, 7, 0)
                        if sps_idx < 0:
                            break
                        if sps_idx > 0:
                            buffer = buffer[sps_idx:]
                        first_sps_found = True
                        continue

                    next_sps_idx = self._find_nal_type(buffer, 7, 5)

                    if next_sps_idx > 0:
                        frame_data = buffer[:next_sps_idx]
                        buffer = buffer[next_sps_idx:]

                        # 直接发送帧 (TCP 保证可靠传输)
                        self._send_frame(frame_data)
                        frame_count += 1

                        # 每秒输出统计
                        now = time.time()
                        if now - last_log_time >= 1.0:
                            elapsed = now - self.stats['start_time']
                            fps = frame_count / elapsed if elapsed > 0 else 0
                            mbps = (self.stats['bytes_sent'] * 8) / (elapsed * 1_000_000) if elapsed > 0 else 0
                            avg_send_ms = self.get_network_stats()['avg_send_time_ms']

                            logger.info(f"帧:{frame_count} | {mbps:.1f}Mbps | {fps:.1f}fps | "
                                        f"发送耗时:{avg_send_ms:.1f}ms")
                            last_log_time = now
                    else:
                        # 缓冲区溢出保护
                        if len(buffer) > StreamingConstants.MAX_BUFFER_SIZE:
                            self._send_frame(buffer)
                            buffer = b''
                            first_sps_found = False
                            frame_count += 1
                            logger.warning("缓冲区溢出，强制发送")
                        break

            except Exception as e:
                logger.error(f"发送循环错误: {e}")
                break

        logger.info(f"发送循环结束, 共发送 {frame_count} 帧")

    def _find_nal_type(self, data: bytes, nal_type: int, start_pos: int = 0) -> int:
        """
        查找特定类型的 NAL 单元

        NAL 单元格式:
        - 4字节 startcode: 00 00 00 01
        - 3字节 startcode: 00 00 01
        - NAL header: 第一个字节的低 5 位是 NAL 类型
          - 7: SPS (Sequence Parameter Set)
          - 8: PPS (Picture Parameter Set)
          - 5: IDR slice (关键帧)
          - 1: P slice (预测帧)
        """
        NAL_STARTCODE_4 = b'\x00\x00\x00\x01'
        NAL_STARTCODE_3 = b'\x00\x00\x01'

        pos = start_pos
        while pos < len(data) - 4:
            if data[pos:pos+4] == NAL_STARTCODE_4:
                if (data[pos+4] & 0x1F) == nal_type:
                    return pos
                pos += 4
            elif data[pos:pos+3] == NAL_STARTCODE_3:
                if (data[pos+3] & 0x1F) == nal_type:
                    return pos
                pos += 3
            else:
                pos += 1
        return -1

    def _send_frame(self, frame_data: bytes):
        """
        发送一个 H.264 帧

        格式: [4字节 Big-Endian 长度][H.264 Annexb 数据]

        TCP 保证可靠有序传输，无需应用层丢帧。
        连接断开时触发回调通知上层。
        """
        if not self.tcp_socket or not frame_data:
            return

        try:
            packet = struct.pack('>I', len(frame_data)) + frame_data

            # 记录发送开始时间
            send_start = time.perf_counter()

            # 发送数据
            self.tcp_socket.sendall(packet)

            # 计算发送耗时 (仅用于统计)
            send_time_ms = (time.perf_counter() - send_start) * 1000
            self._send_times.append(send_time_ms)

            # 发送成功，重置失败计数
            self._consecutive_send_failures = 0

            # 更新统计
            self.stats['frames_sent'] += 1
            self.stats['bytes_sent'] += len(packet)
            self.stats['last_frame_time'] = time.time()

            # 前 10 帧输出详细调试信息
            if self.stats['frames_sent'] <= 10:
                hex_preview = frame_data[:20].hex()
                logger.info(f"[Frame {self.stats['frames_sent']}] {len(frame_data)} bytes, "
                           f"发送耗时: {send_time_ms:.1f}ms, hex: {hex_preview}...")

        except socket.timeout:
            self._consecutive_send_failures += 1
            logger.warning(f"发送超时 ({self._consecutive_send_failures}/{StreamingConstants.CONNECTION_LOST_THRESHOLD})")

            if self._consecutive_send_failures >= StreamingConstants.CONNECTION_LOST_THRESHOLD:
                self._handle_connection_lost("发送超时")

        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError) as e:
            logger.error(f"连接断开: {e}")
            logger.error(f"已发送 {self.stats['frames_sent']} 帧, 共 {self.stats['bytes_sent']} 字节")
            logger.error(f"可能原因:")
            logger.error(f"  1. PICO 端 MediaDecoder 崩溃或被关闭")
            logger.error(f"  2. Unity Client 被切到后台")
            logger.error(f"  3. 网络断开")
            self._handle_connection_lost(str(e))

        except OSError as e:
            logger.error(f"Socket 错误: {e}")
            self._handle_connection_lost(str(e))

        except Exception as e:
            logger.error(f"发送帧失败: {e}")
            self._consecutive_send_failures += 1
            if self._consecutive_send_failures >= StreamingConstants.CONNECTION_LOST_THRESHOLD:
                self._handle_connection_lost(str(e))

    def _handle_connection_lost(self, reason: str):
        """
        处理连接断开

        行为:
        1. 记录断开事件
        2. 停止发送循环
        3. 在单独线程中触发回调 (避免 join 自己的问题)

        关键: 回调在新线程中执行，因为当前是 _send_loop 线程，
              如果回调中调用 stop_streaming() 会导致 join 自己。
        """
        self.stats['connection_lost_count'] += 1
        logger.error(f"连接断开 (第 {self.stats['connection_lost_count']} 次): {reason}")
        logger.info(f"提示: PICO 端可能进入休眠或 Unity Client 被切到后台")

        # 停止发送循环
        self.is_running = False

        # 在新线程中触发回调，避免 "cannot join current thread" 错误
        if self.on_connection_lost:
            def delayed_callback():
                try:
                    # 等待当前线程退出后再执行回调
                    time.sleep(0.1)
                    self.on_connection_lost(reason)
                except Exception as e:
                    logger.warning(f"连接断开回调执行失败: {e}")

            callback_thread = threading.Thread(target=delayed_callback, daemon=True)
            callback_thread.start()

    def get_network_stats(self) -> dict:
        """
        获取网络状态统计

        返回:
        - avg_send_time_ms: 平均发送耗时
        - max_send_time_ms: 最大发送耗时
        """
        if not self._send_times:
            return {
                'avg_send_time_ms': 0,
                'max_send_time_ms': 0,
            }

        return {
            'avg_send_time_ms': sum(self._send_times) / len(self._send_times),
            'max_send_time_ms': max(self._send_times),
        }

    # ============== 清理 ==============

    def stop_streaming(self):
        """
        停止传输并清理资源

        关键: 必须确保 FFmpeg 进程被终止，否则相机会被锁定。
        即使其他步骤失败，也要尽力释放 FFmpeg。
        """
        logger.info("正在停止视频流...")
        self.is_running = False

        # 1. 等待发送线程结束 (可选，不阻塞清理)
        if self.send_thread:
            # 检查是否是当前线程在调用 (避免 join 自己)
            if threading.current_thread() != self.send_thread:
                try:
                    self.send_thread.join(timeout=2)
                except Exception as e:
                    logger.warning(f"等待发送线程结束失败: {e}")
            else:
                logger.debug("当前线程是发送线程，跳过 join")

        # 2. 关键: 必须终止 FFmpeg 以释放相机
        if self.ffmpeg_process:
            try:
                logger.info("正在终止 FFmpeg 进程...")
                self.ffmpeg_process.terminate()
                try:
                    self.ffmpeg_process.wait(timeout=2)
                    logger.info("FFmpeg 进程已正常终止")
                except subprocess.TimeoutExpired:
                    logger.warning("FFmpeg 未响应 terminate，强制 kill...")
                    self.ffmpeg_process.kill()
                    self.ffmpeg_process.wait(timeout=1)
                    logger.info("FFmpeg 进程已强制终止")
            except Exception as e:
                logger.error(f"终止 FFmpeg 失败: {e}")
                # 最后尝试: 强制 kill
                try:
                    self.ffmpeg_process.kill()
                except:
                    pass
            finally:
                self.ffmpeg_process = None

        # 3. 关闭 TCP socket
        if self.tcp_socket:
            try:
                self.tcp_socket.close()
            except Exception as e:
                logger.debug(f"关闭 socket: {e}")
            finally:
                self.tcp_socket = None

        # 输出完整统计
        elapsed = time.time() - self.stats['start_time'] if self.stats['start_time'] > 0 else 0
        net_stats = self.get_network_stats()

        logger.info("=" * 60)
        logger.info("视频流统计报告")
        logger.info("=" * 60)
        logger.info(f"  总时长: {elapsed:.1f} 秒")
        logger.info(f"  发送帧数: {self.stats['frames_sent']}")
        logger.info(f"  发送数据: {self.stats['bytes_sent'] / 1024 / 1024:.1f} MB")
        logger.info(f"  平均码率: {(self.stats['bytes_sent'] * 8) / (elapsed * 1_000_000):.1f} Mbps" if elapsed > 0 else "  平均码率: N/A")
        logger.info(f"  平均帧率: {self.stats['frames_sent'] / elapsed:.1f} fps" if elapsed > 0 else "  平均帧率: N/A")
        logger.info(f"  平均发送耗时: {net_stats['avg_send_time_ms']:.1f} ms")
        logger.info(f"  最大发送耗时: {net_stats['max_send_time_ms']:.1f} ms")
        logger.info("=" * 60)
        logger.info("相机资源已释放")


# ============== 测试入口 ==============

def test_simple_sender():
    """测试 TCP H.264 发送器"""
    print("=" * 60)
    print("TCP H.264 视频发送器测试")
    print("兼容 XRoboToolkit-Unity-Client MediaDecoder")
    print("=" * 60)

    config = VideoConfig(width=2560, height=720, fps=60, bitrate=20971520)
    sender = SimpleH264Sender(config)

    if not sender.initialize(device_id=0):
        print("相机初始化失败")
        return

    print("\n请在 PICO 头显上:")
    print("1. 启动 Unity Client")
    print("2. 选择 USB_STEREO 相机")
    print("3. 点击 Listen 按钮\n")

    target_ip = input("请输入 PICO IP: ").strip() or "192.168.1.100"
    target_port = int(input("请输入端口 (默认 12345): ").strip() or "12345")

    if sender.start_streaming(target_ip, target_port):
        print(f"\n正在发送视频流到 {target_ip}:{target_port}")
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
