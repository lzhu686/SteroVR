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
    """视频配置"""
    width: int = 1920           # 双目拼接宽度 (单眼960)
    height: int = 1080          # 高度
    fps: int = 30               # 帧率 (30fps降低编码压力)
    bitrate: int = 15000000     # 码率 (15 Mbps，减少块效应)
    keyframe_interval: int = 30 # 关键帧间隔

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

    def initialize(self, device_id: int = 0) -> bool:
        """初始化相机"""
        logger.info(f"初始化相机 (设备: {device_id})...")

        self.cap = cv2.VideoCapture(device_id)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.config.fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            logger.error("无法打开相机")
            return False

        # 验证实际参数
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        logger.info(f"相机配置: {actual_w}x{actual_h} @ {actual_fps:.1f}fps")

        logger.info("相机初始化成功")
        return True

    def start_streaming(self, target_ip: str, target_port: int) -> bool:
        """
        开始流式传输 (TCP H.264)

        工作流程:
        1. 连接到 Unity Client 的 MediaDecoder TCP 服务
        2. 使用 FFmpeg 编码 H.264
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

        # FFmpeg 命令 - 输出 H.264 annex-b 格式到 stdout
        # 优化参数：CBR恒定码率 + 低延迟 + 高质量
        bitrate_k = self.config.bitrate // 1000
        ffmpeg_cmd = [
            'ffmpeg',
            '-y',
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-pix_fmt', 'bgr24',
            '-s', f'{self.config.width}x{self.config.height}',
            '-r', str(self.config.fps),
            '-i', '-',
            '-vf', 'format=yuv420p',
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-profile:v', 'baseline',
            '-level', '4.1',
            # CBR 恒定码率配置
            '-b:v', f'{bitrate_k}k',
            '-maxrate', f'{bitrate_k}k',
            '-minrate', f'{bitrate_k}k',
            '-bufsize', f'{bitrate_k // 2}k',  # 缓冲区=码率的一半，降低延迟
            # 关键帧配置
            '-g', '1',
            '-keyint_min', '1',
            '-x264-params', 'repeat-headers=1:nal-hrd=cbr',  # CBR + 每帧带SPS/PPS
            '-f', 'h264',
            '-'
        ]

        try:
            logger.info(f"启动 FFmpeg 编码器...")
            logger.info(f"FFmpeg 命令: {' '.join(ffmpeg_cmd)}")
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE  # 捕获错误输出以便调试
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

        # 启动采集线程 (写入 FFmpeg stdin)
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        # 启动发送线程 (从 FFmpeg stdout 读取并发送)
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
