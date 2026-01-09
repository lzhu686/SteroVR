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
        self.actual_fps = 30.0
        self.camera_name = ""  # Windows DirectShow 设备名

        # 统计信息
        self.stats = {
            'frames_sent': 0,
            'bytes_sent': 0,
            'start_time': 0.0
        }

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
        获取编码器参数

        参数说明:
        - pix_fmt yuv420p: 最通用的像素格式，所有解码器都支持
        - profile baseline: 最大兼容性，禁用 B 帧
        - level 5.1: 支持 4K@30fps 或 1080p@120fps
        - rc cbr: 恒定码率，网络传输更稳定
        - bufsize: VBV 缓冲区大小，影响延迟和码率稳定性
        - g 1: GOP=1，每帧都是关键帧，适合实时传输
        """
        if use_nvenc:
            logger.info(f"使用 NVENC 硬件编码器, 码率: {bitrate_k} kbps")
            # NVENC 参数 (FFmpeg 4.x / 5.x 兼容)
            return [
                '-pix_fmt', 'yuv420p',        # 确保解码器兼容
                '-c:v', 'h264_nvenc',
                '-preset', 'p1',               # 最低延迟 (p1-p7, p1最快)
                '-tune', 'll',                 # low latency 调优
                '-profile:v', 'baseline',      # 最大兼容性
                '-level', '5.1',
                '-rc', 'cbr',                  # 恒定码率
                '-b:v', f'{bitrate_k}k',
                '-maxrate', f'{bitrate_k}k',
                '-bufsize', f'{bitrate_k // 15}k',  # ~1.4Mbit, 低延迟关键
                '-g', '1',                     # GOP=1
                '-keyint_min', '1',
                '-delay', '0',                 # 零编码延迟
                '-zerolatency', '1',           # NVENC 零延迟模式
            ]
        else:
            logger.info(f"使用 libx264 软件编码器, 码率: {bitrate_k} kbps")
            # libx264 参数 (低延迟优化)
            #
            # preset 选择说明:
            # - ultrafast: 最快，质量最差，CPU 占用最低
            # - superfast: 质量稍好，CPU 占用略高
            # - veryfast: 平衡选择，推荐用于软件编码
            #
            # tune zerolatency 做了什么:
            # - 禁用 B 帧 (bframes=0)
            # - 禁用 lookahead
            # - 禁用帧重排序
            # - 减少编码延迟到约 1 帧
            return [
                '-pix_fmt', 'yuv420p',
                '-c:v', 'libx264',
                '-preset', 'veryfast',         # 平衡质量和速度
                '-tune', 'zerolatency',        # 禁用 B 帧和 lookahead
                '-profile:v', 'baseline',      # 禁用 CABAC，使用 CAVLC
                '-level', '5.1',
                '-b:v', f'{bitrate_k}k',
                '-maxrate', f'{bitrate_k}k',
                '-bufsize', f'{bitrate_k // 10}k',  # ~2Mbit, 软编码需要更多缓冲
                '-g', '1',
                '-keyint_min', '1',
                '-x264-params', 'repeat-headers=1:sliced-threads=1',
                # repeat-headers: 每帧包含 SPS/PPS，便于随机访问
                # sliced-threads: 切片级多线程，降低延迟
            ]

    # ============== 相机初始化 ==============

    def initialize(self, device_id: int = 0) -> bool:
        """
        初始化相机

        步骤:
        1. 用 OpenCV 测试相机是否可用
        2. 获取相机实际支持的分辨率和帧率
        3. 关闭 OpenCV，让 FFmpeg 独占相机
        4. (Windows) 获取 DirectShow 设备名称
        """
        logger.info(f"初始化相机 (设备: {device_id})...")
        self.device_id = device_id

        # 用 OpenCV 测试相机
        test_cap = cv2.VideoCapture(device_id)
        test_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        test_cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        test_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        test_cap.set(cv2.CAP_PROP_FPS, self.config.fps)

        if not test_cap.isOpened():
            logger.error("无法打开相机")
            return False

        # 获取实际参数 (相机可能不支持请求的值)
        actual_w = int(test_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(test_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.actual_fps = test_cap.get(cv2.CAP_PROP_FPS)
        logger.info(f"相机配置: {actual_w}x{actual_h} @ {self.actual_fps:.1f}fps")

        # 关闭 OpenCV，让 FFmpeg 独占
        test_cap.release()

        # Windows: 获取 DirectShow 设备名称
        if sys.platform == 'win32':
            self.camera_name = self._get_windows_camera_name(device_id)
            if not self.camera_name:
                self.camera_name = f"video={device_id}"
                logger.warning(f"无法获取相机名称，使用默认: {self.camera_name}")

        logger.info("相机初始化成功 (将使用 FFmpeg 直接采集)")
        return True

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

        # 1. TCP 连接 (带重试)
        if not self._connect_tcp(target_ip, target_port):
            return False

        # 2. 启动 FFmpeg
        if not self._start_ffmpeg():
            self.tcp_socket.close()
            return False

        # 3. 启动发送线程
        self.is_running = True
        self.stats = {'frames_sent': 0, 'bytes_sent': 0, 'start_time': time.time()}
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
                self.tcp_socket.settimeout(10)
                self.tcp_socket.connect((target_ip, target_port))
                logger.info(f"TCP 连接成功: {target_ip}:{target_port}")
                return True
            except Exception as e:
                if attempt < StreamingConstants.TCP_RETRY_COUNT - 1:
                    logger.warning(f"TCP 连接失败 ({attempt+1}/{StreamingConstants.TCP_RETRY_COUNT}): {e}")
                    time.sleep(retry_delay)
                    retry_delay *= 1.5
                else:
                    logger.error(f"TCP 连接失败，已达最大重试次数: {e}")
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
                '-i', self.camera_name,
            ]
        else:
            input_args = [
                '-f', 'v4l2',
                '-input_format', 'mjpeg',
                '-video_size', f'{self.config.width}x{self.config.height}',
                '-framerate', str(actual_fps),
                '-i', f'/dev/video{self.device_id}',
            ]

        ffmpeg_cmd = ['ffmpeg', '-y'] + input_args + encoder_args + ['-f', 'h264', '-']

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
                        self._send_frame(frame_data)
                        frame_count += 1

                        # 每秒输出统计
                        now = time.time()
                        if now - last_log_time >= 1.0:
                            elapsed = now - self.stats['start_time']
                            fps = frame_count / elapsed if elapsed > 0 else 0
                            mbps = (self.stats['bytes_sent'] * 8) / (elapsed * 1_000_000) if elapsed > 0 else 0
                            logger.info(f"已发送 {frame_count} 帧, {mbps:.1f} Mbps, {fps:.1f} fps")
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
        """
        if not self.tcp_socket or not frame_data:
            return

        try:
            packet = struct.pack('>I', len(frame_data)) + frame_data
            self.tcp_socket.sendall(packet)

            self.stats['frames_sent'] += 1
            self.stats['bytes_sent'] += len(packet)

            # 前几帧输出调试信息
            if self.stats['frames_sent'] <= 3:
                hex_preview = frame_data[:20].hex()
                logger.info(f"[Frame {self.stats['frames_sent']}] {len(frame_data)} bytes, {hex_preview}...")

        except Exception as e:
            logger.error(f"发送帧失败: {e}")
            self.is_running = False

    # ============== 清理 ==============

    def stop_streaming(self):
        """停止传输并清理资源"""
        logger.info("正在停止视频流...")
        self.is_running = False

        if self.send_thread:
            self.send_thread.join(timeout=2)

        if self.ffmpeg_process:
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

        elapsed = time.time() - self.stats['start_time'] if self.stats['start_time'] > 0 else 0
        logger.info(f"视频流已停止. 统计: {self.stats['frames_sent']} 帧, "
                   f"{self.stats['bytes_sent'] / 1024 / 1024:.1f} MB, {elapsed:.1f}s")


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
