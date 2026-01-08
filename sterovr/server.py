#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USB双目立体相机 WebSocket 服务器 (SSL/WSS 版本)
支持 HTTPS/WSS 远程访问，让其他设备通过网络访问立体视觉流

功能特性:
1. 支持 SSL/TLS 加密的 WebSocket (WSS)
2. 实时获取USB双目相机数据
3. 分割左右图像并通过 WSS 传输
4. 支持多客户端连接
5. 自动使用项目根目录的 SSL 证书

技术架构:
┌─────────────────────────────────────────────────────────────┐
│  USB双目相机 → Python服务器(WSS) → VR浏览器(HTTPS)           │
│                                                             │
│  本地设备:                                                   │
│    python usb_stereo_websocket_server_ssl.py                │
│                                                             │
│  远程设备访问:                                               │
│    1. 浏览器打开: https://你的IP:8445/                       │
│    2. 信任证书后访问 dual_infrared_vr_viewer.html            │
│    3. WebSocket 自动连接 wss://你的IP:8765                   │
└─────────────────────────────────────────────────────────────┘

依赖安装: pip install opencv-python websockets numpy

作者: Liang ZHU
邮箱: lzhu686@connect.hkust-gz.edu.cn
日期: 2025
"""

import asyncio
import websockets
import json
import cv2
import numpy as np
import base64
import time
import logging
import threading
from typing import Optional, Tuple, Set
import ssl
import os
import sys
import argparse
import socket

# ==================== 相机配置参数 (可直接修改) ====================
STEREO_WIDTH = 2560     # 双目拼接图像宽度
STEREO_HEIGHT = 720     # 双目拼接图像高度
CAMERA_WIDTH = 1280     # 单目图像宽度 (STEREO_WIDTH/2)
CAMERA_HEIGHT = 720     # 单目图像高度
TARGET_FPS = 60         # 目标帧率
JPEG_QUALITY = 85       # JPEG压缩质量 (优化: 100→85, 提升编码速度)
CAMERA_BUFFERSIZE = 1   # 相机缓冲区大小 (减少延迟)

# USB带宽优化设置
USE_MJPG_FORMAT = True     # 使用MJPG格式
FORCE_USB3_MODE = True     # 强制USB3.0模式
AUTO_EXPOSURE = False      # 自动曝光
AUTO_WHITE_BALANCE = True  # 自动白平衡

# 立体校正参数
ENABLE_RECTIFY = False  # 是否启用立体校正
BASELINE_MM = 60.0      # 基线距离(毫米)
# ================================================================

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


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


def get_ssl_context():
    """获取SSL上下文，使用项目根目录的证书"""
    # 查找证书文件
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(script_dir))  # 向上两级到项目根目录

    cert_locations = [
        # 优先使用项目根目录的证书
        (os.path.join(project_root, "webxr_cert.pem"), os.path.join(project_root, "webxr_key.pem")),
        # 当前目录
        (os.path.join(script_dir, "server.crt"), os.path.join(script_dir, "server.key")),
        # 标准位置
        ("server.crt", "server.key"),
    ]

    for cert_file, key_file in cert_locations:
        if os.path.exists(cert_file) and os.path.exists(key_file):
            logger.info(f"🔐 找到SSL证书: {cert_file}")
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(cert_file, key_file)
            return ssl_context

    logger.warning("⚠️ 未找到SSL证书，将尝试自动生成...")
    return generate_self_signed_cert(script_dir)


def generate_self_signed_cert(output_dir):
    """自动生成自签名证书"""
    import subprocess

    cert_file = os.path.join(output_dir, "server.crt")
    key_file = os.path.join(output_dir, "server.key")

    try:
        cmd = [
            'openssl', 'req', '-x509', '-newkey', 'rsa:4096',
            '-keyout', key_file, '-out', cert_file,
            '-days', '365', '-nodes',
            '-subj', '/C=CN/ST=Beijing/L=Beijing/O=WebXR/CN=localhost'
        ]

        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            logger.info(f"✅ 自签名证书已生成: {cert_file}")
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(cert_file, key_file)
            return ssl_context
        else:
            logger.error(f"❌ 证书生成失败: {result.stderr}")
            return None
    except Exception as e:
        logger.error(f"❌ 无法生成证书: {e}")
        return None


class USBStereoWebSocketServerSSL:
    """USB双目立体相机WebSocket服务器 (SSL版本)"""

    def __init__(self, host="0.0.0.0", port=8765, use_ssl=True):
        """
        初始化WebSocket服务器

        Args:
            host: 服务器地址
            port: 服务器端口
            use_ssl: 是否使用SSL加密
        """
        self.host = host
        self.port = port
        self.use_ssl = use_ssl
        self.camera_index = 0
        self.enable_rectify = ENABLE_RECTIFY

        # 相机配置
        self.stereo_width = STEREO_WIDTH
        self.stereo_height = STEREO_HEIGHT
        self.camera_width = CAMERA_WIDTH
        self.camera_height = CAMERA_HEIGHT
        self.target_fps = TARGET_FPS

        # 立体校正配置
        self.stereo_config = None
        if self.enable_rectify:
            self.stereo_config = {
                'baseline_mm': BASELINE_MM,
                'camera_width': self.camera_width,
                'camera_height': self.camera_height
            }

        # WebSocket连接管理
        self.connected_clients: Set[websockets.WebSocketServerProtocol] = set()
        self.client_count = 0

        # 相机管理
        self.cap = None
        self.is_camera_running = False
        self.camera_thread = None
        self.frame_lock = threading.Lock()
        self.latest_frames = (None, None)
        self.current_frame_id = 0  # 添加帧序列号

        # 测试模式
        self.test_mode = False

        # 性能统计
        self.stats = {
            'frames_captured': 0,
            'frames_sent': 0,
            'bytes_sent': 0,
            'start_time': time.time(),
            'last_frame_time': 0,
            'fps_actual': 0,
            'fps_sent': 0,  # 添加发送帧率统计
            'compression_ratio': 0,
            'last_fps_calc_time': time.time(),  # FPS计算时间戳
            'frames_sent_last_second': 0  # 上一秒发送的帧数
        }

        # 图像编码配置
        self.encoding_params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]

    def initialize_camera(self) -> bool:
        """初始化USB双目相机"""
        logger.info(f"初始化USB双目相机 (设备索引: {self.camera_index})...")

        try:
            self.cap = cv2.VideoCapture(0)

            if USE_MJPG_FORMAT:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                logger.info("✅ 已设置MJPG格式")

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.stereo_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.stereo_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)

            if not self.cap.isOpened():
                logger.warning(f"无法打开相机设备0，启用测试模式")
                self.test_mode = True
                return True

            if not AUTO_EXPOSURE:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)

            if AUTO_WHITE_BALANCE:
                self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)

            if FORCE_USB3_MODE:
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFERSIZE)

            # 获取实际配置
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            actual_fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
            fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])

            logger.info(f"✅ 相机配置成功: {actual_width}x{actual_height} @ {actual_fps:.1f}fps")
            logger.info(f"📹 编码格式: {fourcc_str}")
            logger.info(f"🔧 目标配置: {self.stereo_width}x{self.stereo_height} @ {self.target_fps}fps")

            # 性能警告
            if actual_fps < self.target_fps * 0.5:
                logger.warning(f"⚠️ 相机实际帧率({actual_fps:.1f})远低于目标({self.target_fps})，可能受硬件限制")
                logger.warning(f"💡 建议: 1) 检查USB连接(使用USB3.0); 2) 降低分辨率; 3) 降低JPEG_QUALITY")

            # 测试读取
            ret, frame = self.cap.read()
            if ret and frame is not None:
                logger.info(f"✅ 图像读取测试成功")
                return True
            else:
                logger.warning("图像读取测试失败，启用测试模式")
                self.test_mode = True
                return True

        except Exception as e:
            logger.warning(f"相机初始化失败，启用测试模式: {e}")
            self.test_mode = True
            return True

    def generate_test_frames(self):
        """生成测试帧"""
        left_test = np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)
        right_test = np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)

        left_test[:, :] = (0, 100, 200)
        right_test[:, :] = (200, 100, 0)

        timestamp = time.strftime("%H:%M:%S", time.localtime())
        cv2.putText(left_test, f"LEFT TEST", (50, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        cv2.putText(left_test, f"{timestamp}", (50, 200),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(left_test, f"WSS Mode", (50, 300),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.putText(right_test, f"RIGHT TEST", (50, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        cv2.putText(right_test, f"{timestamp}", (50, 200),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(right_test, f"WSS Mode", (50, 300),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        return left_test, right_test

    def camera_thread_function(self):
        """相机数据获取线程"""
        logger.info("启动相机数据获取线程")
        frame_interval = 1.0 / self.target_fps
        frame_count = 0
        fps_start_time = time.time()

        # 性能监控
        total_capture_time = 0
        capture_count = 0

        while self.is_camera_running:
            try:
                loop_start = time.time()

                if self.test_mode:
                    left_image, right_image = self.generate_test_frames()
                else:
                    capture_start = time.time()
                    ret, stereo_frame = self.cap.read()
                    capture_time = time.time() - capture_start
                    total_capture_time += capture_time
                    capture_count += 1

                    if ret and stereo_frame is not None:
                        height = stereo_frame.shape[0]
                        left_image = stereo_frame[0:height, 0:self.camera_width]
                        right_image = stereo_frame[0:height, self.camera_width:self.stereo_width]
                    else:
                        time.sleep(0.001)
                        continue

                with self.frame_lock:
                    self.latest_frames = (left_image.copy(), right_image.copy())
                    self.current_frame_id += 1  # 每捕获一帧就递增帧ID

                self.stats['frames_captured'] += 1
                self.stats['last_frame_time'] = time.time()
                frame_count += 1

                if frame_count % 60 == 0:
                    current_time = time.time()
                    actual_fps = 60.0 / (current_time - fps_start_time)
                    self.stats['fps_actual'] = actual_fps
                    fps_start_time = current_time

                    # 性能诊断
                    if not self.test_mode and capture_count > 0:
                        avg_capture_time = (total_capture_time / capture_count) * 1000
                        logger.info(f"📊 采集性能: FPS={actual_fps:.1f}, 平均采集耗时={avg_capture_time:.1f}ms")
                        total_capture_time = 0
                        capture_count = 0

                elapsed = time.time() - loop_start
                if elapsed < frame_interval:
                    sleep_time = frame_interval - elapsed
                    if sleep_time > 0.001:
                        time.sleep(sleep_time)

            except Exception as e:
                logger.warning(f"获取帧失败: {e}")
                time.sleep(0.01)

        logger.info("相机数据获取线程结束")

    def encode_images(self, left_image: np.ndarray, right_image: np.ndarray,
                     quality: int = 80) -> Tuple[str, str, dict]:
        """编码图像为Base64字符串"""
        encode_start = time.time()

        try:
            if len(left_image.shape) == 2:
                left_bgr = cv2.cvtColor(left_image, cv2.COLOR_GRAY2BGR)
            else:
                left_bgr = left_image

            if len(right_image.shape) == 2:
                right_bgr = cv2.cvtColor(right_image, cv2.COLOR_GRAY2BGR)
            else:
                right_bgr = right_image

            encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]

            _, left_buffer = cv2.imencode('.jpg', left_bgr, encode_params)
            _, right_buffer = cv2.imencode('.jpg', right_bgr, encode_params)

            left_base64 = base64.b64encode(left_buffer).decode('utf-8')
            right_base64 = base64.b64encode(right_buffer).decode('utf-8')

            original_size = left_image.size + right_image.size
            compressed_size = len(left_buffer) + len(right_buffer)
            compression_ratio = compressed_size / original_size

            encode_time = (time.time() - encode_start) * 1000  # 转换为毫秒

            metadata = {
                'width': left_image.shape[1],
                'height': left_image.shape[0],
                'quality': quality,
                'compression_ratio': compression_ratio,
                'compressed_size': compressed_size,
                'timestamp': time.time(),
                'rectified': self.enable_rectify,
                'encode_time_ms': round(encode_time, 2)  # 添加编码耗时
            }

            self.stats['compression_ratio'] = compression_ratio

            return left_base64, right_base64, metadata

        except Exception as e:
            logger.error(f"图像编码失败: {e}")
            return "", "", {}

    def adaptive_quality_adjustment(self, client_count: int,
                                  last_compression_ratio: float) -> int:
        """自适应质量调整"""
        base_quality = 85

        if client_count <= 1:
            quality = base_quality
        elif client_count <= 2:
            quality = base_quality - 5
        else:
            quality = base_quality - 15

        if last_compression_ratio > 0.8:
            quality = max(40, quality - 10)
        elif last_compression_ratio < 0.3:
            quality = min(95, quality + 5)

        return max(40, min(95, quality))

    async def handle_client(self, websocket):
        """处理客户端连接"""
        client_addr = websocket.remote_address
        logger.info(f"🔗 新VR客户端连接: {client_addr}")

        self.connected_clients.add(websocket)
        self.client_count = len(self.connected_clients)

        try:
            # 发送连接确认
            welcome_msg = {
                'type': 'connection_established',
                'camera_info': {
                    'stereo_width': self.stereo_width,
                    'stereo_height': self.stereo_height,
                    'width': self.camera_width,  # 修复字段名
                    'height': self.camera_height,  # 修复字段名
                    'camera_width': self.camera_width,  # 保留兼容性
                    'camera_height': self.camera_height,  # 保留兼容性
                    'fps': self.target_fps,
                    'format': 'usb_stereo_rgb',
                    'rectified': self.enable_rectify,
                    'baseline_mm': self.stereo_config['baseline_mm'] if self.stereo_config else 60.0
                },
                'server_info': {
                    'version': '1.0.0-ssl',
                    'features': ['usb_stereo', 'ssl_encrypted', 'wss', 'vr_optimized'],
                    'mode': 'test' if self.test_mode else 'real_camera',
                    'protocol': 'wss' if self.use_ssl else 'ws'
                }
            }
            await websocket.send(json.dumps(welcome_msg))

            logger.info(f"✅ 已向客户端发送配置信息")

            # 主循环
            frame_interval = 1.0 / self.target_fps
            last_send_time = 0
            last_sent_frame_id = -1  # 跟踪上次发送的帧ID
            fps_counter = 0  # 每秒发送帧计数
            fps_timer = time.time()

            while True:
                current_time = time.time()

                # 帧率限制
                if current_time - last_send_time < frame_interval:
                    await asyncio.sleep(0.005)
                    continue

                # 获取最新帧和帧ID
                with self.frame_lock:
                    left_image, right_image = self.latest_frames
                    current_frame_id = self.current_frame_id

                # 跳过重复帧：如果帧ID与上次发送的相同，说明没有新帧
                if current_frame_id == last_sent_frame_id:
                    await asyncio.sleep(0.001)
                    continue

                if left_image is not None and right_image is not None:
                    quality = self.adaptive_quality_adjustment(
                        self.client_count,
                        self.stats['compression_ratio']
                    )

                    left_b64, right_b64, metadata = self.encode_images(
                        left_image, right_image, quality
                    )

                    if left_b64 and right_b64:
                        message = {
                            'type': 'dual_infrared_frame',
                            'timestamp': current_time,
                            'frame_id': current_frame_id,  # 添加真实帧ID
                            'left_infrared': left_b64,
                            'right_infrared': right_b64,
                            'metadata': metadata,
                            'stats': {
                                'fps_actual': round(self.stats['fps_actual'], 1),
                                'fps_sent': round(self.stats.get('fps_sent', 0), 1),  # 添加发送FPS
                                'frames_captured': self.stats['frames_captured'],
                                'frames_sent': self.stats['frames_sent'],
                                'client_count': self.client_count,
                                'compression_ratio': round(self.stats['compression_ratio'], 3),
                                'mode': 'test' if self.test_mode else 'camera',
                                'protocol': 'wss' if self.use_ssl else 'ws'
                            }
                        }

                        await websocket.send(json.dumps(message))

                        self.stats['frames_sent'] += 1
                        self.stats['bytes_sent'] += len(json.dumps(message))
                        last_send_time = current_time
                        last_sent_frame_id = current_frame_id  # 记录已发送的帧ID
                        fps_counter += 1

                        # 每秒计算一次发送FPS
                        if current_time - fps_timer >= 1.0:
                            self.stats['fps_sent'] = fps_counter / (current_time - fps_timer)
                            fps_counter = 0
                            fps_timer = current_time

                try:
                    pong_waiter = await websocket.ping()
                    await asyncio.wait_for(pong_waiter, timeout=0.5)
                except:
                    break

        except websockets.exceptions.ConnectionClosed:
            logger.info(f"🔌 VR客户端断开连接: {client_addr}")
        except Exception as e:
            logger.error(f"❌ 处理VR客户端出错 {client_addr}: {e}")
        finally:
            self.connected_clients.discard(websocket)
            self.client_count = len(self.connected_clients)
            logger.info(f"📊 剩余连接: {self.client_count}")

    async def stats_reporter(self):
        """统计信息报告器"""
        while True:
            await asyncio.sleep(10)

            runtime = time.time() - self.stats['start_time']

            logger.info("=" * 50)
            logger.info(f"🔐 USB双目相机WSS服务器统计")
            logger.info(f"⏱️  运行时间: {runtime:.1f}秒")
            logger.info(f"🔒 协议: {'WSS (加密)' if self.use_ssl else 'WS'}")
            logger.info(f"📹 模式: {'测试模式' if self.test_mode else '真实相机'}")
            logger.info(f"🖥️  VR客户端: {self.client_count}")
            logger.info(f"📊 捕获/发送: {self.stats['frames_captured']}/{self.stats['frames_sent']}")
            logger.info(f"⚡ 实际FPS: {self.stats['fps_actual']:.1f}")
            logger.info(f"💾 数据传输: {self.stats['bytes_sent'] / 1024 / 1024:.1f} MB")
            logger.info("=" * 50)

    async def start_server(self):
        """启动WebSocket服务器"""
        # 初始化相机
        if not self.initialize_camera():
            logger.error("相机初始化失败")
            return

        # 启动相机线程
        self.is_camera_running = True
        self.camera_thread = threading.Thread(target=self.camera_thread_function)
        self.camera_thread.daemon = True
        self.camera_thread.start()

        await asyncio.sleep(2)

        # SSL配置
        ssl_context = None
        if self.use_ssl:
            ssl_context = get_ssl_context()
            if ssl_context is None:
                logger.error("❌ 无法获取SSL证书，服务器启动失败")
                return

        protocol = "wss" if ssl_context else "ws"
        local_ip = get_local_ip()

        print("\n" + "=" * 60)
        print("🚀 USB双目相机 WebSocket SSL 服务器启动成功!")
        print("=" * 60)
        print(f"🔐 协议: {protocol.upper()} ({'加密' if ssl_context else '未加密'})")
        print(f"📡 本地访问: {protocol}://localhost:{self.port}")
        print(f"🌐 局域网访问: {protocol}://{local_ip}:{self.port}")
        print(f"📹 模式: {'测试模式' if self.test_mode else '真实相机模式'}")
        print(f"🎯 分辨率: {self.stereo_width}x{self.stereo_height}")
        print(f"⚡ 目标帧率: {self.target_fps}fps")
        print("=" * 60)
        print("\n📱 其他设备访问方法:")
        print(f"   1. 在目标设备浏览器打开: https://{local_ip}:8445/")
        print(f"   2. 信任自签名证书")
        print(f"   3. 打开 dual_infrared_vr_viewer.html")
        print(f"   4. 页面会自动连接到 {protocol}://{local_ip}:{self.port}")
        print("=" * 60 + "\n")

        # 启动统计报告器
        stats_task = asyncio.create_task(self.stats_reporter())

        # 启动服务器
        server = await websockets.serve(
            self.handle_client,
            self.host,
            self.port,
            ssl=ssl_context,
            ping_interval=20,
            ping_timeout=10
        )

        logger.info("WebSocket服务器已启动，等待VR客户端连接...")

        await server.wait_closed()

    def cleanup(self):
        """清理资源"""
        logger.info("正在清理资源...")
        self.is_camera_running = False

        if self.camera_thread:
            self.camera_thread.join(timeout=2)

        if self.cap and not self.test_mode:
            try:
                self.cap.release()
            except:
                pass

        logger.info("资源清理完成")


async def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='USB双目相机WebSocket SSL服务器')
    parser.add_argument('--port', '-p', type=int, default=8765,
                       help='WebSocket端口 (默认: 8765)')
    parser.add_argument('--no-ssl', action='store_true',
                       help='禁用SSL加密 (仅用于调试)')
    parser.add_argument('--host', default='0.0.0.0',
                       help='服务器地址 (默认: 0.0.0.0)')

    args = parser.parse_args()

    server = USBStereoWebSocketServerSSL(
        host=args.host,
        port=args.port,
        use_ssl=not args.no_ssl
    )

    try:
        await server.start_server()
    except KeyboardInterrupt:
        logger.info("收到中断信号")
    finally:
        server.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
