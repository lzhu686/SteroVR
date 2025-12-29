#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
USB双目立体相机 WebSocket 服务器
实时传输左右双目图像到VR设备 (独立配置版本)

功能特性:
1. 实时获取USB双目相机数据 (可配置分辨率和帧率)
2. 分割左右图像
3. 通过WebSocket传输原始图像数据到VR浏览器
4. 支持多客户端连接
5. 实时性能监控和自适应质量调整
6. 专为VR视频透视设计

技术架构:
- USB双目相机 → Python后端 → WebSocket → VR前端
- 支持Base64编码传输
- 自适应分辨率和帧率调整
- 低延迟优化

依赖安装: pip install opencv-python websockets numpy

作者: lucas ZHU
日期: 2025年8月13日
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

# ==================== 相机配置参数 (可直接修改) ====================
# USB双目相机配置 - 针对设备0的实际能力调整
STEREO_WIDTH = 2560     # 双目拼接图像宽度 (降低以适应设备0)
STEREO_HEIGHT = 720     # 双目拼接图像高度 (降低以适应设备0)
CAMERA_WIDTH = 1280     # 单目图像宽度 (STEREO_WIDTH/2)
CAMERA_HEIGHT = 720     # 单目图像高度
TARGET_FPS = 60         # 目标帧率 (根据实际情况调整) 📌 降低到15以匹配实际性能
JPEG_QUALITY = 100       # JPEG压缩质量 (降低以提高性能)
CAMERA_BUFFERSIZE = 1   # 相机缓冲区大小 (减少延迟)

# USB带宽优化设置 - 解决帧率减半问题
USE_MJPG_FORMAT = True     # 使用MJPG格式 (建议开启，减少带宽)
FORCE_USB3_MODE = True     # 强制USB3.0模式
AUTO_EXPOSURE = False      # 自动曝光 (关闭可提高帧率稳定性)
AUTO_WHITE_BALANCE = True  # 自动白平衡 (启用以获得更好色彩)

# 立体校正参数 (如果需要校正，可在这里设置)
ENABLE_RECTIFY = False  # 是否启用立体校正
BASELINE_MM = 60.0      # 基线距离(毫米)

# 📖 调参指南:
# 1. 如果实际帧率是目标帧率的一半：
#    - 降低 TARGET_FPS (60→30, 30→15)
#    - 降低 JPEG_QUALITY (85→70)
#    - 确保USB连接是3.0
# 2. 如果图像质量不够：
#    - 提高 JPEG_QUALITY (85→95)
#    - 但可能影响帧率
# 3. 如果延迟太高：
#    - 保持 CAMERA_BUFFERSIZE = 1
#    - 关闭 AUTO_EXPOSURE 和 AUTO_WHITE_BALANCE
# ================================================================

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

logger.info(f"📹 相机配置: {STEREO_WIDTH}x{STEREO_HEIGHT}@{TARGET_FPS}fps")
logger.info(f"🎯 单目尺寸: {CAMERA_WIDTH}x{CAMERA_HEIGHT}")
logger.info(f"⚡ JPEG质量: {JPEG_QUALITY}%, 缓冲区: {CAMERA_BUFFERSIZE}")

def get_camera_config_info():
    """获取相机配置信息，供外部脚本使用"""
    return {
        'stereo_width': STEREO_WIDTH,
        'stereo_height': STEREO_HEIGHT,
        'camera_width': CAMERA_WIDTH,
        'camera_height': CAMERA_HEIGHT,
        'target_fps': TARGET_FPS,
        'jpeg_quality': JPEG_QUALITY,
        'enable_rectify': ENABLE_RECTIFY,
        'camera_index': 0  # 固定使用设备0
    }

class USBStereoWebSocketServer:
    """USB双目立体相机WebSocket服务器"""
    
    def __init__(self, host="0.0.0.0", port=8765, use_ssl=False):
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
        self.camera_index = 0  # 固定使用设备0
        self.enable_rectify = ENABLE_RECTIFY  # 直接使用配置常量
        
        # USB双目相机配置 (直接使用文件顶部的配置常量)
        self.stereo_width = STEREO_WIDTH
        self.stereo_height = STEREO_HEIGHT
        self.camera_width = CAMERA_WIDTH
        self.camera_height = CAMERA_HEIGHT
        self.target_fps = TARGET_FPS
        
        # 立体校正配置 (简化版，不依赖外部文件)
        self.stereo_config = None
        if self.enable_rectify:
            logger.info("⚡ 立体校正: 启用 (简化配置)")
            logger.info(f"   基线距离: {BASELINE_MM:.1f}mm")
            logger.info(f"   图像尺寸: {self.camera_width}x{self.camera_height}")
            # 创建简化的立体校正配置
            self.stereo_config = {
                'baseline_mm': BASELINE_MM,
                'camera_width': self.camera_width,
                'camera_height': self.camera_height
            }
        else:
            logger.info("⚡ 立体校正: 禁用 (原始图像)")

        # WebSocket连接管理
        self.connected_clients: Set[websockets.WebSocketServerProtocol] = set()
        self.client_count = 0
        
        # 相机管理
        self.cap = None
        self.is_camera_running = False
        self.camera_thread = None
        self.frame_lock = threading.Lock()
        self.latest_frames = (None, None)
        
        # 测试模式（当没有真实设备时）
        self.test_mode = False
        
        # 性能统计
        self.stats = {
            'frames_captured': 0,
            'frames_sent': 0,
            'bytes_sent': 0,
            'start_time': time.time(),
            'last_frame_time': 0,
            'fps_actual': 0,
            'compression_ratio': 0
        }
        
        # 图像编码配置
        self.encoding_params = [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
        self.max_image_size = 1024 * 512  # 最大图像大小 512KB
        
    def initialize_camera(self) -> bool:
        """初始化USB双目相机 - 简化版本，固定使用设备0"""
        logger.info(f"初始化USB双目相机 (设备索引: {self.camera_index})...")
        
        try:
            # 创建VideoCapture对象 - 直接使用设备0
            self.cap = cv2.VideoCapture(0)
            
            # 第一步：设置MJPG格式
            if USE_MJPG_FORMAT:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                logger.info("✅ 已设置MJPG格式")
            
            # 第二步：设置分辨率和帧率
            logger.info(f"🎯 设置目标分辨率: {self.stereo_width}x{self.stereo_height}")
            logger.info(f"🎯 设置目标帧率: {self.target_fps}fps")
            
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.stereo_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.stereo_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.target_fps)
            
            # 第三步：检查相机是否可用
            if not self.cap.isOpened():
                logger.warning(f"无法打开相机设备0，启用测试模式")
                self.test_mode = True
                return True
            
            # 第四步：设置其他优化参数
            if not AUTO_EXPOSURE:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 禁用自动曝光
                logger.info("✅ 已禁用自动曝光")
            
            if AUTO_WHITE_BALANCE:
                self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)  # 启用自动白平衡
                logger.info("✅ 已启用自动白平衡")
            else:
                self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)  # 禁用自动白平衡
                logger.info("✅ 已禁用自动白平衡")
            
            # USB优化设置
            if FORCE_USB3_MODE:
                self.cap.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFERSIZE)
                logger.info(f"✅ 设置缓冲区大小: {CAMERA_BUFFERSIZE}")
            
            # 获取实际配置
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            
            logger.info(f"✅ 相机配置成功:")
            logger.info(f"   设备索引: {self.camera_index}")
            logger.info(f"   分辨率: {actual_width}x{actual_height}")
            logger.info(f"   帧率: {actual_fps:.1f} fps")
            logger.info(f"   单目尺寸: {self.camera_width}x{self.camera_height}")
            
            # 测试读取
            ret, frame = self.cap.read()
            if ret and frame is not None:
                h, w = frame.shape[:2]
                logger.info(f"✅ 图像读取测试成功: {w}x{h}")
                return True
            else:
                logger.error("❌ 图像读取测试失败")
                return False
            
        except Exception as e:
            logger.warning(f"相机初始化失败，启用测试模式: {e}")
            self.test_mode = True
            return True
    
    def generate_test_frames(self):
        """生成测试帧 (测试模式)"""
        # 创建测试图像
        left_test = np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)
        right_test = np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)
        
        # 添加不同的颜色以区分左右
        left_test[:, :] = (0, 100, 200)  # 橙色调
        right_test[:, :] = (200, 100, 0)  # 蓝色调
        
        # 添加时间戳和标识
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        cv2.putText(left_test, f"LEFT TEST", (50, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        cv2.putText(left_test, f"{timestamp}", (50, 200), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        cv2.putText(right_test, f"RIGHT TEST", (50, 100), 
                   cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3)
        cv2.putText(right_test, f"{timestamp}", (50, 200), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return left_test, right_test
    
    def camera_thread_function(self):
        """相机数据获取线程 - 优化版本，解决帧率减半问题"""
        logger.info("启动相机数据获取线程")
        frame_interval = 1.0 / self.target_fps
        last_frame_time = time.time()
        frame_count = 0
        fps_start_time = time.time()
        
        while self.is_camera_running:
            try:
                loop_start = time.time()
                
                if self.test_mode:
                    # 测试模式：生成虚拟图像
                    left_image, right_image = self.generate_test_frames()
                else:
                    # 真实相机模式 - 优化读取
                    ret, stereo_frame = self.cap.read()
                    
                    if ret and stereo_frame is not None:
                        # 分割左右图像
                        height = stereo_frame.shape[0]
                        
                        # 左图像：左半部分
                        left_image = stereo_frame[0:height, 0:self.camera_width]
                        
                        # 右图像：右半部分  
                        right_image = stereo_frame[0:height, self.camera_width:self.stereo_width]
                        
                        # 立体校正处理 (简化版本，无外部依赖)
                        if self.enable_rectify:
                            # 这里可以添加简单的立体校正逻辑
                            # 目前跳过，直接使用原始图像
                            pass
                    else:
                        # 读取失败，跳过此帧但不增加过多延迟
                        time.sleep(0.001)  # 1ms延迟
                        continue
                
                # 更新最新帧
                with self.frame_lock:
                    self.latest_frames = (left_image.copy(), right_image.copy())
                
                # 更新统计
                self.stats['frames_captured'] += 1
                self.stats['last_frame_time'] = time.time()
                frame_count += 1
                
                # 每60帧计算一次实际FPS，减少警告频率
                if frame_count % 60 == 0:
                    current_time = time.time()
                    actual_fps = 60.0 / (current_time - fps_start_time)
                    self.stats['fps_actual'] = actual_fps
                    fps_start_time = current_time
                    
                    # 只在明显偏低且每分钟最多输出一次警告
                    if actual_fps < self.target_fps * 0.6 and frame_count % 1800 == 0:  # 每30秒检查一次
                        logger.warning(f"⚠️ 实际FPS({actual_fps:.1f}) 明显低于目标({self.target_fps})")
                        logger.warning(f"💡 建议: 1)降低目标帧率 2)降低JPEG质量 3)检查USB3.0连接")
                
                # 精确的帧率控制 - 减少不必要的sleep
                elapsed = time.time() - loop_start
                if elapsed < frame_interval:
                    sleep_time = frame_interval - elapsed
                    if sleep_time > 0.001:  # 只有当需要sleep超过1ms时才执行
                        time.sleep(sleep_time)
                    
            except Exception as e:
                logger.warning(f"获取帧失败: {e}")
                time.sleep(0.01)  # 发生错误时的短暂延迟
        
        logger.info("相机数据获取线程结束")
    
    def encode_images(self, left_image: np.ndarray, right_image: np.ndarray, 
                     quality: int = 80) -> Tuple[str, str, dict]:
        """
        编码图像为Base64字符串
        
        Args:
            left_image: 左目图像
            right_image: 右目图像
            quality: JPEG压缩质量 (1-100)
            
        Returns:
            (left_base64, right_base64, metadata)
        """
        try:
            # 确保是3通道彩色图像
            if len(left_image.shape) == 2:
                left_bgr = cv2.cvtColor(left_image, cv2.COLOR_GRAY2BGR)
            else:
                left_bgr = left_image
                
            if len(right_image.shape) == 2:
                right_bgr = cv2.cvtColor(right_image, cv2.COLOR_GRAY2BGR)
            else:
                right_bgr = right_image
            
            # JPEG编码
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, quality]
            
            _, left_buffer = cv2.imencode('.jpg', left_bgr, encode_params)
            _, right_buffer = cv2.imencode('.jpg', right_bgr, encode_params)
            
            # Base64编码
            left_base64 = base64.b64encode(left_buffer).decode('utf-8')
            right_base64 = base64.b64encode(right_buffer).decode('utf-8')
            
            # 计算压缩率
            original_size = left_image.size + right_image.size
            compressed_size = len(left_buffer) + len(right_buffer)
            compression_ratio = compressed_size / original_size
            
            metadata = {
                'width': left_image.shape[1],
                'height': left_image.shape[0],
                'quality': quality,
                'original_size': original_size,
                'compressed_size': compressed_size,
                'compression_ratio': compression_ratio,
                'timestamp': time.time(),
                'rectified': self.enable_rectify  # 标识：是否进行立体校正
            }
            
            self.stats['compression_ratio'] = compression_ratio
            
            return left_base64, right_base64, metadata
            
        except Exception as e:
            logger.error(f"图像编码失败: {e}")
            return "", "", {}
    
    def adaptive_quality_adjustment(self, client_count: int, 
                                  last_compression_ratio: float) -> int:
        """
        自适应质量调整
        
        Args:
            client_count: 连接的客户端数量
            last_compression_ratio: 上次压缩率
            
        Returns:
            调整后的JPEG质量
        """
        base_quality = 85  # 提高基础质量，因为VR需要更好的视觉效果
        
        # 根据客户端数量调整
        if client_count <= 1:
            quality = base_quality
        elif client_count <= 2:
            quality = base_quality - 5
        else:
            quality = base_quality - 15
        
        # 根据压缩率调整
        if last_compression_ratio > 0.8:  # 压缩率过高，降低质量
            quality = max(40, quality - 10)
        elif last_compression_ratio < 0.3:  # 压缩率过低，提高质量
            quality = min(95, quality + 5)
        
        return max(40, min(95, quality))
    
    async def handle_client(self, websocket):
        """处理客户端连接"""
        client_addr = websocket.remote_address
        logger.info(f"新VR客户端连接: {client_addr}")
        
        self.connected_clients.add(websocket)
        self.client_count = len(self.connected_clients)
        
        try:
            # 发送连接确认和相机信息
            welcome_msg = {
                'type': 'connection_established',
                'camera_info': {
                    'stereo_width': self.stereo_width,
                    'stereo_height': self.stereo_height,
                    'camera_width': self.camera_width,
                    'camera_height': self.camera_height,
                    'fps': self.target_fps,
                    'format': 'usb_stereo_rgb',
                    'rectified': self.enable_rectify,  # 标识：是否进行立体校正
                    'baseline_mm': self.stereo_config['baseline_mm'] if self.stereo_config else 60.0
                },
                'server_info': {
                    'version': '1.0.0',
                    'features': ['usb_stereo', 'adaptive_quality', 'vr_optimized', 'real_time_stats'],
                    'mode': 'test' if self.test_mode else 'real_camera'
                }
            }
            await websocket.send(json.dumps(welcome_msg))
            
            logger.info(f"已向客户端 {client_addr} 发送配置信息")
            
            # 主循环 - 发送图像数据
            frame_interval = 1.0 / self.target_fps
            last_send_time = 0
            
            while True:
                current_time = time.time()
                
                # 控制发送频率
                if current_time - last_send_time < frame_interval:
                    await asyncio.sleep(0.005)  # 更短的睡眠时间以减少延迟
                    continue
                
                # 获取最新帧
                with self.frame_lock:
                    left_image, right_image = self.latest_frames
                
                if left_image is not None and right_image is not None:
                    # 自适应质量调整
                    quality = self.adaptive_quality_adjustment(
                        self.client_count, 
                        self.stats['compression_ratio']
                    )
                    
                    # 编码图像
                    left_b64, right_b64, metadata = self.encode_images(
                        left_image, right_image, quality
                    )
                    
                    if left_b64 and right_b64:
                        # 构造消息 (与dual_infrared_websocket_server.py兼容)
                        message = {
                            'type': 'dual_infrared_frame',  # 保持与现有HTML兼容
                            'timestamp': current_time,
                            'left_infrared': left_b64,    # 保持与现有字段名兼容
                            'right_infrared': right_b64,  # 保持与现有字段名兼容
                            'metadata': metadata,
                            'stats': {
                                'fps_actual': round(self.stats['fps_actual'], 1),
                                'frames_captured': self.stats['frames_captured'],
                                'frames_sent': self.stats['frames_sent'],
                                'client_count': self.client_count,
                                'compression_ratio': round(self.stats['compression_ratio'], 3),
                                'mode': 'test' if self.test_mode else 'camera'
                            }
                        }
                        
                        # 发送数据
                        await websocket.send(json.dumps(message))
                        
                        self.stats['frames_sent'] += 1
                        self.stats['bytes_sent'] += len(json.dumps(message))
                        last_send_time = current_time
                
                # 检查连接状态
                try:
                    pong_waiter = await websocket.ping()
                    await asyncio.wait_for(pong_waiter, timeout=0.5)  # 更短的超时
                except:
                    break
                    
        except websockets.exceptions.ConnectionClosed:
            logger.info(f"VR客户端断开连接: {client_addr}")
        except Exception as e:
            logger.error(f"处理VR客户端出错 {client_addr}: {e}")
        finally:
            self.connected_clients.discard(websocket)
            self.client_count = len(self.connected_clients)
            logger.info(f"VR客户端已移除: {client_addr}, 剩余连接: {self.client_count}")
    
    async def stats_reporter(self):
        """统计信息报告器"""
        while True:
            await asyncio.sleep(10)
            
            runtime = time.time() - self.stats['start_time']
            fps_ratio = (self.stats['fps_actual'] / self.target_fps) * 100 if self.target_fps > 0 else 0
            
            logger.info("=== USB双目相机服务器统计 ===")
            logger.info(f"运行时间: {runtime:.1f}秒")
            logger.info(f"模式: {'测试模式' if self.test_mode else '真实相机'}")
            logger.info(f"配置: {self.stereo_width}x{self.stereo_height}@{self.target_fps}fps")
            logger.info(f"VR客户端: {self.client_count}")
            logger.info(f"捕获帧数: {self.stats['frames_captured']}")
            logger.info(f"发送帧数: {self.stats['frames_sent']}")
            logger.info(f"目标FPS: {self.target_fps}, 实际FPS: {self.stats['fps_actual']:.1f} ({fps_ratio:.1f}%)")
            logger.info(f"JPEG质量: {JPEG_QUALITY}%")
            logger.info(f"数据传输: {self.stats['bytes_sent'] / 1024 / 1024:.1f} MB")
            logger.info(f"压缩率: {self.stats['compression_ratio']:.3f}")
            
            # 如果帧率明显偏低，给出建议
            if fps_ratio < 80:
                logger.warning(f"⚠️ 帧率偏低建议:")
                logger.warning(f"   1. 降低目标帧率 (当前:{self.target_fps})")
                logger.warning(f"   2. 降低JPEG质量 (当前:{JPEG_QUALITY}%)")
                logger.warning(f"   3. 检查USB连接是否为USB3.0")
                
            logger.info("================================")
    
    async def start_server(self):
        """启动WebSocket服务器"""
        # 初始化相机
        if not self.initialize_camera():
            logger.error("相机初始化失败，无法启动服务器")
            return
        
        # 启动相机线程
        self.is_camera_running = True
        self.camera_thread = threading.Thread(target=self.camera_thread_function)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        
        # 等待相机稳定
        await asyncio.sleep(2)
        
        # SSL配置 (可选)
        ssl_context = None
        if self.use_ssl:
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            cert_file = "server.crt"
            key_file = "server.key"
            if os.path.exists(cert_file) and os.path.exists(key_file):
                ssl_context.load_cert_chain(cert_file, key_file)
                logger.info("已启用SSL加密")
            else:
                logger.warning("SSL证书文件不存在，使用非加密连接")
                ssl_context = None
        
        # 启动WebSocket服务器
        protocol = "wss" if ssl_context else "ws"
        
        logger.info("🚀 USB双目相机WebSocket服务器启动成功!")
        logger.info(f"📡 WebSocket地址: {protocol}://localhost:{self.port}")
        logger.info(f"🎮 模式: {'测试模式' if self.test_mode else '真实相机模式'}")
        logger.info(f"📹 分辨率: {self.stereo_width}x{self.stereo_height} (双目拼接)")
        logger.info(f"🎯 目标帧率: {self.target_fps}fps")
        logger.info(f"� JPEG质量: {JPEG_QUALITY}%")
        logger.info(f"�👁️  左眼: {self.camera_width}x{self.camera_height}")
        logger.info(f"👁️  右眼: {self.camera_width}x{self.camera_height}")
        logger.info(f"🔧 USB优化: MJPG={USE_MJPG_FORMAT}, 缓冲区={CAMERA_BUFFERSIZE}")
        
        if self.enable_rectify and self.stereo_config:
            logger.info(f"⚡ 立体校正: 启用")
            logger.info(f"   基线距离: {self.stereo_config['baseline_mm']:.1f}mm")
        else:
            logger.info(f"⚡ 立体校正: 禁用 (原始图像)")
            
        logger.info("=" * 60)
        logger.info("🌐 推荐VR客户端连接方式:")
        logger.info(f"   WebSocket URL: {protocol}://你的服务器IP:{self.port}")
        logger.info(f"   消息格式: dual_infrared_frame")
        logger.info(f"   数据字段: left_infrared, right_infrared")
        logger.info("=" * 60)
        
        # 启动统计报告器
        stats_task = asyncio.create_task(self.stats_reporter())
        
        # 创建处理器函数
        async def handler(websocket):
            await self.handle_client(websocket)
        
        # 启动服务器
        server = await websockets.serve(
            handler, 
            self.host, 
            self.port,
            ssl=ssl_context,
            ping_interval=20,
            ping_timeout=10
        )
        
        logger.info("WebSocket服务器已启动，等待VR客户端连接...")
        
        # 等待服务器关闭
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
    """主函数 - 简化版本，固定使用设备0"""
    import argparse
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='USB双目相机WebSocket服务器')
    parser.add_argument('--port', '-p', type=int, default=8765, 
                       help='WebSocket端口 (默认: 8765)')
    parser.add_argument('--ssl', action='store_true', 
                       help='启用SSL加密')
    parser.add_argument('--host', default='0.0.0.0', 
                       help='服务器地址 (默认: 0.0.0.0)')
    parser.add_argument('--get-config', action='store_true',
                       help='输出当前配置信息并退出')
    
    args = parser.parse_args()
    
    # 如果只是获取配置信息，直接输出并退出
    if args.get_config:
        config = get_camera_config_info()
        print(f"{config['stereo_width']}x{config['stereo_height']}@{config['target_fps']}fps")
        print(f"{config['camera_width']}x{config['camera_height']}")
        print(f"Camera Index: {config['camera_index']}")
        print(f"Rectify: {config['enable_rectify']}")
        return
    
    # 创建WebSocket服务器 - 不再接受camera_index和enable_rectify参数
    server = USBStereoWebSocketServer(
        host=args.host,
        port=args.port,
        use_ssl=args.ssl
    )
    
    try:
        await server.start_server()
    except KeyboardInterrupt:
        logger.info("收到中断信号")
    finally:
        server.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
