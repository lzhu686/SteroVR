#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 独立发布进程 - 从 V4L2 Loopback 虚拟相机读取并发布

架构说明:
┌─────────────────────────────────────────────────────────────────┐
│  进程 1: PICO 视频流 (h264_sender.py)                            │
│  /dev/stereo_camera → FFmpeg → H.264 → PICO (60fps)          │
│              ↓                                                   │
│         MJPEG → /dev/video99 (30fps)                            │
└─────────────────────────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────────────────────────┐
│  进程 2: ROS2 发布 (本脚本)                                       │
│  /dev/video99 → OpenCV → Split → CompressedImage              │
│                                   ├─ /stereo/left/compressed    │
│                                   └─ /stereo/right/compressed   │
└─────────────────────────────────────────────────────────────────┘

设备说明:
- /dev/stereo_camera: 真实双目相机 (udev 符号链接 → /dev/video3)
- /dev/video99: V4L2 Loopback 虚拟相机 (接收 FFmpeg 的 MJPEG 输出)

优势:
1. 完全独立的进程，不影响 PICO 视频流延迟
2. ROS2 发布崩溃不影响主视频流
3. 可以独立调整 ROS2 发布参数

使用方法:
    # 确保 v4l2loopback 已加载
    sudo modprobe v4l2loopback video_nr=99 card_label="StereoVR_ROS2"

    # 启动主服务器 (会输出到 /dev/video99)
    python start_xrobo_compat.py --device /dev/stereo_camera --loopback /dev/video99 --loopback-fps 30

    # 在另一个终端启动 ROS2 发布
    python -m teleopVision.ros2_loopback_publisher --device /dev/video99 --fps 30

作者: Liang ZHU
邮箱: lzhu686@connect.hkust-gz.edu.cn
日期: 2025
"""

import cv2
import numpy as np
import time
import argparse
import logging
import sys
from typing import Optional

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('ROS2LoopbackPublisher')


class ROS2LoopbackPublisher:
    """
    从 V4L2 Loopback 读取并发布到 ROS2

    独立进程运行，不影响主视频流
    """

    # JPEG 标记
    JPEG_START = b'\xff\xd8'  # JPEG SOI (Start Of Image)
    JPEG_END = b'\xff\xd9'    # JPEG EOI (End Of Image)

    def __init__(self,
                 loopback_device: str = "/dev/video99",
                 jpeg_quality: int = 85,
                 target_fps: float = 30.0):
        """
        初始化

        参数:
            loopback_device: V4L2 Loopback 设备路径
            jpeg_quality: JPEG 压缩质量 (1-100)
            target_fps: 目标发布帧率
        """
        self.loopback_device = loopback_device
        self.jpeg_quality = jpeg_quality
        self.target_fps = target_fps
        self.frame_interval = 1.0 / target_fps

        self.cap: Optional[cv2.VideoCapture] = None
        self.is_running = False

        # ROS2 相关
        self.node = None
        self.pub_left = None
        self.pub_right = None
        self.pub_left_raw = None      # Image Raw 发布者 (BGR8)
        self.pub_right_raw = None     # Image Raw 发布者 (BGR8)
        self.pub_left_rgb = None      # Image RGB 发布者 (RGB8)
        self.pub_right_rgb = None     # Image RGB 发布者 (RGB8)
        self.ros2_available = False

        # 统计
        self.stats = {
            'frames_published': 0,
            'frames_skipped': 0,  # 跳过的损坏帧数
            'start_time': 0.0,
            'last_frame_time': 0.0,
        }

    @staticmethod
    def _is_valid_jpeg(data: bytes) -> bool:
        """
        检查数据是否为有效的 JPEG 格式

        验证:
        1. 以 JPEG SOI 标记 (0xFF 0xD8) 开头
        2. 以 JPEG EOI 标记 (0xFF 0xD9) 结尾
        3. 数据长度合理
        """
        if data is None or len(data) < 2:
            return False

        # 检查起始标记
        if not data.startswith(ROS2LoopbackPublisher.JPEG_START):
            return False

        # 检查结束标记
        if not data.endswith(ROS2LoopbackPublisher.JPEG_END):
            return False

        # 数据长度检查 (最小有效 JPEG 约 100 bytes)
        if len(data) < 100:
            return False

        return True

    @staticmethod
    def _fix_jpeg_frame(data: bytes) -> Optional[bytes]:
        """
        尝试修复损坏的 JPEG 帧

        从数据中提取完整的 JPEG:
        1. 找到所有有效的 JPEG 帧
        2. 返回包含起始和结束标记的完整帧
        """
        if data is None or len(data) < 2:
            return None

        # 如果数据以 JPEG 开头但没有结束标记，尝试修复
        if data.startswith(ROS2LoopbackPublisher.JPEG_START):
            # 查找结束标记
            end_pos = data.rfind(ROS2LoopbackPublisher.JPEG_END)
            if end_pos >= 0:
                # 返回完整的帧（包括结束标记）
                return data[:end_pos + 2]

            # 如果没有找到结束标记，添加它
            return data + ROS2LoopbackPublisher.JPEG_END

        return None

    def _read_frame_with_retry(self, max_retries: int = 3) -> Optional[cv2.Mat]:
        """
        带重试机制的帧读取

        读取帧并验证 JPEG 完整性，如果损坏则重试
        """
        for attempt in range(max_retries):
            ret, frame = self.cap.read()
            if not ret or frame is None:
                time.sleep(0.01 * (attempt + 1))
                continue

            h, w = frame.shape[:2] if len(frame.shape) == 2 else frame.shape[:2]

            # 检查图像尺寸是否有效
            if w < 100 or h < 100:
                self.stats['frames_skipped'] += 1
                logger.debug(f"跳过异常尺寸帧: {w}x{h}")
                time.sleep(0.01 * (attempt + 1))
                continue

            return frame

        self.stats['frames_skipped'] += max_retries
        return None

    def _init_ros2(self) -> bool:
        """初始化 ROS2 节点和发布者"""
        try:
            import rclpy
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
            from sensor_msgs.msg import CompressedImage, Image

            if not rclpy.ok():
                rclpy.init()

            self.node = rclpy.create_node('stereo_camera_publisher')

            # 图像数据 QoS Profile - 使用 BEST_EFFORT 实现低延迟传输
            # 原理:
            #   RELIABLE: 丢包时重传，保证送达，但增加延迟 (适合小数据/关键数据)
            #   BEST_EFFORT: 不重传，允许丢帧，低延迟 (适合实时视频流)
            # 对于 30fps 视频流，丢 1 帧 (33ms) 比等待重传 (可能 >100ms) 更好
            image_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,  # 不重传，低延迟
                durability=DurabilityPolicy.VOLATILE,        # 不保存历史消息
                history=HistoryPolicy.KEEP_LAST,             # 只保留最新
                depth=1                                      # 队列深度为 1，防止积压
            )

            # 创建 CompressedImage 发布者 (用于网络传输，带宽优化)
            self.pub_left = self.node.create_publisher(
                CompressedImage,
                '/stereo/left/compressed',
                image_qos
            )
            self.pub_right = self.node.create_publisher(
                CompressedImage,
                '/stereo/right/compressed',
                image_qos
            )

            # 创建 Image Raw 发布者 (BGR8, 用于本地 RViz2 显示，OpenCV 原生格式)
            self.pub_left_raw = self.node.create_publisher(
                Image,
                '/stereo/left/image_raw',
                image_qos
            )
            self.pub_right_raw = self.node.create_publisher(
                Image,
                '/stereo/right/image_raw',
                image_qos
            )

            # 创建 RGB8 发布者 (用于深度学习框架，如 PyTorch/TensorFlow)
            self.pub_left_rgb = self.node.create_publisher(
                Image,
                '/stereo/left/image_rgb',
                image_qos
            )
            self.pub_right_rgb = self.node.create_publisher(
                Image,
                '/stereo/right/image_rgb',
                image_qos
            )

            self.ros2_available = True
            logger.info("ROS2 节点初始化成功")
            logger.info("  QoS Profile: BEST_EFFORT (低延迟，允许丢帧)")
            logger.info("  发布话题:")
            logger.info("    - /stereo/left/compressed  (CompressedImage, JPEG, 网络优化)")
            logger.info("    - /stereo/right/compressed (CompressedImage, JPEG, 网络优化)")
            logger.info("    - /stereo/left/image_raw   (Image, BGR8, OpenCV/RViz2)")
            logger.info("    - /stereo/right/image_raw  (Image, BGR8, OpenCV/RViz2)")
            logger.info("    - /stereo/left/image_rgb   (Image, RGB8, 深度学习框架)")
            logger.info("    - /stereo/right/image_rgb  (Image, RGB8, 深度学习框架)")
            return True

        except ImportError as e:
            logger.error(f"ROS2 不可用: {e}")
            logger.error("请确保已安装 ROS2 并 source 了 setup.bash")
            return False
        except Exception as e:
            logger.error(f"ROS2 初始化失败: {e}")
            return False

    def _open_loopback(self) -> bool:
        """打开 V4L2 Loopback 设备 (BGR24 格式，OpenCV 原生格式)"""
        logger.info(f"正在打开 V4L2 Loopback 设备: {self.loopback_device}")
        logger.info("注意: 使用 BGR24 格式，OpenCV 原生格式，零转换开销")

        # 等待设备可用 (主进程可能还没开始输出)
        max_retries = 30
        for attempt in range(max_retries):
            try:
                # 从设备路径提取数字
                if self.loopback_device.startswith('/dev/video'):
                    device_num = int(self.loopback_device.replace('/dev/video', ''))
                else:
                    # 符号链接情况，尝试查找实际设备
                    import os
                    if os.path.islink(self.loopback_device):
                        real_path = os.path.realpath(self.loopback_device)
                        if real_path.startswith('/dev/video'):
                            device_num = int(real_path.replace('/dev/video', ''))
                        else:
                            logger.error(f"无法识别的设备路径: {real_path}")
                            return False
                    else:
                        # 尝试作为索引打开
                        device_num = int(self.loopback_device)

                # 使用 V4L2 后端打开
                self.cap = cv2.VideoCapture(device_num, cv2.CAP_V4L2)

                if self.cap.isOpened():
                    # 尝试读取一帧来确认设备可用
                    ret, frame = self.cap.read()
                    if ret and frame is not None:
                        h, w = frame.shape[:2]
                        channels = frame.shape[2] if len(frame.shape) == 3 else 1
                        logger.info(f"V4L2 Loopback 设备已打开: {w}x{h}x{channels}")

                        # BGR24 格式应该是 3 通道
                        if channels == 3:
                            logger.info(f"数据格式: BGR24 (OpenCV 原生格式)，dtype={frame.dtype}")
                        else:
                            logger.warning(f"意外的通道数: {channels}，期望 3 (BGR24)")

                        return True

                    # 释放并重试
                    self.cap.release()
                    self.cap = None

                logger.debug(f"设备已打开但无数据 (尝试 {attempt+1}/{max_retries})")

            except Exception as e:
                logger.debug(f"打开设备失败: {e}")

            time.sleep(1.0)

        logger.error(f"无法打开 V4L2 Loopback 设备: {self.loopback_device}")
        logger.error("请检查:")
        logger.error("  1. v4l2loopback 模块是否已加载")
        logger.error("  2. 主视频流服务器是否已启动并输出到此设备")
        logger.error(f"  3. 设备路径是否正确 ({self.loopback_device})")
        return False

    def start(self) -> bool:
        """启动发布"""
        # 初始化 ROS2
        if not self._init_ros2():
            return False

        # 打开 Loopback 设备
        if not self._open_loopback():
            return False

        self.is_running = True
        self.stats['start_time'] = time.time()

        logger.info("=" * 60)
        logger.info("ROS2 Loopback Publisher 已启动")
        logger.info("=" * 60)
        logger.info(f"  设备: {self.loopback_device}")
        logger.info(f"  目标帧率: {self.target_fps} fps")
        logger.info(f"  JPEG 质量: {self.jpeg_quality}")
        logger.info("=" * 60)

        self._publish_loop()
        return True

    def _publish_loop(self):
        """发布循环"""
        import rclpy
        from sensor_msgs.msg import CompressedImage, Image
        from builtin_interfaces.msg import Time

        frame_count = 0
        last_log_time = time.time()
        last_frame_time = time.time()

        while self.is_running and rclpy.ok():
            try:
                # 控制帧率
                now = time.time()
                elapsed = now - last_frame_time
                if elapsed < self.frame_interval:
                    time.sleep(self.frame_interval - elapsed)
                    now = time.time()
                last_frame_time = now

                # 使用带重试的帧读取
                frame = self._read_frame_with_retry(max_retries=3)
                if frame is None:
                    logger.debug("无法读取有效帧，继续等待...")
                    continue

                h, w = frame.shape[:2]
                half_w = w // 2

                # 检查是否是有效的双目图像
                if half_w < 100:
                    self.stats['frames_skipped'] += 1
                    logger.warning(f"图像宽度异常: {w}，可能不是双目图像")
                    time.sleep(0.1)
                    continue

                # BGR24 格式：OpenCV 直接读取为 3 通道 BGR，无需颜色转换
                if len(frame.shape) == 3 and frame.shape[2] == 3:
                    frame_bgr = frame  # 已经是 BGR，直接使用
                elif len(frame.shape) == 2:
                    # 灰度图 (不应该发生)
                    logger.warning("收到灰度图，转换为 BGR")
                    frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                else:
                    logger.warning(f"意外的帧格式: shape={frame.shape}，跳过")
                    self.stats['frames_skipped'] += 1
                    continue

                # 分割左右眼
                left_frame = frame_bgr[:, :half_w]
                right_frame = frame_bgr[:, half_w:]

                # 编码为 JPEG
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
                _, left_jpeg = cv2.imencode('.jpg', left_frame, encode_params)
                _, right_jpeg = cv2.imencode('.jpg', right_frame, encode_params)

                # 创建时间戳
                timestamp = self.node.get_clock().now().to_msg()

                # 发布左眼
                msg_left = CompressedImage()
                msg_left.header.stamp = timestamp
                msg_left.header.frame_id = 'stereo_left'
                msg_left.format = 'jpeg'
                msg_left.data = left_jpeg.tobytes()
                self.pub_left.publish(msg_left)

                # 发布右眼
                msg_right = CompressedImage()
                msg_right.header.stamp = timestamp
                msg_right.header.frame_id = 'stereo_right'
                msg_right.format = 'jpeg'
                msg_right.data = right_jpeg.tobytes()
                self.pub_right.publish(msg_right)

                # 发布 Image Raw (用于 RViz2 本地显示，共享内存传输，不占网络带宽)
                # 左眼 Raw
                msg_left_raw = Image()
                msg_left_raw.header.stamp = timestamp
                msg_left_raw.header.frame_id = 'stereo_left'
                msg_left_raw.height = left_frame.shape[0]
                msg_left_raw.width = left_frame.shape[1]
                msg_left_raw.encoding = 'bgr8'
                msg_left_raw.is_bigendian = False
                msg_left_raw.step = left_frame.shape[1] * 3  # width * channels
                msg_left_raw.data = left_frame.tobytes()
                self.pub_left_raw.publish(msg_left_raw)

                # 右眼 Raw
                msg_right_raw = Image()
                msg_right_raw.header.stamp = timestamp
                msg_right_raw.header.frame_id = 'stereo_right'
                msg_right_raw.height = right_frame.shape[0]
                msg_right_raw.width = right_frame.shape[1]
                msg_right_raw.encoding = 'bgr8'
                msg_right_raw.is_bigendian = False
                msg_right_raw.step = right_frame.shape[1] * 3  # width * channels
                msg_right_raw.data = right_frame.tobytes()
                self.pub_right_raw.publish(msg_right_raw)

                # 可选: 发布 RGB8 格式 (用于深度学习框架)
                if self.pub_left_rgb and self.pub_right_rgb:
                    # BGR → RGB 转换
                    left_rgb = cv2.cvtColor(left_frame, cv2.COLOR_BGR2RGB)
                    right_rgb = cv2.cvtColor(right_frame, cv2.COLOR_BGR2RGB)

                    # 左眼 RGB
                    msg_left_rgb = Image()
                    msg_left_rgb.header.stamp = timestamp
                    msg_left_rgb.header.frame_id = 'stereo_left'
                    msg_left_rgb.height = left_rgb.shape[0]
                    msg_left_rgb.width = left_rgb.shape[1]
                    msg_left_rgb.encoding = 'rgb8'
                    msg_left_rgb.is_bigendian = False
                    msg_left_rgb.step = left_rgb.shape[1] * 3
                    msg_left_rgb.data = left_rgb.tobytes()
                    self.pub_left_rgb.publish(msg_left_rgb)

                    # 右眼 RGB
                    msg_right_rgb = Image()
                    msg_right_rgb.header.stamp = timestamp
                    msg_right_rgb.header.frame_id = 'stereo_right'
                    msg_right_rgb.height = right_rgb.shape[0]
                    msg_right_rgb.width = right_rgb.shape[1]
                    msg_right_rgb.encoding = 'rgb8'
                    msg_right_rgb.is_bigendian = False
                    msg_right_rgb.step = right_rgb.shape[1] * 3
                    msg_right_rgb.data = right_rgb.tobytes()
                    self.pub_right_rgb.publish(msg_right_rgb)

                frame_count += 1
                self.stats['frames_published'] = frame_count

                # 每秒输出统计
                if now - last_log_time >= 1.0:
                    elapsed_total = now - self.stats['start_time']
                    fps = frame_count / elapsed_total if elapsed_total > 0 else 0
                    skipped = self.stats['frames_skipped']
                    logger.info(f"已发布: {frame_count} 帧 | {fps:.1f} fps | "
                               f"左眼: {len(left_jpeg)} bytes, 右眼: {len(right_jpeg)} bytes | "
                               f"跳过: {skipped}")
                    last_log_time = now

                # 处理 ROS2 回调 (确保消息发送)
                rclpy.spin_once(self.node, timeout_sec=0)

            except KeyboardInterrupt:
                logger.info("收到中断信号")
                break
            except Exception as e:
                logger.error(f"发布循环错误: {e}")
                time.sleep(0.1)

        logger.info(f"发布循环结束，共发布 {frame_count} 帧")

    def stop(self):
        """停止发布"""
        logger.info("正在停止 ROS2 Loopback Publisher...")
        self.is_running = False

        if self.cap:
            self.cap.release()
            self.cap = None

        if self.node:
            try:
                self.node.destroy_node()
            except:
                pass

        try:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

        # 输出统计
        elapsed = time.time() - self.stats['start_time'] if self.stats['start_time'] > 0 else 0
        skipped = self.stats['frames_skipped']
        published = self.stats['frames_published']
        total = published + skipped
        valid_rate = (published / total * 100) if total > 0 else 0

        logger.info("=" * 60)
        logger.info("ROS2 Loopback Publisher 统计")
        logger.info("=" * 60)
        logger.info(f"  运行时长: {elapsed:.1f} 秒")
        logger.info(f"  发布帧数: {published}")
        logger.info(f"  跳过帧数: {skipped}")
        logger.info(f"  有效率: {valid_rate:.1f}%")
        if elapsed > 0:
            logger.info(f"  平均帧率: {published / elapsed:.1f} fps")
        logger.info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='ROS2 Loopback Publisher - 从 V4L2 Loopback 读取并发布到 ROS2',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 使用默认设备 /dev/video99 (V4L2 Loopback)
  python -m teleopVision.ros2_loopback_publisher

  # 指定设备和参数
  python -m teleopVision.ros2_loopback_publisher --device /dev/video99 --fps 30 --quality 85

发布的话题 (共 6 个):
  CompressedImage (JPEG, 网络传输优化):
    /stereo/left/compressed
    /stereo/right/compressed

  Image BGR8 (OpenCV 原生格式, RViz2):
    /stereo/left/image_raw
    /stereo/right/image_raw

  Image RGB8 (深度学习框架, PyTorch/TensorFlow):
    /stereo/left/image_rgb
    /stereo/right/image_rgb

前置条件:
  1. v4l2loopback 模块已加载:
     sudo modprobe v4l2loopback video_nr=99 card_label="StereoVR_ROS2"

  2. 主视频流服务器已启动并输出到 loopback 设备:
     python start_xrobo_compat.py --device /dev/stereo_camera --loopback /dev/video99 --loopback-fps 30

  3. ROS2 环境已配置:
     source /opt/ros/humble/setup.bash
"""
    )
    parser.add_argument('--device', '-d', type=str, default='/dev/video99',
                        help='V4L2 Loopback 设备路径 (默认: /dev/video99)')
    parser.add_argument('--fps', '-f', type=float, default=30.0,
                        help='目标发布帧率 (默认: 30)')
    parser.add_argument('--quality', '-q', type=int, default=85,
                        help='JPEG 压缩质量 1-100 (默认: 85)')

    args = parser.parse_args()

    print()
    print("=" * 60)
    print("ROS2 Loopback Publisher")
    print("从 V4L2 Loopback 虚拟相机读取并发布到 ROS2")
    print("=" * 60)
    print()

    publisher = ROS2LoopbackPublisher(
        loopback_device=args.device,
        jpeg_quality=args.quality,
        target_fps=args.fps
    )

    try:
        publisher.start()
    except KeyboardInterrupt:
        print("\n收到中断信号")
    finally:
        publisher.stop()


if __name__ == '__main__':
    main()
