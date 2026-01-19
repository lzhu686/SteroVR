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
        self.ros2_available = False

        # 统计
        self.stats = {
            'frames_published': 0,
            'start_time': 0.0,
            'last_frame_time': 0.0,
        }

    def _init_ros2(self) -> bool:
        """初始化 ROS2 节点和发布者"""
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import CompressedImage

            if not rclpy.ok():
                rclpy.init()

            self.node = rclpy.create_node('stereo_camera_publisher')

            # 创建发布者
            self.pub_left = self.node.create_publisher(
                CompressedImage,
                '/stereo/left/compressed',
                10
            )
            self.pub_right = self.node.create_publisher(
                CompressedImage,
                '/stereo/right/compressed',
                10
            )

            self.ros2_available = True
            logger.info("ROS2 节点初始化成功")
            logger.info("  发布话题:")
            logger.info("    - /stereo/left/compressed")
            logger.info("    - /stereo/right/compressed")
            return True

        except ImportError as e:
            logger.error(f"ROS2 不可用: {e}")
            logger.error("请确保已安装 ROS2 并 source 了 setup.bash")
            return False
        except Exception as e:
            logger.error(f"ROS2 初始化失败: {e}")
            return False

    def _open_loopback(self) -> bool:
        """打开 V4L2 Loopback 设备"""
        logger.info(f"正在打开 V4L2 Loopback 设备: {self.loopback_device}")

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
                # 验证设备是否可打开
                test_cap = cv2.VideoCapture(device_num)
                if test_cap.isOpened():
                    ret, frame = test_cap.read()
                    test_cap.release()
                    if ret and frame is not None:
                        h, w = frame.shape[:2]
                        self.cap = cv2.VideoCapture(device_num)
                        logger.info(f"V4L2 Loopback 设备已打开: {w}x{h}")
                        return True
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
        from sensor_msgs.msg import CompressedImage
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

                # 读取帧
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    logger.warning("读取帧失败，等待数据...")
                    time.sleep(0.1)
                    continue

                h, w = frame.shape[:2]
                half_w = w // 2

                # 检查是否是有效的双目图像
                if half_w < 100:
                    logger.warning(f"图像宽度异常: {w}，可能不是双目图像")
                    time.sleep(0.1)
                    continue

                # 分割左右眼
                left_frame = frame[:, :half_w]
                right_frame = frame[:, half_w:]

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

                frame_count += 1
                self.stats['frames_published'] = frame_count

                # 每秒输出统计
                if now - last_log_time >= 1.0:
                    elapsed_total = now - self.stats['start_time']
                    fps = frame_count / elapsed_total if elapsed_total > 0 else 0
                    logger.info(f"已发布: {frame_count} 帧 | {fps:.1f} fps | "
                               f"左眼: {len(left_jpeg)} bytes, 右眼: {len(right_jpeg)} bytes")
                    last_log_time = now

                # 处理 ROS2 回调
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
        logger.info("=" * 60)
        logger.info("ROS2 Loopback Publisher 统计")
        logger.info("=" * 60)
        logger.info(f"  运行时长: {elapsed:.1f} 秒")
        logger.info(f"  发布帧数: {self.stats['frames_published']}")
        if elapsed > 0:
            logger.info(f"  平均帧率: {self.stats['frames_published'] / elapsed:.1f} fps")
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
