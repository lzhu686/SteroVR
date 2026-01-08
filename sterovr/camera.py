#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
StereoVR 相机采集模块

统一的相机采集接口，支持:
- USB 双目相机
- 测试模式 (无相机时生成测试图像)

作者: Liang ZHU
邮箱: lzhu686@connect.hkust-gz.edu.cn
"""

import cv2
import numpy as np
import time
import threading
import logging
from typing import Optional, Tuple, Callable
from dataclasses import dataclass

from .config import CameraConfig

logger = logging.getLogger(__name__)


@dataclass
class FrameData:
    """帧数据"""
    frame: np.ndarray           # 原始帧 (双目拼接)
    left_eye: np.ndarray        # 左眼图像
    right_eye: np.ndarray       # 右眼图像
    frame_id: int               # 帧序号
    timestamp: float            # 时间戳


class StereoCamera:
    """
    立体相机采集类

    特性:
    - 异步采集，不阻塞主线程
    - 自动跳帧，始终获取最新帧
    - 支持测试模式
    """

    def __init__(self, config: Optional[CameraConfig] = None):
        self.config = config or CameraConfig()
        self.cap: Optional[cv2.VideoCapture] = None
        self.is_running = False
        self.test_mode = False

        # 帧缓冲
        self._frame_lock = threading.Lock()
        self._latest_frame: Optional[FrameData] = None
        self._frame_id = 0

        # 采集线程
        self._capture_thread: Optional[threading.Thread] = None

        # 统计
        self.stats = {
            'frames_captured': 0,
            'fps_actual': 0.0,
            'capture_time_ms': 0.0,
            'dropped_frames': 0
        }

        # 回调
        self.on_frame: Optional[Callable[[FrameData], None]] = None

    def initialize(self) -> bool:
        """
        初始化相机

        返回:
            True 成功, False 失败 (将启用测试模式)
        """
        logger.info(f"初始化相机 (设备: {self.config.device_id})...")
        logger.info(f"目标配置: {self.config.stereo_width}x{self.config.stereo_height} @ {self.config.fps}fps")

        try:
            self.cap = cv2.VideoCapture(self.config.device_id)

            # 设置 MJPG 格式 (减少 USB 带宽)
            if self.config.use_mjpg:
                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
                logger.info("使用 MJPG 格式")

            # 设置分辨率和帧率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.stereo_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.stereo_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.config.fps)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, self.config.buffer_size)

            # 曝光和白平衡
            if not self.config.auto_exposure:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)

            if not self.cap.isOpened():
                logger.warning("无法打开相机，启用测试模式")
                self.test_mode = True
                return True

            # 验证实际参数
            actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
            fourcc_str = "".join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])

            logger.info(f"实际配置: {actual_w}x{actual_h} @ {actual_fps:.1f}fps ({fourcc_str})")

            # 测试读取
            ret, frame = self.cap.read()
            if not ret:
                logger.warning("相机读取测试失败，启用测试模式")
                self.test_mode = True
                return True

            logger.info("相机初始化成功")
            return True

        except Exception as e:
            logger.error(f"相机初始化异常: {e}")
            self.test_mode = True
            return True

    def start(self):
        """启动采集"""
        if self.is_running:
            return

        self.is_running = True
        self._capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        logger.info("相机采集已启动")

    def stop(self):
        """停止采集"""
        self.is_running = False

        if self._capture_thread:
            self._capture_thread.join(timeout=2)

        if self.cap:
            self.cap.release()

        logger.info("相机已停止")

    def get_frame(self) -> Optional[FrameData]:
        """获取最新帧 (非阻塞)"""
        with self._frame_lock:
            return self._latest_frame

    def _capture_loop(self):
        """采集循环"""
        frame_interval = 1.0 / self.config.fps
        fps_counter = 0
        fps_start_time = time.time()

        while self.is_running:
            loop_start = time.time()

            # 获取帧
            if self.test_mode:
                frame = self._generate_test_frame()
                ret = True
            else:
                ret, frame = self.cap.read()

            if ret and frame is not None:
                # 分割左右眼
                height = frame.shape[0]
                half_width = self.config.single_eye_width
                left_eye = frame[0:height, 0:half_width]
                right_eye = frame[0:height, half_width:self.config.stereo_width]

                # 创建帧数据
                self._frame_id += 1
                frame_data = FrameData(
                    frame=frame,
                    left_eye=left_eye,
                    right_eye=right_eye,
                    frame_id=self._frame_id,
                    timestamp=time.time()
                )

                # 更新缓冲
                with self._frame_lock:
                    self._latest_frame = frame_data

                # 回调
                if self.on_frame:
                    try:
                        self.on_frame(frame_data)
                    except Exception as e:
                        logger.error(f"帧回调异常: {e}")

                # 统计
                self.stats['frames_captured'] += 1
                self.stats['capture_time_ms'] = (time.time() - loop_start) * 1000
                fps_counter += 1

                # 每秒更新 FPS
                elapsed = time.time() - fps_start_time
                if elapsed >= 1.0:
                    self.stats['fps_actual'] = fps_counter / elapsed
                    fps_counter = 0
                    fps_start_time = time.time()

            # 帧率控制
            elapsed = time.time() - loop_start
            if elapsed < frame_interval:
                time.sleep(frame_interval - elapsed)

    def _generate_test_frame(self) -> np.ndarray:
        """生成测试帧"""
        frame = np.zeros((self.config.stereo_height, self.config.stereo_width, 3), dtype=np.uint8)

        # 左眼: 红色调
        frame[:, :self.config.single_eye_width] = (50, 50, 150)
        # 右眼: 蓝色调
        frame[:, self.config.single_eye_width:] = (150, 50, 50)

        # 添加文字
        timestamp = time.strftime("%H:%M:%S")
        cv2.putText(frame, f"LEFT - {timestamp}", (50, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        cv2.putText(frame, f"RIGHT - {timestamp}", (self.config.single_eye_width + 50, 100),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 2)
        cv2.putText(frame, "TEST MODE", (self.config.stereo_width // 2 - 100, 200),
                   cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)

        return frame


# === 便捷函数 ===

def create_camera(config: Optional[CameraConfig] = None) -> StereoCamera:
    """创建并初始化相机"""
    camera = StereoCamera(config)
    camera.initialize()
    return camera


if __name__ == '__main__':
    # 测试
    logging.basicConfig(level=logging.INFO)

    print("=== 相机采集测试 ===\n")

    camera = create_camera()
    camera.start()

    try:
        for i in range(5):
            time.sleep(1)
            frame = camera.get_frame()
            if frame:
                print(f"帧 #{frame.frame_id}: {frame.frame.shape}, FPS: {camera.stats['fps_actual']:.1f}")
    finally:
        camera.stop()
