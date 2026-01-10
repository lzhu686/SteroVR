#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XRoboToolkit 协议兼容服务器
模拟 XRoboToolkit PC-Service 的行为，让 PICO 头显可以直接连接并接收视频流

协议说明:
1. TCP 端口 13579: 接收来自 Unity Client 的控制命令
2. TCP 端口 (动态): 向 Unity Client 发送 H.264 视频流

命令格式:
- Unity Client 发送 JSON 命令: {"functionName": "StartReceivePcCamera", "value": {...}}
- 本服务器解析命令并启动视频流发送

生命周期管理:
- 线程安全: 使用 threading.Lock 保护共享状态
- 资源清理: 客户端断开时自动停止视频流
- 重复命令: 正确处理快速连续的 OPEN/CLOSE 命令

使用方法:
    python xrobo_compat_server.py

    # 或指定相机设备
    python xrobo_compat_server.py --device 0

作者: Liang ZHU
邮箱: lzhu686@connect.hkust-gz.edu.cn
日期: 2025
"""

import socket
import threading
import json
import struct
import time
import logging
import argparse
import sys
from typing import Optional, Callable
from dataclasses import dataclass
from enum import Enum

# 导入 H.264 发送器
from .h264_sender import SimpleH264Sender, VideoConfig

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('XRoboCompat')


# ============== 状态枚举 ==============

class StreamingState(Enum):
    """视频流状态"""
    IDLE = "idle"                # 空闲，没有视频流
    STARTING = "starting"        # 正在启动视频流
    STREAMING = "streaming"      # 视频流运行中
    STOPPING = "stopping"        # 正在停止视频流


# ============== XRoboToolkit 协议常量 ==============

class ProtocolConstants:
    """协议常量 (参考 XRoboToolkit-Unity-Client 的 TcpManager.cs)"""

    # 端口 (Unity Client 使用 13579)
    TCP_PORT = 13579

    # 包头包尾
    PACKET_HEAD_SEND = 0x3F      # 发送包头
    PACKET_HEAD_RECV = 0xCF      # 接收包头
    PACKET_END = 0xA5            # 包尾

    # 命令码
    CMD_CONNECT = 0x19           # 连接
    CMD_FUNCTION = 0x6D          # 函数调用
    CMD_HEARTBEAT = 0x23         # 心跳
    CMD_VERSION = 0x6C           # 版本


@dataclass
class CameraRequest:
    """相机请求配置"""
    ip: str = ""                 # 客户端 IP
    port: int = 12345            # 目标端口
    width: int = 2560            # 视频宽度
    height: int = 720            # 视频高度
    fps: int = 60                # 帧率
    bitrate: int = 8000000       # 码率
    camera_type: str = "USB"     # 相机类型

    @classmethod
    def from_json(cls, data: dict) -> 'CameraRequest':
        """从 JSON 创建"""
        return cls(
            ip=data.get('ip', ''),
            port=data.get('port', 12345),
            width=data.get('width', 2560),
            height=data.get('height', 720),
            fps=data.get('fps', 60),
            bitrate=data.get('bitrate', 8000000),
            camera_type=data.get('cameraType', 'USB')
        )


class PacketParser:
    """
    协议包解析器
    支持两种格式:
    1. XRoboToolkit 格式: [Head:1][Cmd:1][Length:4][Data:n][Timestamp:8][End:1]
    2. NetworkDataProtocol 格式: [cmd_len:4][command:str][data_len:4][data:bytes]
    """

    @staticmethod
    def parse(data: bytes) -> tuple:
        """
        解析数据包
        返回: (command, json_data) 或 (None, None)
        """
        if len(data) < 8:
            return None, None

        # 首先尝试 NetworkDataProtocol 格式 (Unity Client 使用)
        result = PacketParser._try_parse_network_data_protocol(data)
        if result[0] is not None:
            return result

        # 然后尝试 XRoboToolkit 格式
        result = PacketParser._try_parse_xrobo_protocol(data)
        if result[0] is not None:
            return result

        # 最后尝试直接解析 JSON
        return PacketParser._try_parse_json(data)

    @staticmethod
    def _try_parse_network_data_protocol(data: bytes) -> tuple:
        """
        解析 NetworkDataProtocol 格式

        Unity Client 发送格式:
        [total_length: 4 bytes Big-Endian][cmd_len: 4 bytes Little-Endian][command][data_len: 4 bytes Little-Endian][data]
        """
        try:
            hex_preview = data[:min(64, len(data))].hex()
            logger.debug(f"[DEBUG] 收到数据 ({len(data)} bytes): {hex_preview}...")

            if len(data) < 12:
                return None, None

            offset = 0

            # 检查是否有 Big-Endian 长度前缀
            potential_total_len = struct.unpack('>I', data[0:4])[0]
            if 10 < potential_total_len < 1000 and potential_total_len <= len(data):
                offset = 4
                logger.debug(f"[DEBUG] 检测到长度前缀: {potential_total_len} bytes")

            # 读取命令长度 (Little-Endian)
            cmd_len = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            # 验证命令长度合理性
            if cmd_len <= 0 or cmd_len > 100:
                return None, None

            if offset + cmd_len > len(data):
                return None, None

            # 读取命令字符串
            command = data[offset:offset+cmd_len].decode('utf-8')
            offset += cmd_len

            if offset + 4 > len(data):
                return None, None

            # 读取数据长度
            data_len = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            if data_len < 0 or offset + data_len > len(data):
                return None, None

            # 读取数据
            payload = data[offset:offset+data_len]

            logger.info(f"解析命令: {command}, 数据长度: {data_len}")

            # 根据命令类型处理
            if command == "OPEN_CAMERA":
                camera_config = PacketParser._parse_camera_request(payload)
                if camera_config:
                    return ProtocolConstants.CMD_FUNCTION, {
                        "functionName": "OpenCamera",
                        "value": camera_config
                    }

            elif command == "CLOSE_CAMERA":
                return ProtocolConstants.CMD_FUNCTION, {
                    "functionName": "StopReceivePcCamera"
                }

            return None, None

        except Exception as e:
            logger.debug(f"NetworkDataProtocol 解析失败: {e}")
            return None, None

    @staticmethod
    def _parse_camera_request(data: bytes) -> dict:
        """
        解析 CameraRequest 二进制数据
        """
        try:
            if len(data) < 10:
                logger.warning(f"CameraRequest 数据太短: {len(data)} bytes")
                return None

            offset = 0

            # 检查魔数 0xCA 0xFE
            if data[0] == 0xCA and data[1] == 0xFE:
                offset = 2
                version = data[offset]
                offset += 1
                logger.debug(f"CameraRequest 协议版本: {version}")
            else:
                logger.debug("无魔数头，尝试直接解析")

            # 读取整数字段 (小端序)
            width = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            height = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            fps = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            bitrate = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            enable_hevc = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            render_mode = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            port = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4

            # 读取 camera 字符串
            camera_len = data[offset]
            offset += 1
            camera = data[offset:offset+camera_len].decode('utf-8') if camera_len > 0 else "USB"
            offset += camera_len

            # 读取 ip 字符串
            ip_len = data[offset]
            offset += 1
            ip = data[offset:offset+ip_len].decode('utf-8') if ip_len > 0 else ""
            offset += ip_len

            logger.info(f"CameraRequest: {width}x{height}@{fps}fps, {bitrate//1000000}Mbps, ip={ip}:{port}")

            return {
                "width": width,
                "height": height,
                "fps": fps,
                "bitrate": bitrate,
                "ip": ip,
                "port": port,
                "cameraType": camera
            }

        except Exception as e:
            logger.error(f"解析 CameraRequest 失败: {e}")
            import traceback
            traceback.print_exc()
            return None

    @staticmethod
    def _try_parse_xrobo_protocol(data: bytes) -> tuple:
        """尝试解析 XRoboToolkit 标准协议格式"""
        head_idx = -1
        for i in range(len(data)):
            if data[i] == ProtocolConstants.PACKET_HEAD_RECV or data[i] == ProtocolConstants.PACKET_HEAD_SEND:
                head_idx = i
                break

        if head_idx < 0:
            return None, None

        try:
            cmd = data[head_idx + 1]
            length = struct.unpack('<I', data[head_idx + 2:head_idx + 6])[0]
            data_start = head_idx + 6
            data_end = data_start + length

            if data_end > len(data):
                return None, None

            payload = data[data_start:data_end]

            try:
                json_str = payload.decode('utf-8')
                json_data = json.loads(json_str)
                return cmd, json_data
            except:
                pass

        except Exception as e:
            logger.debug(f"XRobo 协议解析失败: {e}")

        return None, None

    @staticmethod
    def _try_parse_json(data: bytes) -> tuple:
        """尝试直接从数据中提取 JSON"""
        try:
            text = data.decode('utf-8', errors='ignore')
            json_start = text.find('{')
            json_end = text.rfind('}')

            if json_start >= 0 and json_end > json_start:
                json_str = text[json_start:json_end + 1]
                json_data = json.loads(json_str)
                return ProtocolConstants.CMD_FUNCTION, json_data

        except Exception as e:
            logger.debug(f"JSON 解析失败: {e}")

        return None, None

    @staticmethod
    def build_response(cmd: int, data: dict) -> bytes:
        """构建响应包"""
        json_str = json.dumps(data)
        json_bytes = json_str.encode('utf-8')

        packet = bytearray()
        packet.append(ProtocolConstants.PACKET_HEAD_SEND)
        packet.append(cmd)
        packet.extend(struct.pack('<I', len(json_bytes)))
        packet.extend(json_bytes)
        packet.extend(struct.pack('<Q', int(time.time() * 1000)))
        packet.append(ProtocolConstants.PACKET_END)

        return bytes(packet)


class XRoboCompatServer:
    """
    XRoboToolkit 兼容服务器

    生命周期管理:
    - 线程安全: 使用 _state_lock 保护所有共享状态
    - 状态机: IDLE -> STARTING -> STREAMING -> STOPPING -> IDLE
    - 客户端断开: 自动检测并清理资源
    - 重复命令: 正确处理快速连续的命令

    工作流程:
    1. 启动 TCP 服务器监听 13579 端口
    2. Unity Client 连接并发送 OPEN_CAMERA 命令
    3. 解析命令，获取目标 IP 和端口
    4. 启动 H.264 视频流发送
    5. 收到 CLOSE_CAMERA 或客户端断开时停止
    """

    def __init__(self, device_id: int = 0):
        self.device_id = device_id
        self.tcp_server: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
        self.video_sender: Optional[SimpleH264Sender] = None
        self.is_running = False
        self.client_thread: Optional[threading.Thread] = None
        self.adb_connected = False

        # 线程安全: 状态锁
        self._state_lock = threading.RLock()
        self._streaming_state = StreamingState.IDLE
        self._active_client_id: Optional[str] = None  # 当前活跃客户端标识

        # 回调
        self.on_client_connected: Optional[Callable[[str], None]] = None
        self.on_streaming_started: Optional[Callable[[str, int], None]] = None
        self.on_streaming_stopped: Optional[Callable[[], None]] = None

    def _get_state(self) -> StreamingState:
        """获取当前状态 (线程安全)"""
        with self._state_lock:
            return self._streaming_state

    def _set_state(self, state: StreamingState):
        """设置状态 (线程安全)"""
        with self._state_lock:
            old_state = self._streaming_state
            self._streaming_state = state
            logger.info(f"状态变化: {old_state.value} -> {state.value}")

    def _check_adb_connection(self) -> bool:
        """检测是否有 ADB 设备连接"""
        try:
            import subprocess
            result = subprocess.run(['adb', 'devices'], capture_output=True, text=True, timeout=5)
            lines = result.stdout.strip().split('\n')[1:]
            devices = [l.split('\t')[0] for l in lines if '\tdevice' in l]
            return len(devices) > 0
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    def _setup_adb_forward(self, port: int) -> bool:
        """设置 ADB forward (PC -> PICO)"""
        try:
            import subprocess
            result = subprocess.run(
                ['adb', 'forward', f'tcp:{port}', f'tcp:{port}'],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                logger.info(f"ADB forward 已设置: PC:127.0.0.1:{port} -> PICO:127.0.0.1:{port}")
                return True
            else:
                logger.warning(f"ADB forward 失败: {result.stderr}")
                return False
        except Exception as e:
            logger.warning(f"ADB forward 异常: {e}")
            return False

    def _setup_adb_reverse(self, port: int) -> bool:
        """设置 ADB reverse (PICO -> PC)"""
        try:
            import subprocess
            result = subprocess.run(
                ['adb', 'reverse', f'tcp:{port}', f'tcp:{port}'],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                logger.info(f"ADB reverse 已设置: PICO:127.0.0.1:{port} -> PC:127.0.0.1:{port}")
                return True
            else:
                logger.warning(f"ADB reverse 失败: {result.stderr}")
                return False
        except Exception as e:
            logger.warning(f"ADB reverse 异常: {e}")
            return False

    def start(self) -> bool:
        """启动服务器"""
        try:
            # 检测 ADB 连接
            self.adb_connected = self._check_adb_connection()
            if self.adb_connected:
                logger.info("检测到 ADB USB 连接，将使用 127.0.0.1 进行视频传输")
                self._setup_adb_reverse(ProtocolConstants.TCP_PORT)
            else:
                logger.info("未检测到 ADB 连接，将使用 WiFi IP 进行视频传输")

            # 创建 TCP 服务器
            self.tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.tcp_server.bind(('0.0.0.0', ProtocolConstants.TCP_PORT))
            self.tcp_server.listen(1)

            self.is_running = True

            logger.info("=" * 60)
            logger.info("XRoboToolkit 兼容服务器已启动")
            logger.info("=" * 60)
            logger.info(f"TCP 监听端口: {ProtocolConstants.TCP_PORT}")
            logger.info(f"相机设备: {self.device_id}")
            logger.info("")
            logger.info("等待 PICO 头显连接...")
            logger.info("请在 Unity Client 中:")
            logger.info("  1. 选择视频源: ADB 或 WIFI")
            logger.info("  2. 确认 IP 地址 (ADB: 127.0.0.1, WIFI: 服务器IP)")
            logger.info("  3. 点击 Listen 按钮")
            logger.info("")
            logger.info("生命周期: 断开后自动清理，可重复连接")
            logger.info("=" * 60)

            # 接受连接
            while self.is_running:
                try:
                    self.tcp_server.settimeout(1.0)
                    client, addr = self.tcp_server.accept()
                    client_id = f"{addr[0]}:{addr[1]}"
                    logger.info(f"客户端已连接: {client_id}")

                    if self.on_client_connected:
                        self.on_client_connected(addr[0])

                    # 在新线程中处理客户端
                    self.client_socket = client
                    self.client_thread = threading.Thread(
                        target=self._handle_client,
                        args=(client, addr, client_id),
                        daemon=True
                    )
                    self.client_thread.start()

                except socket.timeout:
                    continue
                except Exception as e:
                    if self.is_running:
                        logger.error(f"接受连接错误: {e}")

            return True

        except Exception as e:
            logger.error(f"启动服务器失败: {e}")
            return False

    def _handle_client(self, client: socket.socket, addr: tuple, client_id: str):
        """
        处理客户端连接 (线程安全)

        Unity Client 使用"一次性命令"模式:
        1. 连接 -> 2. 发送命令 -> 3. 立即断开

        生命周期管理:
        - 记录哪个客户端启动了视频流
        - 客户端断开时，如果是它启动的流，则停止流
        """
        buffer = b''
        started_streaming = False  # 此客户端是否启动了视频流

        try:
            client.settimeout(5.0)

            while self.is_running:
                try:
                    data = client.recv(4096)

                    if not data:
                        logger.info(f"客户端完成命令发送: {client_id}")
                        break

                    buffer += data
                    _, json_data = PacketParser.parse(buffer)

                    if json_data:
                        buffer = b''
                        func_name = json_data.get('functionName', '')

                        # 处理命令并记录是否启动了视频流
                        if func_name in ['OpenCamera', 'StartReceivePcCamera']:
                            started_streaming = True
                            with self._state_lock:
                                self._active_client_id = client_id

                        self._process_command(json_data, client)

                except socket.timeout:
                    break

                except ConnectionResetError:
                    logger.info(f"客户端连接重置: {client_id}")
                    break

        except Exception as e:
            logger.warning(f"客户端处理异常: {e}")

        finally:
            client.close()

            # 检查是否需要停止视频流
            # 只有当视频流正在运行，且没有收到显式的 CLOSE 命令时才停止
            with self._state_lock:
                current_state = self._streaming_state
                active_client = self._active_client_id

            # 如果此客户端启动了视频流，但没有发送关闭命令就断开了
            # 我们选择让视频流继续运行，因为:
            # 1. Unity Client 的"一次性命令"模式会在发送后立即断开
            # 2. 真正的关闭应该由显式的 CLOSE_CAMERA 命令触发
            # 3. 如果需要"客户端断开就停止"的行为，取消下面的注释

            # if started_streaming and current_state == StreamingState.STREAMING:
            #     if active_client == client_id:
            #         logger.info(f"客户端 {client_id} 断开，停止视频流")
            #         self._stop_video_stream_internal()

            logger.debug(f"客户端连接已关闭: {client_id}")

    def _process_command(self, json_data: dict, client: socket.socket):
        """处理命令 (线程安全)"""
        func_name = json_data.get('functionName', '')
        value = json_data.get('value', {})

        if isinstance(value, str):
            try:
                value = json.loads(value)
            except:
                pass

        logger.info(f"处理命令: {func_name}")

        if func_name == 'StartReceivePcCamera':
            self._handle_start_camera(value, client)

        elif func_name == 'StopReceivePcCamera':
            self._handle_stop_camera()

        elif func_name == 'requestCameraList':
            self._handle_camera_list(client)

        elif func_name == 'OpenCamera':
            self._handle_open_camera(value, client)

        else:
            logger.warning(f"未知命令: {func_name}")

    def _handle_start_camera(self, params: dict, client: socket.socket):
        """处理 StartReceivePcCamera 命令"""
        self._start_video_stream(params, client)

    def _handle_open_camera(self, params: dict, client: socket.socket):
        """处理 OpenCamera 命令"""
        self._start_video_stream(params, client)

    def _start_video_stream(self, params: dict, client: socket.socket):
        """
        启动视频流 (线程安全)

        状态机: IDLE/STREAMING -> STARTING -> STREAMING
        """
        with self._state_lock:
            current_state = self._streaming_state

            # 如果已经在启动中，忽略重复命令
            if current_state == StreamingState.STARTING:
                logger.warning("视频流正在启动中，忽略重复命令")
                return

            # 如果正在停止中，等待停止完成
            if current_state == StreamingState.STOPPING:
                logger.warning("视频流正在停止中，等待完成...")
                # 释放锁，等待停止完成
                self._state_lock.release()
                time.sleep(0.5)
                self._state_lock.acquire()

            # 如果已经在运行，先停止
            if current_state == StreamingState.STREAMING:
                logger.info("已有视频流运行，先停止旧流")
                self._stop_video_stream_internal_unlocked()
                time.sleep(0.3)

            # 设置状态为启动中
            self._streaming_state = StreamingState.STARTING

        # 解析参数
        target_ip = params.get('clientIp', params.get('ip', ''))
        target_port = params.get('clientPort', params.get('port', 12345))

        # 等待 PICO MediaDecoder 准备好
        # MediaDecoder.startServer() 需要一些时间来启动 TCP 监听
        # 这个延迟是关键，否则会出现 Broken pipe
        logger.info("等待 PICO MediaDecoder 启动监听...")
        wait_time = 3.0  # 增加到 3 秒，确保 MediaDecoder 准备好
        logger.info(f"等待 {wait_time} 秒...")
        time.sleep(wait_time)

        # ADB 模式处理
        if self.adb_connected:
            logger.info(f"ADB 模式: 将目标 IP 从 {target_ip} 改为 127.0.0.1")
            target_ip = "127.0.0.1"
            self._setup_adb_forward(target_port)

        logger.info(f"启动视频流: {target_ip}:{target_port}")
        logger.info(f"参数: {params.get('width', 2560)}x{params.get('height', 720)} @ "
                    f"{params.get('fps', 60)}fps, {params.get('bitrate', 8000000)//1000000}Mbps")

        # 创建视频发送器
        config = VideoConfig(
            width=params.get('width', 2560),
            height=params.get('height', 720),
            fps=params.get('fps', 60),
            bitrate=params.get('bitrate', 8000000)
        )

        try:
            sender = SimpleH264Sender(config)

            if not sender.initialize(self.device_id):
                logger.error("相机初始化失败")
                self._send_error(client, "Camera initialization failed")
                self._set_state(StreamingState.IDLE)
                return

            # 设置连接断开回调
            sender.on_connection_lost = self._on_video_connection_lost

            if sender.start_streaming(target_ip, target_port):
                with self._state_lock:
                    self.video_sender = sender
                    self._streaming_state = StreamingState.STREAMING

                logger.info("视频流已启动")

                if self.on_streaming_started:
                    self.on_streaming_started(target_ip, target_port)
            else:
                logger.error("启动视频流失败")
                self._send_error(client, "Failed to start video stream")
                self._set_state(StreamingState.IDLE)

        except Exception as e:
            logger.error(f"启动视频流异常: {e}")
            self._send_error(client, f"Exception: {e}")
            self._set_state(StreamingState.IDLE)

    def _on_video_connection_lost(self, reason: str):
        """
        视频连接断开回调

        这个回调在单独线程中执行 (由 h264_sender._handle_connection_lost 启动)
        用于自动清理资源，准备接受新的连接
        """
        logger.warning(f"视频连接断开: {reason}")
        logger.info("正在清理资源，准备接受新连接...")

        # 在新线程中清理，避免阻塞回调
        def cleanup():
            try:
                self._stop_video_stream_internal()
                logger.info("资源已清理，可以接受新的连接")
            except Exception as e:
                logger.error(f"清理资源失败: {e}")

        cleanup_thread = threading.Thread(target=cleanup, daemon=True)
        cleanup_thread.start()

    def _handle_stop_camera(self):
        """处理 StopReceivePcCamera 命令"""
        logger.info("收到停止视频流命令")
        self._stop_video_stream_internal()

        if self.on_streaming_stopped:
            self.on_streaming_stopped()

    def _stop_video_stream_internal(self):
        """停止视频流 (线程安全，外部调用)"""
        with self._state_lock:
            self._stop_video_stream_internal_unlocked()

    def _stop_video_stream_internal_unlocked(self):
        """
        停止视频流 (假设已持有锁)

        确保无论发生什么错误，都能正确清理资源
        """
        if self._streaming_state == StreamingState.IDLE:
            logger.debug("视频流已经停止")
            return

        if self._streaming_state == StreamingState.STOPPING:
            logger.debug("视频流正在停止中")
            return

        self._streaming_state = StreamingState.STOPPING
        logger.info("正在停止视频流...")

        # 清理 video_sender
        if self.video_sender:
            try:
                self.video_sender.stop_streaming()
                logger.info("视频发送器已停止")
            except Exception as e:
                logger.warning(f"停止视频发送器时出现异常: {e}")
            finally:
                self.video_sender = None

        self._streaming_state = StreamingState.IDLE
        self._active_client_id = None
        logger.info("视频流已停止，等待新连接")

    def _handle_camera_list(self, client: socket.socket):
        """处理相机列表请求"""
        response = PacketParser.build_response(
            ProtocolConstants.CMD_FUNCTION,
            {
                "functionName": "onCameraList",
                "cameras": [
                    {
                        "name": "ADB",
                        "type": "USB",
                        "description": "USB有线模式 (ADB)"
                    },
                    {
                        "name": "WIFI",
                        "type": "USB",
                        "description": "WiFi无线模式"
                    }
                ]
            }
        )
        client.send(response)

    def _send_error(self, client: socket.socket, message: str):
        """发送错误响应"""
        response = PacketParser.build_response(
            ProtocolConstants.CMD_FUNCTION,
            {
                "functionName": "OnError",
                "error": message
            }
        )
        try:
            client.send(response)
        except:
            pass

    def stop(self):
        """停止服务器"""
        logger.info("正在停止服务器...")
        self.is_running = False

        self._stop_video_stream_internal()

        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass

        if self.tcp_server:
            try:
                self.tcp_server.close()
            except:
                pass

        if self.client_thread:
            self.client_thread.join(timeout=2)

        logger.info("服务器已停止")


def get_local_ip() -> str:
    """获取本机局域网 IP"""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return "127.0.0.1"


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='XRoboToolkit 协议兼容服务器 - 让 PICO 头显接收 StereoVR 视频流'
    )
    parser.add_argument(
        '--device', '-d',
        type=int,
        default=0,
        help='相机设备 ID (默认: 0)'
    )
    parser.add_argument(
        '--test',
        action='store_true',
        help='测试模式 (不启动服务器，仅测试相机)'
    )

    args = parser.parse_args()

    # 显示本机 IP
    local_ip = get_local_ip()
    print()
    print("=" * 60)
    print("StereoVR XRoboToolkit 兼容服务器")
    print("=" * 60)
    print(f"本机 IP: {local_ip}")
    print(f"请在 PICO 头显的 Unity Client 中输入此 IP")
    print("=" * 60)
    print()

    if args.test:
        print("测试模式: 检测相机...")
        import cv2
        cap = cv2.VideoCapture(args.device)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"相机正常: {frame.shape}")
            cap.release()
        else:
            print("无法打开相机")
        return

    # 启动服务器
    server = XRoboCompatServer(device_id=args.device)

    try:
        server.start()
    except KeyboardInterrupt:
        print("\n收到中断信号")
    finally:
        server.stop()


if __name__ == '__main__':
    main()
