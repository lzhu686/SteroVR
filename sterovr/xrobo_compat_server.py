#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
XRoboToolkit 协议兼容服务器
模拟 XRoboToolkit PC-Service 的行为，让 PICO 头显可以直接连接并接收视频流

协议说明:
1. TCP 端口 13579: 接收来自 Unity Client 的控制命令
2. UDP 端口 (动态): 向 Unity Client 发送 H.264 视频流

命令格式:
- Unity Client 发送 JSON 命令: {"functionName": "StartReceivePcCamera", "value": {...}}
- 本服务器解析命令并启动视频流发送

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

# 导入 H.264 发送器
from .h264_sender import SimpleH264Sender, VideoConfig

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('XRoboCompat')


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

        例如 OPEN_CAMERA:
        00 00 00 43  - total length (67 bytes, Big-Endian)
        0b 00 00 00  - cmd_len (11, Little-Endian)
        4f 50 45 4e 5f 43 41 4d 45 52 41  - "OPEN_CAMERA"
        30 00 00 00  - data_len (48 bytes, Little-Endian)
        ca fe 01 ... - CameraRequest data
        """
        try:
            # 打印收到的原始数据前64字节（用于调试）
            hex_preview = data[:min(64, len(data))].hex()
            logger.info(f"[DEBUG] 收到数据 ({len(data)} bytes): {hex_preview}...")

            if len(data) < 12:
                logger.debug(f"[DEBUG] 数据太短: {len(data)} < 12")
                return None, None

            offset = 0

            # 检查是否有 Big-Endian 长度前缀
            # 如果前4字节作为 Big-Endian 是合理的长度值，则跳过它
            potential_total_len = struct.unpack('>I', data[0:4])[0]
            if 10 < potential_total_len < 1000 and potential_total_len <= len(data):
                # 有总长度前缀，跳过它
                offset = 4
                logger.info(f"[DEBUG] 检测到长度前缀: {potential_total_len} bytes")

            # 读取命令长度 (Little-Endian)
            cmd_len = struct.unpack('<I', data[offset:offset+4])[0]
            offset += 4
            logger.info(f"[DEBUG] cmd_len = {cmd_len}")

            # 验证命令长度合理性
            if cmd_len <= 0 or cmd_len > 100:
                logger.info(f"[DEBUG] cmd_len 不合理: {cmd_len}")
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

            logger.info(f"解析 NetworkDataProtocol: command={command}, data_len={data_len}")

            # 根据命令类型处理
            if command == "OPEN_CAMERA":
                # 解析 CameraRequest 二进制数据
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
        参考 Unity CameraRequestSerializer

        格式:
        [Magic: 0xCA 0xFE][Version: 1byte][width:4][height:4][fps:4][bitrate:4]
        [enableMvHevc:4][renderMode:4][port:4][camera_len:1][camera][ip_len:1][ip]
        """
        try:
            if len(data) < 10:
                logger.warning(f"CameraRequest 数据太短: {len(data)} bytes")
                return None

            offset = 0

            # 检查魔数 0xCA 0xFE
            if data[0] == 0xCA and data[1] == 0xFE:
                offset = 2
                # 读取版本号
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

            # 读取 camera 字符串 (1字节长度前缀)
            camera_len = data[offset]
            offset += 1
            camera = data[offset:offset+camera_len].decode('utf-8') if camera_len > 0 else "USB"
            offset += camera_len

            # 读取 ip 字符串 (1字节长度前缀)
            ip_len = data[offset]
            offset += 1
            ip = data[offset:offset+ip_len].decode('utf-8') if ip_len > 0 else ""
            offset += ip_len

            logger.info(f"CameraRequest: {width}x{height}@{fps}fps, {bitrate}bps, ip={ip}:{port}, camera={camera}")

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
        # 查找包头
        head_idx = -1
        for i in range(len(data)):
            if data[i] == ProtocolConstants.PACKET_HEAD_RECV or data[i] == ProtocolConstants.PACKET_HEAD_SEND:
                head_idx = i
                break

        if head_idx < 0:
            return None, None

        try:
            # 解析包头后的命令字节
            cmd = data[head_idx + 1]

            # 解析长度 (4字节小端)
            length = struct.unpack('<I', data[head_idx + 2:head_idx + 6])[0]

            # 提取数据部分
            data_start = head_idx + 6
            data_end = data_start + length

            if data_end > len(data):
                return None, None

            payload = data[data_start:data_end]

            # 尝试解析为 JSON
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
            # 查找 JSON 起始和结束
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

        # 构建包
        packet = bytearray()
        packet.append(ProtocolConstants.PACKET_HEAD_SEND)
        packet.append(cmd)
        packet.extend(struct.pack('<I', len(json_bytes)))
        packet.extend(json_bytes)
        packet.extend(struct.pack('<Q', int(time.time() * 1000)))  # 时间戳
        packet.append(ProtocolConstants.PACKET_END)

        return bytes(packet)


class XRoboCompatServer:
    """
    XRoboToolkit 兼容服务器

    工作流程:
    1. 启动 TCP 服务器监听 63901 端口
    2. Unity Client 连接并发送 StartReceivePcCamera 命令
    3. 解析命令，获取目标 IP 和端口
    4. 启动 H.264 视频流发送
    5. 收到 StopReceivePcCamera 时停止
    """

    def __init__(self, device_id: int = 0):
        self.device_id = device_id
        self.tcp_server: Optional[socket.socket] = None
        self.client_socket: Optional[socket.socket] = None
        self.video_sender: Optional[SimpleH264Sender] = None
        self.is_running = False
        self.client_thread: Optional[threading.Thread] = None
        self.adb_connected = False  # ADB 连接状态

        # 回调
        self.on_client_connected: Optional[Callable[[str], None]] = None
        self.on_streaming_started: Optional[Callable[[str, int], None]] = None
        self.on_streaming_stopped: Optional[Callable[[], None]] = None

    def _check_adb_connection(self) -> bool:
        """检测是否有 ADB 设备连接"""
        try:
            import subprocess
            result = subprocess.run(['adb', 'devices'], capture_output=True, text=True, timeout=5)
            lines = result.stdout.strip().split('\n')[1:]  # 跳过标题行
            devices = [l.split('\t')[0] for l in lines if '\tdevice' in l]
            return len(devices) > 0
        except (FileNotFoundError, subprocess.TimeoutExpired):
            return False

    def _setup_adb_forward(self, port: int) -> bool:
        """
        设置 ADB forward (PC -> PICO)
        用于视频端口: PC 连接 127.0.0.1:port -> PICO 的 127.0.0.1:port
        """
        try:
            import subprocess
            result = subprocess.run(
                ['adb', 'forward', f'tcp:{port}', f'tcp:{port}'],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                logger.info(f"✅ ADB forward 已设置: PC:127.0.0.1:{port} -> PICO:127.0.0.1:{port}")
                return True
            else:
                logger.warning(f"ADB forward 失败: {result.stderr}")
                return False
        except Exception as e:
            logger.warning(f"ADB forward 异常: {e}")
            return False

    def _setup_adb_reverse(self, port: int) -> bool:
        """
        设置 ADB reverse (PICO -> PC)
        用于控制端口: PICO 连接 127.0.0.1:port -> PC 的 127.0.0.1:port
        """
        try:
            import subprocess
            result = subprocess.run(
                ['adb', 'reverse', f'tcp:{port}', f'tcp:{port}'],
                capture_output=True, text=True, timeout=5
            )
            if result.returncode == 0:
                logger.info(f"✅ ADB reverse 已设置: PICO:127.0.0.1:{port} -> PC:127.0.0.1:{port}")
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
                logger.info("✅ 检测到 ADB USB 连接，将使用 127.0.0.1 进行视频传输")
                # 设置控制端口的 reverse (PICO 连接 127.0.0.1:13579 -> PC 的服务器)
                self._setup_adb_reverse(ProtocolConstants.TCP_PORT)
            else:
                logger.info("ℹ️  未检测到 ADB 连接，将使用 WiFi IP 进行视频传输")

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
            logger.info("  1. 选择视频源: USB_STEREO")
            logger.info("  2. 输入本机 IP 地址")
            logger.info("  3. 点击 Listen 按钮")
            logger.info("=" * 60)

            # 接受连接
            while self.is_running:
                try:
                    self.tcp_server.settimeout(1.0)
                    client, addr = self.tcp_server.accept()
                    logger.info(f"客户端已连接: {addr}")

                    if self.on_client_connected:
                        self.on_client_connected(addr[0])

                    # 在新线程中处理客户端
                    self.client_socket = client
                    self.client_thread = threading.Thread(
                        target=self._handle_client,
                        args=(client, addr),
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

    def _handle_client(self, client: socket.socket, addr: tuple):
        """
        处理客户端连接

        Unity Client 使用"一次性命令"模式:
        1. 连接 → 2. 发送命令 → 3. 立即断开
        这是正常行为，不是错误。
        """
        buffer = b''

        try:
            # 设置较短的超时，因为 Unity Client 发送命令后会立即断开
            client.settimeout(5.0)

            while self.is_running:
                try:
                    data = client.recv(4096)

                    if not data:
                        # 客户端正常关闭连接
                        logger.info(f"客户端完成命令发送: {addr}")
                        break

                    buffer += data

                    # 解析命令
                    _, json_data = PacketParser.parse(buffer)

                    if json_data:
                        buffer = b''
                        self._process_command(json_data, client)
                        # 命令处理完成后，Unity Client 会断开
                        # 不需要继续等待更多数据

                except socket.timeout:
                    # 超时 = 没有更多数据，正常退出
                    break

                except ConnectionResetError:
                    # Unity Client 发送命令后立即关闭连接，这是正常的
                    logger.info(f"客户端命令发送完成 (连接已关闭): {addr}")
                    break

        except Exception as e:
            logger.warning(f"客户端处理异常: {e}")

        finally:
            client.close()
            # 注意：不要在这里停止视频流，让它继续运行

    def _process_command(self, json_data: dict, client: socket.socket):
        """处理命令"""
        func_name = json_data.get('functionName', '')
        value = json_data.get('value', {})

        # 如果 value 是字符串，尝试解析为 JSON
        if isinstance(value, str):
            try:
                value = json.loads(value)
            except:
                pass

        logger.info(f"收到命令: {func_name}")
        logger.debug(f"命令参数: {value}")

        if func_name == 'StartReceivePcCamera':
            self._handle_start_camera(value, client)

        elif func_name == 'StopReceivePcCamera':
            self._handle_stop_camera()

        elif func_name == 'requestCameraList':
            self._handle_camera_list(client)

        elif func_name == 'OpenCamera':
            # 处理 OpenCamera 命令 (来自 NetworkCommander)
            self._handle_open_camera(value, client)

        else:
            logger.warning(f"未知命令: {func_name}")

    def _handle_start_camera(self, params: dict, client: socket.socket):
        """处理 StartReceivePcCamera 命令"""
        request = CameraRequest.from_json(params)

        # 如果检测到 ADB 连接，使用 127.0.0.1 替代客户端报告的 IP
        original_ip = request.ip
        if self.adb_connected:
            request.ip = "127.0.0.1"
            logger.info(f"🔌 ADB 模式: 将目标 IP 从 {original_ip} 改为 127.0.0.1")
            # 等待 MediaDecoder 启动监听
            logger.info("⏳ 等待 PICO MediaDecoder 启动监听 (2秒)...")
            import time
            time.sleep(2.0)
            # 设置端口转发 (视频端口: PC连接PICO，用forward)
            self._setup_adb_forward(request.port)

        logger.info(f"开始视频流: {request.ip}:{request.port}")
        logger.info(f"参数: {request.width}x{request.height} @ {request.fps}fps, {request.bitrate//1000000}Mbps")

        # 停止现有流
        self._stop_video_stream()

        # 创建视频发送器
        config = VideoConfig(
            width=request.width,
            height=request.height,
            fps=request.fps,
            bitrate=request.bitrate
        )

        self.video_sender = SimpleH264Sender(config)

        if not self.video_sender.initialize(self.device_id):
            logger.error("相机初始化失败")
            self._send_error(client, "Camera initialization failed")
            return

        if self.video_sender.start_streaming(request.ip, request.port):
            logger.info("视频流已启动")

            # 发送成功响应
            response = PacketParser.build_response(
                ProtocolConstants.CMD_FUNCTION,
                {
                    "functionName": "OnCameraStarted",
                    "status": "success",
                    "width": request.width,
                    "height": request.height,
                    "fps": request.fps
                }
            )
            client.send(response)

            if self.on_streaming_started:
                self.on_streaming_started(request.ip, request.port)
        else:
            logger.error("启动视频流失败")
            self._send_error(client, "Failed to start video stream")

    def _handle_open_camera(self, params: dict, client: socket.socket):
        """处理 OpenCamera 命令 (来自新版协议)"""
        # 新版协议的参数格式可能不同，需要适配
        target_ip = params.get('clientIp', params.get('ip', ''))
        target_port = params.get('clientPort', params.get('port', 12345))

        # 如果检测到 ADB 连接，使用 127.0.0.1 替代客户端报告的 IP
        if self.adb_connected:
            logger.info(f"🔌 ADB 模式: 将目标 IP 从 {target_ip} 改为 127.0.0.1")
            target_ip = "127.0.0.1"
            # 等待 MediaDecoder 启动监听
            logger.info("⏳ 等待 PICO MediaDecoder 启动监听 (2秒)...")
            import time
            time.sleep(2.0)
            # 设置端口转发 (视频端口: PC连接PICO，用forward)
            self._setup_adb_forward(target_port)

        # 直接启动视频流，不再调用 _handle_start_camera 避免重复检测
        logger.info(f"开始视频流: {target_ip}:{target_port}")
        logger.info(f"参数: {params.get('width', 2560)}x{params.get('height', 720)} @ {params.get('fps', 60)}fps, {params.get('bitrate', 8000000)//1000000}Mbps")

        # 停止现有流
        self._stop_video_stream()

        # 创建视频发送器
        config = VideoConfig(
            width=params.get('width', 2560),
            height=params.get('height', 720),
            fps=params.get('fps', 60),
            bitrate=params.get('bitrate', 8000000)
        )

        self.video_sender = SimpleH264Sender(config)

        if not self.video_sender.initialize(self.device_id):
            logger.error("相机初始化失败")
            self._send_error(client, "Camera initialization failed")
            return

        if self.video_sender.start_streaming(target_ip, target_port):
            logger.info("视频流已启动")

            if self.on_streaming_started:
                self.on_streaming_started(target_ip, target_port)
        else:
            logger.error("启动视频流失败")
            self._send_error(client, "Failed to start video stream")

    def _handle_stop_camera(self):
        """处理 StopReceivePcCamera 命令"""
        logger.info("停止视频流")
        self._stop_video_stream()

        if self.on_streaming_stopped:
            self.on_streaming_stopped()

    def _handle_camera_list(self, client: socket.socket):
        """处理相机列表请求"""
        response = PacketParser.build_response(
            ProtocolConstants.CMD_FUNCTION,
            {
                "functionName": "onCameraList",
                "cameras": [
                    {
                        "name": "USB_STEREO",
                        "type": "USB",
                        "description": "USB Stereo Camera (StereoVR)"
                    }
                ]
            }
        )
        client.send(response)

    def _stop_video_stream(self):
        """停止视频流"""
        if self.video_sender:
            self.video_sender.stop_streaming()
            self.video_sender = None

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

        self._stop_video_stream()

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
    print("StereoVR → XRoboToolkit 兼容服务器")
    print("=" * 60)
    print(f"本机 IP: {local_ip}")
    print(f"请在 PICO 头显的 Unity Client 中输入此 IP")
    print("=" * 60)
    print()

    if args.test:
        # 测试模式
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
