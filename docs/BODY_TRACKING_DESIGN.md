# PICO 体感追踪 + OpenCV 双目相机集成方案

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              完整数据流                                      │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                     PICO 头显 (修改版 Unity Client)                  │   │
│  │                                                                      │   │
│  │   ┌─────────────────────┐      ┌─────────────────────────────────┐  │   │
│  │   │ 追踪数据采集 (原有)  │      │ 立体视觉显示 (新增)              │  │   │
│  │   │ - 头部 6DoF         │      │ - WebSocket 接收相机帧           │  │   │
│  │   │ - 手柄 6DoF         │      │ - 左右眼分离渲染                 │  │   │
│  │   │ - 全身 24 关节      │      │ - VR 立体显示                    │  │   │
│  │   └─────────┬───────────┘      └──────────────┬──────────────────┘  │   │
│  └─────────────┼─────────────────────────────────┼──────────────────────┘   │
│                │ gRPC (追踪数据)                  │ WebSocket (视频帧)       │
│                ▼                                  ▼                          │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │                         Linux PC (Ubuntu 22.04)                      │   │
│  │                                                                      │   │
│  │   ┌─────────────────────┐      ┌─────────────────────────────────┐  │   │
│  │   │ XRoboToolkit        │      │ 你的 server.py                   │  │   │
│  │   │ PC-Service          │      │                                  │  │   │
│  │   │ - 接收追踪数据       │      │ ┌─────────────────────────────┐ │  │   │
│  │   │ - gRPC 服务         │      │ │ OpenCV 双目相机采集          │ │  │   │
│  │   └─────────┬───────────┘      │ │ - cv2.VideoCapture()        │ │  │   │
│  │             │                   │ │ - 左右图像分离               │ │  │   │
│  │   ┌─────────▼───────────┐      │ │ - JPEG 编码                  │ │  │   │
│  │   │ Python 绑定 (pxrea) │      │ └─────────────────────────────┘ │  │   │
│  │   │ - 获取追踪数据       │─────>│                                  │  │   │
│  │   │ - 回调函数          │      │ ┌─────────────────────────────┐ │  │   │
│  │   └─────────────────────┘      │ │ WebSocket 服务器             │ │  │   │
│  │                                 │ │ - 发送相机帧                 │ │  │   │
│  │                                 │ │ - 发送追踪数据               │ │  │   │
│  │                                 │ └─────────────────────────────┘ │  │   │
│  │                                 └─────────────────────────────────┘  │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

# 第一部分：PC 端环境搭建

## 1.1 安装 XRoboToolkit PC-Service

PC-Service 是运行在电脑上的后台服务，负责接收 PICO 头显发来的追踪数据。

```bash
# === 方法 A：直接安装预编译包 (推荐) ===

# 下载最新版本
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.1.1/xrobotoolkit-pc-service_1.1.1_amd64.deb

# 安装
sudo dpkg -i xrobotoolkit-pc-service_1.1.1_amd64.deb

# 如果有依赖问题，运行：
sudo apt-get install -f

# 启动服务 (会在后台运行)
xrobotoolkit-pc-service
```

```bash
# === 方法 B：从源码编译 ===

# 克隆仓库
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git
cd XRoboToolkit-PC-Service

# 安装依赖
sudo apt-get install build-essential cmake qt6-base-dev libgrpc++-dev protobuf-compiler-grpc

# 编译
./build_linux.sh

# 运行
./build/xrobotoolkit-pc-service
```

**验证安装**：
```bash
# 查看服务是否运行
ps aux | grep xrobotoolkit

# 查看监听端口 (默认 50051)
netstat -tlnp | grep 50051
```

---

## 1.2 安装 Python 绑定 (pxrea)

pxrea 是 Python 接口，让你的 server.py 能够获取追踪数据。

```bash
# 创建 Python 环境
conda create -n xrobot python=3.10
conda activate xrobot

# 克隆 Python 绑定
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git
cd XRoboToolkit-PC-Service-Pybind

# 安装
pip install .

# 验证安装
python -c "import pxrea; print('pxrea 安装成功')"
```

---

## 1.3 修改你的 server.py

在你现有的 server.py 中添加追踪数据接收功能：

```python
# server.py - 完整示例

import asyncio
import websockets
import json
import cv2
import threading
import ssl
from typing import Optional

# ========== 追踪模块 ==========

try:
    import pxrea
    HAS_TRACKING = True
    print("[OK] pxrea 模块加载成功")
except ImportError:
    HAS_TRACKING = False
    print("[警告] pxrea 未安装，追踪功能不可用")


class BodyTracker:
    """PICO 体感追踪数据接收器"""

    # 关节名称映射
    JOINT_NAMES = [
        'Pelvis', 'SpineLower', 'SpineMiddle', 'SpineUpper',
        'Chest', 'Neck', 'Head',
        'LeftShoulder', 'LeftUpperArm', 'LeftLowerArm', 'LeftHand',
        'RightShoulder', 'RightUpperArm', 'RightLowerArm', 'RightHand',
        'LeftUpperLeg', 'LeftLowerLeg', 'LeftFoot',
        'RightUpperLeg', 'RightLowerLeg', 'RightFoot',
        'LeftToes', 'RightToes', 'LeftFingers', 'RightFingers'
    ]

    def __init__(self):
        self.body_data = None
        self.head_data = None
        self.controller_data = {'left': None, 'right': None}
        self.lock = threading.Lock()
        self.frame_id = 0
        self.connected = False

    def _data_callback(self, data_type: int, data: dict):
        """接收追踪数据的回调函数"""
        with self.lock:
            if data_type == 0x01:  # HEAD
                self.head_data = data
            elif data_type == 0x02:  # CONTROLLER
                self.controller_data = data
            elif data_type == 0x08:  # BODY (全身追踪)
                self.body_data = data
                self.frame_id += 1

    def start(self) -> bool:
        """启动追踪服务"""
        if not HAS_TRACKING:
            print("[错误] pxrea 未安装")
            return False

        # 初始化 pxrea
        # 参数: (config, callback, data_mask)
        # data_mask: 0x01=HEAD, 0x02=CONTROLLER, 0x04=HAND, 0x08=BODY, 0xFF=ALL
        result = pxrea.PXREAInit(None, self._data_callback, 0xFF)

        if result == 0:
            self.connected = True
            print("[OK] 追踪服务已启动，等待 PICO 连接...")
            return True
        else:
            print(f"[错误] 追踪服务启动失败，错误码: {result}")
            return False

    def get_tracking_frame(self) -> Optional[dict]:
        """获取最新一帧追踪数据"""
        with self.lock:
            if self.body_data is None:
                return None

            joints = {}
            raw_joints = self.body_data.get('joints', [])

            for i, joint in enumerate(raw_joints):
                if i < len(self.JOINT_NAMES):
                    joints[self.JOINT_NAMES[i]] = {
                        'position': joint.get('position', [0, 0, 0]),
                        'rotation': joint.get('rotation', [0, 0, 0, 1])
                    }

            return {
                'type': 'body_tracking',
                'frame_id': self.frame_id,
                'timestamp': self.body_data.get('timestamp', 0),
                'joints': joints,
                'head': self.head_data,
                'controllers': self.controller_data
            }

    def stop(self):
        """停止追踪服务"""
        if HAS_TRACKING and self.connected:
            pxrea.PXREADeinit()
            self.connected = False
            print("[OK] 追踪服务已停止")


# ========== 双目相机模块 ==========

class StereoCamera:
    """OpenCV 双目相机采集"""

    def __init__(self, device_id: int = 0, width: int = 2560, height: int = 720):
        """
        初始化双目相机

        参数:
            device_id: 相机设备 ID
            width: 双目图像总宽度 (左右拼接)
            height: 图像高度
        """
        self.cap = cv2.VideoCapture(device_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.width = width
        self.height = height
        self.half_width = width // 2

        self.lock = threading.Lock()
        self.left_frame = None
        self.right_frame = None
        self.frame_id = 0
        self.running = False

    def start(self):
        """启动相机采集线程"""
        if not self.cap.isOpened():
            print("[错误] 无法打开相机")
            return False

        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        print(f"[OK] 双目相机已启动 ({self.width}x{self.height})")
        return True

    def _capture_loop(self):
        """相机采集循环"""
        while self.running:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    # 分离左右图像
                    self.left_frame = frame[:, :self.half_width]
                    self.right_frame = frame[:, self.half_width:]
                    self.frame_id += 1

    def get_stereo_frame(self, quality: int = 80) -> Optional[dict]:
        """
        获取编码后的立体帧

        参数:
            quality: JPEG 压缩质量 (1-100)

        返回:
            包含左右眼 base64 图像的字典
        """
        import base64

        with self.lock:
            if self.left_frame is None or self.right_frame is None:
                return None

            # JPEG 编码
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            _, left_jpg = cv2.imencode('.jpg', self.left_frame, encode_param)
            _, right_jpg = cv2.imencode('.jpg', self.right_frame, encode_param)

            return {
                'type': 'stereo_frame',
                'frame_id': self.frame_id,
                'left': base64.b64encode(left_jpg).decode('utf-8'),
                'right': base64.b64encode(right_jpg).decode('utf-8'),
                'width': self.half_width,
                'height': self.height
            }

    def get_stereo_frame_binary(self) -> Optional[bytes]:
        """
        获取二进制格式的立体帧 (更高效)

        返回:
            格式: [4字节左图大小][左图JPEG][4字节右图大小][右图JPEG]
        """
        import struct

        with self.lock:
            if self.left_frame is None or self.right_frame is None:
                return None

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
            _, left_jpg = cv2.imencode('.jpg', self.left_frame, encode_param)
            _, right_jpg = cv2.imencode('.jpg', self.right_frame, encode_param)

            left_bytes = left_jpg.tobytes()
            right_bytes = right_jpg.tobytes()

            # 打包: [left_size(4B)][left_data][right_size(4B)][right_data]
            return (
                struct.pack('<I', len(left_bytes)) + left_bytes +
                struct.pack('<I', len(right_bytes)) + right_bytes
            )

    def stop(self):
        """停止相机"""
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
        self.cap.release()
        print("[OK] 相机已停止")


# ========== WebSocket 服务器 ==========

class StereoVRServer:
    """WebSocket 服务器，发送双目视频和追踪数据"""

    def __init__(self, host: str = '0.0.0.0', port: int = 8765):
        self.host = host
        self.port = port
        self.camera = StereoCamera()
        self.tracker = BodyTracker()
        self.clients = set()

    async def handler(self, websocket):
        """处理 WebSocket 连接"""
        self.clients.add(websocket)
        client_ip = websocket.remote_address[0]
        print(f"[连接] 客户端已连接: {client_ip}")

        try:
            while True:
                # 发送相机帧 (二进制格式)
                frame_data = self.camera.get_stereo_frame_binary()
                if frame_data:
                    await websocket.send(frame_data)

                # 发送追踪数据 (JSON 格式)
                tracking_data = self.tracker.get_tracking_frame()
                if tracking_data:
                    await websocket.send(json.dumps(tracking_data))

                # 控制帧率 (~30fps)
                await asyncio.sleep(1/30)

        except websockets.exceptions.ConnectionClosed:
            print(f"[断开] 客户端已断开: {client_ip}")
        finally:
            self.clients.discard(websocket)

    async def start(self):
        """启动服务器"""
        # 启动相机
        if not self.camera.start():
            return

        # 启动追踪
        self.tracker.start()

        # 配置 SSL (可选)
        ssl_context = None
        # ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        # ssl_context.load_cert_chain('cert.pem', 'key.pem')

        # 启动 WebSocket 服务器
        print(f"[OK] WebSocket 服务器启动: ws://{self.host}:{self.port}")

        async with websockets.serve(
            self.handler,
            self.host,
            self.port,
            ssl=ssl_context,
            max_size=10*1024*1024  # 10MB max message size
        ):
            await asyncio.Future()  # 永久运行

    def stop(self):
        """停止服务器"""
        self.camera.stop()
        self.tracker.stop()


# ========== 主函数 ==========

if __name__ == '__main__':
    server = StereoVRServer(host='0.0.0.0', port=8765)

    try:
        asyncio.run(server.start())
    except KeyboardInterrupt:
        print("\n[OK] 服务器已停止")
        server.stop()
```

---

## 1.4 测试 PC 端

```bash
# 1. 启动 PC-Service (如果不是自动启动)
xrobotoolkit-pc-service &

# 2. 运行你的 server.py
conda activate xrobot
python server.py

# 预期输出:
# [OK] pxrea 模块加载成功
# [OK] 双目相机已启动 (2560x720)
# [OK] 追踪服务已启动，等待 PICO 连接...
# [OK] WebSocket 服务器启动: ws://0.0.0.0:8765
```

---

# 第二部分：Unity Client 修改

## 2.1 获取 XRoboToolkit Unity Client

```bash
# 方法 1: Fork 到自己的账号 (推荐，方便后续更新)
gh repo fork XR-Robotics/XRoboToolkit-Unity-Client --clone
cd XRoboToolkit-Unity-Client

# 方法 2: 直接克隆
git clone https://github.com/XR-Robotics/XRoboToolkit-Unity-Client.git
cd XRoboToolkit-Unity-Client

# 创建开发分支
git checkout -b feature/opencv-stereo-vision
```

---

## 2.2 Unity 环境要求

| 要求 | 版本 |
|------|------|
| Unity | 2021.3 LTS 或更高 |
| Android Build Support | 已安装 |
| PICO Unity Integration SDK | 已配置 |

打开 Unity Hub → Add → 选择克隆的 XRoboToolkit-Unity-Client 文件夹

---

## 2.3 安装 WebSocket 依赖

在 Unity 中，打开 `Window → Package Manager`：

1. 点击 `+` → `Add package from git URL`
2. 输入: `https://github.com/endel/NativeWebSocket.git#upm`
3. 点击 `Add`

---

## 2.4 创建立体视觉接收脚本

在 `Assets/Scripts/` 下创建 `StereoVisionReceiver.cs`:

```csharp
// Assets/Scripts/StereoVisionReceiver.cs

using UnityEngine;
using NativeWebSocket;
using System;
using System.Threading.Tasks;

public class StereoVisionReceiver : MonoBehaviour
{
    [Header("服务器设置")]
    [Tooltip("PC 的 IP 地址")]
    public string serverIP = "192.168.1.100";

    [Tooltip("WebSocket 端口")]
    public int serverPort = 8765;

    [Tooltip("使用 WSS (安全连接)")]
    public bool useSSL = false;

    [Header("显示设置")]
    [Tooltip("左眼渲染材质")]
    public Material leftEyeMaterial;

    [Tooltip("右眼渲染材质")]
    public Material rightEyeMaterial;

    [Tooltip("渲染到的 Quad 对象")]
    public GameObject displayQuad;

    [Header("状态")]
    [SerializeField] private bool isConnected = false;
    [SerializeField] private int framesReceived = 0;
    [SerializeField] private float fps = 0;

    private WebSocket websocket;
    private Texture2D leftTexture;
    private Texture2D rightTexture;

    // 双缓冲，避免主线程阻塞
    private byte[] pendingLeftData;
    private byte[] pendingRightData;
    private bool hasNewFrame = false;
    private readonly object frameLock = new object();

    // FPS 计算
    private int frameCount = 0;
    private float fpsTimer = 0;

    async void Start()
    {
        // 初始化纹理
        leftTexture = new Texture2D(1280, 720, TextureFormat.RGB24, false);
        rightTexture = new Texture2D(1280, 720, TextureFormat.RGB24, false);

        // 应用纹理到材质
        if (leftEyeMaterial != null)
            leftEyeMaterial.mainTexture = leftTexture;
        if (rightEyeMaterial != null)
            rightEyeMaterial.mainTexture = rightTexture;

        await ConnectToServer();
    }

    async Task ConnectToServer()
    {
        string protocol = useSSL ? "wss" : "ws";
        string url = $"{protocol}://{serverIP}:{serverPort}";

        Debug.Log($"[StereoVision] 正在连接: {url}");

        websocket = new WebSocket(url);

        websocket.OnOpen += () =>
        {
            Debug.Log("[StereoVision] 连接成功!");
            isConnected = true;
        };

        websocket.OnError += (e) =>
        {
            Debug.LogError($"[StereoVision] 错误: {e}");
        };

        websocket.OnClose += (e) =>
        {
            Debug.Log($"[StereoVision] 连接关闭: {e}");
            isConnected = false;
        };

        websocket.OnMessage += (bytes) =>
        {
            ProcessMessage(bytes);
        };

        try
        {
            await websocket.Connect();
        }
        catch (Exception e)
        {
            Debug.LogError($"[StereoVision] 连接失败: {e.Message}");
        }
    }

    void ProcessMessage(byte[] data)
    {
        // 判断消息类型
        // 二进制数据 (视频帧): 以 4 字节长度开头
        // JSON 数据 (追踪): 以 '{' 开头

        if (data.Length > 8 && data[0] != '{')
        {
            // 二进制视频帧
            ProcessBinaryFrame(data);
        }
        else
        {
            // JSON 追踪数据
            ProcessTrackingData(data);
        }
    }

    void ProcessBinaryFrame(byte[] data)
    {
        try
        {
            // 格式: [4字节左图大小][左图JPEG][4字节右图大小][右图JPEG]
            int leftSize = BitConverter.ToInt32(data, 0);
            int rightSize = BitConverter.ToInt32(data, 4 + leftSize);

            byte[] leftData = new byte[leftSize];
            byte[] rightData = new byte[rightSize];

            Array.Copy(data, 4, leftData, 0, leftSize);
            Array.Copy(data, 8 + leftSize, rightData, 0, rightSize);

            lock (frameLock)
            {
                pendingLeftData = leftData;
                pendingRightData = rightData;
                hasNewFrame = true;
            }

            framesReceived++;
        }
        catch (Exception e)
        {
            Debug.LogError($"[StereoVision] 解析视频帧失败: {e.Message}");
        }
    }

    void ProcessTrackingData(byte[] data)
    {
        try
        {
            string json = System.Text.Encoding.UTF8.GetString(data);
            // 这里可以解析追踪数据并应用
            // TrackingData tracking = JsonUtility.FromJson<TrackingData>(json);
            // Debug.Log($"[Tracking] Frame: {tracking.frame_id}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[StereoVision] 解析追踪数据失败: {e.Message}");
        }
    }

    void Update()
    {
        // WebSocket 消息分发 (必须在主线程)
        #if !UNITY_WEBGL || UNITY_EDITOR
        websocket?.DispatchMessageQueue();
        #endif

        // 更新纹理 (必须在主线程)
        lock (frameLock)
        {
            if (hasNewFrame && pendingLeftData != null && pendingRightData != null)
            {
                leftTexture.LoadImage(pendingLeftData);
                rightTexture.LoadImage(pendingRightData);
                hasNewFrame = false;
                frameCount++;
            }
        }

        // 计算 FPS
        fpsTimer += Time.deltaTime;
        if (fpsTimer >= 1.0f)
        {
            fps = frameCount / fpsTimer;
            frameCount = 0;
            fpsTimer = 0;
        }
    }

    async void OnDestroy()
    {
        if (websocket != null && websocket.State == WebSocketState.Open)
        {
            await websocket.Close();
        }
    }

    // 提供给 UI 调用的方法
    public void SetServerIP(string ip)
    {
        serverIP = ip;
    }

    public async void Reconnect()
    {
        if (websocket != null)
        {
            await websocket.Close();
        }
        await ConnectToServer();
    }
}
```

---

## 2.5 创建立体显示 Shader

在 `Assets/Shaders/` 下创建 `StereoDisplay.shader`:

```hlsl
// Assets/Shaders/StereoDisplay.shader

Shader "Custom/StereoDisplay"
{
    Properties
    {
        _LeftTex ("Left Eye Texture", 2D) = "white" {}
        _RightTex ("Right Eye Texture", 2D) = "white" {}
    }

    SubShader
    {
        Tags { "RenderType"="Opaque" "Queue"="Geometry" }
        LOD 100

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing

            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct v2f
            {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                UNITY_VERTEX_OUTPUT_STEREO
            };

            sampler2D _LeftTex;
            sampler2D _RightTex;
            float4 _LeftTex_ST;

            v2f vert (appdata v)
            {
                v2f o;

                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_INITIALIZE_OUTPUT(v2f, o);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);

                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = TRANSFORM_TEX(v.uv, _LeftTex);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);

                // 根据当前渲染的眼睛选择对应纹理
                // unity_StereoEyeIndex: 0 = 左眼, 1 = 右眼
                if (unity_StereoEyeIndex == 0)
                {
                    return tex2D(_LeftTex, i.uv);
                }
                else
                {
                    return tex2D(_RightTex, i.uv);
                }
            }
            ENDCG
        }
    }

    FallBack "Diffuse"
}
```

---

## 2.6 Unity 场景设置

### 步骤 1: 创建显示平面

1. 在 Hierarchy 中右键 → `3D Object` → `Quad`
2. 重命名为 `StereoDisplayQuad`
3. 设置 Transform:
   - Position: (0, 0, 2) — 放在相机前方 2 米
   - Rotation: (0, 180, 0) — 面向相机
   - Scale: (3.2, 1.8, 1) — 16:9 比例

### 步骤 2: 创建材质

1. 在 Project 窗口右键 → `Create` → `Material`
2. 重命名为 `StereoDisplayMaterial`
3. 选择 Shader: `Custom/StereoDisplay`
4. 将材质拖到 `StereoDisplayQuad` 上

### 步骤 3: 挂载脚本

1. 选择场景中的某个 GameObject (如 Main Camera 或新建空对象)
2. `Add Component` → 搜索 `StereoVisionReceiver`
3. 配置参数:
   - Server IP: 你的 PC IP 地址
   - Server Port: 8765
   - Left Eye Material: 拖入 StereoDisplayMaterial
   - Right Eye Material: 拖入 StereoDisplayMaterial
   - Display Quad: 拖入 StereoDisplayQuad

### 步骤 4: 配置 XR 设置

1. `Edit` → `Project Settings` → `XR Plug-in Management`
2. 勾选 `PICO`
3. 确保 Single Pass Instanced 渲染模式启用

---

## 2.7 打包 APK

```
1. File → Build Settings
2. 选择 Android 平台
3. 点击 Switch Platform
4. Player Settings:
   - Company Name: 你的名字
   - Product Name: StereoVR
   - Minimum API Level: Android 10.0 (API 29)
   - Target API Level: Android 12 (API 31)
5. 点击 Build
6. 选择保存位置，生成 APK
```

---

## 2.8 安装到 PICO

```bash
# 1. 连接 PICO (USB 数据线)
# 2. PICO 上选择 "传输文件"

# 3. 检查设备连接
adb devices

# 4. 安装 APK
adb install -r StereoVR.apk

# 5. 查看安装的应用
adb shell pm list packages | grep stereo
```

---

# 第三部分：系统集成测试

## 3.1 启动顺序

```bash
# === PC 端 ===

# 终端 1: 启动 PC-Service
xrobotoolkit-pc-service

# 终端 2: 启动你的 server.py
conda activate xrobot
python server.py


# === PICO 端 ===

# 1. 确保 PICO 和 PC 在同一 WiFi 网络
# 2. 打开 "StereoVR" 应用
# 3. 在应用中输入 PC 的 IP 地址
# 4. 点击连接
```

## 3.2 验证清单

| 检查项 | 预期结果 |
|--------|----------|
| PC-Service 运行 | `ps aux \| grep xrobotoolkit` 有输出 |
| server.py 运行 | 显示 "WebSocket 服务器启动" |
| PICO 显示画面 | 看到双目相机的立体图像 |
| 追踪数据 | server.py 终端显示 "收到追踪数据" |

## 3.3 网络调试

```bash
# 查看 PC IP 地址
ip addr show | grep inet

# 测试 WebSocket 端口
nc -zv <PC_IP> 8765

# 查看 PICO 连接状态
adb logcat | grep StereoVision
```

---

# 第四部分：数据格式参考

## 追踪数据 JSON 格式

```json
{
    "type": "body_tracking",
    "frame_id": 42,
    "timestamp": 1704326400000,
    "joints": {
        "Head": {
            "position": [0.0, 1.7, 0.0],
            "rotation": [0.0, 0.0, 0.0, 1.0]
        },
        "Neck": {
            "position": [0.0, 1.55, 0.0],
            "rotation": [0.0, 0.0, 0.0, 1.0]
        },
        "LeftHand": {
            "position": [-0.3, 1.2, 0.3],
            "rotation": [0.0, 0.0, 0.0, 1.0]
        }
        // ... 24 个关节
    },
    "head": {
        "position": [0.0, 1.7, 0.0],
        "rotation": [0.0, 0.0, 0.0, 1.0]
    },
    "controllers": {
        "left": { "position": [...], "rotation": [...], "buttons": {...} },
        "right": { "position": [...], "rotation": [...], "buttons": {...} }
    }
}
```

## 视频帧二进制格式

```
[4 字节: 左图 JPEG 大小 (little-endian uint32)]
[N 字节: 左图 JPEG 数据]
[4 字节: 右图 JPEG 大小 (little-endian uint32)]
[M 字节: 右图 JPEG 数据]
```

## 坐标系

```
      Y (上)
      |
      |
      +------ X (右)
     /
    /
   Z (前)

- 右手坐标系
- 单位: 米
- 旋转: 四元数 [x, y, z, w]
- 原点: 启动时头部位置
```

---

# 第五部分：参考资源

## 官方仓库

| 仓库 | 链接 | 说明 |
|------|------|------|
| Unity Client | https://github.com/XR-Robotics/XRoboToolkit-Unity-Client | PICO 端应用 |
| PC-Service | https://github.com/XR-Robotics/XRoboToolkit-PC-Service | PC 后台服务 |
| Python 绑定 | https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind | pxrea API |
| 官方文档 | https://xr-robotics.github.io/ | 框架文档 |

## 设备要求

| 设备 | 要求 |
|------|------|
| PICO 头显 | PICO 4 / 4 Pro / 4 Ultra (消费版即可) |
| PC | Ubuntu 22.04, x86_64 |
| 双目相机 | 任意 OpenCV 兼容的 USB 双目相机 |
| 网络 | PICO 和 PC 在同一局域网 |

## 常见问题

**Q: 消费版 PICO 能用吗？**
A: 能！PICO 4、4 Pro、4 Ultra 消费版都支持体感追踪。

**Q: 必须买 Motion Tracker 吗？**
A: 不必须。没有 Tracker 也能追踪全身，只是腿部精度稍低。

**Q: 延迟有多少？**
A: 追踪约 20ms，视频取决于网络，通常 50-100ms。

**Q: pxrea 是什么意思？**
A: PICO XR Enterprise Assistant 的缩写。
