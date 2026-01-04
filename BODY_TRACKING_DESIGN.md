# PICO 体感追踪集成方案

## 一句话总结

**问题**: PICO体感追踪器不支持网页(WebXR)
**解决**: 用PC做中转站，把追踪数据通过WebSocket发给浏览器

---

# ⚠️ 重要限制：XRoboToolkit 与 WebXR 不能同时运行

## 问题确认

经过官方文档验证，**XRoboToolkit-Unity-Client 需要占用 VR 显示**：

| 事实 | 说明 |
|------|------|
| Unity Client 是独立 VR 应用 | 有自己的 UI 界面，显示 "WORKING" 等状态 |
| 占用 VR 渲染资源 | 不能与 PICO 浏览器同时在前台运行 |
| PICO 系统限制 | 同一时间只能有一个 VR 应用在前台 |

**结论**: 你不能一边运行 XRoboToolkit APP 采集追踪数据，一边用浏览器看 WebXR 立体视觉。

---

## 解决方案对比

### 方案 A：使用 XRoboToolkit 内置 Remote Vision (最简单！)

**重要发现**: XRoboToolkit **已经内置立体视觉功能**！

根据[官方论文](https://arxiv.org/html/2508.00097v1)，Remote Vision 支持两种视频源：

| 视频源 | 说明 | 你的情况 |
|--------|------|----------|
| **PICO 头显摄像头** | PICO 4 Ultra 内置 VST 相机 | 需要企业版权限 |
| **ZED Mini 相机** | 通过 XRoboToolkit-Robot-Vision 模块 | ✅ 可用！ |

**关键信息**：
- 立体视觉通过 `XRoboToolkit-Robot-Vision` 模块实现
- 支持 ZED Mini 等外置立体相机
- 使用自定义 Shader 调整瞳距，焦点约 3.3 英尺 (适合遥操作)

```
┌─────────────────────────────────────────────────────────────────┐
│  Linux PC                                                        │
│  ┌─────────────────┐     ┌─────────────────────────────────┐    │
│  │ 你的立体相机     │────>│ XRoboToolkit-Robot-Vision       │    │
│  │ (USB相机/ZED)   │     │ - 编码立体视频                   │    │
│  └─────────────────┘     │ - 发送到头显                     │    │
│                          └─────────────────┬───────────────┘    │
│                                             │                    │
│  ┌─────────────────────────────────────────┐│                    │
│  │ XRoboToolkit PC-Service                 ││                    │
│  │ - 接收追踪数据                           ││                    │
│  └─────────────────────────────────────────┘│                    │
└──────────────────────────────────────────────┼──────────────────┘
                                               │ WiFi
┌──────────────────────────────────────────────┼──────────────────┐
│  PICO 头显                                    ▼                  │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ XRoboToolkit Unity Client                                │    │
│  │ - Remote Vision: Listen 接收视频                         │    │
│  │ - Tracking: 发送追踪数据                                 │    │
│  │ - 全部功能在一个APP内                                    │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

**优点**:
- ✅ **官方支持**，不需要改代码
- ✅ 追踪 + 视觉在同一个应用
- ✅ 延迟优化过 (<100ms)
- ✅ 支持 ZED Mini 等立体相机

**缺点**:
- ❌ 你的相机可能需要适配 Robot-Vision 模块
- ❌ 需要研究 Robot-Vision 的接口

**需要确认**: 你的立体相机是什么型号？如果不是 ZED，可能需要写适配器

---

### 方案 B：两阶段切换模式

**思路**: 两个应用交替运行

```
阶段1: 校准/采集追踪数据
┌─────────────────────────────────────────┐
│ 运行 XRoboToolkit APP                    │
│ - 采集追踪数据发送到 PC                  │
│ - PC 保存追踪数据到文件/内存             │
└─────────────────────────────────────────┘
          ↓ 用户手动切换
阶段2: 查看立体视觉
┌─────────────────────────────────────────┐
│ 打开 PICO 浏览器                         │
│ - 显示 WebXR 立体视觉                    │
│ - 使用之前采集的追踪数据                 │
└─────────────────────────────────────────┘
```

**优点**:
- ✅ 不需要大改代码
- ✅ 保留 WebXR 方案

**缺点**:
- ❌ 不能实时追踪 (数据是之前采集的)
- ❌ 用户体验差 (需要切换应用)

---

### 方案 C：使用 PICO Streaming (PICO Link)

**思路**: 让 PC 运行 VR 应用，PICO 只做显示器

```
┌─────────────────────────────────────────────────────────────────┐
│  Linux/Windows PC                                                │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ USB 相机        │  │ SteamVR/OpenXR  │  │ XRoboToolkit    │  │
│  │                 │  │ VR 应用         │  │ PC-Service      │  │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘  │
│           │                    │                     │           │
│           └──────────┬─────────┴─────────────────────┘           │
│                      ▼                                            │
│              ┌─────────────────┐                                  │
│              │ PICO Link/串流   │                                  │
│              └────────┬────────┘                                  │
└───────────────────────┼─────────────────────────────────────────┘
                        │ WiFi/USB
┌───────────────────────┼─────────────────────────────────────────┐
│  PICO 头显            ▼                                          │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ PICO Link APP (串流接收器)                               │    │
│  │ - 只显示 PC 传来的画面                                   │    │
│  │ - 追踪数据发回 PC                                        │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

**优点**:
- ✅ 所有计算在 PC
- ✅ 追踪 + 视觉可以同时工作
- ✅ PICO Link 官方支持体感追踪数据传输

**缺点**:
- ❌ 需要高带宽低延迟网络
- ❌ 需要 PC 端 VR 应用 (非浏览器)

---

### 方案 D：修改 XRoboToolkit Unity Client (最灵活)

**思路**: 在 XRoboToolkit Unity Client 中集成你的立体相机显示功能

```
┌─────────────────────────────────────────────────────────────────┐
│  修改后的 XRoboToolkit Unity Client                              │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ 新增模块: WebSocket 客户端                               │    │
│  │ - 连接到你的 server.py                                   │    │
│  │ - 接收相机帧                                             │    │
│  │ - 渲染到 VR 眼镜                                         │    │
│  └─────────────────────────────────────────────────────────┘    │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ 原有模块: 追踪数据采集                                    │    │
│  │ - 照常发送到 PC-Service                                  │    │
│  └─────────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

#### Fork 和开发环境设置

```bash
# 1. Fork 仓库 (在 GitHub 网页上点 Fork，或用 gh 命令)
gh repo fork XR-Robotics/XRoboToolkit-Unity-Client --clone

# 2. 或者直接克隆官方仓库
git clone https://github.com/XR-Robotics/XRoboToolkit-Unity-Client.git
cd XRoboToolkit-Unity-Client

# 3. 创建你的开发分支
git checkout -b feature/custom-stereo-vision

# 4. 用 Unity 打开项目 (需要 Unity 2021.3+ 或查看项目要求)
# Unity Hub → Add → 选择克隆的文件夹
```

#### Unity 项目中添加 WebSocket 接收模块

**步骤 1**: 安装 WebSocket 库

```
在 Unity Package Manager 中添加:
- NativeWebSocket: https://github.com/endel/NativeWebSocket.git
或
- WebSocketSharp (通过 NuGet)
```

**步骤 2**: 创建立体视觉接收脚本

```csharp
// Assets/Scripts/StereoVisionReceiver.cs
using UnityEngine;
using NativeWebSocket;
using System;

public class StereoVisionReceiver : MonoBehaviour
{
    [Header("Server Settings")]
    public string serverIP = "192.168.1.100";
    public int serverPort = 8765;

    [Header("Display")]
    public Material leftEyeMaterial;
    public Material rightEyeMaterial;

    private WebSocket ws;
    private Texture2D leftTexture;
    private Texture2D rightTexture;
    private byte[] pendingLeftData;
    private byte[] pendingRightData;
    private bool hasNewFrame = false;

    async void Start()
    {
        // 初始化纹理
        leftTexture = new Texture2D(1280, 720, TextureFormat.RGB24, false);
        rightTexture = new Texture2D(1280, 720, TextureFormat.RGB24, false);

        // 连接 WebSocket
        ws = new WebSocket($"wss://{serverIP}:{serverPort}");

        ws.OnMessage += (bytes) =>
        {
            // 解析你的 server.py 发送的帧格式
            ProcessFrame(bytes);
        };

        ws.OnOpen += () => Debug.Log("[StereoVision] Connected to server");
        ws.OnError += (e) => Debug.LogError($"[StereoVision] Error: {e}");
        ws.OnClose += (e) => Debug.Log("[StereoVision] Disconnected");

        await ws.Connect();
    }

    void ProcessFrame(byte[] data)
    {
        // 根据你 server.py 的数据格式解析
        // 假设格式: [4字节左图大小][左图JPEG][4字节右图大小][右图JPEG]
        try
        {
            int leftSize = BitConverter.ToInt32(data, 0);
            pendingLeftData = new byte[leftSize];
            Array.Copy(data, 4, pendingLeftData, 0, leftSize);

            int rightSize = BitConverter.ToInt32(data, 4 + leftSize);
            pendingRightData = new byte[rightSize];
            Array.Copy(data, 8 + leftSize, pendingRightData, 0, rightSize);

            hasNewFrame = true;
        }
        catch (Exception e)
        {
            Debug.LogError($"[StereoVision] Parse error: {e.Message}");
        }
    }

    void Update()
    {
        #if !UNITY_WEBGL || UNITY_EDITOR
        ws?.DispatchMessageQueue();
        #endif

        // 在主线程更新纹理
        if (hasNewFrame)
        {
            leftTexture.LoadImage(pendingLeftData);
            rightTexture.LoadImage(pendingRightData);

            leftEyeMaterial.mainTexture = leftTexture;
            rightEyeMaterial.mainTexture = rightTexture;

            hasNewFrame = false;
        }
    }

    async void OnDestroy()
    {
        if (ws != null && ws.State == WebSocketState.Open)
        {
            await ws.Close();
        }
    }
}
```

**步骤 3**: 创建立体显示 Shader

```hlsl
// Assets/Shaders/StereoDisplay.shader
Shader "Custom/StereoDisplay"
{
    Properties
    {
        _LeftTex ("Left Eye Texture", 2D) = "white" {}
        _RightTex ("Right Eye Texture", 2D) = "white" {}
        _IPD ("Interpupillary Distance", Range(0.058, 0.072)) = 0.064
    }

    SubShader
    {
        Tags { "RenderType"="Opaque" }

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            sampler2D _LeftTex;
            sampler2D _RightTex;
            float _IPD;

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

            v2f vert (appdata v)
            {
                v2f o;
                UNITY_SETUP_INSTANCE_ID(v);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(o);
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv = v.uv;
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(i);

                // 根据当前渲染的眼睛选择纹理
                if (unity_StereoEyeIndex == 0)
                    return tex2D(_LeftTex, i.uv);  // 左眼
                else
                    return tex2D(_RightTex, i.uv); // 右眼
            }
            ENDCG
        }
    }
}
```

**步骤 4**: 场景设置

```
1. 创建一个 Quad 放在相机前方
2. 将 StereoDisplay 材质应用到 Quad
3. 将 StereoVisionReceiver 脚本挂载到某个 GameObject
4. 配置 server IP 和端口
```

**步骤 5**: 打包 APK

```bash
# 在 Unity 中:
# File → Build Settings → Android → Switch Platform
# Player Settings → XR Plug-in Management → 启用 PICO XR
# Build → 生成 APK

# 安装到 PICO
adb install -r your_modified_app.apk
```

**优点**:
- ✅ 追踪 + 视觉完全集成
- ✅ 保留你现有的 server.py
- ✅ 最低延迟
- ✅ 完全可控

**缺点**:
- ❌ 需要 Unity 开发经验
- ❌ 需要维护 Fork

---

## 推荐选择

| 你的需求 | 推荐方案 |
|----------|----------|
| **最简单，官方支持** | 方案 A (XRoboToolkit Remote Vision) |
| 保留现有代码，愿意开发 | 方案 D (修改 Unity Client) |
| 快速验证，不改代码 | 方案 B (两阶段切换) |
| 已有 PC VR 应用 | 方案 C (PICO Link) |

### 方案 A vs 方案 D 详细对比

| 对比项 | 方案 A (Remote Vision) | 方案 D (修改 Unity Client) |
|--------|------------------------|---------------------------|
| **视频传输** | XRoboToolkit-Robot-Vision | 你的 server.py + WebSocket |
| **改动位置** | PC 端 (适配 Robot-Vision) | 头显端 (Unity Client) |
| **官方支持** | ✅ 官方模块 | ❌ 需要自己维护 |
| **你的 server.py** | 替换掉 | ✅ 保留 |
| **相机兼容性** | ZED Mini 等已支持 | ✅ 任意相机 |
| **开发工作量** | 小 (如果相机兼容) | 中 (需要写 Unity 代码) |
| **灵活性** | 受限于 Robot-Vision | ✅ 完全自定义 |

**建议决策流程**：

```
你的立体相机是 ZED Mini 吗？
    ├─ 是 → 方案 A (直接用 Remote Vision)
    │
    └─ 否 → 你的相机可以适配 Robot-Vision 吗？
              ├─ 可以 → 方案 A (写适配器)
              │
              └─ 不行/太麻烦 → 方案 D (改 Unity Client)
```

---

# XRoboToolkit 完整仓库地图

## 官方资源入口

| 资源 | 链接 | 说明 |
|------|------|------|
| **GitHub组织** | https://github.com/XR-Robotics | 所有仓库汇总 |
| **官方文档** | https://xr-robotics.github.io/ | 框架介绍和API文档 |
| **YouTube** | @XRRobotics | 演示视频 |
| **Discord** | 官网获取 | 技术支持社区 |

---

## 仓库全景图 (按安装顺序)

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                        XRoboToolkit 仓库依赖关系                                 │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │ 第1层: PICO头显端 (必装)                                                 │   │
│  │                                                                          │   │
│  │  ┌────────────────────────────────────────────────────────────────┐     │   │
│  │  │ XRoboToolkit-Unity-Client                                      │     │   │
│  │  │ https://github.com/XR-Robotics/XRoboToolkit-Unity-Client       │     │   │
│  │  │                                                                 │     │   │
│  │  │ ⭐ 30 stars | 语言: C#/Unity                                   │     │   │
│  │  │ 功能: 采集头部/手柄/手势/全身追踪数据                            │     │   │
│  │  │ 输出: APK安装包 (需自行编译或找Release)                         │     │   │
│  │  │ 支持: PICO 4 Ultra (推荐) / PICO 4 Pro / PICO 4                │     │   │
│  │  └────────────────────────────────────────────────────────────────┘     │   │
│  │                                                                          │   │
│  │  ┌────────────────────────────────────────────────────────────────┐     │   │
│  │  │ XRoboToolkit-Unity-Client-Quest (备选)                         │     │   │
│  │  │ https://github.com/XR-Robotics/XRoboToolkit-Unity-Client-Quest │     │   │
│  │  │ ⭐ 18 stars | 支持: Meta Quest 3/Pro                           │     │   │
│  │  └────────────────────────────────────────────────────────────────┘     │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                      │                                          │
│                                      ▼ WiFi/USB传输                             │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │ 第2层: PC服务端 (必装)                                                   │   │
│  │                                                                          │   │
│  │  ┌────────────────────────────────────────────────────────────────┐     │   │
│  │  │ XRoboToolkit-PC-Service                                        │     │   │
│  │  │ https://github.com/XR-Robotics/XRoboToolkit-PC-Service         │     │   │
│  │  │                                                                 │     │   │
│  │  │ ⭐ 19 stars | 语言: C++/Qt6 | 许可: Apache 2.0                 │     │   │
│  │  │                                                                 │     │   │
│  │  │ 功能:                                                          │     │   │
│  │  │ - gRPC服务器接收PICO数据                                        │     │   │
│  │  │ - 设备管理和数据记录                                            │     │   │
│  │  │ - 低延迟视频流 (<100ms)                                         │     │   │
│  │  │                                                                 │     │   │
│  │  │ 支持平台:                                                       │     │   │
│  │  │ - Windows x64                                                   │     │   │
│  │  │ - Linux x86_64 (Ubuntu 22.04)                                  │     │   │
│  │  │ - Linux ARM64 (Jetson/Orin)                                    │     │   │
│  │  │                                                                 │     │   │
│  │  │ 安装: sudo dpkg -i xrobotoolkit-pc-service_xxx.deb             │     │   │
│  │  └────────────────────────────────────────────────────────────────┘     │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                      │                                          │
│                                      ▼ 本地API调用                              │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │ 第3层: 语言绑定 (选一个)                                                 │   │
│  │                                                                          │   │
│  │  ┌────────────────────────────────────────────────────────────────┐     │   │
│  │  │ XRoboToolkit-PC-Service-Pybind  ← 推荐！                       │     │   │
│  │  │ https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind  │     │   │
│  │  │                                                                 │     │   │
│  │  │ ⭐ 5 stars | 语言: Python | 最新版: v1.0.2                     │     │   │
│  │  │                                                                 │     │   │
│  │  │ 核心API:                                                        │     │   │
│  │  │ - xrt.init() / xrt.close()                                     │     │   │
│  │  │ - xrt.get_headset_pose()                                       │     │   │
│  │  │ - xrt.get_left/right_controller_pose()                         │     │   │
│  │  │ - xrt.is_body_data_available()                                 │     │   │
│  │  │ - xrt.get_body_joint_poses() → 24个关节                        │     │   │
│  │  │                                                                 │     │   │
│  │  │ 安装:                                                          │     │   │
│  │  │ conda create -n xr python=3.10                                 │     │   │
│  │  │ pip install .                                                   │     │   │
│  │  └────────────────────────────────────────────────────────────────┘     │   │
│  │                                                                          │   │
│  │  ┌────────────────────────────────────────────────────────────────┐     │   │
│  │  │ XRoboToolkit-Teleop-ROS (ROS2用户)                             │     │   │
│  │  │ https://github.com/XR-Robotics/XRoboToolkit-Teleop-ROS         │     │   │
│  │  │                                                                 │     │   │
│  │  │ ⭐ 7 stars | 语言: C++ | 支持: ROS1 & ROS2                     │     │   │
│  │  │                                                                 │     │   │
│  │  │ 发布话题:                                                       │     │   │
│  │  │ - /xr/head (float32[7] pose)                                   │     │   │
│  │  │ - /xr/left_controller                                          │     │   │
│  │  │ - /xr/right_controller                                         │     │   │
│  │  │                                                                 │     │   │
│  │  │ 运行: ros2 run picoxr talker                                   │     │   │
│  │  └────────────────────────────────────────────────────────────────┘     │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                      │                                          │
│                                      ▼ 应用开发                                 │
│  ┌─────────────────────────────────────────────────────────────────────────┐   │
│  │ 第4层: 示例和应用 (参考学习)                                             │   │
│  │                                                                          │   │
│  │  ┌────────────────────────────────────────────────────────────────┐     │   │
│  │  │ XRoboToolkit-Teleop-Sample-Python                              │     │   │
│  │  │ https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python│     │   │
│  │  │                                                                 │     │   │
│  │  │ ⭐ 61 stars | 最受欢迎的示例仓库                                │     │   │
│  │  │                                                                 │     │   │
│  │  │ 仿真示例:                                                       │     │   │
│  │  │ - teleop_dual_ur5e_mujoco.py (UR5e双臂)                        │     │   │
│  │  │ - teleop_shadow_hand_mujoco.py (Shadow灵巧手)                  │     │   │
│  │  │ - teleop_x7s_placo.py (Placo可视化)                            │     │   │
│  │  │                                                                 │     │   │
│  │  │ 真机示例:                                                       │     │   │
│  │  │ - teleop_dual_ur5e_hardware.py                                 │     │   │
│  │  │ - teleop_dual_arx_r5_hardware.py                               │     │   │
│  │  │ - teleop_r1lite_hardware.py (Galaxea人形机器人)                │     │   │
│  │  └────────────────────────────────────────────────────────────────┘     │   │
│  │                                                                          │   │
│  │  ┌────────────────────────────────────────────────────────────────┐     │   │
│  │  │ XRoboToolkit-Teleop-Sample-Cpp                                 │     │   │
│  │  │ https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Cpp  │     │   │
│  │  │ C++版双臂机器人遥操作示例                                       │     │   │
│  │  └────────────────────────────────────────────────────────────────┘     │   │
│  └─────────────────────────────────────────────────────────────────────────┘   │
│                                                                                 │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## 你需要什么设备？

| 设备 | 价格 | 必须？ | 说明 |
|------|------|--------|------|
| PICO 4 / 4 Pro / 4 Ultra | ¥2000-4000 | ✅ 是 | **消费版就够用**，不需要企业版 |
| PICO Motion Tracker (2个) | ¥500 | ❌ 可选 | 绑在脚踝，提高腿部追踪精度 |
| Linux电脑 (Ubuntu 22.04) | - | ✅ 是 | 运行服务程序 |

**重要**: 消费版PICO 4完全支持体感追踪，不需要花更多钱买企业版！

---

## 方案对比：哪个适合你？

```
简单程度: XRoboToolkit > ALVR > Unity方案
推荐指数: ⭐⭐⭐⭐⭐   ⭐⭐⭐    ⭐⭐
```

| 方案 | 延迟 | 难度 | Linux | ROS2 | 适合谁 |
|------|------|------|-------|------|--------|
| **XRoboToolkit** | 20ms | 简单 | ✅ | ✅ | 机器人研究、你的项目 |
| ALVR | 30-50ms | 中等 | ✅ | ❌ | VR游戏玩家 |
| Unity | 30ms | 复杂 | ❌ | 部分 | Windows快速原型 |

**推荐: XRoboToolkit** - PICO官方出品，专为机器人设计

---

## XRoboToolkit 是什么？

简单说：**PICO官方做的一套工具，让你能在Linux电脑上获取VR头显的追踪数据**

```
数据流动过程：

  [PICO头显]                    [Linux电脑]                    [浏览器]
      |                              |                              |
   戴在头上                     运行服务程序                    显示骨骼
   追踪身体                     接收追踪数据                    渲染画面
      |                              |                              |
      +-------- WiFi传输 ----------->+-------- WebSocket ---------->+
                                     |
                              (你的server.py)
```

---

## 能获取哪些数据？

XRoboToolkit 可以给你 **5种追踪数据**：

| 数据类型 | 内容 | 帧率 |
|----------|------|------|
| 头部 | 位置 + 朝向 (6自由度) | 90Hz |
| 手柄 | 位置 + 按钮 + 摇杆 | 90Hz |
| 手势 | 26个手指关节 | 60Hz |
| **全身** | **24个骨骼关节** | 90Hz |
| 追踪器 | Motion Tracker位置 | 200Hz |

**24个骨骼关节包括**：
- 头、颈、胸、腰、骨盆
- 左右肩、上臂、前臂、手
- 左右大腿、小腿、脚

---

## 安装步骤 (5分钟)

### 第1步：PICO头显端

```bash
# 1. 开启开发者模式
#    设置 → 通用 → 开发者模式 → 开启

# 2. 用数据线连接电脑，安装APP
adb install XRoboToolkit-PICO-1.1.1.apk
```

### 第2步：Linux电脑端

```bash
# 方法A：直接安装预编译包 (推荐)
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.1.1/xrobotoolkit-pc-service_1.1.1_amd64.deb
sudo dpkg -i xrobotoolkit-pc-service_1.1.1_amd64.deb

# 方法B：从源码编译
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service.git
cd XRoboToolkit-PC-Service
./build_linux.sh
```

### 第3步：安装Python绑定

```bash
# 克隆并编译Python绑定
git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git
cd XRoboToolkit-PC-Service-Pybind
pip install .
```

### 第4步：测试连接

```python
# test_tracking.py
import pxrea

def on_data(data_type, data):
    if data_type == 0x08:  # 全身追踪
        print(f"收到骨骼数据: {len(data['joints'])}个关节")

# 启动
pxrea.PXREAInit(None, on_data, 0xFF)
input("按回车退出...")
pxrea.PXREADeinit()
```

---

## 与你的项目集成

### 架构图

```
┌────────────────────────────────────────────────────────────────┐
│  PICO 头显                                                      │
│  ┌──────────────────────┐  ┌────────────────────────────────┐  │
│  │ XRoboToolkit APP     │  │ 浏览器 (你的VR Viewer)          │  │
│  │ 采集追踪数据          │  │ 显示相机画面 + 骨骼             │  │
│  └──────────┬───────────┘  └─────────────────┬──────────────┘  │
└─────────────┼────────────────────────────────┼─────────────────┘
              │ WiFi                            │ WebSocket
┌─────────────┼────────────────────────────────┼─────────────────┐
│  Linux 电脑 │                                │                 │
│  ┌──────────▼───────────┐  ┌─────────────────▼──────────────┐  │
│  │ XRoboToolkit服务     │  │ 你的 server.py                 │  │
│  │ (C++后台运行)        │  │ - USB相机流 (已有)             │  │
│  └──────────┬───────────┘  │ - 追踪数据 (新增)              │  │
│             │               │ - WebSocket发送                │  │
│  ┌──────────▼───────────┐  │                                │  │
│  │ Python绑定 (pxrea)   │──┘                                │  │
│  └──────────────────────┘                                    │  │
└──────────────────────────────────────────────────────────────────┘
```

### 修改你的 server.py

在现有代码基础上，添加追踪数据接收：

```python
# === 新增：体感追踪模块 ===

import threading
from typing import Optional, Dict, Any

# 尝试导入XRoboToolkit
try:
    import pxrea
    HAS_TRACKING = True
except ImportError:
    HAS_TRACKING = False
    print("[提示] 未安装pxrea，体感追踪功能不可用")


class BodyTracker:
    """体感追踪数据接收器"""

    def __init__(self):
        self.data = None
        self.lock = threading.Lock()
        self.frame_id = 0

    def _callback(self, data_type: int, data: dict):
        """接收追踪数据的回调函数"""
        with self.lock:
            if data_type == 0x08:  # BODY = 全身追踪
                self.data = data
                self.frame_id += 1

    def start(self):
        """启动追踪"""
        if not HAS_TRACKING:
            return False

        result = pxrea.PXREAInit(None, self._callback, 0x08)
        return result == 0

    def get_frame(self) -> Optional[dict]:
        """获取最新一帧追踪数据"""
        with self.lock:
            if self.data is None:
                return None

            return {
                'type': 'body_tracking',
                'frame_id': self.frame_id,
                'joints': self._format_joints(self.data.get('joints', []))
            }

    def _format_joints(self, joints: list) -> dict:
        """把关节数组转成字典，方便前端使用"""
        names = [
            'Pelvis', 'SpineLower', 'SpineMiddle', 'SpineUpper',
            'Chest', 'Neck', 'Head',
            'LeftShoulder', 'LeftUpperArm', 'LeftLowerArm', 'LeftHand',
            'RightShoulder', 'RightUpperArm', 'RightLowerArm', 'RightHand',
            'LeftUpperLeg', 'LeftLowerLeg', 'LeftFoot',
            'RightUpperLeg', 'RightLowerLeg', 'RightFoot'
        ]

        result = {}
        for i, joint in enumerate(joints):
            if i < len(names):
                result[names[i]] = {
                    'pos': joint.get('position', [0, 0, 0]),
                    'rot': joint.get('rotation', [0, 0, 0, 1])
                }
        return result

    def stop(self):
        if HAS_TRACKING:
            pxrea.PXREADeinit()


# === 在你的WebSocket服务器中使用 ===

class USBStereoWebSocketServerSSL:
    def __init__(self, ...):
        # ... 现有代码 ...

        # 新增：体感追踪
        self.tracker = BodyTracker()

    async def start_server(self):
        # ... 现有代码 ...

        # 新增：启动追踪
        if self.tracker.start():
            print("[OK] 体感追踪已启动")

    async def handle_client(self, websocket):
        while True:
            # 1. 发送相机帧 (现有)
            camera_frame = self.get_camera_frame()
            if camera_frame:
                await websocket.send(json.dumps(camera_frame))

            # 2. 发送追踪数据 (新增)
            body_frame = self.tracker.get_frame()
            if body_frame:
                await websocket.send(json.dumps(body_frame))

            await asyncio.sleep(1/60)
```

### 前端接收数据

```javascript
// 在你的 dual_infrared_vr_viewer.html 中

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);

    if (data.type === 'dual_infrared_frame') {
        // 现有：处理相机帧
        updateCameraView(data);
    }
    else if (data.type === 'body_tracking') {
        // 新增：处理骨骼数据
        updateSkeleton(data.joints);
    }
};

function updateSkeleton(joints) {
    // 示例：打印头部位置
    if (joints.Head) {
        console.log('头部位置:', joints.Head.pos);
    }

    // TODO: 用Three.js渲染骨骼
}
```

---

## ROS2 集成 (可选)

如果你需要接入机器人系统：

```bash
# 1. 克隆ROS2包
cd ~/ros2_ws/src
git clone https://github.com/XR-Robotics/XRoboToolkit-Teleop-ROS.git

# 2. 编译
cd ~/ros2_ws
colcon build --packages-select picoxr xr_msgs

# 3. 运行
source install/setup.bash
ros2 run picoxr talker

# 4. 查看数据
ros2 topic echo /xr/body_tracking
```

**发布的ROS2话题**：
- `/xr/head` - 头部位姿
- `/xr/left_controller` - 左手柄
- `/xr/right_controller` - 右手柄
- `/xr/body_tracking` - 全身骨骼

---

## 数据格式参考

### WebSocket消息格式

```json
{
    "type": "body_tracking",
    "frame_id": 42,
    "joints": {
        "Head": {
            "pos": [0.0, 1.7, 0.0],
            "rot": [0.0, 0.0, 0.0, 1.0]
        },
        "Neck": {
            "pos": [0.0, 1.55, 0.0],
            "rot": [0.0, 0.0, 0.0, 1.0]
        },
        "LeftHand": {
            "pos": [-0.3, 1.2, 0.3],
            "rot": [0.0, 0.0, 0.0, 1.0]
        }
        // ... 其他21个关节
    }
}
```

### 坐标系说明

```
      Y (上)
      |
      |
      +------ X (右)
     /
    /
   Z (前)

- 右手坐标系
- 单位：米
- 旋转：四元数 [x, y, z, w]
- 原点：启动时头部位置
```

---

## 常见问题

### Q: 消费版PICO能用吗？
**A: 能！** PICO 4、4 Pro、4 Ultra都支持，不需要企业版。

### Q: 必须买Motion Tracker吗？
**A: 不必须。** 没有Tracker也能追踪全身，只是腿部精度稍低。

### Q: 延迟有多少？
**A: 约20ms。** 从身体动作到数据到达电脑约20毫秒。

### Q: 支持Windows吗？
**A: 支持。** XRoboToolkit同时支持Windows和Linux。

### Q: pxrea是什么意思？
**A: PICO XR Enterprise Assistant** 的缩写，是XRoboToolkit的核心API。

---

## 后续开发计划

### 阶段1：基础集成 (1周)
- [ ] 安装XRoboToolkit环境
- [ ] 测试追踪数据接收
- [ ] 修改server.py添加追踪模块
- [ ] 验证WebSocket数据传输

### 阶段2：前端渲染 (1周)
- [ ] Three.js骨骼可视化
- [ ] 坐标系对齐调试
- [ ] 与相机画面叠加显示

### 阶段3：ROS2集成 (可选)
- [ ] 编译ROS2节点
- [ ] 订阅追踪话题
- [ ] 对接机器人控制

---

## 参考链接

### 官方资源
- [XRoboToolkit GitHub](https://github.com/XR-Robotics)
- [XRoboToolkit 论文](https://arxiv.org/html/2508.00097)
- [PICO开发者中心](https://developer-cn.picoxr.com/)

### SDK文档
- [PICO Integration SDK 3.0](https://developer-cn.picoxr.com/document/)
- [XR_BD_body_tracking OpenXR扩展](https://registry.khronos.org/OpenXR/specs/1.1/man/html/XR_BD_body_tracking.html)

### 相关项目
- [TWIST2 人形机器人遥操作](https://github.com/amazon-far/TWIST2) - 使用相同的XRoboToolkit
- [ALVR](https://github.com/alvr-org/ALVR) - 备选方案

---

## SDK生态系统图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          PICO SDK 生态系统                               │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                     你的应用层                                   │   │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │   │
│  │  │ VR游戏          │  │ 机器人遥操作    │  │ 你的WebXR项目   │  │   │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘  │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                               ↑                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    框架层 (选一个)                               │   │
│  │                                                                   │   │
│  │  ┌─────────────────────────────────────────────────────────┐    │   │
│  │  │ XRoboToolkit ← 推荐！                                    │    │   │
│  │  │ github.com/XR-Robotics                                   │    │   │
│  │  │ - 轻量级C++服务                                          │    │   │
│  │  │ - Python绑定 (pxrea)                                     │    │   │
│  │  │ - ROS2支持                                               │    │   │
│  │  └─────────────────────────────────────────────────────────┘    │   │
│  │                           或                                     │   │
│  │  ┌─────────────────────────────────────────────────────────┐    │   │
│  │  │ Unity/Unreal SDK                                         │    │   │
│  │  │ github.com/Pico-Developer                                │    │   │
│  │  │ - 游戏引擎集成                                           │    │   │
│  │  │ - 更重但功能更全                                         │    │   │
│  │  └─────────────────────────────────────────────────────────┘    │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                               ↑                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    底层 SDK                                      │   │
│  │  PICO Integration SDK 3.0                                        │   │
│  │  - Unity SDK / Unreal SDK / OpenXR SDK / Native SDK              │   │
│  │  - 体感追踪、手势、MR、空间音频等功能                             │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                               ↑                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    设备运行时                                    │   │
│  │  PICO OpenXR Runtime (运行在头显上)                              │   │
│  │  - XR_BD_body_tracking 扩展                                      │   │
│  │  - XR_PICO_hand_tracking 扩展                                    │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                               ↑                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                    硬件层                                        │   │
│  │  PICO 4 / 4 Pro / 4 Ultra + Motion Tracker (可选)                │   │
│  └─────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘

GitHub组织对比：
┌──────────────────┬──────────────────────┬─────────────────────────┐
│ github.com/      │ 定位                  │ 主要内容                │
├──────────────────┼──────────────────────┼─────────────────────────┤
│ Pico-Developer   │ 官方SDK              │ Unity/Unreal/OpenXR SDK │
│ picoxr           │ 社区支持             │ 示例代码、技术支持       │
│ XR-Robotics      │ 机器人专用           │ XRoboToolkit全套        │
└──────────────────┴──────────────────────┴─────────────────────────┘
```

---

## 总结

1. **用XRoboToolkit** - PICO官方出品，简单好用
2. **消费版够用** - 不需要买企业版设备
3. **改动量小** - 只需在server.py添加一个模块
4. **ROS2友好** - 天然支持机器人系统集成
