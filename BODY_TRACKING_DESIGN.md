# PICO 体感追踪集成方案

## 一句话总结

**问题**: PICO体感追踪器不支持网页(WebXR)
**解决**: 用PC做中转站，把追踪数据通过WebSocket发给浏览器

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
