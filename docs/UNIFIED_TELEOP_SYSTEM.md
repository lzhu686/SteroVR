# 统一遥操作系统架构

## 系统愿景

构建一个完整的 VR 遥操作系统，让用户：
- **眼睛** → 通过 PICO 头显看到机器人端的立体视觉
- **手臂** → 4个 Motion Tracker (2腕部+2肘部) 控制天机双臂姿态
- **手指** → Manus 手套 20 DoF (Mediapipe 格式，不含腕部)
- **输出** → 天机双臂 + 舞肌灵巧手执行动作

## 追踪方案说明

| 部位 | 追踪设备 | 输出 | 说明 |
|------|----------|------|------|
| **头部** | PICO 头显 | 头部 6DoF | 仅用于视觉显示，不控制机器人 |
| **左腕** | Motion Tracker #1 | 左臂末端 6DoF | 控制天机左臂末端位姿 |
| **右腕** | Motion Tracker #2 | 右臂末端 6DoF | 控制天机右臂末端位姿 |
| **左肘** | Motion Tracker #3 | 左臂肘角 | 提供臂型约束 (Elbow Hint) |
| **右肘** | Motion Tracker #4 | 右臂肘角 | 提供臂型约束 (Elbow Hint) |
| **左手指** | Manus 左手套 | 20 DoF | Mediapipe 格式 (5指×4关节) |
| **右手指** | Manus 右手套 | 20 DoF | Mediapipe 格式 (5指×4关节) |

**注意**: 腕部旋转由 Motion Tracker 提供，Manus 手套 **不控制腕部**，仅控制手指

---

## 完整系统架构

```
┌─────────────────────────────────────────────────────────────────────────────────────────┐
│                              VR 遥操作统一系统                                            │
├─────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                         │
│  ┌───────────────────────────────────────────────────────────────────────────────────┐ │
│  │                              用户端 (操作者)                                        │ │
│  │                                                                                    │ │
│  │   ┌─────────────────────────────────────────────────────────────────────────────┐ │ │
│  │   │                           穿戴设备布局                                        │ │ │
│  │   │                                                                              │ │ │
│  │   │                      ┌─────────────────┐                                     │ │ │
│  │   │                      │   PICO 头显      │                                     │ │ │
│  │   │                      │   (立体视觉)     │                                     │ │ │
│  │   │                      └─────────────────┘                                     │ │ │
│  │   │                                                                              │ │ │
│  │   │    ┌──────────────┐                        ┌──────────────┐                  │ │ │
│  │   │    │ Tracker #3   │                        │ Tracker #4   │                  │ │ │
│  │   │    │ (左肘)        │                        │ (右肘)        │                  │ │ │
│  │   │    └──────┬───────┘                        └───────┬──────┘                  │ │ │
│  │   │           │                                        │                         │ │ │
│  │   │    ┌──────▼───────┐                        ┌───────▼──────┐                  │ │ │
│  │   │    │ Tracker #1   │                        │ Tracker #2   │                  │ │ │
│  │   │    │ (左腕)        │                        │ (右腕)        │                  │ │ │
│  │   │    └──────┬───────┘                        └───────┬──────┘                  │ │ │
│  │   │           │                                        │                         │ │ │
│  │   │    ┌──────▼───────┐                        ┌───────▼──────┐                  │ │ │
│  │   │    │ Manus 左手套  │                        │ Manus 右手套  │                  │ │ │
│  │   │    │ 20 DoF       │                        │ 20 DoF       │                  │ │ │
│  │   │    │ (仅手指)      │                        │ (仅手指)      │                  │ │ │
│  │   │    └──────────────┘                        └──────────────┘                  │ │ │
│  │   └─────────────────────────────────────────────────────────────────────────────┘ │ │
│  │                                                                                    │ │
│  │   数据输出:                                                                         │ │
│  │   ├── Tracker #1,#2 (腕部) → 末端位姿 (position + orientation)                     │ │
│  │   ├── Tracker #3,#4 (肘部) → 臂型约束 (elbow hint position)                        │ │
│  │   └── Manus 手套 → 20 DoF (5指×4关节, Mediapipe 格式, 无腕部)                       │ │
│  │                                                                                    │ │
│  └───────────────────────────────────────────────────────────────────────────────────┘ │
│                                                                                         │
│               │ gRPC (Tracker×4)              │ USB/BT (Manus)                         │
│               ▼                               ▼                                         │
│  ┌───────────────────────────────────────────────────────────────────────────────────┐ │
│  │                         Linux 工作站 (Docker 容器)                                  │ │
│  │                                                                                    │ │
│  │   ┌───────────────────────────────────────────────────────────────────────────┐   │ │
│  │   │                    wuji-system-docker 容器                                 │   │ │
│  │   │                                                                            │   │ │
│  │   │   ┌─────────────────────────────────────────────────────────────────────┐ │   │ │
│  │   │   │ 输入层 (Input Layer)                                                 │ │   │ │
│  │   │   │                                                                      │ │   │ │
│  │   │   │ ┌────────────────────────────┐  ┌────────────────────────────────┐  │ │   │ │
│  │   │   │ │ XRoboToolkit PC-Service    │  │ Manus ROS2 Node                │  │ │   │ │
│  │   │   │ │                            │  │                                │  │ │   │ │
│  │   │   │ │ • Tracker #1 (左腕) 6DoF   │  │ • 左手 20 DoF (Mediapipe)      │  │ │   │ │
│  │   │   │ │ • Tracker #2 (右腕) 6DoF   │  │ • 右手 20 DoF (Mediapipe)      │  │ │   │ │
│  │   │   │ │ • Tracker #3 (左肘) 位置   │  │ • 不含腕部旋转                  │  │ │   │ │
│  │   │   │ │ • Tracker #4 (右肘) 位置   │  │                                │  │ │   │ │
│  │   │   │ └──────────────┬─────────────┘  └──────────────┬─────────────────┘  │ │   │ │
│  │   │   │                │                               │                    │ │   │ │
│  │   │   └────────────────┼───────────────────────────────┼────────────────────┘ │   │ │
│  │   │                    │                               │                      │   │ │
│  │   │   ┌────────────────▼───────────────────────────────▼────────────────────┐ │   │ │
│  │   │   │ 融合层 (Fusion Layer)                                                │ │   │ │
│  │   │   │                                                                      │ │   │ │
│  │   │   │ ┌──────────────────────────────────────────────────────────────────┐│ │   │ │
│  │   │   │ │ 机械臂 IK 求解器                                                  ││ │   │ │
│  │   │   │ │                                                                  ││ │   │ │
│  │   │   │ │ 输入:                                                            ││ │   │ │
│  │   │   │ │ • 腕部 Tracker → 末端目标位姿 (target_pose)                      ││ │   │ │
│  │   │   │ │ • 肘部 Tracker → 臂型约束 (elbow_hint)                           ││ │   │ │
│  │   │   │ │                                                                  ││ │   │ │
│  │   │   │ │ 输出:                                                            ││ │   │ │
│  │   │   │ │ • 天机臂 7 关节角度                                              ││ │   │ │
│  │   │   │ └──────────────────────────────────────────────────────────────────┘│ │   │ │
│  │   │   │                                                                      │ │   │ │
│  │   │   │ ┌──────────────────────────────────────────────────────────────────┐│ │   │ │
│  │   │   │ │ 灵巧手映射 (wuji_retargeting)                                    ││ │   │ │
│  │   │   │ │                                                                  ││ │   │ │
│  │   │   │ │ 输入:                                                            ││ │   │ │
│  │   │   │ │ • Manus 20 DoF (Mediapipe 格式)                                  ││ │   │ │
│  │   │   │ │                                                                  ││ │   │ │
│  │   │   │ │ 输出:                                                            ││ │   │ │
│  │   │   │ │ • 舞肌手 20 关节角度 (5指×4关节)                                 ││ │   │ │
│  │   │   │ └──────────────────────────────────────────────────────────────────┘│ │   │ │
│  │   │   └────────────────────────────────────────────────────────────────────┘ │   │ │
│  │   │                                                                            │   │ │
│  │   │   ┌─────────────────────────────────────────────────────────────────────┐ │   │ │
│  │   │   │ 输出层 (Output Layer)                                                │ │   │ │
│  │   │   │                                                                      │ │   │ │
│  │   │   │ ROS2 话题:                                                           │ │   │ │
│  │   │   │ • /tianji_arm/left/joint_command   → 天机左臂 7 DoF                  │ │   │ │
│  │   │   │ • /tianji_arm/right/joint_command  → 天机右臂 7 DoF                  │ │   │ │
│  │   │   │ • /wuji_hand/left/joint_command    → 舞肌左手 20 DoF                 │ │   │ │
│  │   │   │ • /wuji_hand/right/joint_command   → 舞肌右手 20 DoF                 │ │   │ │
│  │   │   └─────────────────────────────────────────────────────────────────────┘ │   │ │
│  │   │                                                                            │   │ │
│  │   │   ┌─────────────────────────────────────────────────────────────────────┐ │   │ │
│  │   │   │ 视觉层 (Vision Layer)                                                │ │   │ │
│  │   │   │                                                                      │ │   │ │
│  │   │   │ ┌────────────────────┐    ┌────────────────────────────────────┐    │ │   │ │
│  │   │   │ │ OpenCV 双目相机    │───>│ stereo_vision_server.py            │    │ │   │ │
│  │   │   │ │ (机器人头部)       │    │ • WebSocket 发送到 PICO 头显       │    │ │   │ │
│  │   │   │ └────────────────────┘    └────────────────────────────────────┘    │ │   │ │
│  │   │   └─────────────────────────────────────────────────────────────────────┘ │   │ │
│  │   └───────────────────────────────────────────────────────────────────────────┘   │ │
│  └───────────────────────────────────────────────────────────────────────────────────┘ │
│                                                                                         │
│               │ ROS2 控制指令                                                           │
│               ▼                                                                         │
│  ┌───────────────────────────────────────────────────────────────────────────────────┐ │
│  │                              机器人端 (执行)                                        │ │
│  │                                                                                    │ │
│  │   ┌─────────────────────────────────────────────────────────────────────────────┐ │ │
│  │   │                        天机双臂 + 舞肌灵巧手                                  │ │ │
│  │   │                                                                              │ │ │
│  │   │   ┌──────────────────────────┐      ┌──────────────────────────┐            │ │ │
│  │   │   │ 天机左臂 (7 DoF)          │      │ 天机右臂 (7 DoF)          │            │ │ │
│  │   │   │                          │      │                          │            │ │ │
│  │   │   │ J1: 肩部偏摆             │      │ J1: 肩部偏摆             │            │ │ │
│  │   │   │ J2: 肩部俯仰             │      │ J2: 肩部俯仰             │            │ │ │
│  │   │   │ J3: 肩部旋转             │      │ J3: 肩部旋转             │            │ │ │
│  │   │   │ J4: 肘部俯仰             │      │ J4: 肘部俯仰             │            │ │ │
│  │   │   │ J5: 前臂旋转             │      │ J5: 前臂旋转             │            │ │ │
│  │   │   │ J6: 腕部俯仰             │      │ J6: 腕部俯仰             │            │ │ │
│  │   │   │ J7: 腕部偏摆             │      │ J7: 腕部偏摆             │            │ │ │
│  │   │   │                          │      │                          │            │ │ │
│  │   │   │ ┌──────────────────────┐ │      │ ┌──────────────────────┐ │            │ │ │
│  │   │   │ │ 舞肌左手 (20 DoF)    │ │      │ │ 舞肌右手 (20 DoF)    │ │            │ │ │
│  │   │   │ │                      │ │      │ │                      │ │            │ │ │
│  │   │   │ │ 拇指: MCP, PIP, DIP, │ │      │ │ 拇指: MCP, PIP, DIP, │ │            │ │ │
│  │   │   │ │       Spread (4DoF)  │ │      │ │       Spread (4DoF)  │ │            │ │ │
│  │   │   │ │ 食指: MCP, PIP, DIP, │ │      │ │ 食指: MCP, PIP, DIP, │ │            │ │ │
│  │   │   │ │       Spread (4DoF)  │ │      │ │       Spread (4DoF)  │ │            │ │ │
│  │   │   │ │ 中指: MCP, PIP, DIP, │ │      │ │ 中指: MCP, PIP, DIP, │ │            │ │ │
│  │   │   │ │       Spread (4DoF)  │ │      │ │       Spread (4DoF)  │ │            │ │ │
│  │   │   │ │ 无名指: MCP,PIP,DIP, │ │      │ │ 无名指: MCP,PIP,DIP, │ │            │ │ │
│  │   │   │ │         Spread(4DoF) │ │      │ │         Spread(4DoF) │ │            │ │ │
│  │   │   │ │ 小指: MCP, PIP, DIP, │ │      │ │ 小指: MCP, PIP, DIP, │ │            │ │ │
│  │   │   │ │       Spread (4DoF)  │ │      │ │       Spread (4DoF)  │ │            │ │ │
│  │   │   │ └──────────────────────┘ │      │ └──────────────────────┘ │            │ │ │
│  │   │   └──────────────────────────┘      └──────────────────────────┘            │ │ │
│  │   │                                                                              │ │ │
│  │   │                     ┌──────────────────────┐                                 │ │ │
│  │   │                     │ 双目相机 (机器人头部) │                                 │ │ │
│  │   │                     │ → 立体视觉回传 PICO   │                                 │ │ │
│  │   │                     └──────────────────────┘                                 │ │ │
│  │   └─────────────────────────────────────────────────────────────────────────────┘ │ │
│  └───────────────────────────────────────────────────────────────────────────────────┘ │
│                                                                                         │
└─────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 现有 vs 规划对比

### 现有系统 (wuji-system-docker)

| 组件 | 状态 | 说明 |
|------|------|------|
| Manus 手套输入 | ✅ 已实现 | `manus_input` 节点 |
| HTC Vive Tracker | ✅ 已实现 | 手臂追踪 |
| Apple Vision Pro | ✅ 已实现 | 手部 + 手臂 |
| 舞肌灵巧手控制 | ✅ 已实现 | `wuji_hand_description` |
| 天机双臂控制 | ✅ 已实现 | 通过 ROS2 话题 |
| 运动重定向 | ✅ 已实现 | `wuji_retargeting` |
| 立体视觉回传 | ❌ 缺失 | **需要新增** |
| PICO 追踪输入 | ❌ 缺失 | **需要新增** |

### 需要集成的新组件

| 组件 | 来源 | 作用 |
|------|------|------|
| XRoboToolkit PC-Service | XR-Robotics | 接收 PICO 追踪数据 |
| pxrea Python 绑定 | XR-Robotics | Python API 获取追踪 |
| stereo_vision_server | 新开发 | 双目视频 → PICO |
| pico_tracking_node | 新开发 | PICO 追踪 → ROS2 |
| 修改版 Unity Client | Fork XRoboToolkit | 显示立体视觉 |

---

## Dockerfile 修改方案

在现有 `wuji-system-docker/Dockerfile` 基础上添加 XRoboToolkit 支持：

```dockerfile
# ===========================================
# Wuji System-Level Docker Image
# Extended with XRoboToolkit + Stereo Vision
# ===========================================

FROM osrf/ros:humble-desktop-full

# ... 现有配置保持不变 ...

# ===== 新增: XRoboToolkit 依赖 =====

# 安装 XRoboToolkit PC-Service
RUN wget -q https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.1.1/xrobotoolkit-pc-service_1.1.1_amd64.deb \
    && dpkg -i xrobotoolkit-pc-service_1.1.1_amd64.deb || apt-get install -f -y \
    && rm xrobotoolkit-pc-service_1.1.1_amd64.deb

# 克隆并安装 Python 绑定
RUN git clone https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git /tmp/pybind \
    && cd /tmp/pybind \
    && pip3 install . \
    && rm -rf /tmp/pybind

# ===== 新增: 立体视觉依赖 =====
RUN pip3 install websockets opencv-python-headless

# ... 其余配置保持不变 ...
```

---

## 新增 ROS2 包结构

```
ros2_ws/src/
├── wuji-hand-teleop-ros2/          # 现有: 手部遥操作
│   └── src/input_devices/
│       ├── manus_input/            # 现有: Manus 手套
│       ├── avp_input/              # 现有: Apple Vision Pro
│       └── pico_input/             # 新增: PICO 追踪输入
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── pico_input/
│           │   ├── __init__.py
│           │   └── pico_tracking_node.py
│           └── launch/
│               └── pico_tracking.launch.py
│
├── wuji_retargeting/               # 现有: 运动重定向
│
└── stereo_vision/                  # 新增: 立体视觉服务
    ├── CMakeLists.txt
    ├── package.xml
    ├── stereo_vision/
    │   ├── __init__.py
    │   ├── stereo_camera.py        # 双目相机采集
    │   └── vision_server.py        # WebSocket 服务器
    └── launch/
        └── stereo_vision.launch.py
```

---

## pico_tracker_node.py 代码

```python
#!/usr/bin/env python3
"""
PICO Motion Tracker ROS2 节点
从 XRoboToolkit 获取 4 个 Tracker 数据并发布为 ROS2 话题

Tracker 配置:
- Tracker #1: 左腕 (left_wrist) → 左臂末端位姿
- Tracker #2: 右腕 (right_wrist) → 右臂末端位姿
- Tracker #3: 左肘 (left_elbow) → 左臂臂型约束
- Tracker #4: 右肘 (right_elbow) → 右臂臂型约束
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import threading

try:
    import pxrea
    HAS_PXREA = True
except ImportError:
    HAS_PXREA = False


class PicoTrackerNode(Node):
    """PICO Motion Tracker 数据发布节点"""

    # Tracker ID 映射 (需要根据实际配对顺序调整)
    TRACKER_MAP = {
        0: 'left_wrist',   # Tracker #1
        1: 'right_wrist',  # Tracker #2
        2: 'left_elbow',   # Tracker #3
        3: 'right_elbow',  # Tracker #4
    }

    def __init__(self):
        super().__init__('pico_tracker_node')

        # 声明参数 (允许运行时重新映射 Tracker)
        self.declare_parameter('tracker_0_role', 'left_wrist')
        self.declare_parameter('tracker_1_role', 'right_wrist')
        self.declare_parameter('tracker_2_role', 'left_elbow')
        self.declare_parameter('tracker_3_role', 'right_elbow')

        # 发布者: 4 个 Tracker
        self.publishers = {
            'left_wrist': self.create_publisher(PoseStamped, '/pico/tracker/left_wrist', 10),
            'right_wrist': self.create_publisher(PoseStamped, '/pico/tracker/right_wrist', 10),
            'left_elbow': self.create_publisher(PoseStamped, '/pico/tracker/left_elbow', 10),
            'right_elbow': self.create_publisher(PoseStamped, '/pico/tracker/right_elbow', 10),
        }

        # 数据缓存
        self.lock = threading.Lock()
        self.tracker_data = {}  # {tracker_id: pose_data}

        # 定时发布 (200Hz, Motion Tracker 原生频率)
        self.timer = self.create_timer(1.0/200.0, self.publish_data)

        # 启动 pxrea (仅订阅 Tracker 数据: 0x10)
        if HAS_PXREA:
            # data_mask: 0x10 = TRACKER
            result = pxrea.PXREAInit(None, self._data_callback, 0x10)
            if result == 0:
                self.get_logger().info('PICO Tracker 服务已启动，等待 4 个 Tracker 连接...')
            else:
                self.get_logger().error(f'PICO Tracker 启动失败: {result}')
        else:
            self.get_logger().error('pxrea 模块未安装')

    def _data_callback(self, data_type: int, data: dict):
        """pxrea 数据回调"""
        if data_type != 0x10:  # 仅处理 Tracker 数据
            return

        with self.lock:
            # data 格式: {'trackers': [{id, position, rotation}, ...]}
            trackers = data.get('trackers', [])
            for tracker in trackers:
                tracker_id = tracker.get('id', -1)
                if 0 <= tracker_id <= 3:
                    self.tracker_data[tracker_id] = {
                        'position': tracker.get('position', [0, 0, 0]),
                        'rotation': tracker.get('rotation', [0, 0, 0, 1]),
                    }

    def publish_data(self):
        """发布 Tracker 数据"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'pico_world'

        with self.lock:
            for tracker_id, role in self.TRACKER_MAP.items():
                if tracker_id not in self.tracker_data:
                    continue

                data = self.tracker_data[tracker_id]
                msg = PoseStamped()
                msg.header = header
                msg.header.frame_id = f'tracker_{role}'

                pos = data['position']
                rot = data['rotation']

                msg.pose.position.x = float(pos[0])
                msg.pose.position.y = float(pos[1])
                msg.pose.position.z = float(pos[2])
                msg.pose.orientation.x = float(rot[0])
                msg.pose.orientation.y = float(rot[1])
                msg.pose.orientation.z = float(rot[2])
                msg.pose.orientation.w = float(rot[3])

                if role in self.publishers:
                    self.publishers[role].publish(msg)

    def destroy_node(self):
        if HAS_PXREA:
            pxrea.PXREADeinit()
        super().destroy_node()


def main():
    rclpy.init()
    node = PicoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## IK 求解器与 Elbow Hint

### 问题背景

7 DoF 机械臂的逆运动学 (IK) 存在**冗余自由度**问题：给定末端位姿 (6 DoF)，关节配置不唯一。
通过引入 **Elbow Hint** (肘部位置约束)，可以唯一确定臂型姿态，使机器人动作更自然。

### 数据流

```
Motion Tracker #1/#2 (腕部)                Motion Tracker #3/#4 (肘部)
        ↓                                           ↓
  末端目标位姿                                   Elbow Hint 位置
  (position + orientation)                     (position only)
        ↓                                           ↓
        └───────────────────┬───────────────────────┘
                            ↓
                   ┌────────────────────┐
                   │  IK 求解器         │
                   │                    │
                   │  minimize:         │
                   │  ‖elbow - hint‖²  │
                   │                    │
                   │  subject to:       │
                   │  FK(q) = target    │
                   └────────┬───────────┘
                            ↓
                   天机臂 7 关节角度
```

### arm_ik_solver 示例代码

```python
#!/usr/bin/env python3
"""
天机臂 IK 求解器 (带 Elbow Hint 约束)
"""

import numpy as np
import nlopt
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


class ArmIKSolver(Node):
    """使用 Elbow Hint 约束的 7-DoF 机械臂 IK 求解器"""

    def __init__(self):
        super().__init__('arm_ik_solver')

        # 参数
        self.declare_parameter('use_elbow_hint', True)
        self.declare_parameter('elbow_weight', 0.1)  # Elbow Hint 权重

        self.use_elbow_hint = self.get_parameter('use_elbow_hint').value
        self.elbow_weight = self.get_parameter('elbow_weight').value

        # 订阅 Tracker 数据
        self.left_wrist_sub = self.create_subscription(
            PoseStamped, '/pico/tracker/left_wrist',
            lambda msg: self._wrist_callback(msg, 'left'), 10)
        self.right_wrist_sub = self.create_subscription(
            PoseStamped, '/pico/tracker/right_wrist',
            lambda msg: self._wrist_callback(msg, 'right'), 10)
        self.left_elbow_sub = self.create_subscription(
            PoseStamped, '/pico/tracker/left_elbow',
            lambda msg: self._elbow_callback(msg, 'left'), 10)
        self.right_elbow_sub = self.create_subscription(
            PoseStamped, '/pico/tracker/right_elbow',
            lambda msg: self._elbow_callback(msg, 'right'), 10)

        # 发布关节命令
        self.left_joint_pub = self.create_publisher(
            JointState, '/tianji_arm/left/joint_command', 10)
        self.right_joint_pub = self.create_publisher(
            JointState, '/tianji_arm/right/joint_command', 10)

        # 数据缓存
        self.wrist_targets = {'left': None, 'right': None}
        self.elbow_hints = {'left': None, 'right': None}
        self.current_joints = {'left': np.zeros(7), 'right': np.zeros(7)}

        # 定时 IK 求解 (100Hz)
        self.timer = self.create_timer(0.01, self._solve_ik)

        self.get_logger().info('ARM IK Solver 已启动 (Elbow Hint: %s)' % self.use_elbow_hint)

    def _wrist_callback(self, msg: PoseStamped, side: str):
        """更新腕部目标位姿"""
        self.wrist_targets[side] = msg

    def _elbow_callback(self, msg: PoseStamped, side: str):
        """更新肘部 Hint"""
        self.elbow_hints[side] = msg

    def _solve_ik(self):
        """求解双臂 IK"""
        for side in ['left', 'right']:
            if self.wrist_targets[side] is None:
                continue

            # 提取目标位姿
            target_pose = self.wrist_targets[side].pose
            target_pos = np.array([
                target_pose.position.x,
                target_pose.position.y,
                target_pose.position.z
            ])
            target_rot = Rotation.from_quat([
                target_pose.orientation.x,
                target_pose.orientation.y,
                target_pose.orientation.z,
                target_pose.orientation.w
            ])

            # 提取 Elbow Hint (如果有)
            elbow_hint = None
            if self.use_elbow_hint and self.elbow_hints[side] is not None:
                ep = self.elbow_hints[side].pose.position
                elbow_hint = np.array([ep.x, ep.y, ep.z])

            # 求解 IK
            joint_angles = self._solve_single_arm(
                side, target_pos, target_rot, elbow_hint
            )

            # 发布结果
            if joint_angles is not None:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = [f'{side}_joint_{i}' for i in range(7)]
                msg.position = joint_angles.tolist()

                if side == 'left':
                    self.left_joint_pub.publish(msg)
                else:
                    self.right_joint_pub.publish(msg)

    def _solve_single_arm(self, side: str, target_pos: np.ndarray,
                          target_rot: Rotation, elbow_hint: np.ndarray = None) -> np.ndarray:
        """
        求解单臂 IK (带 Elbow Hint 约束)

        使用 NLopt 优化器，目标函数:
        f(q) = ||FK_position(q) - target_pos||²
             + ||FK_orientation(q) - target_rot||²
             + w * ||FK_elbow(q) - elbow_hint||²  (如果有 hint)
        """
        # 初始猜测: 使用上一帧结果
        q0 = self.current_joints[side].copy()

        # 这里应该实现完整的 IK 求解器
        # 使用 nlopt 或 pinocchio 等库
        # 此处为伪代码示意

        # 实际项目中应调用 pinocchio 等机器人学库
        # q_solution = solve_ik_with_nlopt(...)

        # 暂时返回初始值 (需要实际实现)
        q_solution = q0

        self.current_joints[side] = q_solution
        return q_solution


def main():
    rclpy.init()
    node = ArmIKSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## 新的 Launch 文件

### wuji_teleop_pico.launch.py

```python
#!/usr/bin/env python3
"""
PICO + Manus 遥操作启动文件
使用 4×Motion Tracker + Manus 手套

Tracker 配置:
- Tracker #1,#2: 腕部 → 末端位姿
- Tracker #3,#4: 肘部 → Elbow Hint
- Manus: 仅手指 20 DoF (无腕部)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 参数
    enable_rviz = LaunchConfiguration('enable_rviz', default='false')
    enable_stereo = LaunchConfiguration('enable_stereo', default='true')

    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument('enable_rviz', default_value='false'),
        DeclareLaunchArgument('enable_stereo', default_value='true'),

        # 1. PICO Motion Tracker 节点 (4个 Tracker)
        Node(
            package='pico_input',
            executable='pico_tracker_node',
            name='pico_tracker',
            output='screen',
            parameters=[{
                'tracker_0_role': 'left_wrist',
                'tracker_1_role': 'right_wrist',
                'tracker_2_role': 'left_elbow',
                'tracker_3_role': 'right_elbow',
            }],
        ),

        # 2. Manus 手套节点 (仅手指, 无腕部)
        Node(
            package='manus_input',
            executable='manus_ros2_node',
            name='manus_input',
            output='screen',
        ),

        # 3. 立体视觉服务 (可选)
        Node(
            package='stereo_vision',
            executable='vision_server',
            name='stereo_vision',
            output='screen',
            condition=LaunchConfiguration('enable_stereo'),
        ),

        # 4. 机械臂 IK 求解器 (使用 Elbow Hint)
        Node(
            package='tianji_arm_ik',
            executable='arm_ik_solver',
            name='arm_ik_solver',
            output='screen',
            parameters=[{
                'use_elbow_hint': True,  # 启用肘部约束
            }],
            remappings=[
                # 输入: Tracker 数据
                ('/left_wrist_target', '/pico/tracker/left_wrist'),
                ('/right_wrist_target', '/pico/tracker/right_wrist'),
                ('/left_elbow_hint', '/pico/tracker/left_elbow'),
                ('/right_elbow_hint', '/pico/tracker/right_elbow'),
                # 输出: 关节命令
                ('/left_joint_cmd', '/tianji_arm/left/joint_command'),
                ('/right_joint_cmd', '/tianji_arm/right/joint_command'),
            ],
        ),

        # 5. 手部运动重定向 (Manus → 舞肌手)
        Node(
            package='wuji_retargeting',
            executable='retarget_node',
            name='hand_retargeting',
            output='screen',
            parameters=[{
                'hand_input': 'manus',
                'input_format': 'mediapipe',  # Manus 使用 Mediapipe 格式
            }],
        ),

        # 6. 天机双臂控制
        Node(
            package='tianji_arm_control',
            executable='arm_controller',
            name='tianji_arm',
            output='screen',
        ),

        # 7. 舞肌灵巧手控制
        Node(
            package='wuji_hand_control',
            executable='hand_controller',
            name='wuji_hand',
            output='screen',
        ),
    ])
```

---

## 使用方式更新

### 启动完整遥操作系统

```bash
# 进入容器
docker compose exec wuji-system bash

# 方式 1: PICO 体感追踪 + Manus 手套 (新增)
ros2 launch wuji_teleop_bringup wuji_teleop_pico.launch.py

# 方式 2: PICO 体感 + Manus 手套 + 立体视觉
ros2 launch wuji_teleop_bringup wuji_teleop_pico.launch.py enable_stereo:=true

# 方式 3: 传统模式 (HTC Vive + Manus)
ros2 launch wuji_teleop_bringup wuji_teleop.launch.py hand_input:=manus arm_input:=tracker
```

---

## ROS2 话题总览

### 输入话题 (从设备采集)

| 话题 | 类型 | 来源 | 频率 | 说明 |
|------|------|------|------|------|
| `/pico/tracker/left_wrist` | PoseStamped | Motion Tracker #1 | 200Hz | 左臂末端目标位姿 |
| `/pico/tracker/right_wrist` | PoseStamped | Motion Tracker #2 | 200Hz | 右臂末端目标位姿 |
| `/pico/tracker/left_elbow` | PoseStamped | Motion Tracker #3 | 200Hz | 左臂肘部约束 (Elbow Hint) |
| `/pico/tracker/right_elbow` | PoseStamped | Motion Tracker #4 | 200Hz | 右臂肘部约束 (Elbow Hint) |
| `/manus/left_hand` | JointState | Manus 左手套 | 60Hz | 20 DoF 手指关节 (Mediapipe) |
| `/manus/right_hand` | JointState | Manus 右手套 | 60Hz | 20 DoF 手指关节 (Mediapipe) |

**注意**:
- Motion Tracker 提供腕部 6DoF 位姿，用于 IK 求解天机臂末端位置
- 肘部 Tracker 提供 Elbow Hint，用于约束 IK 求解时的臂型姿态
- Manus 手套仅提供手指数据，**不包含腕部旋转**

### 输出话题 (发送到机器人)

| 话题 | 类型 | 目标 | 频率 |
|------|------|------|------|
| `/tianji_arm/left/joint_command` | JointState | 天机左臂 | 100Hz |
| `/tianji_arm/right/joint_command` | JointState | 天机右臂 | 100Hz |
| `/wuji_hand/left/joint_command` | JointState | 舞肌左手 | 100Hz |
| `/wuji_hand/right/joint_command` | JointState | 舞肌右手 | 100Hz |

---

## 实施路线图

### 阶段 1: 环境准备

- [ ] 更新 Dockerfile 添加 XRoboToolkit 依赖
- [ ] 测试 PC-Service 在容器内运行
- [ ] 验证 pxrea Python 绑定

### 阶段 2: PICO 追踪集成

- [ ] 创建 `pico_input` ROS2 包
- [ ] 实现 `pico_tracking_node.py`
- [ ] 测试 PICO → ROS2 数据流
- [ ] 与现有 `wuji_retargeting` 对接

### 阶段 3: 立体视觉集成

- [ ] 创建 `stereo_vision` ROS2 包
- [ ] 实现 WebSocket 视频服务器
- [ ] Fork 并修改 XRoboToolkit Unity Client
- [ ] 测试双目视频 → PICO 头显

### 阶段 4: 系统集成

- [ ] 创建统一 launch 文件
- [ ] 端到端测试
- [ ] 延迟优化
- [ ] 文档更新

---

## 硬件清单

| 设备 | 数量 | 用途 | 状态 |
|------|------|------|------|
| PICO 4 Ultra | 1 | VR 头显 (立体视觉显示) | 需采购 |
| PICO Motion Tracker | 4 | 2×腕部 + 2×肘部 追踪 | 需采购 |
| Manus Quantum Metaglove | 1对 | 手指追踪 (20 DoF/手) | ✅ 已有 |
| 天机双臂 | 1套 | 机械臂 (7 DoF/臂) | ✅ 已有 |
| 舞肌灵巧手 | 1对 | 灵巧手 (20 DoF/手) | ✅ 已有 |
| USB 双目相机 | 1 | 立体视觉采集 | ✅ 已有 |
| Linux 工作站 | 1 | 运行 Docker 容器 | ✅ 已有 |

### Motion Tracker 佩戴位置

| Tracker ID | 佩戴位置 | 功能 |
|------------|----------|------|
| #1 | 左手腕 | 左臂末端 6DoF 目标位姿 |
| #2 | 右手腕 | 右臂末端 6DoF 目标位姿 |
| #3 | 左上臂/肘部 | 左臂 Elbow Hint |
| #4 | 右上臂/肘部 | 右臂 Elbow Hint |

---

## 参考资源

- [XRoboToolkit GitHub](https://github.com/XR-Robotics)
- [XRoboToolkit 论文](https://arxiv.org/html/2508.00097v1)
- [wuji-system-docker](本仓库)
- [TWIST2 人形机器人遥操作](https://github.com/amazon-far/TWIST2)
