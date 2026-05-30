# stargazer

实时 3D 网页端机器人集群可视化工具。直接读取 ROS 2 话题，实时渲染轨迹、点云和可动 URDF 模型 —— 无需写任何前端代码。

基于 [three.js](https://threejs.org) + [rosbridge](https://github.com/RobotWebTools/rosbridge_suite)。默认暗色主题。

![stargazer](https://img.shields.io/badge/技术栈-three.js%20%2B%20rosbridge-blue)

---

## 架构

```
浏览器（手机/电脑）
    │
    ▼ HTTP :8080
┌──────────────┐      ws://host/ws       ┌──────────────┐
│   nginx      │ ─────────────────────── │  rosbridge   │
│  (静态页面 + │                         │  (WebSocket) │
│   模型文件)  │                         └──────┬───────┘
└──────────────┘                                │
                                          ┌─────┴─────┐
                                          │  ROS 2     │
                                          │  topics    │
                                          └───────────┘
```

- **nginx** 提供静态页面和模型文件，并代理 WebSocket，单端口搞定。
- **rosbridge** 把 ROS 2 话题通过 WebSocket 桥接到浏览器。
- **three.js** 纯前端渲染——包括 GLTF 机器人模型 + 关节动画。

---

## 快速开始

### 1. 在你的 ROS 2 环境中启动 rosbridge

```bash
# 在 ROS 2 容器或宿主机中
sudo apt install ros-humble-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

rosbridge 默认监听 `ws://localhost:9090`。

### 2. Docker 启动 stargazer

```bash
docker run -d --name stargazer \
  -p 8080:8080 \
  -v $(pwd):/usr/share/nginx/html:ro \
  -v $(pwd)/nginx.conf:/etc/nginx/nginx.conf:ro \
  --add-host=host.docker.internal:host-gateway \
  nginx:alpine
```

### 3. 打开浏览器

```
http://localhost:8080
```

左上角 HUD 显示连接状态。绿色 = 已连接 rosbridge。

---

## 机器人模型 —— URDF 转 GLB

stargazer 从 GLB 文件加载可动关节的机器人模型。转换器 `urdf2glb.py` 把 URDF + Collada DAE 网格转成带完整运动学节点层级的 GLB——顶点保留在 link 局部坐标系，关节 origin 作为 GLTF node transform，运行时独立驱动每个关节。

### 转换 URDF

```bash
# 安装依赖
pip install -r requirements.txt

# 转换
python3 urdf2glb.py models/go1/go1.urdf models/go1/go1.glb
```

### 或用 Docker

```bash
docker build -t stargazer-converter .
docker run --rm -v $(pwd):/work stargazer-converter models/go1/go1.urdf models/go1/go1.glb
```

### 转换流程

```
DAE 原始网格
  → 应用 DAE scene node 变换    （修正 CAD 导出朝向）
  → 应用 URDF visual origin     （link 内放置位置）
  → 存储为 link-local 顶点数据

GLTF node = joint origin         （运动学链，每个关节独立矩阵）
GLTF 层级 = URDF 关节树          （base → trunk → FR_hip → FR_thigh → FR_calf）
```

相同 (DAE 文件, visual origin, scale) 的网格在转换时自动去重。

---

## 关节动画

关节定义在 `index.html` → `JOINT_DEFS`。每个条目把 GLTF 节点名映射到旋转轴（来自 URDF joint axis）。

```js
const JOINT_DEFS = {
  FR_hip:  { axis: [1, 0, 0] },  // 绕 X 轴旋转 (髋关节)
  FR_thigh: { axis: [0, 1, 0] },  // 绕 Y 轴旋转 (大腿)
  FR_calf:  { axis: [0, 1, 0] },  // 绕 Y 轴旋转 (小腿)
  // ... 共 12 个关节 (4 条腿 × 3 自由度)
};
```

步态动画 (`updateGait()`) 用 sin/cos 驱动关节角度，trot 步态（对角腿同相）。修改 `gait` 参数可调速度和幅度。

机器人底座 (`base` 节点) 跟随 SLAM 里程计——位置和朝向直接从 `/odom` 消息设置（ROS 坐标系）。`robotRoot` group 统一做 ROS→three.js 坐标变换。

---

## ROS 2 话题

| 话题 | 消息类型 | 显示 |
|---|---|---|
| `/dlio/odom_node/odom` | `nav_msgs/Odometry` | 机器人位姿 + 轨迹线 |
| `/dlio/odom_node/pointcloud/deskewed` | `sensor_msgs/PointCloud2` | 实时点云 |

### 添加自定义话题

编辑 `index.html` 中的 `connect()` 函数：

```js
ws.send(JSON.stringify({
  op: 'subscribe',
  topic: '/你的机器人/odom',
  type: 'nav_msgs/msg/Odometry'
}));
```

然后在 `onmessage` 回调中添加对应处理函数和渲染逻辑。

---

## 坐标系转换

ROS 使用 `X-前, Y-左, Z-上`。three.js 默认 Y 轴朝上。

**GLB 模型**存储在 ROS/URDF 坐标系中。`robotRoot` group 在运行时施加 `-π/2` X 轴旋转，将整个机器人转换到 three.js 空间。

**点云和轨迹**由 `rosToThree()` 逐点转换：

```
three_x =  ros_x
three_y =  ros_z
three_z = -ros_y
```

---

## 手机访问（WSL / Windows）

### 1. 配置 Windows 端口转发（一次）

```powershell
# 在 PowerShell（管理员）中运行
$wsl_ip = (wsl hostname -I).Trim().Split()[0]
netsh interface portproxy add v4tov4 listenport=8080 listenaddress=0.0.0.0 connectport=8080 connectaddress=$wsl_ip
```

### 2. 放行 Windows 防火墙

```powershell
New-NetFirewallRule -DisplayName "stargazer" -Direction Inbound -Protocol TCP -LocalPort 8080 -Action Allow
```

### 3. 手机浏览器打开

```
http://<你电脑的局域网IP>:8080
```

确保手机和电脑连同一个 WiFi，**关闭 Clash / 代理**——大部分移动端代理会拦截 WebSocket 连接。

---

## 多机器人配置

stargazer 从架构上就是为集群可视化设计的。添加机器人：

1. 订阅每个机器人的命名空间（如 `/robot1/odom`, `/robot2/odom`）
2. 在渲染函数中给每个机器人分配不同的颜色
3. 添加标签或模型

示例骨架：

```js
const robots = {
  'robot1': { color: 0x44aaff, traj: [], mesh: null },
  'robot2': { color: 0xff8844, traj: [], mesh: null },
};

// 在 onmessage 中:
if (topic.startsWith('/robot1')) robots.robot1.traj.push(...);
if (topic.startsWith('/robot2')) robots.robot2.traj.push(...);
```

---

## 自定义

所有渲染逻辑在单个 HTML 文件中。可调的参数：

| 改什么 | 在哪改 |
|---|---|
| 话题订阅 | `connect()` 函数 |
| 坐标系映射 | `rosToThree()` 函数 |
| 关节定义 | `JOINT_DEFS` 对象 |
| 步态速度/幅度 | `gait` 对象 |
| 场景颜色/雾/网格 | Scene Setup 区域 |
| 点云大小 | `pcMat` 的 `size` 属性 |
| 轨迹颜色/粗细 | `trajMat` 和 `trajLine` 材质 |

---

## 依赖

- [three.js](https://threejs.org) (v0.160，通过 importmap CDN)
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) (ROS 2 Humble)
- [nginx:alpine](https://hub.docker.com/_/nginx) (Docker，用于 Web 部署)

URDF → GLB 转换：
- [trimesh](https://trimesh.org) (DAE 加载 + GLTF 导出)
- [pycollada](https://github.com/pycollada/pycollada) (DAE scene transform 提取)
- numpy

---

## 许可证

MIT
