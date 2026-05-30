# stargazer

[中文文档](README-zh.md)

Realtime 3D web viewer for robot swarms. Visualizes trajectories, point clouds, and URDF models directly from ROS 2 topics — no frontend code required.

Built on [three.js](https://threejs.org) + [rosbridge](https://github.com/RobotWebTools/rosbridge_suite). Dark by default.

![stargazer](https://img.shields.io/badge/stack-three.js%20%2B%20rosbridge-blue)

---

## Architecture

```
Browser (phone / desktop)
    │
    ▼ HTTP :8080
┌──────────────┐      ws://host/ws       ┌──────────────┐
│   nginx      │ ─────────────────────── │  rosbridge   │
│  (static +   │                         │  (WebSocket) │
│   ws proxy)  │                         └──────┬───────┘
└──────────────┘                                │
                                          ┌─────┴─────┐
                                          │  ROS 2     │
                                          │  topics    │
                                          └───────────┘
```

- **nginx** serves the single-page app and proxies WebSocket traffic on a single port (no CORS / firewall headaches).
- **rosbridge** bridges ROS 2 topics to the browser via WebSocket.
- **three.js** renders everything client-side — no server-side rendering, zero backend state.

---

## Quick Start

### 1. Start rosbridge inside your ROS 2 workspace

```bash
# In your ROS 2 container / host
sudo apt install ros-humble-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

rosbridge listens on `ws://localhost:9090` by default.

### 2. Launch stargazer with Docker

```bash
docker run -d --name stargazer \
  -p 8080:8080 \
  -v $(pwd)/index.html:/usr/share/nginx/html/index.html:ro \
  -v $(pwd)/nginx.conf:/etc/nginx/nginx.conf:ro \
  --add-host=host.docker.internal:host-gateway \
  nginx:alpine
```

### 3. Open your browser

```
http://localhost:8080
```

The HUD shows connection status. Green = connected to rosbridge.

---

## ROS 2 Topics

stargazer subscribes to standard ROS 2 message types. Topic names are hardcoded in `index.html` — edit the `subscribe` calls to match your setup.

| Topic | Message Type | What It Shows |
|---|---|---|
| `/dlio/odom_node/odom` | `nav_msgs/Odometry` | Robot pose + trajectory line |
| `/dlio/odom_node/pointcloud/deskewed` | `sensor_msgs/PointCloud2` | Real-time point cloud |

### Adding your own topics

Edit the `connect()` function in `index.html`:

```js
ws.send(JSON.stringify({
  op: 'subscribe',
  topic: '/your_robot/odom',
  type: 'nav_msgs/msg/Odometry'
}));
```

Then add a handler in the `onmessage` callback and a corresponding render function.

---

## Coordinate Frames

ROS uses `X-forward, Y-left, Z-up`. three.js defaults to `Y-up`. stargazer remaps:

```
three_x =  ros_x
three_y =  ros_z
three_z = -ros_y
```

The remap function lives at the top of the odometry and point cloud handlers — easy to adjust for your convention.

---

## Mobile Access (WSL / Windows)

If you're running stargazer on WSL and want to view it on your phone:

### 1. Set up Windows port forwarding (once)

```powershell
# Run in PowerShell as Administrator
$wsl_ip = (wsl hostname -I).Trim().Split()[0]
netsh interface portproxy add v4tov4 listenport=8080 listenaddress=0.0.0.0 connectport=8080 connectaddress=$wsl_ip
```

### 2. Allow through Windows Firewall

```powershell
New-NetFirewallRule -DisplayName "stargazer" -Direction Inbound -Protocol TCP -LocalPort 8080 -Action Allow
```

### 3. Open on your phone

```
http://<your-windows-lan-ip>:8080
```

Make sure your phone is on the same WiFi and any VPN/proxy (Clash, etc.) is disabled — WebSocket connections don't survive most mobile proxies.

---

## Multi-Robot Setup

stargazer is designed for swarm visualization from day one. To add robots:

1. Subscribe to each robot's namespace (e.g. `/robot1/odom`, `/robot2/odom`)
2. Give each robot a unique color in its render function
3. Add a label or model per robot

Example skeleton:

```js
const robots = {
  'robot1': { color: 0x44aaff, traj: [], mesh: null },
  'robot2': { color: 0xff8844, traj: [], mesh: null },
};

// In onmessage:
if (topic.startsWith('/robot1')) robots.robot1.traj.push(...);
if (topic.startsWith('/robot2')) robots.robot2.traj.push(...);
```

---

## Customization

All rendering logic is in a single HTML file. Key knobs:

| What | Where |
|---|---|
| Topic subscriptions | `connect()` function |
| Coordinate mapping | `rosToThree()` helper |
| Scene colors / fog / grid | Scene Setup section |
| Point cloud size | `pcMat` material `size` |
| Trajectory color / width | `trajMat` and `trajLine` materials |
| Robot model | Replace `BoxGeometry` with a GLTF or URDF loader |

---

## Dependencies

Everything is loaded from CDN at runtime — no `npm install` required.

- [three.js](https://threejs.org) (v0.160, via importmap CDN)
- [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite) (ROS 2 Humble)
- [nginx:alpine](https://hub.docker.com/_/nginx) (Docker)

---

## License

MIT
