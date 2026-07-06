# 01-CAD-to-MJCF Demo

两个入门演示，体验 ElectronBot MuJoCo 仿真模型。

## 前提

```bash
cd ElectronBot_SIM
pip install mujoco
```

## Demo 1: 手动控制

启动 MuJoCo viewer，右侧 Control 面板拖动 actuator slider 控制关节。

**有桌面（直接运行）：**

```bash
python demos/01-CAD-to-MJCF_Demo/01_manual_control.py
```

**SSH / 无桌面（Xvfb 虚拟桌面 + 浏览器查看）：**

<details>
<summary>🔧 环境部署（只需一次）</summary>

```bash
# 安装 Xvfb + x11vnc
sudo apt install xvfb x11vnc -y

# 安装 websockify
pip install websockify

# 下载 noVNC
git clone https://github.com/novnc/noVNC.git /tmp/novnc
```

</details>

**每次运行：**

```bash
# 1. 启动虚拟桌面
Xvfb :99 -screen 0 1024x768x24 &

# 2. 启动 VNC → Web 桥接 (VNC:5901 → 网页:6080)
x11vnc -display :99 -forever -nopw &
/tmp/novnc/utils/novnc_proxy --vnc localhost:5901 --listen 6080 &

# 3. 运行 viewer
DISPLAY=:99 python3 demos/01-CAD-to-MJCF_Demo/01_manual_control.py

# 4. 浏览器打开 http://localhost:6080/vnc.html → 点 Connect
```

</details>

**关闭（按需）：**

```bash
pkill -f Xvfb; pkill -f x11vnc; pkill -f novnc_proxy
```

**简化版（只看 GIF，不需要 viewer 窗口，用 Demo 2 替代）：**

```bash
MUJOCO_GL=egl python3 demos/01-CAD-to-MJCF_Demo/02_sequence_demo.py
```

| slider | 关节 | 范围 |
|--------|------|------|
| `act_body` | 腰部旋转 (Z轴) | ±90° |
| `act_head` | 头部俯仰 (Y轴) | ±15° |
| `act_left_pitch` | 左臂 Pitch (Y轴) | ±90° |
| `act_left_roll` | 左臂 Roll (X轴) | ±45° |
| `act_right_pitch` | 右臂 Pitch (Y轴) | ±90° |
| `act_right_roll` | 右臂 Roll (X轴) | ±45° |

快捷键：空格=暂停/继续，R=重置，滚轮=缩放，右键拖拽=旋转视角

## Demo 2: 程序控制

自动播放 12 种预设动作序列（挥手、点头、比心、再见等）。

**有桌面（交互窗口）：**

```bash
python demos/01-CAD-to-MJCF_Demo/02_sequence_demo.py --interactive
```

**无桌面（生成 GIF）：**

```bash
MUJOCO_GL=egl python demos/01-CAD-to-MJCF_Demo/02_sequence_demo.py
```

输出目录默认为 `demos/01-CAD-to-MJCF_Demo/02_sequence_demo_gif/`，可用 `--output` 自定义：

```bash
MUJOCO_GL=egl python demos/01-CAD-to-MJCF_Demo/02_sequence_demo.py --output ./my_demo
```
