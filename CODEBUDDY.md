# CODEBUDDY.md This file provides guidance to CodeBuddy when working with code in this repository.

## Common Commands

### Installation
```bash
# Basic installation (simulation core)
pip install -e .
# Full installation (AI training + all optional dependencies)
pip install -e ".[full]"
# Selective installs
pip install -e ".[ai]"        # AI training pipeline
pip install -e ".[dev]"       # dev tools (pytest, ruff)
pip install -e ".[sensors]"   # OpenCV for camera
```

### Testing
```bash
# Run all tests
pytest tests/ -v
# Run a single test file
pytest tests/test_env.py -v
# Run tests matching a keyword
pytest tests/ -v -k "reset"
# Skip slow or GPU tests
pytest tests/ -v -m "not slow and not gpu"
# With coverage
pytest tests/ -v --cov=electronbot_sim --cov-report=html
```

### Linting
```bash
ruff check src/
```

### Environment Variables
- `MUJOCO_GL=egl` — **must be set to `egl`** for correct RGB rendering (OSMesa produces grayscale in this environment)
- `ELECTRONBOT_SIM_ASSETS_PATH` — override default `assets/mjcf/` path
- `ELECTRONBOT_SIM_DISABLE_DR=1` — disable domain randomization for baseline experiments
- `ELECTRONBOT_SIM_DT` — override simulation timestep (default `0.02`)
- `ELECTRONBOT_SIM_HEADLESS=1` — headless mode for interactive script

### Running Demos and Scripts
```bash
# Manual control with MuJoCo viewer
python demos/01-CAD-to-MJCF_Demo/01_manual_control.py
# Auto-play 12 preset actions
python demos/01-CAD-to-MJCF_Demo/02_sequence_demo.py --interactive
# Keyboard interactive control
python -m electronbot_sim.interactive
# Benchmark
python -m electronbot_benchmark.run --tasks all --episodes 10
```

---

## Architecture Overview

ElectronBot-SIM is a MuJoCo-based physics simulation and AI training platform for the ElectronBot desktop robot — a 6-DOF ESP32-driven robot with dual arms (3 joints each), a rotating body, and a nodding head. The project provides a complete pipeline from CAD model to sim-to-real deployment.

### Four Sub-Packages (src/)

1. **`electronbot_sim`** — Core simulation layer: Gymnasium `ElectronBotEnv`, MCP JSON-RPC bridge (`McpSimBridge`), unified `ElectronBotBackend` (sim/cloud mode switching), sensor system (camera, joint, contact), 12 preset actions, and keyboard interactive control.

2. **`electronbot_ai`** — AI training: 7 standard tasks (Reach/Push/PickPlace/Stack/Follow/Gesture/VoiceCmd), imitation learning (BC + ACT), reinforcement learning (PPO with SubprocVecEnv 64-way parallelism), and VLA planning via LLM. Tasks extend `BaseTask` abstract class and are gym-compatible.

3. **`electronbot_benchmark`** — Standardized evaluation suite with success rate, completion time, and trajectory smoothness metrics. Supports batch evaluation across tasks × algorithms.

4. **`electronbot_sim2real`** — Deployment to real hardware: cloud API proxy, ONNX model deployment, servo calibration, and capability downgrade strategies.

### Layered Architecture

The system is organized in five layers, with the strategy that upper layers never need to know whether the backend is simulation or real hardware:

```
Layer 5: Sensors (camera / joint / contact)
Layer 4: Actions (12 preset actions, 1:1 aligned with ESP32 firmware)
Layer 3: MCP Bridge (JSON-RPC 2.0, WebSocket, 12 tools)
Layer 2: MuJoCo Simulation (Gymnasium RL Env, domain randomization)
Layer 1: CAD/MJCF Physical Models
```

The key integration point is `ElectronBotBackend` — AI strategies interact with the robot exclusively through `backend.call(method, params)`, regardless of whether the backend connects to a local MuJoCo simulation or a cloud API for the real robot.

### Angle Unit Convention (Critical)

This is the single most important convention in the codebase and violating it causes 57.3× errors:

- **Python layer** (`mcp_bridge`, `actions`, `sensors`, `observation`): all joint angles use **degrees**
- **MuJoCo layer** (`data.qpos`, `data.ctrl`, `data.qvel`): always uses **radians** (regardless of MJCF `angle="degree"` setting)
- **Conversion boundaries**: read qpos → `np.degrees()`, write ctrl → `np.radians()`
- The only legal path to write control targets is `env.apply_joint_targets_deg()` (defined in `env.py`), which takes degrees and converts to radians internally

### Servo↔Joint Mapping (Single Source of Truth)

`env.py` defines the canonical mapping constants (`SERVO_CENTER`, `SERVO_RATIO`, `SERVO_DIRECTION`, `SERVO_LIMITS`, `SERVO_HOME`) with conversion functions (`servo_to_joint`, `joint_to_servo`, `servo_array_to_joint_array`). All other modules (`mcp_bridge.py`, `actions/`, `sensors/`) must import and reuse these — never redefine them.

Joint order (6D, project-wide): `[RP, RR, LP, LR, BODY, HEAD]`
- RP/RR = Right arm Pitch/Roll, LP/LR = Left arm Pitch/Roll
- Conversion formula: `joint_angle = (servo_angle - SERVO_CENTER[i]) * SERVO_RATIO[i] * SERVO_DIRECTION[i]`

### MCP Bridge Protocol

The bridge (`McpSimBridge` in `mcp_bridge.py`) implements the exact same JSON-RPC 2.0 protocol as the real ESP32 firmware (`xiaozhi-esp32 release v2.2.6`). It provides 12 tools: 8 aligned with real firmware (hand_action, body_turn, head_move, stop, get_status, set_trim, get_trims, battery.get_level) plus 4 simulation-only tools (servo_move, servo_sequences, home, get_ip) and 1 combo_action tool. The bridge uses linear interpolation matching the firmware's `MoveServos()` implementation with 10ms stepping.

### Simulation Environment (`ElectronBotEnv`)

- Gymnasium-compliant with `reset()`, `step(action)`, `render()`, `close()`
- Control frequency: 50Hz (`dt=0.02s`), 10 physics substeps per control step
- Action space: 6D joint angle increments, ±5°/step, degrees
- Two observation modes: `"full"` (joint positions, velocities, end-effector positions, camera images, depth, segmentation, contact forces) for research; `"realistic"` (only commanded positions, moving flag, battery info) for Sim2Real training where the real robot has no encoders or camera
- Domain randomization: 7-dimensional (friction ±20%, actuator gain ±10%, mass ±15%, servo deadband 2-5°, battery voltage 3.5-4.2V, corresponding gain scale, timing noise)
- Auto-reset on NaN detection or physics explosion (joint angles exceeding limits by 20° buffer)
- Rendering: EGL preferred, auto-fallback to OSMesa; camera defaults `distance=0.25, azimuth=145, elevation=-25, lookat=[0,0,0.04]`

### MuJoCo Models (`assets/mjcf/`)

- `scene_tabletop.xml`: the main scene file. Includes `electronbot.xml` (the robot model), sets up a table with 4 legs, a ground plane, 4 interactive objects (red/blue cubes, green cylinder, yellow box) with free joints. **Critical rendering parameters**: `znear=0.001`, `zfar=100.0`, headlight `ambient="0.3 0.3 0.35" diffuse="0.6 0.6 0.65"`. Model units are meters (robot is ~70mm tall, ~24mm wide, using mm-scale values in meters).
- `electronbot.xml`: the robot body definition with box/cylinder/capsule geoms (FreeCAD STL meshes had coordinate errors so were replaced).
- Robot joints use position actuators with `ctrlrange="-1.57 1.57"` (±90°) and `forcerange="-0.166 0.166"` (matching SG90 servo specs at 0.166 N·m ≈ 1.7 kg·cm).

### AI Training Pipeline

Tasks are defined as `BaseTask` subclasses (`src/electronbot_ai/tasks/`). Each task implements `reset()`, `get_observation()`, `compute_reward()`, `is_success()`, and `get_demo_action()`. Tasks bind to an `ElectronBotEnv` via `task.bind(env)`. For RL training, tasks are wrapped in `TaskWrapper` (`parallel_env.py`) to provide flattened observations for Stable-Baselines3, then run via `SubprocVecEnv` for multi-process parallelism (default 64 envs).

### Asset Pipeline

`scripts/` contains build and diagnostic tools:
- `build_fc_mjcf.py`: convert FreeCAD STL → MuJoCo MJCF
- `split_arm_mesh.py`: separate arm meshes for independent joint control
- `validate_model.py`: validate MJCF model structure
- `benchmark.py`: FPS performance benchmark
- `calc_inertia.py`: compute inertial parameters from mesh geometry
- `setup_env.sh`: one-click environment setup
- Many `diagnose_render*.py` and `render_candidates*.py` scripts for rendering debugging

### Dev Notes Convention

After solving a technical problem or fixing a bug, create a dev note at `docs/notes/dev-note-YYYY-MM-DD-{short-description}.md` following the template at `docs/notes/TEMPLATE_开发笔记.md`. Include background, root cause, solution, impact scope, and lessons learned. For visual changes, save before/after screenshots to `docs/notes/screenshots/` using the EGL renderer.
