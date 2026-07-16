"""ElectronBot Gymnasium 强化学习环境。

对齐 docs/tasks/02-MuJoCo-Env 详细设计说明书。
关键参数:
  - 控制频率 50Hz (dt=0.02s), 10 子步 × 0.002s
  - 6 关节增量式动作空间 [-5, 5] 度/步
  - 观测模式: full (仿真研究) / realistic (Sim2Real 对齐)
  - Home 姿态 qpos=[0,-45,0,-45,0,0] 度 (对应舵机 [180,180,0,0,90,90])

═══════════════════════════════════════════════════════════════════
  角度单位约定 (全项目统一, 嵌入式固件工程师强制规范)
═══════════════════════════════════════════════════════════════════
  1. Python 层 (mcp_bridge / actions / sensors / observation):
     所有关节角度一律使用【度数 °】运算, 与真机固件舵机角度一致
  2. MuJoCo 层 (data.qpos / data.ctrl / data.qvel):
     始终使用【弧度 rad】 (即使 MJCF 设了 angle="degree", 运行时仍为弧度)
  3. 转换边界:
     - 读 qpos → np.degrees() 转度数 (见 _get_joint_angles_deg)
     - 写 ctrl → np.radians() 转弧度 (见 apply_joint_targets_deg)
  4. 严禁任何模块直接写度数到 data.ctrl (会导致 57.3 倍偏差)
═══════════════════════════════════════════════════════════════════

  舵机↔关节映射 (单一数据源, 对齐真机固件 movements.cc + CAD)
  ─────────────────────────────────────────────────────────────────
  关节顺序: [RP, RR, LP, LR, BODY, HEAD]
  转换公式: joint_angle = (servo_angle - SERVO_CENTER[i]) * SERVO_RATIO[i] * SERVO_DIRECTION[i]
  反向公式: servo_angle = joint_angle / (SERVO_RATIO[i] * SERVO_DIRECTION[i]) + SERVO_CENTER[i]

  McpSimBridge / ElectronBotActions 必须从此处导入这些常量,
  禁止在多处重复定义 (避免数据源不一致).
"""
from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import gymnasium as gym
import numpy as np

logger = logging.getLogger("electronbot_sim.env")

# ═══════════════════════════════════════════════════════════════════
#  舵机↔关节映射常量 (全项目单一数据源, 对齐真机固件)
# ═══════════════════════════════════════════════════════════════════
# 关节顺序: [RP=右臂Pitch, RR=右臂Roll, LP=左臂Pitch, LR=左臂Roll, BODY=身体, HEAD=头部]
# 对齐: xiaozhi-esp32/main/boards/electron-bot/config.h + movements.cc

# 舵机中心角度 (度), 对齐固件 servo_initial / center
#   RP=180, RR=140 (注意: 安全中心与初始位不同), LP=0, LR=40, BODY=90, HEAD=90
SERVO_CENTER = np.array([180.0, 140.0, 0.0, 40.0, 90.0, 90.0], dtype=np.float32)

# 舵机→机械关节映射比 (固件安全范围 / CAD机械范围)
#   HEAD:  30°/60° = 2.0,  BODY: 120°/180° = 1.5
#   RP/LP: 180°/180° = 1.0,  RR/LR: 80°/90° ≈ 1.125
SERVO_RATIO = np.array([1.0, 1.125, 1.0, 1.125, 1.5, 2.0], dtype=np.float32)

# 方向 (反向=固件舵机增大时机械关节角度减小)
#   RP/RR 反向 (-1), LP/LR/BODY/HEAD 正向 (+1)
SERVO_DIRECTION = np.array([-1.0, -1.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float32)

# 舵机安全范围 (度), 对齐固件 ClampServoTarget()
#   索引顺序与 SERVO_CENTER 一致: [RP, RR, LP, LR, BODY, HEAD]
SERVO_LIMITS = (
    (0, 180),    # 0: right_pitch
    (100, 180),  # 1: right_roll
    (0, 180),    # 2: left_pitch
    (0, 80),     # 3: left_roll
    (30, 150),   # 4: body
    (75, 105),   # 5: head
)

# 舵机 Home 位置 (度), 对齐固件 servo_initial = [180,180,0,0,90,90]
SERVO_HOME = np.array([180.0, 180.0, 0.0, 0.0, 90.0, 90.0], dtype=np.float32)

# 舵机简称 → 索引映射 (MCP 工具 servo_type 参数取值)
SERVO_NAME_TO_INDEX = {
    "right_pitch": 0, "rp": 0,
    "right_roll":  1, "rr": 1,
    "left_pitch":  2, "lp": 2,
    "left_roll":   3, "lr": 3,
    "body":        4, "b":  4,
    "head":        5, "h":  5,
}

# ─── 派生: 机械关节限位 (度), 由 SERVO_LIMITS + 映射比推算 ───
# 关节顺序 [RP, RR, LP, LR, BODY, HEAD]
JOINT_MIN = np.array([-90.0, -45.0, -90.0, -45.0, -90.0, -30.0], dtype=np.float32)
JOINT_MAX = np.array([90.0, 45.0, 90.0, 45.0, 90.0, 30.0], dtype=np.float32)
# Home 姿态 (度) — 由 SERVO_HOME 经映射转换得到: [0, -45, 0, -45, 0, 0]
HOME_QPOS = np.array([0.0, -45.0, 0.0, -45.0, 0.0, 0.0], dtype=np.float32)


# ═══════════════════════════════════════════════════════════════════
#  舵机↔关节转换函数 (供 McpSimBridge / Actions 复用, 单一数据源)
# ═══════════════════════════════════════════════════════════════════
def servo_to_joint(servo_index: int, servo_angle: float) -> float:
    """舵机角度 (度) → 机械关节角度 (度).

    对齐固件: joint = (servo - center) * ratio * direction
    """
    return float(
        (servo_angle - SERVO_CENTER[servo_index])
        * SERVO_RATIO[servo_index]
        * SERVO_DIRECTION[servo_index]
    )


def joint_to_servo(servo_index: int, joint_angle: float) -> float:
    """机械关节角度 (度) → 舵机角度 (度).

    对齐固件: servo = joint / (ratio * direction) + center
    """
    return float(
        joint_angle / (SERVO_RATIO[servo_index] * SERVO_DIRECTION[servo_index])
        + SERVO_CENTER[servo_index]
    )


def servo_array_to_joint_array(servo_angles: np.ndarray) -> np.ndarray:
    """6 维舵机角度 → 6 维机械关节角度 (批量转换)."""
    return (servo_angles - SERVO_CENTER) * SERVO_RATIO * SERVO_DIRECTION


def joint_array_to_servo_array(joint_angles: np.ndarray) -> np.ndarray:
    """6 维机械关节角度 → 6 维舵机角度 (批量转换)."""
    return joint_angles / (SERVO_RATIO * SERVO_DIRECTION) + SERVO_CENTER


def clamp_servo_target(servo_index: int, angle: float) -> int:
    """安全角度裁剪, 对齐固件 ClampServoTarget().

    返回 int (固件舵机角度均为整数 PWM 值).
    """
    lo, hi = SERVO_LIMITS[servo_index]
    clamped = max(lo, min(hi, int(angle)))
    if clamped != int(angle):
        logger.warning(
            "舵机 %d 角度 %d 超出安全范围 [%d, %d], 已裁剪到 %d",
            servo_index, int(angle), lo, hi, clamped,
        )
    return clamped


# 仿真参数
DEFAULT_DT = 0.02  # 50Hz
DEFAULT_NSUBSTEPS = 10


@dataclass
class DomainRandomizationParams:
    """域随机化参数容器 (对齐 Phase 2 §7.1.1)"""
    friction_range: tuple = (0.8, 1.2)
    gain_range: tuple = (0.9, 1.1)
    mass_range: tuple = (0.85, 1.15)
    servo_deadband: float = 0.0  # 归一化死区 (0~1), 对应约 0-5°
    battery_voltage: float = 4.2  # V
    actuator_gain_scale: float = 1.0  # 由 battery_voltage/4.2 派生


class ElectronBotEnv(gym.Env):
    """ElectronBot Gymnasium 强化学习环境。

    Layer 2 物理仿真封装, 继承 gymnasium.Env, 严格遵循标准 API,
    可被 Stable-Baselines3 / CleanRL / Ray RLLib / gym.vector 直接使用。

    参数:
        render_mode: None / "human" / "rgb_array"
        obs_mode: "full" (仿真研究) / "realistic" (Sim2Real 对齐, 仅真机可获取数据)
        **kwargs: 透传域随机化与物理参数
            friction_range: 摩擦系数缩放范围, 默认 (0.8, 1.2)
            gain_range: 执行器增益缩放范围, 默认 (0.9, 1.1)
            mass_range: 物体质量缩放范围, 默认 (0.85, 1.15)
            servo_deadband: 舵机死区 (归一化), 默认 0.0
            battery_voltage: 电池电压 (V), 默认 4.2
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(self, render_mode: Optional[str] = None, **kwargs: Any):
        super().__init__()

        # ─── 动作空间: 6 关节角度增量 (度/步) ───
        self.action_space = gym.spaces.Box(
            low=np.array([-5.0] * 6, dtype=np.float32),
            high=np.array([5.0] * 6, dtype=np.float32),
            dtype=np.float32,
        )

        # ─── 观测空间 (对齐 §6.2.2) ───
        self.obs_mode = kwargs.get("obs_mode", "realistic")
        self.observation_space = self._build_observation_space(self.obs_mode, gym)

        # ─── 域随机化参数 ───
        self.dr_params = DomainRandomizationParams(
            friction_range=kwargs.get("friction_range", (0.8, 1.2)),
            gain_range=kwargs.get("gain_range", (0.9, 1.1)),
            mass_range=kwargs.get("mass_range", (0.85, 1.15)),
            servo_deadband=kwargs.get("servo_deadband", 0.0),
            battery_voltage=kwargs.get("battery_voltage", 4.2),
        )
        self.enable_domain_rand = kwargs.get("enable_domain_rand", True)
        self.enable_perf_log = kwargs.get("enable_perf_log", False)

        # ─── 仿真参数 (可由环境变量覆写) ───
        self.dt = float(os.environ.get("ELECTRONBOT_SIM_DT", DEFAULT_DT))
        self.nsubsteps = int(os.environ.get("ELECTRONBOT_SIM_NSUBSTEPS", DEFAULT_NSUBSTEPS))
        self.max_episode_steps = kwargs.get("max_episode_steps", 1000)

        # ─── MuJoCo 加载 ───
        assets_path = os.environ.get(
            "ELECTRONBOT_SIM_ASSETS_PATH",
            str(Path(__file__).parent.parent.parent / "assets" / "mjcf"),
        )
        model_file = kwargs.get("model_file", "scene_tabletop.xml")
        model_path = Path(assets_path) / model_file
        if not model_path.exists():
            # 回退到 mesh scene
            model_path = Path(assets_path) / "scene_mesh.xml"

        import mujoco
        self._mujoco = mujoco  # 保存引用, 供 actions 等模块复用
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)

        # 可选相机距离 (None=自动 extent*1.8)
        self._camera_distance = kwargs.get("camera_distance", None)
        self._joint_min = JOINT_MIN
        self._joint_max = JOINT_MAX

        # 关节/执行器 name → id 映射 (对齐 MJCF 命名)
        # 兼容 STEP 转换 MJCF: body 关节可能叫 "body_joint" 而非 "joint_body"
        self._joint_ids = {}
        self._actuator_ids = {}
        _joint_candidates = {
            "joint_rp": ["joint_rp"],
            "joint_rr": ["joint_rr"],
            "joint_lp": ["joint_lp"],
            "joint_lr": ["joint_lr"],
            "joint_body": ["joint_body", "body_joint"],
            "joint_head": ["joint_head"],
        }
        for canon, candidates in _joint_candidates.items():
            for name in candidates:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if jid >= 0:
                    self._joint_ids[canon] = jid
                    break
        for name in ["act_rp", "act_rr", "act_lp", "act_lr",
                     "act_body", "act_head"]:
            aid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            if aid >= 0:
                self._actuator_ids[name] = aid

        # 关节 qpos 索引 (按 [RP, RR, LP, LR, BODY, HEAD] 顺序)
        # 兼容 STEP 转换 MJCF: body 关节可能叫 "body_joint" 而非 "joint_body"
        _joint_name_map = {
            "joint_rp": ["joint_rp"],
            "joint_rr": ["joint_rr"],
            "joint_lp": ["joint_lp"],
            "joint_lr": ["joint_lr"],
            "joint_body": ["joint_body", "body_joint"],
            "joint_head": ["joint_head"],
        }
        def _resolve(canon):
            for n in _joint_name_map[canon]:
                if n in self._joint_ids:
                    return self._joint_ids[n]
            return None
        self._qpos_addr = np.array([
            self.model.jnt_qposadr[_resolve(n)]
            for n in ["joint_rp", "joint_rr", "joint_lp", "joint_lr",
                      "joint_body", "joint_head"]
            if _resolve(n) is not None
        ], dtype=int)
        self._qvel_addr = np.array([
            self.model.jnt_dofadr[_resolve(n)]
            for n in ["joint_rp", "joint_rr", "joint_lp", "joint_lr",
                      "joint_body", "joint_head"]
            if _resolve(n) is not None
        ], dtype=int)
        # 执行器顺序 (与 action 顺序一致)
        self._act_order = ["act_rp", "act_rr", "act_lp", "act_lr",
                           "act_body", "act_head"]
        self._act_ids_sorted = [self._actuator_ids[n] for n in self._act_order
                                if n in self._actuator_ids]

        # ─── 渲染器 ───
        self.render_mode = render_mode
        self.renderer = None
        self._viewer = None
        if render_mode == "rgb_array":
            self._init_renderer()
        elif render_mode == "human":
            # human 模式延迟到首次 render 时初始化 (避免无头环境崩溃)
            pass

        # ─── 状态追踪 ───
        self.step_count = 0
        self._last_log_time = time.time()
        self._last_commanded = HOME_QPOS.copy()
        self._is_moving = False
        self.nan_reset_count = 0
        self.explosion_reset_count = 0

        # ─── 关闭域随机化 (基线对照实验) ───
        if os.environ.get("ELECTRONBOT_SIM_DISABLE_DR") == "1":
            self.enable_domain_rand = False

        logger.info(
            "ElectronBotEnv 初始化: model=%s, obs_mode=%s, dt=%.3f, nsub=%d",
            model_path.name, self.obs_mode, self.dt, self.nsubsteps,
        )

    # ================================================================
    #  Gymnasium API
    # ================================================================

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        """重置环境到 home 姿态, 触发域随机化。

        返回: (obs, info)
            info 含 domain_randomization 参数快照、object_positions、seed
        """
        # 通过父类标准机制初始化 np_random
        super().reset(seed=seed)

        # 1. 清零 qfrc_applied (关键, 防残留, 对齐 §8.2.3)
        self.data.qfrc_applied[:] = 0
        # 2. mj_resetData
        self._mujoco.mj_resetData(self.model, self.data)
        # 3. 设置 home 姿态 (度 → 弧度)
        home_rad = np.radians(HOME_QPOS)
        for i, addr in enumerate(self._qpos_addr):
            self.data.qpos[addr] = home_rad[i]
        # 4. 域随机化
        dr_snapshot = {}
        if self.enable_domain_rand:
            dr_snapshot = self.randomize_domain()
        # 5. mj_forward 刷新派生量
        self._mujoco.mj_forward(self.model, self.data)
        # 6. 重置计数器
        self.step_count = 0
        self._last_commanded = HOME_QPOS.copy()
        self._is_moving = False
        # 7. 构建观测
        obs = self._build_observation()
        info = {
            "domain_randomization": dr_snapshot,
            "object_positions": np.array([], dtype=np.float32),
            "seed": seed if seed is not None else -1,
        }
        return obs, info

    def step(self, action: np.ndarray):
        """执行一个 50Hz 控制步。

        参数:
            action: (6,) 关节角度增量 (度), 范围 [-5, 5]

        返回: (obs, reward, terminated, truncated, info)
        """
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        if action.shape[0] != 6:
            logger.warning("E006 动作维度不匹配: shape=%s, 期望 (6,)", action.shape)
            action = np.zeros(6, dtype=np.float32)

        # 1. 增量 → 绝对角度 (裁剪到限位)
        current_angles = self._get_joint_angles_deg()
        target_angles = np.clip(
            current_angles + action, self._joint_min, self._joint_max
        )

        # 2. 应用舵机死区 (仿真真机 SG90 deadband)
        if self.dr_params.servo_deadband > 0:
            delta = np.abs(target_angles - current_angles)
            deadband_deg = self.dr_params.servo_deadband * 180.0
            small_move = delta < deadband_deg
            target_angles = np.where(small_move, current_angles, target_angles)

        # 3. 设置执行器目标 (度 → 弧度)
        target_rad = np.radians(target_angles)
        for i, aid in enumerate(self._act_ids_sorted):
            self.data.ctrl[aid] = target_rad[i]
        self._last_commanded = target_angles.copy()
        self._is_moving = bool(np.any(np.abs(action) > 0.1))

        # 4. 推进物理仿真 (nsubsteps 子步)
        try:
            substeps = max(1, int(self.dt / self.model.opt.timestep))
            for _ in range(substeps):
                self._mujoco.mj_step(self.model, self.data)
        except Exception as e:
            logger.error("mj_step 异常: %s", e)
            self.explosion_reset_count += 1
            obs, info = self.reset()
            info["auto_reset_reason"] = "E002_explosion"
            return obs, 0.0, True, False, info

        self.step_count += 1

        # 5. 状态合法性检测 (NaN / 爆炸)
        if not self._check_state_validity():
            self.nan_reset_count += 1
            logger.warning("E001 NaN 检测: qpos/qvel 含 NaN, 触发自动 reset")
            obs, info = self.reset()
            info["auto_reset_reason"] = "E001_NaN"
            return obs, 0.0, True, False, info

        # 6. 构建观测
        obs = self._build_observation()

        # 7. 渲染 (可选)
        if self.render_mode == "human":
            self._render_human()
        elif self.render_mode == "rgb_array":
            self._render_rgb()

        # 8. 性能日志 (每 1000 步)
        if self.enable_perf_log and self.step_count % 1000 == 0:
            elapsed = time.time() - self._last_log_time
            fps = 1000 / elapsed if elapsed > 0 else 0
            logger.info("仿真步进 fps=%.0f Hz (步数=%d)", fps, self.step_count)
            self._last_log_time = time.time()

        # 9. 返回 (reward=0, terminated=False, 由 task wrapper 覆盖)
        info = self._build_step_info(current_angles, target_angles)
        truncated = self.step_count >= self.max_episode_steps
        return obs, 0.0, False, truncated, info

    def _compute_scene_camera(self):
        """根据 model.stat.center / extent 动态计算相机参数。"""
        import mujoco
        cam = mujoco.MjvCamera()
        cam.lookat[:] = self.model.stat.center
        extent = self.model.stat.extent
        cam.distance = float(self._camera_distance) if self._camera_distance else 0.3
        cam.azimuth = 90 if self._camera_distance else 135
        cam.elevation = -10
        # 让 lookat 对准模型底部，确保看到地面接触
        if not self._camera_distance:
            cam.lookat[2] = 0.05
        return cam

    def render(self):
        """渲染当前帧。rgb_array 返回 (480,480,3) uint8; human 返回 None。"""
        if self.render_mode == "rgb_array":
            return self._render_rgb()
        elif self.render_mode == "human":
            self._render_human()
        return None

    def close(self):
        """释放渲染器, 关闭窗口, 清理 MuJoCo 上下文。"""
        if self.renderer is not None:
            self.renderer = None
        if self._viewer is not None:
            try:
                self._viewer.close()
            except Exception:
                pass
            self._viewer = None
        logger.info("ElectronBotEnv 已关闭")

    # ================================================================
    #  域随机化 / 物理参数
    # ================================================================

    def randomize_domain(self) -> dict:
        """域随机化入口, 返回本次随机化参数快照 (对齐 §3 Step 4)。"""
        rng = self._np_random
        snapshot = {}

        # 关节摩擦 (dof_damping) ±20%
        if self.model.njnt > 0:
            scale = rng.uniform(*self.dr_params.friction_range)
            for i in range(self.model.njnt):
                if i < len(self.model.dof_damping):
                    base = self.model.dof_damping[i] / scale if scale != 0 else self.model.dof_damping[i]
                    self.model.dof_damping[i] = base * scale
            snapshot["friction_scale"] = float(scale)

        # 执行器增益 ±10%
        if self.model.nu > 0:
            scale = rng.uniform(*self.dr_params.gain_range)
            for i in range(self.model.nu):
                if i < self.model.actuator_gainprm.shape[0]:
                    self.model.actuator_gainprm[i, 0] *= scale
            snapshot["gain_scale"] = float(scale)

        # 物体质量 ±15%
        if self.model.nbody > 0:
            for i in range(self.model.nbody):
                scale = rng.uniform(*self.dr_params.mass_range)
                if i < len(self.model.body_mass):
                    self.model.body_mass[i] *= scale

        # 舵机死区 2-5° (归一化 0.011-0.028)
        self.dr_params.servo_deadband = rng.uniform(0.011, 0.028)
        snapshot["servo_deadband"] = float(self.dr_params.servo_deadband)

        # 电池电压 3.5-4.2V
        self.dr_params.battery_voltage = rng.uniform(3.5, 4.2)
        self.dr_params.actuator_gain_scale = self.dr_params.battery_voltage / 4.2
        snapshot["battery_voltage"] = float(self.dr_params.battery_voltage)
        snapshot["actuator_gain_scale"] = float(self.dr_params.actuator_gain_scale)

        # 电池增益缩放叠加到执行器
        for i in range(self.model.nu):
            if i < self.model.actuator_gainprm.shape[0]:
                self.model.actuator_gainprm[i, 0] *= self.dr_params.actuator_gain_scale

        logger.info(
            "域随机化: friction=%.2f, gain=%.2f, voltage=%.2fV, deadband=%.4f",
            snapshot.get("friction_scale", 1.0),
            snapshot.get("gain_scale", 1.0),
            self.dr_params.battery_voltage,
            self.dr_params.servo_deadband,
        )
        return snapshot

    def set_physics_params(self, params: dict) -> None:
        """运行时热更新物理参数 (摩擦/增益/阻尼等), 无需重建环境。"""
        if "dof_damping" in params:
            for i, v in enumerate(params["dof_damping"]):
                if i < len(self.model.dof_damping):
                    self.model.dof_damping[i] = v
        if "actuator_gainprm" in params:
            for i, v in enumerate(params["actuator_gainprm"]):
                if i < self.model.actuator_gainprm.shape[0]:
                    self.model.actuator_gainprm[i, 0] = v
        logger.info("物理参数已热更新: %s", list(params.keys()))

    def get_state(self) -> dict:
        """序列化当前 qpos/qvel/time, 用于检查点保存。"""
        return {
            "qpos": self.data.qpos.copy(),
            "qvel": self.data.qvel.copy(),
            "time": float(self.data.time),
            "step_count": self.step_count,
        }

    def set_state(self, state: dict) -> None:
        """恢复检查点状态, 用于回放与调试。"""
        self.data.qpos[:] = state["qpos"]
        self.data.qvel[:] = state["qvel"]
        self.data.time = state.get("time", 0.0)
        self.step_count = state.get("step_count", 0)
        self._mujoco.mj_forward(self.model, self.data)

    # ================================================================
    #  内部辅助
    # ================================================================

    def _build_observation_space(self, obs_mode: str, gym):
        """构建观测空间 (对齐 §6.2.2)。"""
        if obs_mode == "full":
            return gym.spaces.Dict({
                "joint_pos": gym.spaces.Box(
                    low=JOINT_MIN, high=JOINT_MAX, dtype=np.float32),
                "joint_vel": gym.spaces.Box(
                    low=-360.0, high=360.0, shape=(6,), dtype=np.float32),
                "ee_left_pos": gym.spaces.Box(
                    low=-1.0, high=1.0, shape=(3,), dtype=np.float32),
                "ee_right_pos": gym.spaces.Box(
                    low=-1.0, high=1.0, shape=(3,), dtype=np.float32),
                "head_angle": gym.spaces.Box(
                    low=-30.0, high=30.0, shape=(1,), dtype=np.float32),
            })
        else:  # realistic
            return gym.spaces.Dict({
                "commanded_joint_pos": gym.spaces.Box(
                    low=JOINT_MIN, high=JOINT_MAX, dtype=np.float32),
                "is_moving": gym.spaces.Box(
                    low=0, high=1, shape=(1,), dtype=np.float32),
                "battery_voltage": gym.spaces.Box(
                    low=3.0, high=4.2, shape=(1,), dtype=np.float32),
                "battery_percent": gym.spaces.Box(
                    low=0, high=100, shape=(1,), dtype=np.float32),
            })

    def _get_joint_angles_deg(self) -> np.ndarray:
        """读取 6 关节当前角度 (度)。"""
        rad = np.array([self.data.qpos[a] for a in self._qpos_addr], dtype=np.float64)
        return np.degrees(rad).astype(np.float32)

    def _get_joint_velocities_deg(self) -> np.ndarray:
        """读取 6 关节角速度 (度/秒)。⚠️ 真机无编码器, realistic 模式不可用。"""
        rad = np.array([self.data.qvel[a] for a in self._qvel_addr], dtype=np.float64)
        return np.degrees(rad).astype(np.float32)

    def _get_ee_position(self, body_name: str) -> np.ndarray:
        """获取末端执行器世界坐标 (m)。"""
        bid = self._mujoco.mj_name2id(
            self.model, self._mujoco.mjtObj.mjOBJ_BODY, body_name
        )
        if bid < 0:
            return np.zeros(3, dtype=np.float32)
        return self.data.xpos[bid].astype(np.float32).copy()

    def _build_observation(self) -> dict:
        """构建观测字典 (full / realistic)。"""
        if self.obs_mode == "full":
            return {
                "joint_pos": self._get_joint_angles_deg(),
                "joint_vel": self._get_joint_velocities_deg(),
                "ee_left_pos": self._get_ee_position("left_hand"),
                "ee_right_pos": self._get_ee_position("right_hand"),
                "head_angle": self._get_joint_angles_deg()[5:6],
            }
        else:  # realistic
            battery_v = self.dr_params.battery_voltage
            battery_pct = max(0.0, min(100.0, (battery_v - 3.0) / 1.2 * 100.0))
            return {
                "commanded_joint_pos": self._last_commanded.copy(),
                "is_moving": np.array([1.0 if self._is_moving else 0.0], dtype=np.float32),
                "battery_voltage": np.array([battery_v], dtype=np.float32),
                "battery_percent": np.array([battery_pct], dtype=np.float32),
            }

    def _build_step_info(self, current: np.ndarray, target: np.ndarray) -> dict:
        info = {
            "joint_limits_hit": np.array([
                bool(self._joint_min[i] <= target[i] <= self._joint_max[i])
                for i in range(6)
            ]),
            "ctrl_cost": float(np.sum(np.abs(target - current) * 0.01)),
        }
        if self.enable_perf_log:
            info["fps"] = 1.0 / max(1e-6, self.dt)
        return info

    def _check_state_validity(self) -> bool:
        """检测仿真状态是否合法 (NaN), 返回 False 表示需要 reset。"""
        if np.any(np.isnan(self.data.qpos)) or np.any(np.isnan(self.data.qvel)):
            return False
        # 仿真爆炸检测: 关节超出物理限位 ±20° 缓冲区
        angles_deg = self._get_joint_angles_deg()
        safe_min = self._joint_min - 20.0
        safe_max = self._joint_max + 20.0
        if np.any(angles_deg < safe_min) or np.any(angles_deg > safe_max):
            logger.error(
                "E002 仿真爆炸: 关节角度 %s 超出安全范围",
                np.round(angles_deg, 1).tolist(),
            )
            return False
        return True

    def _init_renderer(self):
        """初始化渲染器, 支持 EGL → OSMesa 自动回退 (对齐 §8.2.4)。"""
        try:
            os.environ["MUJOCO_GL"] = "egl"
            self.renderer = self._mujoco.Renderer(self.model, 480, 480)
            logger.debug("渲染器: EGL 初始化成功")
        except (RuntimeError, ImportError) as e:
            logger.warning("E004 EGL 渲染不可用 (%s), 回退到 OSMesa", e)
            os.environ["MUJOCO_GL"] = "osmesa"
            try:
                self.renderer = self._mujoco.Renderer(self.model, 480, 480)
                logger.debug("渲染器: OSMesa 初始化成功")
            except RuntimeError as e2:
                logger.error("OSMesa 也不可用, 禁用渲染: %s", e2)
                self.renderer = None
                self.render_mode = None

    def _render_rgb(self) -> Optional[np.ndarray]:
        if self.renderer is None:
            self._init_renderer()
        if self.renderer is None:
            return None
        # 头灯已在 scene_tabletop.xml 中设置, 此处仅微调
        cam = self._mujoco.MjvCamera()
        cam.lookat[:] = [0, 0, 0.04]       # 机器人中心 z≈40mm
        cam.distance = 0.25                 # 25cm 距离 (可看清整个桌面场景)
        cam.azimuth = 145                   # 四分之三视角
        cam.elevation = -25                 # 俯视25度
        self._mujoco.mjv_updateScene(
            self.model, self.data,
            self._mujoco.MjvOption(),
            self._mujoco.MjvPerturb(),
            cam,
            self._mujoco.mjtCatBit.mjCAT_ALL,
            self.renderer.scene,
        )
        return self.renderer.render()

    def _render_human(self):
        if self._viewer is None:
            try:
                import mujoco.viewer
                self._viewer = mujoco.viewer.launch_passive(self.model, self.data)
                if self._viewer is not None and hasattr(self, '_compute_scene_camera'):
                    cam = self._compute_scene_camera()
                    logger.info("setting camera: dist=%.0f az=%.0f el=%.0f lookat=(%.0f,%.0f,%.0f)",
                                cam.distance, cam.azimuth, cam.elevation,
                                cam.lookat[0], cam.lookat[1], cam.lookat[2])
                    self._viewer.cam.lookat[:] = cam.lookat
                    self._viewer.cam.distance = cam.distance
                    self._viewer.cam.azimuth = cam.azimuth
                    self._viewer.cam.elevation = cam.elevation
                    # 打印 arm geom 的世界位置, 确认有手臂
                    for gn in ['left_arm_geom', 'right_arm_geom']:
                        gid = mujoco.mj_name2id(self.model, 5, gn)
                        if gid >= 0:
                            pos = self.data.geom_xpos[gid]
                            mid = self.model.geom_dataid[gid]
                            vnum = self.model.mesh_vertnum[mid]
                            logger.info("  %s: pos=(%.0f,%.0f,%.0f) verts=%d", gn, pos[0], pos[1], pos[2], vnum)
            except Exception as e:
                logger.warning("human viewer 初始化失败: %s, 回退 rgb_array", e)
                self.render_mode = "rgb_array"
                self._init_renderer()
                return
        if self._viewer is not None:
            try:
                with self._viewer.lock():
                    self._viewer.sync()
            except Exception as e:
                logger.error("viewer.sync 异常: %s", e)

    # ================================================================
    #  便捷方法 (供 Task / Actions 模块复用)
    # ================================================================

    def get_joint_positions(self) -> np.ndarray:
        """6 关节当前角度 (度), 别名。"""
        return self._get_joint_angles_deg()

    def get_ee_position(self, body_name: str = "right_hand") -> np.ndarray:
        """末端执行器位置 (m), 别名。"""
        return self._get_ee_position(body_name)

    @property
    def joint_min(self) -> np.ndarray:
        return self._joint_min

    @property
    def joint_max(self) -> np.ndarray:
        return self._joint_max

    @property
    def np_random(self):
        if not hasattr(self, "_np_random"):
            self._np_random = np.random.default_rng()
        return self._np_random

    # ================================================================
    #  Bridge / Actions 专用接口 (统一角度单位入口, 严禁绕过)
    # ================================================================

    def apply_joint_targets_deg(self, joint_angles_deg: np.ndarray) -> None:
        """【Bridge/Actions 唯一合法的写 ctrl 入口】

        接收 6 维机械关节角度 (度), 内部转弧度写入 data.ctrl.
        严禁任何模块直接写度数到 data.ctrl (会导致 57.3 倍偏差).

        参数:
            joint_angles_deg: (6,) 机械关节角度 (度), 顺序 [RP,RR,LP,LR,BODY,HEAD]
        """
        angles = np.asarray(joint_angles_deg, dtype=np.float32).reshape(-1)
        if angles.shape[0] != 6:
            logger.warning("apply_joint_targets_deg: 维度不匹配 %s, 跳过", angles.shape)
            return
        # 安全裁剪到关节限位 (对齐固件 ClampServoTarget 的等价物)
        angles = np.clip(angles, self._joint_min, self._joint_max)
        # 度 → 弧度
        target_rad = np.radians(angles)
        for i, aid in enumerate(self._act_ids_sorted):
            self.data.ctrl[aid] = target_rad[i]
        self._last_commanded = angles.copy()

    def step_simulation(self, substeps: Optional[int] = None) -> bool:
        """【Bridge/Actions 推进物理仿真的唯一入口】

        推进 nsubsteps 个 MuJoCo 子步 (默认 self.nsubsteps).
        与 step() 不同, 本方法不构建观测/不计算奖励/不递增 step_count,
        仅供 Bridge 在执行多步插值时调用.

        返回: True 若仿真状态合法 (无 NaN/爆炸), False 表示需要 reset.
        """
        substeps = substeps or max(1, int(self.dt / self.model.opt.timestep))
        # human 模式: 用 viewer.lock() 上下文管理器串行化 mj_step + sync。
        # 注: MuJoCo 3.x viewer.lock 是上下文管理器函数, 不是 threading.Lock,
        # 必须用 with viewer.lock(): 而不能手动 .acquire()/.release()。
        if self.render_mode == "human" and self._viewer is not None:
            try:
                with self._viewer.lock():
                    for _ in range(substeps):
                        self._mujoco.mj_step(self.model, self.data)
                    self._viewer.sync()
            except Exception as e:
                logger.error("human step/sync 异常: %s", e)
                self.explosion_reset_count += 1
                return False
            return self._check_state_validity()
        # 非 human 模式: 仅推进物理
        try:
            for _ in range(substeps):
                self._mujoco.mj_step(self.model, self.data)
        except Exception as e:
            logger.error("mj_step 异常: %s", e)
            self.explosion_reset_count += 1
            return False
        return self._check_state_validity()

    def is_state_valid(self) -> bool:
        """公开的仿真状态合法性检测 (NaN / 爆炸)."""
        return self._check_state_validity()

    def get_commanded_joint_pos(self) -> np.ndarray:
        """获取最后指令的关节角度 (度), 供 realistic 观测模式使用."""
        return self._last_commanded.copy()

    def set_moving_state(self, is_moving: bool) -> None:
        """设置运动状态标志, 供 Actions 在插值开始/结束时调用."""
        self._is_moving = bool(is_moving)

    def get_battery_info(self) -> dict:
        """获取电池信息 (电压/百分比), 供 realistic 观测 + battery.get_level 工具使用.

        返回:
            {"voltage": float, "percent": float, "is_charging": False}
        """
        v = float(self.dr_params.battery_voltage)
        pct = max(0.0, min(100.0, (v - 3.0) / 1.2 * 100.0))
        return {"voltage": v, "percent": pct, "is_charging": False}

    def get_servo_angles(self) -> np.ndarray:
        """获取当前 6 舵机角度 (度), 由机械关节角度反推.

        供 get_trims / get_status 等 MCP 工具使用.
        """
        joint_angles = self._get_joint_angles_deg()
        return joint_array_to_servo_array(joint_angles)
