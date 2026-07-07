"""域随机化工具模块。

对齐 Phase 2 §3 Step 4 + §7.1.1 域随机化参数结构。
独立于 ElectronBotEnv, 提供可复用的域随机化函数。
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, asdict
from typing import Optional

import numpy as np

logger = logging.getLogger("electronbot_sim.domain_randomizer")


@dataclass
class DomainRandomizationParams:
    """域随机化参数容器 (对齐 Phase 2 §7.1.1)。

    字段:
        friction_range: 摩擦系数缩放范围, 默认 (0.8, 1.2) — 舵机齿轮箱润滑差异
        gain_range: 执行器增益缩放范围, 默认 (0.9, 1.1) — 舵机个体差异+电池波动
        mass_range: 物体质量缩放范围, 默认 (0.85, 1.15) — 3D打印件重量公差
        servo_deadband: 舵机死区 (归一化 0~1), 默认 0.0
        battery_voltage: 电池电压 (V), 默认 4.2
        actuator_gain_scale: 电池电压派生增益, 默认 1.0 (=4.2/4.2)
    """
    friction_range: tuple = (0.8, 1.2)
    gain_range: tuple = (0.9, 1.1)
    mass_range: tuple = (0.85, 1.15)
    servo_deadband: float = 0.0
    battery_voltage: float = 4.2
    actuator_gain_scale: float = 1.0


class DomainRandomizer:
    """域随机化执行器。

    用法:
        randomizer = DomainRandomizer(params)
        dr_model = randomizer.randomize_model(model)
        dr_data = randomizer.randomize_data(data)
        snapshot = randomizer.get_snapshot()
    """

    def __init__(
        self,
        params: Optional[DomainRandomizationParams] = None,
        rng: Optional[np.random.Generator] = None,
    ):
        self.params = params or DomainRandomizationParams()
        self.rng = rng or np.random.default_rng()
        self._snapshot: dict = {}
        self._baseline_saved: bool = False

    # ================================================================
    #  核心接口
    # ================================================================

    def randomize_model(self, model) -> dict:
        """对 mjModel 执行域随机化, 返回参数快照。

        副作用: 修改 model.dof_damping, model.actuator_gainprm, model.body_mass。
        注意: mjModel 是共享的, 多线程需要加锁保护。
        """
        snapshot = {}

        # 1. 关节阻尼随机化 (摩擦 ±20%)
        if model.njnt > 0:
            scale = self.rng.uniform(*self.params.friction_range)
            for i in range(model.njnt):
                if i < len(model.dof_damping):
                    model.dof_damping[i] *= scale
            snapshot["friction_scale"] = float(scale)
            logger.debug("关节阻尼缩放: %.3f", scale)

        # 2. 执行器增益随机化 (±10%)
        if model.nu > 0:
            scale = self.rng.uniform(*self.params.gain_range)
            for i in range(model.nu):
                if i < model.actuator_gainprm.shape[0]:
                    model.actuator_gainprm[i, 0] *= scale
            snapshot["gain_scale"] = float(scale)
            logger.debug("执行器增益缩放: %.3f", scale)

        # 3. 物体质量随机化 (±15%)
        mass_scales = []
        if model.nbody > 0:
            for i in range(model.nbody):
                scale = self.rng.uniform(*self.params.mass_range)
                if i < len(model.body_mass):
                    model.body_mass[i] *= scale
                mass_scales.append(float(scale))
        snapshot["mass_scales"] = mass_scales[:10]  # 仅记录前 10 个

        # 4. 舵机死区 (2-5° 归一化 0.011-0.028)
        self.params.servo_deadband = self.rng.uniform(0.011, 0.028)
        snapshot["servo_deadband"] = float(self.params.servo_deadband)

        # 5. 电池电压 (3.5-4.2V) → 增益缩放 0.83-1.0
        self.params.battery_voltage = self.rng.uniform(3.5, 4.2)
        self.params.actuator_gain_scale = self.params.battery_voltage / 4.2
        snapshot["battery_voltage"] = float(self.params.battery_voltage)
        snapshot["actuator_gain_scale"] = float(self.params.actuator_gain_scale)

        # 电池增益叠加到执行器
        for i in range(model.nu):
            if i < model.actuator_gainprm.shape[0]:
                model.actuator_gainprm[i, 0] *= self.params.actuator_gain_scale

        self._snapshot = snapshot
        return snapshot

    def randomize_data(self, data, qpos_noise: float = 0.0,
                       qvel_noise: float = 0.0) -> dict:
        """对 mjData 执行状态随机化, 返回噪声快照。

        参数:
            data: mjData 实例。
            qpos_noise: 关节角度噪声标准差 (度), 0 表示不添加噪声。
            qvel_noise: 关节速度噪声标准差 (度/秒), 0 表示不添加噪声。
        """
        noise_snapshot: dict = {"qpos_noise": [], "qvel_noise": []}

        if qpos_noise > 0:
            noise = self.rng.normal(0, np.radians(qpos_noise), size=data.nq)
            data.qpos[:] += noise
            noise_snapshot["qpos_noise"] = noise.tolist()

        if qvel_noise > 0:
            noise = self.rng.normal(0, np.radians(qvel_noise), size=data.nv)
            data.qvel[:] += noise
            noise_snapshot["qvel_noise"] = noise.tolist()

        return noise_snapshot

    def randomize_object_positions(self, scene_body_ids: list[int],
                                   model, data,
                                   xy_range: tuple = (-0.03, 0.03),
                                   z_height: float = 47.0) -> None:
        """随机化桌面物体的位置。

        参数:
            scene_body_ids: 桌面物体 body 的 ID 列表。
            model, data: MuJoCo 模型/数据。
            xy_range: x/y 随机范围 (mm 单位)。
            z_height: 物体放置高度 (mm 单位)。
        """
        for bid in scene_body_ids:
            # 获取 free joint 的 qpos 地址
            jntadr = model.jnt_qposadr[bid] if bid < model.njnt else -1
            if jntadr < 0:
                continue

            # 位置: [x, y, z] 随机
            x = self.rng.uniform(*xy_range)
            y = self.rng.uniform(*xy_range)
            data.qpos[jntadr:jntadr + 3] = [x, y, z_height]

            # 姿态: 随机绕 z 轴旋转
            angle = self.rng.uniform(0, 2 * np.pi)
            data.qpos[jntadr + 3:jntadr + 7] = [
                np.cos(angle / 2), 0, 0, np.sin(angle / 2)
            ]

            logger.debug("物体 %d 位置: (%.4f, %.4f, %.2f)", bid, x, y, z_height)

    # ================================================================
    #  工具方法
    # ================================================================

    def get_snapshot(self) -> dict:
        """返回最近一次域随机化的参数快照, 用于日志与可复现。"""
        return dict(self._snapshot)

    def reset(self, rng_seed: Optional[int] = None) -> None:
        """重置 RNG 状态 (用于可复现性)。"""
        self.rng = np.random.default_rng(rng_seed)
        self._snapshot = {}

    def compute_deadband_mask(self, delta_action: np.ndarray) -> np.ndarray:
        """根据伺服死区, 计算哪些维度的动作为零的有效 mask。

        返回: bool 数组, True=动作有效, False=被死区吞噬。
        """
        deadband_deg = self.params.servo_deadband * 180.0
        return np.abs(delta_action) >= deadband_deg

    @staticmethod
    def progressive_schedule(step: int, total_steps: int, start_range: float,
                            end_range: float) -> float:
        """渐进式域随机化调度: 从窄范围逐步扩到宽范围。

        参数:
            step: 当前步数。
            total_steps: 总步数。
            start_range: 起始范围 (如 0.05 = 5%)。
            end_range: 终止范围 (如 0.20 = 20%)。

        返回: 当前步的缩放范围。
        """
        progress = min(1.0, step / total_steps)
        return start_range + (end_range - start_range) * progress

    def to_dict(self) -> dict:
        """序列化参数为字典。"""
        return asdict(self.params)
