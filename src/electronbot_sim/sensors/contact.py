"""ContactSensor — 接触力传感器.

对齐 docs/tasks/05-Sensors-Observation 详细设计说明书 §5.

═══════════════════════════════════════════════════════════════════
  实现说明
═══════════════════════════════════════════════════════════════════
  通过 MuJoCo 的 contact 数组读取接触力, 用于:
  - 检测末端是否接触物体 (RL 奖励函数)
  - 检测是否发生碰撞 (安全约束)
  - 接触力大小 (任务完成判定)

  ⚠️ 真机无接触力传感器, 此数据仅仿真可用.
  ⚠️ 防穿透卡死: max_contacts_traverse = 1000
"""
from __future__ import annotations

import logging
from typing import Optional

import numpy as np

logger = logging.getLogger("electronbot_sim.sensors.contact")

# 防穿透卡死上限 (对齐设计文档 §6.1)
MAX_CONTACTS_TRAVERSE = 1000


class ContactSensor:
    """接触力传感器.

    参数:
        env:       ElectronBotEnv 实例
        body_name: 监测的 body 名称, 如 "left_hand" / "right_hand"
    """

    def __init__(self, env, body_name: str):
        self.env = env
        self.body_name = body_name
        mujoco = env._mujoco
        self._body_id = mujoco.mj_name2id(
            env.model, mujoco.mjtObj.mjOBJ_BODY, body_name
        )
        if self._body_id < 0:
            logger.warning("body '%s' 未找到, 接触传感器返回零", body_name)

    def is_in_contact(self, threshold: float = 0.01) -> bool:
        """检测是否发生接触 (总力 > 阈值 N).

        参数:
            threshold: 力阈值 (N), 默认 0.01 (对齐 contact_threshold)

        返回: True 若总接触力 > threshold
        """
        return self.get_total_contact_force() > threshold

    def get_total_contact_force(self) -> float:
        """获取合力大小 (N).

        遍历 MuJoCo contact 数组, 累加涉及本 body 的接触力.
        """
        if self._body_id < 0:
            return 0.0

        mujoco = self.env._mujoco
        data = self.env.data
        model = self.env.model
        total_force = 0.0

        # 遍历 contact 数组 (带上限保护)
        n_contact = min(data.ncon, MAX_CONTACTS_TRAVERSE)
        for i in range(n_contact):
            contact = data.contact[i]
            # 检查接触的两个 geom 是否属于本 body
            geom1_body = model.geom_bodyid[contact.geom1]
            geom2_body = model.geom_bodyid[contact.geom2]
            if geom1_body != self._body_id and geom2_body != self._body_id:
                continue

            # 计算接触力 (使用 mj_contactForce)
            force = np.zeros(6, dtype=np.float64)
            try:
                mujoco.mj_contactForce(model, data, i, force)
                # force[0:3] 为接触力 (N), force[3:6] 为扭矩 (N·m)
                # 取力的合力大小
                total_force += float(np.linalg.norm(force[0:3]))
            except Exception as e:
                logger.debug("接触力计算失败 (contact %d): %s", i, e)

        return total_force

    def get_contact_forces(self) -> np.ndarray:
        """获取所有接触点的力向量 (N).

        返回: (K, 3) float64, K 为接触点数, 每行为 [fx, fy, fz]
        """
        if self._body_id < 0:
            return np.zeros((0, 3), dtype=np.float64)

        mujoco = self.env._mujoco
        data = self.env.data
        model = self.env.model
        forces = []

        n_contact = min(data.ncon, MAX_CONTACTS_TRAVERSE)
        for i in range(n_contact):
            contact = data.contact[i]
            geom1_body = model.geom_bodyid[contact.geom1]
            geom2_body = model.geom_bodyid[contact.geom2]
            if geom1_body != self._body_id and geom2_body != self._body_id:
                continue

            force = np.zeros(6, dtype=np.float64)
            try:
                mujoco.mj_contactForce(model, data, i, force)
                forces.append(force[0:3].copy())
            except Exception:
                pass

        if not forces:
            return np.zeros((0, 3), dtype=np.float64)
        return np.array(forces, dtype=np.float64)
