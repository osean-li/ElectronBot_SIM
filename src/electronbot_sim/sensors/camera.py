"""CameraSensor — RGB + Depth + Segmentation 摄像头传感器.

对齐 docs/tasks/05-Sensors-Observation 详细设计说明书 §3.
对齐真机: GC9A01 240×240 圆形 LCD (注意: 真机 ElectronBot 无摄像头, 仅仿真)

═══════════════════════════════════════════════════════════════════
  关键参数 (对齐 GC9A01 硬件规格)
═══════════════════════════════════════════════════════════════════
  - 默认分辨率 240×240 (对齐 GC9A01 屏幕)
  - 默认 fovy=60°
  - fx = fy = 240 / 2 / tan(30°) ≈ 207.85
  - cx = cy = 120.0
  - 渲染后端: EGL (GPU) → OSMesa (CPU) 自动回退

  ⚠️ 真机约束: ElectronBot 无摄像头硬件, 视觉 VLA 不可部署到真机.
     obs_mode="realistic" 会移除 image/depth/segmentation 字段.
"""
from __future__ import annotations

import logging
import os
from typing import Dict, Optional, Tuple

import numpy as np

logger = logging.getLogger("electronbot_sim.sensors.camera")


class CameraSensor:
    """RGB + Depth + Segmentation 摄像头传感器.

    参数:
        env:    ElectronBotEnv 实例
        name:   摄像头名称 (对应 MJCF 中的 camera), 默认 "head_cam"
        width:  图像宽度, 默认 240 (对齐 GC9A01)
        height: 图像高度, 默认 240 (对齐 GC9A01)
    """

    def __init__(self, env, name: str = "head_cam",
                 width: int = 240, height: int = 240):
        self.env = env
        self.name = name
        # 环境变量覆盖 (对齐设计文档 §6.1)
        self.width = int(os.environ.get("ELECTRONBOT_CAM_WIDTH", width))
        self.height = int(os.environ.get("ELECTRONBOT_CAM_HEIGHT", height))
        # 上限保护 (对齐 max_render_resolution=512)
        self.width = max(64, min(512, self.width))
        self.height = max(64, min(512, self.height))

        self.fovy = 60.0  # 度
        self._renderer = None
        self._seg_enabled = False
        self._init_renderer()

    def _init_renderer(self) -> None:
        """初始化 MuJoCo 渲染器, 支持 EGL → OSMesa 自动回退."""
        try:
            mujoco = self.env._mujoco
            os.environ.setdefault("MUJOCO_GL", "egl")
            self._renderer = mujoco.Renderer(
                self.env.model, self.height, self.width
            )
            logger.info("CameraSensor 初始化: %dx%d, 后端=%s, cam=%s",
                        self.width, self.height, os.environ.get("MUJOCO_GL"),
                        self.name)
        except (RuntimeError, ImportError) as e:
            logger.warning("EGL 渲染不可用 (%s), 回退到 OSMesa", e)
            os.environ["MUJOCO_GL"] = "osmesa"
            try:
                mujoco = self.env._mujoco
                self._renderer = mujoco.Renderer(
                    self.env.model, self.height, self.width
                )
                logger.info("CameraSensor 初始化 (OSMesa): %dx%d",
                            self.width, self.height)
            except RuntimeError as e2:
                logger.error("OSMesa 也不可用, 禁用摄像头: %s", e2)
                self._renderer = None

    def capture(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """捕获一帧, 返回 (rgb, depth, segmentation).

        返回:
            rgb:           (H, W, 3) uint8, 0-255
            depth:         (H, W)    float32, 单位米 (m)
            segmentation:  (H, W, 3) uint8, 每像素 (body_id, geom_id, 0)
        """
        if self._renderer is None:
            # 渲染器不可用, 返回零数组
            return (
                np.zeros((self.height, self.width, 3), dtype=np.uint8),
                np.zeros((self.height, self.width), dtype=np.float32),
                np.zeros((self.height, self.width, 3), dtype=np.uint8),
            )

        mujoco = self.env._mujoco
        # 更新场景
        self._renderer.update_scene(self.env.data, camera=self.name)

        # RGB 渲染
        rgb = self._renderer.render()

        # 深度渲染
        try:
            self._renderer.enable_depth_rendering()
            depth = self._renderer.render()
            self._renderer.disable_depth_rendering()
            # MuJoCo 深度: 正交距离 (m), inf 表示无物体
            depth = np.asarray(depth, dtype=np.float32)
            # 将 inf 替换为 0 (对齐设计文档 depth_zero_ratio < 0.2)
            depth = np.where(np.isinf(depth), 0.0, depth)
        except Exception as e:
            logger.debug("深度渲染失败: %s, 返回零数组", e)
            depth = np.zeros((self.height, self.width), dtype=np.float32)

        # 分割图渲染
        seg = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        try:
            self._renderer.enable_segmentation_rendering()
            seg_raw = self._renderer.render()
            self._renderer.disable_segmentation_rendering()
            if seg_raw is not None:
                seg = np.asarray(seg_raw, dtype=np.uint8)
                if seg.ndim == 3 and seg.shape[2] >= 3:
                    seg = seg[:, :, :3]
                else:
                    seg = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        except Exception as e:
            logger.debug("分割图渲染失败: %s", e)

        return rgb, depth, seg

    def get_intrinsics(self) -> Dict:
        """获取相机内参.

        返回:
            {"fx": float, "fy": float, "cx": float, "cy": float,
             "width": int, "height": int, "fovy": float}
        """
        # fx = fy = height / 2 / tan(fovy/2)
        fx = fy = self.height / 2.0 / np.tan(np.radians(self.fovy / 2.0))
        return {
            "fx": float(fx),
            "fy": float(fy),
            "cx": float(self.width / 2.0),
            "cy": float(self.height / 2.0),
            "width": self.width,
            "height": self.height,
            "fovy": self.fovy,
        }

    def close(self) -> None:
        """释放渲染器资源."""
        self._renderer = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass
