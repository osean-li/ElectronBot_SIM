#!/usr/bin/env python3
"""
VLA 双模式后端: VLAModel 抽象基类 + QwenVLAModel + OpenVLAModel

支持运行时切换 VLA 后端:
  - 模式 A: Qwen2-VL 7B AWQ (本地 vLLM 推理, OpenAI 兼容 API)
  - 模式 B: OpenVLA (开源 VLA 框架, 自定义 HTTP API)

统一接口: predict(rgb: np.ndarray, text_prompt: str) -> joint_angles (6,) in rad
"""

from abc import ABC, abstractmethod
import base64
import io
import json
import logging
import re
import time
from typing import Optional, Dict, Any, List

import numpy as np
from PIL import Image

logger = logging.getLogger(__name__)

# 导入提示词模板 (从同包)
try:
    from .prompt_templates import SYSTEM_PROMPT, build_vla_messages, parse_vlm_output
except ImportError:
    # 作为脚本独立运行时
    from prompt_templates import SYSTEM_PROMPT, build_vla_messages, parse_vlm_output


# ============================================================
# 抽象基类
# ============================================================

class VLAModel(ABC):
    """VLA 模型抽象基类"""

    @abstractmethod
    def predict(self, image: np.ndarray, prompt: str) -> np.ndarray:
        """
        视觉语言推理 → 6 维关节角度

        参数:
          image:  RGB 图像 (HxWx3, uint8)
          prompt: 自然语言指令

        返回:
          joint_angles: 6 维关节角度 (rad)
        """
        pass

    @abstractmethod
    def is_ready(self) -> bool:
        """检查模型是否就绪"""
        pass

    def warmup(self, image: Optional[np.ndarray] = None) -> bool:
        """预热模型 (可选)"""
        try:
            test_img = image or np.random.randint(0, 255, (240, 240, 3), dtype=np.uint8)
            self.predict(test_img, "保持不动")
            return True
        except Exception as e:
            logger.warning(f"预热失败: {e}")
            return False


# ============================================================
# 模式 A: Qwen2-VL 7B AWQ (vLLM, OpenAI 兼容 API)
# ============================================================

class QwenVLAModel(VLAModel):
    """
    Qwen2-VL 7B AWQ 量化版本地推理

    两种使用方式:
    1. vLLM OpenAI API (推荐): 先用 deploy_qwen_vl.sh 启动服务，然后传 server_url
    2. vLLM Python 内嵌 (备选): 无需独立服务，直接在进程中加载模型

    例:
      # 方式 1: 连接已有的 vLLM 服务
      model = QwenVLAModel(
          model_path="./models/qwen_vl/Qwen2-VL-7B-Instruct-AWQ",
          server_url="http://localhost:8000/v1",
      )
    """

    def __init__(
        self,
        model_path: str = "Qwen/Qwen2-VL-7B-Instruct-AWQ",
        server_url: Optional[str] = "http://localhost:8000/v1",
        use_inline: bool = False,       # True = 内嵌加载, False = 连接 API 服务
    ):
        self.model_path = model_path
        self.server_url = server_url.rstrip("/") if server_url else None
        self.use_inline = use_inline
        self._client: Optional[Any] = None
        self._ready = False

    # -------------------------------------------------------
    # 方式 1: vLLM API 服务 (推荐)
    # -------------------------------------------------------

    def _ensure_api_client(self) -> bool:
        """检查 vLLM API 服务是否可达"""
        if not self.server_url:
            return False

        import requests
        try:
            resp = requests.get(
                self.server_url.replace("/v1", "/health"),
                timeout=3,
            )
            if resp.status_code == 200:
                self._ready = True
                logger.info(f"vLLM API 就绪: {self.server_url}")
                return True
        except Exception:
            pass
        return False

    def _predict_via_api(self, image: np.ndarray, prompt: str) -> np.ndarray:
        """通过 HTTP API 推理"""
        import requests

        # 编码图像
        img_b64 = _encode_image_base64(image)

        # 构建多模态消息
        messages = _build_vllm_messages(prompt, img_b64)

        payload = {
            "model": self.model_path.split("/")[-1],
            "messages": messages,
            "max_tokens": 256,
            "temperature": 0.1,
            "top_p": 0.9,
        }

        t0 = time.time()
        try:
            resp = requests.post(
                f"{self.server_url}/chat/completions",
                json=payload,
                timeout=60,
            )
            elapsed = time.time() - t0
        except requests.exceptions.ConnectionError:
            logger.warning("vLLM API 不可达, 降级到 mock 模式")
            return _mock_predict(prompt)
        except Exception as e:
            logger.error(f"API 请求失败: {e}")
            return _mock_predict(prompt)

        if resp.status_code != 200:
            logger.error(f"vLLM 返回 {resp.status_code}: {resp.text[:300]}")
            return _mock_predict(prompt)

        result = resp.json()
        raw_output = result["choices"][0]["message"]["content"]
        logger.info(f"vLLM 推理 ({elapsed:.2f}s): {raw_output[:200]}")

        # 解析角度
        angles = parse_vlm_output(raw_output)
        if angles is not None:
            return _clip_angles(angles)

        logger.warning("无法解析 vLLM 输出中的角度, 回退 mock")
        return _mock_predict(prompt)

    # -------------------------------------------------------
    # 方式 2: vLLM 内嵌加载 (备选)
    # -------------------------------------------------------

    def _ensure_inline_client(self) -> bool:
        """内嵌加载 vLLM 模型到当前进程"""
        if self._client is True:
            return True
        if self._client is False:
            return False

        try:
            from vllm import LLM, SamplingParams
            logger.info(f"内嵌加载模型: {self.model_path}")
            t0 = time.time()

            self._llm = LLM(
                model=self.model_path,
                max_model_len=4096,
                gpu_memory_utilization=0.85,
                trust_remote_code=True,
                enforce_eager=True,
            )
            self._sampling_params = SamplingParams(
                temperature=0.1,
                max_tokens=256,
                top_p=0.9,
            )
            logger.info(f"模型加载完成 ({time.time() - t0:.1f}s)")
            self._client = True
            self._ready = True
            return True

        except Exception as e:
            logger.error(f"vLLM 内嵌加载失败: {e}")
            self._client = False
            return False

    def _predict_via_inline(self, image: np.ndarray, prompt: str) -> np.ndarray:
        """内嵌 vLLM 推理"""
        # Encode image as data URL
        img_b64 = _encode_image_base64(image)
        img_url = f"data:image/jpeg;base64,{img_b64}"

        # Build chat template with image
        # vLLM 的多模态输入使用类似 OpenAI 的 chat format
        messages = _build_vllm_messages(prompt, img_b64)

        try:
            t0 = time.time()
            outputs = self._llm.chat(
                messages,
                sampling_params=self._sampling_params,
                use_tqdm=False,
            )
            raw_output = outputs[0].outputs[0].text
            logger.info(f"vLLM 推理 ({time.time() - t0:.2f}s): {raw_output[:200]}")

            angles = parse_vlm_output(raw_output)
            if angles is not None:
                return _clip_angles(angles)
            return _mock_predict(prompt)

        except Exception as e:
            logger.error(f"内嵌推理失败: {e}")
            return _mock_predict(prompt)

    # -------------------------------------------------------
    # 统一接口
    # -------------------------------------------------------

    def is_ready(self) -> bool:
        if self._ready:
            return True

        if self.use_inline:
            self._ready = self._ensure_inline_client()
        else:
            self._ready = self._ensure_api_client()

        return self._ready

    def predict(self, image: np.ndarray, prompt: str) -> np.ndarray:
        """VLA 推理"""
        if self.use_inline:
            if not self._ensure_inline_client():
                return _mock_predict(prompt)
            return self._predict_via_inline(image, prompt)

        # API 模式
        if self._ensure_api_client():
            return self._predict_via_api(image, prompt)

        logger.warning("vLLM 不可用, 使用 mock 模式")
        return _mock_predict(prompt)


# ============================================================
# 模式 B: OpenVLA (开源 VLA 框架, 自定义 HTTP API)
# ============================================================

class OpenVLAModel(VLAModel):
    """
    OpenVLA 开源 VLA 框架

    使用方式:
    1. 先用 deploy_openvla.sh serve 启动 OpenVLA HTTP API 服务
    2. 然后:
       model = OpenVLAModel(server_url="http://localhost:8001")
    """

    def __init__(
        self,
        model_path: str = "openvla/openvla-7b",
        server_url: Optional[str] = "http://localhost:8001",
        use_inline: bool = False,
    ):
        self.model_path = model_path
        self.server_url = server_url.rstrip("/") if server_url else None
        self.use_inline = use_inline
        self._model = None
        self._processor = None
        self._ready = False

    def is_ready(self) -> bool:
        """检查 OpenVLA 是否就绪"""
        if self._ready:
            return True

        if self.use_inline:
            return self._model is not None

        # API 模式
        import requests
        try:
            resp = requests.get(f"{self.server_url}/health", timeout=3)
            if resp.status_code == 200:
                self._ready = True
                logger.info(f"OpenVLA API 就绪: {self.server_url}")
                return True
        except Exception:
            pass
        return False

    def _ensure_inline_model(self) -> bool:
        """内嵌加载 OpenVLA (慎用, 显存占用大)"""
        if self._model is not None:
            return True

        try:
            import torch
            from transformers import AutoModelForVision2Seq, AutoProcessor

            logger.info(f"加载 OpenVLA: {self.model_path}")
            t0 = time.time()

            self._processor = AutoProcessor.from_pretrained(
                self.model_path, trust_remote_code=True
            )
            self._model = AutoModelForVision2Seq.from_pretrained(
                self.model_path,
                torch_dtype=torch.bfloat16,
                device_map="auto",
                trust_remote_code=True,
                low_cpu_mem_usage=True,
            )
            self._model.eval()
            logger.info(f"OpenVLA 加载完成 ({time.time() - t0:.1f}s)")
            self._ready = True
            return True

        except Exception as e:
            logger.error(f"OpenVLA 加载失败: {e}")
            return False

    def predict(self, image: np.ndarray, prompt: str) -> np.ndarray:
        """OpenVLA 推理"""

        # API 模式优先
        if self.server_url and not self.use_inline:
            return self._predict_via_api(image, prompt)

        # 内嵌模式
        if self.use_inline:
            if not self._ensure_inline_model():
                return _mock_predict(prompt)
            return self._predict_via_inline(image, prompt)

        logger.warning("OpenVLA 不可用")
        return _mock_predict(prompt)

    def _predict_via_api(self, image: np.ndarray, prompt: str) -> np.ndarray:
        """通过 deploy_openvla.sh 启动的 HTTP API 推理"""
        import requests

        img_b64 = _encode_image_base64(image)

        payload = {
            "image": f"data:image/jpeg;base64,{img_b64}",
            "prompt": prompt,
        }

        t0 = time.time()
        try:
            resp = requests.post(
                f"{self.server_url}/predict",
                json=payload,
                timeout=120,
            )
            elapsed = time.time() - t0
        except Exception as e:
            logger.error(f"OpenVLA API 失败: {e}")
            return _mock_predict(prompt)

        if resp.status_code != 200:
            logger.error(f"OpenVLA API HTTP {resp.status_code}")
            return _mock_predict(prompt)

        data = resp.json()
        if data.get("success"):
            angles_rad = data.get("joint_angles_rad", [])
            if len(angles_rad) == 6:
                logger.info(f"OpenVLA 推理 ({elapsed:.2f}s): {np.rad2deg(angles_rad)}°")
                return _clip_angles(np.array(angles_rad))
            angles_deg = data.get("joint_angles_deg", [])
            if len(angles_deg) == 6:
                logger.info(f"OpenVLA 推理 ({elapsed:.2f}s): {angles_deg}°")
                return _clip_angles(np.radians(np.array(angles_deg)))

        logger.warning(f"OpenVLA 返回异常: {data.get('error', 'unknown')}")
        return _mock_predict(prompt)

    def _predict_via_inline(self, image: np.ndarray, prompt: str) -> np.ndarray:
        """内嵌 OpenVLA 推理"""
        import torch

        try:
            # 预处理图像
            pil_image = Image.fromarray(image).convert("RGB").resize((224, 224))

            # 构建 prompt
            lang_prompt = f"In: What should the robot do to {prompt}?\nOut:"

            # 处理输入
            inputs = self._processor(lang_prompt, pil_image).to(self._model.device)

            # 生成
            with torch.no_grad():
                outputs = self._model.generate(
                    **inputs,
                    max_new_tokens=128,
                    do_sample=False,
                )

            raw_output = self._processor.decode(outputs[0], skip_special_tokens=True)
            logger.info(f"OpenVLA (inline): {raw_output[:200]}")

            # 解析 7-DoF → 取前 6 个
            nums = re.findall(r'-?\d+\.?\d*', raw_output)
            if len(nums) >= 6:
                angles = np.radians([float(n) for n in nums[:6]])
                return _clip_angles(angles)

            return _mock_predict(prompt)

        except Exception as e:
            logger.error(f"OpenVLA 内嵌推理失败: {e}")
            return _mock_predict(prompt)


# ============================================================
# 工具函数
# ============================================================

def _encode_image_base64(image: np.ndarray) -> str:
    """将 numpy 图像编码为 base64 JPEG 字符串"""
    import cv2
    success, buf = cv2.imencode('.jpg', cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    if not success:
        raise ValueError("图像编码失败")
    return base64.b64encode(buf).decode("ascii")


def _build_vllm_messages(prompt: str, img_b64: str) -> List[Dict]:
    """构建 vLLM OpenAI 兼容格式的多模态消息"""
    return [
        {
            "role": "system",
            "content": SYSTEM_PROMPT,
        },
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"},
                },
                {
                    "type": "text",
                    "text": prompt,
                },
            ],
        },
    ]


def _mock_predict(prompt: str) -> np.ndarray:
    """
    Mock 降级模式: 基于关键词返回预定义动作角度 (rad)
    当 VLM 不可用时自动使用
    """
    prompt_lower = prompt.lower()

    # 15 组关键词 → 预定义角度映射
    if "wave" in prompt_lower or "挥手" in prompt_lower or "挥手" in prompt_lower:
        return np.deg2rad([0, 0, 0, 0, 80, 10])
    elif "point" in prompt_lower or "指" in prompt_lower or "指着" in prompt_lower:
        return np.deg2rad([0, 0, 0, 10, 60, 0])
    elif "nod" in prompt_lower or "点头" in prompt_lower:
        return np.deg2rad([0, 10, 0, 0, 0, 0])
    elif "shake" in prompt_lower or "摇头" in prompt_lower:
        return np.deg2rad([30, 0, 0, 0, 0, 0])
    elif "heart" in prompt_lower or "比心" in prompt_lower or "爱心" in prompt_lower:
        return np.deg2rad([0, 0, 60, 20, 60, 20])
    elif "look" in prompt_lower or "看" in prompt_lower or "观察" in prompt_lower:
        return np.deg2rad([45, 5, 0, 0, 0, 0])
    elif "tired" in prompt_lower or "累" in prompt_lower or "疲倦" in prompt_lower:
        return np.deg2rad([0, -10, 0, 0, 0, 0])
    elif "excited" in prompt_lower or "兴奋" in prompt_lower or "开心" in prompt_lower:
        return np.deg2rad([20, 10, 50, 10, 50, 10])
    elif "greet" in prompt_lower or "hello" in prompt_lower or "你好" in prompt_lower:
        return np.deg2rad([0, 5, 0, 0, 60, 0])
    elif "bye" in prompt_lower or "再见" in prompt_lower or "拜拜" in prompt_lower:
        return np.deg2rad([0, 0, 0, 0, 80, 15])
    elif "touch" in prompt_lower or "碰" in prompt_lower or "摸" in prompt_lower:
        return np.deg2rad([0, 0, 0, 0, 90, 0])
    elif "push" in prompt_lower or "推" in prompt_lower:
        return np.deg2rad([0, 0, 0, 0, 80, 0])
    elif "rest" in prompt_lower or "休息" in prompt_lower or "归零" in prompt_lower:
        return np.zeros(6)
    else:
        # 安全默认: 保持当前姿态 (零位)
        return np.zeros(6)


def _clip_angles(angles: np.ndarray) -> np.ndarray:
    """安全裁切角度到模型范围"""
    # ElectronBot 机械角度范围 (rad)
    mins = np.deg2rad([-90, -15, -20, 0, -20, 0])
    maxs = np.deg2rad([90, 15, 180, 30, 180, 30])
    return np.clip(angles, mins, maxs)


# ============================================================
# 工厂函数
# ============================================================

def create_vla_backend(
    mode: str = "qwen",
    server_url: Optional[str] = None,
    model_path: Optional[str] = None,
    use_inline: bool = False,
    **kwargs,
) -> VLAModel:
    """
    VLA 后端工厂函数

    参数:
      mode:         "qwen" | "openvla"
      server_url:   VLA API 服务地址 (默认根据 mode 选择)
                       - qwen:    http://localhost:8000/v1
                       - openvla: http://localhost:8001
      model_path:   模型路径或 HF ID
      use_inline:   是否内嵌加载模型 (默认 False, 使用 API 服务)
    """
    if mode == "qwen":
        url = server_url or "http://localhost:8000/v1"
        path = model_path or "Qwen/Qwen2-VL-7B-Instruct-AWQ"
        return QwenVLAModel(
            model_path=path,
            server_url=url,
            use_inline=use_inline,
            **kwargs,
        )
    elif mode == "openvla":
        url = server_url or "http://localhost:8001"
        path = model_path or "openvla/openvla-7b"
        return OpenVLAModel(
            model_path=path,
            server_url=url,
            use_inline=use_inline,
            **kwargs,
        )
    else:
        raise ValueError(f"未知 VLA 模式: '{mode}'. 可选: 'qwen', 'openvla'")


# ============================================================
# 快速测试
# ============================================================

if __name__ == "__main__":
    import argparse
    logging.basicConfig(level=logging.INFO)

    parser = argparse.ArgumentParser(description="VLA Backend Test")
    parser.add_argument("--mode", default="qwen", choices=["qwen", "openvla"])
    parser.add_argument("--server-url", default=None)
    parser.add_argument("--prompt", default="挥手打招呼")
    args = parser.parse_args()

    print(f"创建 VLA 后端: mode={args.mode}")
    backend = create_vla_backend(
        mode=args.mode,
        server_url=args.server_url,
    )

    print(f"检查就绪状态...")
    if backend.is_ready():
        print(f"  ✅ 就绪")
        # Warmup
        print(f"  预热中...")
        backend.warmup()
        # Test
        test_img = np.random.randint(0, 255, (240, 240, 3), dtype=np.uint8)
        angles = backend.predict(test_img, args.prompt)
        print(f"  Prompt: {args.prompt}")
        print(f"  Angles (deg): {np.rad2deg(angles).round(1)}")
    else:
        print(f"  ❌ 未就绪 (将使用 mock 降级)")
        test_img = np.random.randint(0, 255, (240, 240, 3), dtype=np.uint8)
        angles = backend.predict(test_img, args.prompt)
        print(f"  Prompt: {args.prompt}")
        print(f"  Angles (deg): {np.rad2deg(angles).round(1)} (mock)")
