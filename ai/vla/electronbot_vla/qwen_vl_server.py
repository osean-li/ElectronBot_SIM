#!/usr/bin/env python3
"""
Qwen2-VL 7B AWQ 部署与推理服务

使用 vLLM 作为推理后端，支持:
- 模型下载 (HuggingFace / ModelScope 镜像)
- vLLM Server 启动 (前台 / 后台守护)
- 健康检查 + 推理测试

用法:
  python qwen_vl_server.py --action download     # 下载模型
  python qwen_vl_server.py --action serve        # 启动服务 (前台)
  python qwen_vl_server.py --action daemon       # 启动服务 (后台)
  python qwen_vl_server.py --action stop         # 停止服务
  python qwen_vl_server.py --action status       # 查看状态
  python qwen_vl_server.py --action test         # 测试推理
"""

import os
import sys
import json
import argparse
import subprocess
import time
import signal
import atexit
from pathlib import Path
from typing import Optional


# 模型配置
MODEL_CONFIGS = {
    "qwen2-vl-7b-awq": {
        "hf_id": "Qwen/Qwen2-VL-7B-Instruct-AWQ",
        "ms_id": "qwen/Qwen2-VL-7B-Instruct-AWQ",
        "local_dir": "./models/qwen_vl/Qwen2-VL-7B-Instruct-AWQ",
        "vram_required_gb": 7.0,
    },
    "qwen2-vl-2b": {
        "hf_id": "Qwen/Qwen2-VL-2B-Instruct",
        "ms_id": "qwen/Qwen2-VL-2B-Instruct",
        "local_dir": "./models/qwen_vl/Qwen2-VL-2B-Instruct",
        "vram_required_gb": 4.0,
    },
}

PID_FILE = "/tmp/electronbot_vllm.pid"
LOG_FILE = "/tmp/electronbot_vllm.log"


def download_model(model_name: str, use_mirror: bool = True):
    """下载模型权重"""
    config = MODEL_CONFIGS[model_name]

    # 检查磁盘空间
    import shutil
    disk = shutil.disk_usage(".")
    free_gb = disk.free / (1024 ** 3)
    need_gb = config["vram_required_gb"] * 3
    if free_gb < need_gb:
        print(f"[WARN] 磁盘可用: {free_gb:.1f} GB, 建议 ≥{need_gb:.0f} GB")

    local_dir = Path(config["local_dir"])
    if local_dir.exists() and (local_dir / "config.json").exists():
        size_gb = sum(f.stat().st_size for f in local_dir.rglob("*") if f.is_file()) / (1024**3)
        print(f"[INFO] 模型已存在: {local_dir} ({size_gb:.1f} GB)")
        return True

    print(f"[INFO] 下载模型: {config['hf_id']} → {local_dir}")
    local_dir.parent.mkdir(parents=True, exist_ok=True)

    if use_mirror:
        try:
            import modelscope
            from modelscope import snapshot_download
            cache = snapshot_download(config["ms_id"], cache_dir=str(local_dir.parent))
            print(f"[INFO] ModelScope 缓存: {cache}")
            # Symlink cache files → local_dir
            local_dir.mkdir(parents=True, exist_ok=True)
            for f in Path(cache).iterdir():
                dst = local_dir / f.name
                if not dst.exists():
                    dst.symlink_to(f)
            print(f"[SUCCESS] 模型已下载 (ModelScope 镜像)")
            return True
        except Exception as e:
            print(f"[WARN] ModelScope 下载失败: {e}")
            print("[INFO] 回退到 HuggingFace Hub...")

    # HuggingFace Hub
    try:
        from huggingface_hub import snapshot_download
        snapshot_download(
            config["hf_id"],
            local_dir=str(local_dir),
            local_dir_use_symlinks=False,
            resume_download=True,
        )
        print(f"[SUCCESS] 模型已下载")
        return True
    except Exception as e:
        print(f"[ERROR] 下载失败: {e}")
        print(f"[INFO] 手动: git lfs clone https://huggingface.co/{config['hf_id']} {local_dir}")
        return False


def start_vllm_server(model_name: str, port: int = 8000, daemon: bool = False):
    """启动 vLLM 推理服务"""
    config = MODEL_CONFIGS[model_name]
    model_path = str(Path(config["local_dir"]).absolute())

    if not Path(model_path).exists():
        print(f"[ERROR] 模型未下载: {model_path}")
        print(f"[INFO] 先运行: python {__file__} --action download")
        return

    # 检查端口
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if sock.connect_ex(("localhost", port)) == 0:
        print(f"[ERROR] 端口 {port} 已被占用")
        sock.close()
        return
    sock.close()

    cmd = [
        sys.executable, "-m", "vllm.entrypoints.openai.api_server",
        "--model", model_path,
        "--host", "0.0.0.0",
        "--port", str(port),
        "--max-model-len", "4096",
        "--gpu-memory-utilization", "0.85",
        "--trust-remote-code",
        "--dtype", "auto",
        "--enforce-eager",
    ]

    print(f"[INFO] 模型: {model_path}")
    print(f"[INFO] API:  http://localhost:{port}/v1")
    print(f"[INFO] Cmd:  {' '.join(cmd)}")

    if daemon:
        import signal as sig
        # 后台启动
        with open(LOG_FILE, "w") as log_fp:
            proc = subprocess.Popen(
                cmd,
                stdout=log_fp,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setpgrp,  # detach
            )
        # 写入 PID
        Path(PID_FILE).write_text(str(proc.pid))
        print(f"[INFO] 后台服务已启动 (PID {proc.pid})")
        print(f"[INFO] 日志: {LOG_FILE}")
        print(f"[INFO] 等待模型加载 (30-90s)...")

        # 轮询等待就绪
        max_wait = 120
        for i in range(0, max_wait, 5):
            time.sleep(5)
            if proc.poll() is not None:
                print(f"[ERROR] 服务异常退出 (exit code {proc.returncode})")
                with open(LOG_FILE) as f:
                    print(f.read()[-2000:])
                return
            try:
                import requests
                resp = requests.get(f"http://localhost:{port}/health", timeout=3)
                if resp.status_code == 200:
                    print(f"[SUCCESS] 服务就绪 (耗时 {i+5}s)")
                    return
            except Exception:
                pass
            if (i + 5) % 15 == 0:
                print(f"  等待中... ({i+5}s / {max_wait}s)")

        print(f"[ERROR] 启动超时")
        proc.kill()
        Path(PID_FILE).unlink(missing_ok=True)
    else:
        try:
            subprocess.run(cmd, check=True)
        except KeyboardInterrupt:
            print("\n[INFO] 服务已停止")


def stop_vllm_server(port: int = 8000):
    """停止 vLLM 服务"""
    stopped = False

    # PID 文件
    pid_file = Path(PID_FILE)
    if pid_file.exists():
        try:
            pid = int(pid_file.read_text().strip())
            os.kill(pid, signal.SIGTERM)
            time.sleep(2)
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
            print(f"[INFO] 已停止 (PID {pid})")
            stopped = True
        except (ProcessLookupError, ValueError):
            pass
        pid_file.unlink(missing_ok=True)

    # 端口释放
    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if sock.connect_ex(("localhost", port)) == 0:
        print(f"[WARN] 端口 {port} 仍有进程, 尝试释放...")
        subprocess.run(["fuser", "-k", f"{port}/tcp"], check=False)
    sock.close()

    if not stopped:
        print("[INFO] 无运行中的服务")


def check_status(port: int = 8000):
    """查看服务状态"""
    pid_file = Path(PID_FILE)

    print(f"── vLLM 服务状态 ──")
    if pid_file.exists():
        try:
            pid = int(pid_file.read_text().strip())
            os.kill(pid, 0)
            print(f"  PID:     {pid} (运行中)")
            # uptime
            try:
                import subprocess as sp
                etime = sp.check_output(["ps", "-p", str(pid), "-o", "etime="], text=True).strip()
                print(f"  运行:    {etime}")
            except Exception:
                pass
        except (ProcessLookupError, ValueError):
            print(f"  PID 文件存在但进程已消失")
    else:
        print(f"  PID:     无")

    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if sock.connect_ex(("localhost", port)) == 0:
        print(f"  端口:    {port} (活跃)")
    else:
        print(f"  端口:    {port} (无监听)")
    sock.close()

    # 健康检查
    try:
        import requests
        resp = requests.get(f"http://localhost:{port}/health", timeout=3)
        print(f"  健康:    OK (HTTP {resp.status_code})")
    except Exception:
        print(f"  健康:    FAIL")

    print(f"  日志:    {LOG_FILE}")


def test_inference(model_name: str, port: int = 8000):
    """测试推理"""
    import requests
    import base64
    import numpy as np

    print(f"[INFO] 测试推理 (模型: {model_name})")

    # 生成测试图像
    test_image = np.random.randint(0, 255, (240, 240, 3), dtype=np.uint8)
    import cv2
    _, img_encoded = cv2.imencode('.jpg', test_image)
    img_base64 = base64.b64encode(img_encoded).decode()

    prompt = "请挥手打招呼，返回6个关节角度(度): [head, left_arm_pitch, left_arm_roll, right_arm_pitch, right_arm_roll, body]"

    payload = {
        "model": model_name,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": f"data:image/jpeg;base64,{img_base64}"},
                    {"type": "text", "text": prompt},
                ]
            }
        ],
        "max_tokens": 256,
        "temperature": 0.1,
    }

    try:
        resp = requests.post(
            f"http://localhost:{port}/v1/chat/completions",
            json=payload,
            timeout=60,
        )
        if resp.status_code == 200:
            result = resp.json()
            output = result["choices"][0]["message"]["content"]
            print(f"[SUCCESS] 推理完成!")
            print(f"[OUTPUT] {output[:500]}")

            # 解析角度
            import re
            nums = re.findall(r'-?\d+\.?\d*', output)
            if len(nums) >= 6:
                angles = [float(n) for n in nums[:6]]
                print(f"[ANGLES] {angles} (度)")

            usage = result.get("usage", {})
            elapsed = resp.elapsed.total_seconds()
            print(f"[STATS] {usage.get('prompt_tokens','?')} prompt + {usage.get('completion_tokens','?')} completion tokens, {elapsed:.2f}s")
            return True
        else:
            print(f"[ERROR] HTTP {resp.status_code}: {resp.text[:500]}")
            return False
    except requests.exceptions.ConnectionError:
        print(f"[ERROR] 无法连接 (端口 {port}), 请先启动服务: python {__file__} --action daemon")
        return False
    except Exception as e:
        print(f"[ERROR] {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description="Qwen2-VL 部署管理")
    parser.add_argument("--action", type=str, default="download",
                        choices=["download", "serve", "daemon", "stop", "status", "test"],
                        help="操作类型")
    parser.add_argument("--model", type=str, default="qwen2-vl-7b-awq",
                        choices=list(MODEL_CONFIGS.keys()),
                        help="模型选择")
    parser.add_argument("--port", type=int, default=8000,
                        help="API 端口 (默认 8000)")
    parser.add_argument("--no-mirror", action="store_true",
                        help="不使用 ModelScope 镜像")
    args = parser.parse_args()

    actions = {
        "download": lambda: download_model(args.model, use_mirror=not args.no_mirror),
        "serve":    lambda: start_vllm_server(args.model, args.port, daemon=False),
        "daemon":   lambda: start_vllm_server(args.model, args.port, daemon=True),
        "stop":     lambda: stop_vllm_server(args.port),
        "status":   lambda: check_status(args.port),
        "test":     lambda: test_inference(args.model, args.port),
    }

    actions[args.action]()


if __name__ == "__main__":
    main()
