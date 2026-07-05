"""Benchmark 核心类 — 标准评估套件.

对齐 docs/tasks/07-Benchmark §3.

BenchmarkResult: 单次 task×algorithm 评估结果
ElectronBotBenchmark: 批量评估、结果汇总、表格打印、JSON 保存
"""
from __future__ import annotations

import json
import logging
import os
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

import numpy as np

logger = logging.getLogger("electronbot_benchmark.suite")


@dataclass
class BenchmarkResult:
    """单次 Benchmark 结果 (task × algorithm).

    对齐 §7.1.1 字段定义.
    """
    # 标识字段
    task_name: str
    algorithm: str

    # 核心指标
    success_rate: float                          # [0, 1]
    mean_completion_time: float                  # 秒
    trajectory_smoothness: float                 # 动作增量 L2 均值
    generalization_gap: float = 0.0              # (ID-OOD)/ID

    # 统计元数据
    num_episodes: int = 100
    num_successes: int = 0
    seed: int = 42

    # 逐 episode 明细
    per_episode_times: List[float] = field(default_factory=list)
    per_episode_smoothness: List[float] = field(default_factory=list)
    per_episode_success: List[bool] = field(default_factory=list)

    # Sim2Real 指标 (可选)
    sim2real_gap: Optional[float] = None

    def to_dict(self) -> dict:
        """转为可序列化的字典."""
        return {
            "task": self.task_name,
            "algorithm": self.algorithm,
            "success_rate": round(self.success_rate, 4),
            "mean_time": round(self.mean_completion_time, 2),
            "smoothness": round(self.trajectory_smoothness, 4),
            "generalization_gap": round(self.generalization_gap, 4),
            "sim2real_gap": self.sim2real_gap,
            "num_episodes": self.num_episodes,
            "num_successes": self.num_successes,
            "seed": self.seed,
        }


class ElectronBotBenchmark:
    """标准 Benchmark Suite.

    对齐 §3.1 核心类.

    使用方式:
        bench = ElectronBotBenchmark(output_dir="results")
        result = bench.run_task(task, policy, num_episodes=100)
        bench.run_all(tasks, policies, num_episodes=100)
        bench.print_table()
    """

    def __init__(self, output_dir: str = "results", seed: int = 42):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.seed = seed
        self.results: List[BenchmarkResult] = []
        np.random.seed(seed)

    def run_task(self, task: Any, policy: Any, num_episodes: int = 100,
                 render: bool = False, timeout: int = 60,
                 env: Optional[Any] = None) -> BenchmarkResult:
        """运行单个 task × algorithm 评估.

        参数:
            task:         BaseTask 实例 (须实现 reset/step/is_success, 有 name 属性)
            policy:       策略实例 (须实现 predict(obs) → action, 有 name 属性, reset())
            num_episodes: 评估回合数
            render:       是否渲染
            timeout:      单 episode 超时 (秒)
            env:          ElectronBotEnv 实例 (若 task 未绑定)

        返回:
            BenchmarkResult
        """
        successes = 0
        times: List[float] = []
        smoothness_vals: List[float] = []
        per_success: List[bool] = []
        failed_episodes: List[dict] = []
        crash_count = 0

        for ep in range(num_episodes):
            try:
                # 重置
                if hasattr(task, "reset"):
                    if env is not None:
                        obs = task.reset(env)
                    else:
                        obs = task.reset()
                else:
                    obs = env.reset()

                if hasattr(policy, "reset"):
                    policy.reset()

                start_time = time.time()
                done = False
                prev_action: Optional[np.ndarray] = None
                ep_steps = 0

                while not done:
                    # 超时检测
                    if time.time() - start_time > timeout:
                        logger.debug("B002 episode 超时 (%.1fs)", time.time() - start_time)
                        break

                    # 策略推理
                    action = policy.predict(obs)

                    # 计算轨迹平滑度
                    if prev_action is not None:
                        smoothness = float(np.linalg.norm(
                            np.asarray(action) - np.asarray(prev_action)
                        ))
                        smoothness_vals.append(smoothness)
                    prev_action = np.asarray(action)

                    # 执行
                    if hasattr(task, "step"):
                        obs, _, done, truncated, info = task.step(action)
                    else:
                        obs, _, done, truncated, info = env.step(action)

                    ep_steps += 1

                    # 成功检测
                    if hasattr(task, "is_success") and task.is_success():
                        successes += 1
                        times.append(time.time() - start_time)
                        done = True
                        break

                    if truncated:
                        break

                per_success.append(successes > len(per_success))

            except RuntimeError as e:
                # 环境崩溃
                crash_count += 1
                logger.error("B003 环境崩溃 [%s/%s] ep=%d: %s",
                            task.name, getattr(policy, "name", "?"), ep, e)
                if crash_count >= 10:
                    logger.error("崩溃次数过多 (≥10), 放弃此组合")
                    break
                per_success.append(False)
                continue
            except Exception as e:
                # 策略推理异常
                logger.warning("B001 策略推理异常 ep=%d: %s", ep, e)
                failed_episodes.append({"episode": ep, "error": str(e)})
                per_success.append(False)
                continue

        # 计算结果
        success_rate = successes / max(1, num_episodes)
        mean_time = float(np.mean(times)) if times else float(timeout)
        mean_smoothness = float(np.mean(smoothness_vals)) if smoothness_vals else 0.0

        result = BenchmarkResult(
            task_name=getattr(task, "name", "Unknown"),
            algorithm=getattr(policy, "name", "unknown"),
            success_rate=success_rate,
            mean_completion_time=mean_time,
            trajectory_smoothness=mean_smoothness,
            num_episodes=num_episodes,
            num_successes=successes,
            seed=self.seed,
            per_episode_times=times,
            per_episode_smoothness=smoothness_vals,
            per_episode_success=per_success,
        )

        if failed_episodes:
            logger.warning("[%s/%s] 异常 episode: %d 个",
                          result.task_name, result.algorithm, len(failed_episodes))

        return result

    def run_all(self, tasks: List[Any], policies: Dict[str, Any],
                num_episodes: int = 100, env: Optional[Any] = None) -> List[BenchmarkResult]:
        """运行完整 Benchmark 矩阵 (tasks × policies 笛卡尔积).

        参数:
            tasks:        任务列表
            policies:     {算法名: 策略实例} 字典
            num_episodes: 每个组合的评估 episode 数
            env:          共享环境实例

        返回:
            List[BenchmarkResult]
        """
        total = len(tasks) * len(policies)
        combo_idx = 0

        for task in tasks:
            for algo_name, policy in policies.items():
                combo_idx += 1
                logger.info("🏃 运行 %s / %s ... (组合 %d/%d)",
                           getattr(task, "name", "?"), algo_name, combo_idx, total)
                result = self.run_task(task, policy, num_episodes, env=env)
                self.results.append(result)
                logger.info("✅ %s / %s: SR=%.1%%, time=%.1fs, smooth=%.3f",
                           result.task_name, result.algorithm,
                           result.success_rate * 100,
                           result.mean_completion_time,
                           result.trajectory_smoothness)

        self._save_results()
        return self.results

    def print_table(self, results: Optional[List[BenchmarkResult]] = None) -> None:
        """打印结果矩阵到终端."""
        results = results or self.results
        if not results:
            print("无结果可显示")
            return

        tasks = sorted(set(r.task_name for r in results))
        algos = sorted(set(r.algorithm for r in results))

        # 表头
        header = f"│ {'Task':<14s}"
        for a in algos:
            header += f"│ {a:>8s} "
        print(header + "│")

        # 分隔线
        sep = "├" + "─" * 16
        for _ in algos:
            sep += "┼" + "─" * 10
        print(sep + "┤")

        # 数据行
        for task in tasks:
            row = f"│ {task:<14s}"
            for algo in algos:
                matched = [r for r in results
                          if r.task_name == task and r.algorithm == algo]
                if matched:
                    row += f"│ {matched[0].success_rate * 100:>7.0f}% "
                else:
                    row += "│    ─     "
            print(row + "│")

    def _save_results(self, path: Optional[str] = None) -> Path:
        """保存结果为 JSON."""
        path = path or str(self.output_dir / "benchmark_results.json")
        data = {
            "metadata": {
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "seed": self.seed,
                "total_combinations": len(self.results),
            },
            "results": [r.to_dict() for r in self.results],
        }
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        logger.info("结果已保存: %s", path)
        return Path(path)

    def load_results(self, path: str) -> List[BenchmarkResult]:
        """从 JSON 加载历史结果."""
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        results = []
        for item in data.get("results", []):
            results.append(BenchmarkResult(
                task_name=item["task"],
                algorithm=item["algorithm"],
                success_rate=item["success_rate"],
                mean_completion_time=item["mean_time"],
                trajectory_smoothness=item["smoothness"],
                generalization_gap=item.get("generalization_gap", 0.0),
                sim2real_gap=item.get("sim2real_gap"),
                num_episodes=item.get("num_episodes", 100),
                num_successes=item.get("num_successes", 0),
                seed=item.get("seed", 42),
            ))
        return results

    @staticmethod
    def compare(baseline: BenchmarkResult, candidate: BenchmarkResult) -> dict:
        """对比两个结果的指标差异."""
        return {
            "task": baseline.task_name,
            "baseline_algo": baseline.algorithm,
            "candidate_algo": candidate.algorithm,
            "success_rate_diff": candidate.success_rate - baseline.success_rate,
            "time_diff": candidate.mean_completion_time - baseline.mean_completion_time,
            "smoothness_diff": candidate.trajectory_smoothness - baseline.trajectory_smoothness,
        }


# 基线策略 (用于测试 Benchmark 系统本身)
class RandomPolicy:
    """随机策略 — Benchmark 基线."""

    name = "random"

    def __init__(self, action_dim: int = 6):
        self.action_dim = action_dim

    def predict(self, obs: Any) -> np.ndarray:
        return np.random.uniform(-2.0, 2.0, self.action_dim).astype(np.float32)

    def reset(self):
        pass


class HomePolicy:
    """Home 策略 — 永远输出零动作 (回到 home 姿态)."""

    name = "home"

    def __init__(self, action_dim: int = 6):
        self.action_dim = action_dim

    def predict(self, obs: Any) -> np.ndarray:
        return np.zeros(self.action_dim, dtype=np.float32)

    def reset(self):
        pass
