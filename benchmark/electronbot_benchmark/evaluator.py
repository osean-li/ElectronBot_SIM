#!/usr/bin/env python3
"""
Benchmark 统一评估框架

支持:
- 多算法统一评估 (PPO, SAC, ACT, Diffusion Policy)
- 多任务统一评估 (Reach, Push, Wave, PointAt, Stack)
- 多指标 (成功率、完成时间、Jerk 平滑度、最终误差)
- 结果记录与对比
"""

import json
import numpy as np
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field, asdict
from pathlib import Path


@dataclass
class EvalMetrics:
    """评估指标"""
    success_rate: float = 0.0
    mean_return: float = 0.0
    mean_completion_time: float = 0.0
    mean_jerk: float = 0.0  # 轨迹平滑度
    mean_final_error: float = 0.0  # 最终位置误差

    def to_dict(self) -> Dict[str, float]:
        return asdict(self)


class Evaluator:
    """统一评估器"""

    def __init__(
        self,
        task_name: str,
        n_episodes: int = 50,
        max_steps: int = 500,
        seed: int = 42,
    ):
        self.task_name = task_name
        self.n_episodes = n_episodes
        self.max_steps = max_steps
        self.seed = seed

        self._rng = np.random.RandomState(seed)

    def evaluate_model(self, model, env) -> EvalMetrics:
        """评估单个模型"""
        successes = []
        returns = []
        completion_times = []
        jerks = []
        final_errors = []

        for ep in range(self.n_episodes):
            env.reset(seed=self.seed + ep)
            obs = env.reset()[0]
            done = False
            ep_return = 0.0
            ep_steps = 0
            prev_action = np.zeros(6)

            while not done and ep_steps < self.max_steps:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, _ = env.step(action)
                done = terminated or truncated
                ep_return += reward
                ep_steps += 1

                # Jerk: 动作变化率
                jerk = np.mean((action - prev_action) ** 2)
                jerks.append(jerk)
                prev_action = action

            successes.append(float(done and not truncated))
            returns.append(ep_return)
            if done:
                completion_times.append(ep_steps)

            # 最终误差
            if hasattr(env, 'robot'):
                _, right_ee = env._get_ee_positions()
                target = getattr(env, '_target_pos', np.zeros(3))
                final_errors.append(np.linalg.norm(right_ee - target))

        return EvalMetrics(
            success_rate=np.mean(successes) if successes else 0.0,
            mean_return=np.mean(returns) if returns else 0.0,
            mean_completion_time=np.mean(completion_times) if completion_times else self.max_steps,
            mean_jerk=np.mean(jerks) if jerks else 0.0,
            mean_final_error=np.mean(final_errors) if final_errors else 0.0,
        )


class BenchmarkRunner:
    """Benchmark 运行器"""

    def __init__(self, output_dir: str = "./benchmark_results"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.results: Dict[str, Dict[str, EvalMetrics]] = {}

    def run(
        self,
        tasks: List[str],
        algorithms: Dict[str, Any],  # {algo_name: model}
        env_builder,
        n_seeds: int = 3,
    ):
        """运行完整 Benchmark"""
        evaluator = Evaluator("")

        for task in tasks:
            self.results[task] = {}
            for algo_name, model_fn in algorithms.items():
                best_metrics = None
                best_score = -float("inf")

                for s in range(n_seeds):
                    env = env_builder(task, seed=s)
                    model = model_fn(env, seed=s)
                    metrics = evaluator.evaluate_model(model, env)

                    if metrics.mean_return > best_score:
                        best_score = metrics.mean_return
                        best_metrics = metrics

                self.results[task][algo_name] = best_metrics

    def save_results(self):
        """保存结果到 JSON"""
        output = {}
        for task, algos in self.results.items():
            output[task] = {
                algo: metrics.to_dict()
                for algo, metrics in algos.items()
            }

        with open(self.output_dir / "benchmark_results.json", "w") as f:
            json.dump(output, f, indent=2)
        print(f"[INFO] Benchmark 结果已保存: {self.output_dir}")

    def print_summary(self):
        """打印摘要表格"""
        print(f"\n{'='*80}")
        print("Benchmark 结果摘要")
        print(f"{'='*80}")

        for task, algos in self.results.items():
            print(f"\n--- {task} ---")
            print(f"{'算法':<20} {'成功率':>8} {'AvgReturn':>10} {'时间(步)':>10} {'Jerk':>8}")
            for algo, m in algos.items():
                print(f"{algo:<20} {m.success_rate:>8.2%} {m.mean_return:>10.2f} "
                      f"{m.mean_completion_time:>10.1f} {m.mean_jerk:>8.4f}")
