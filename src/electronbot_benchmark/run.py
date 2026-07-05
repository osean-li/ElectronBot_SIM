"""Benchmark CLI 运行脚本.

对齐 docs/tasks/07-Benchmark §4.

使用方式:
  # 最小 Benchmark
  python -m electronbot_benchmark.run --tasks reach --algorithms random --episodes 10

  # 完整 Benchmark
  python -m electronbot_benchmark.run \
      --tasks reach push pick_place stack follow gesture voice \
      --algorithms bc act ppo \
      --episodes 100 \
      --output results/full_benchmark.json
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path

from .suite import ElectronBotBenchmark, RandomPolicy, HomePolicy

logger = logging.getLogger("electronbot_benchmark.run")


def build_policies(algorithm_names: list[str], task_name: str):
    """根据算法名列表构建策略实例.

    支持的算法:
      - random: 随机策略 (基线)
      - home:   Home 策略 (零动作)
      - bc:     Behavior Cloning (需加载权重)
      - act:    ACT (需加载权重)
      - ppo:    PPO (需加载权重)
    """
    policies = {}
    for algo in algorithm_names:
        algo = algo.lower().strip()
        if algo == "random":
            policies["random"] = RandomPolicy()
        elif algo == "home":
            policies["home"] = HomePolicy()
        elif algo == "bc":
            policy = _load_bc_policy(task_name)
            if policy is not None:
                policies["bc"] = policy
        elif algo == "act":
            policy = _load_act_policy(task_name)
            if policy is not None:
                policies["act"] = policy
        elif algo == "ppo":
            policy = _load_ppo_policy(task_name)
            if policy is not None:
                policies["ppo"] = policy
        else:
            logger.warning("未知算法: %s, 跳过", algo)
    return policies


def _load_bc_policy(task_name: str):
    """加载 BC 策略权重."""
    try:
        from electronbot_ai.il.train_bc import BCPolicy
        path = Path(f"checkpoints/bc_{task_name}.pt")
        if not path.exists():
            logger.warning("BC 权重不存在: %s, 跳过", path)
            return None
        # 需要知道 obs_dim, 暂用默认值
        policy = BCPolicy(obs_dim=13, act_dim=6)
        policy.load(str(path))
        return policy
    except Exception as e:
        logger.warning("加载 BC 策略失败: %s", e)
        return None


def _load_act_policy(task_name: str):
    """加载 ACT 策略权重."""
    try:
        from electronbot_ai.il.train_act import ACTPolicy
        path = Path(f"checkpoints/act_{task_name}.pt")
        if not path.exists():
            logger.warning("ACT 权重不存在: %s, 跳过", path)
            return None
        policy = ACTPolicy(obs_dim=13, act_dim=6)
        policy.load(str(path))
        return policy
    except Exception as e:
        logger.warning("加载 ACT 策略失败: %s", e)
        return None


def _load_ppo_policy(task_name: str):
    """加载 PPO 策略."""
    try:
        from stable_baselines3 import PPO
        path = Path(f"checkpoints/ppo_{task_name}.zip")
        if not path.exists():
            logger.warning("PPO 权重不存在: %s, 跳过", path)
            return None
        model = PPO.load(str(path))

        class PPOWrapper:
            name = "ppo"
            def predict(self, obs):
                action, _ = model.predict(obs, deterministic=True)
                return action
            def reset(self):
                pass

        return PPOWrapper()
    except Exception as e:
        logger.warning("加载 PPO 策略失败: %s", e)
        return None


def build_tasks(task_names: list[str], env, seed: int = 42):
    """构建任务列表."""
    from electronbot_ai.tasks import create_task, TASK_DISPLAY_NAMES

    tasks = []
    for name in task_names:
        name = name.lower().strip()
        try:
            task = create_task(name, seed=seed)
            task.bind(env)
            # 覆盖 name 为显示名
            task.name = TASK_DISPLAY_NAMES.get(name, name)
            tasks.append(task)
            logger.info("任务已加载: %s", task.name)
        except ValueError as e:
            logger.warning("B004 任务未注册: %s (%s)", name, e)
    return tasks


def main():
    """CLI 入口."""
    parser = argparse.ArgumentParser(description="ElectronBot Benchmark 评估")
    parser.add_argument("--tasks", nargs="+", default=["reach"],
                        help="任务名列表 (默认: reach)")
    parser.add_argument("--algorithms", nargs="+", default=["random"],
                        help="算法名列表 (默认: random)")
    parser.add_argument("--episodes", type=int, default=100,
                        help="每个组合的评估 episode 数")
    parser.add_argument("--seed", type=int, default=42, help="全局随机种子")
    parser.add_argument("--output", type=str, default=None, help="结果输出路径")
    parser.add_argument("--render", action="store_true", help="开启渲染")
    parser.add_argument("--timeout", type=int, default=60, help="单 episode 超时 (秒)")
    parser.add_argument("--report", type=str, default=None, help="报告输出路径")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                       format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    # 创建环境
    from electronbot_sim.env import ElectronBotEnv
    env = ElectronBotEnv(render_mode="human" if args.render else None)

    # 构建任务和策略
    tasks = build_tasks(args.tasks, env, args.seed)
    if not tasks:
        logger.error("无可用任务, 退出")
        sys.exit(1)

    # 为每个任务构建策略 (部分策略需要任务特定的权重)
    all_policies = {}
    for task in tasks:
        task_key = task.name.lower().replace("eb-", "")
        policies = build_policies(args.algorithms, task_key)
        all_policies.update(policies)

    if not all_policies:
        logger.error("无可用策略, 退出")
        sys.exit(1)

    # 运行 Benchmark
    bench = ElectronBotBenchmark(
        output_dir=str(Path(args.output).parent) if args.output else "results",
        seed=args.seed,
    )
    bench.run_all(tasks, all_policies, num_episodes=args.episodes, env=env)
    bench.print_table()

    # 生成报告
    if args.report:
        from .report import generate_report
        report = generate_report(bench.results, fmt="markdown")
        Path(args.report).parent.mkdir(parents=True, exist_ok=True)
        Path(args.report).write_text(report, encoding="utf-8")
        logger.info("报告已生成: %s", args.report)

    env.close()


if __name__ == "__main__":
    main()
