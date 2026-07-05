"""Benchmark 系统测试 — 对齐 Phase 7.

测试:
  - BenchmarkResult 数据结构
  - ElectronBotBenchmark 评估流程
  - 结果保存/加载
  - 报告生成 (Markdown/HTML)
"""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np
import pytest

os.environ.setdefault("MUJOCO_GL", "osmesa")


class TestBenchmarkResult:
    """BenchmarkResult 数据结构测试."""

    def test_create_result(self):
        from electronbot_benchmark.suite import BenchmarkResult
        r = BenchmarkResult(
            task_name="EB-Reach",
            algorithm="bc",
            success_rate=0.87,
            mean_completion_time=2.3,
            trajectory_smoothness=0.15,
        )
        assert r.task_name == "EB-Reach"
        assert r.algorithm == "bc"
        assert r.success_rate == 0.87

    def test_to_dict(self):
        from electronbot_benchmark.suite import BenchmarkResult
        r = BenchmarkResult("EB-Reach", "ppo", 0.98, 1.5, 0.12)
        d = r.to_dict()
        assert d["task"] == "EB-Reach"
        assert d["algorithm"] == "ppo"
        assert d["success_rate"] == 0.98


class TestBenchmarkSuite:
    """ElectronBotBenchmark 测试."""

    @pytest.fixture
    def bench(self, tmp_path):
        from electronbot_benchmark.suite import ElectronBotBenchmark
        return ElectronBotBenchmark(output_dir=str(tmp_path), seed=42)

    def test_run_task_random_policy(self, bench):
        """用随机策略评估 Reach 任务."""
        from electronbot_benchmark.suite import RandomPolicy
        from electronbot_ai.tasks import create_task
        from electronbot_sim.env import ElectronBotEnv

        env = ElectronBotEnv(render_mode=None)
        task = create_task("reach", seed=42)
        task.bind(env)
        task.reset(env)

        policy = RandomPolicy()
        result = bench.run_task(task, policy, num_episodes=5, env=env)

        assert result.task_name == "EB-Reach"
        assert result.algorithm == "random"
        assert 0 <= result.success_rate <= 1
        env.close()

    def test_run_all(self, bench):
        """批量评估."""
        from electronbot_benchmark.suite import RandomPolicy, HomePolicy
        from electronbot_ai.tasks import create_task
        from electronbot_sim.env import ElectronBotEnv

        env = ElectronBotEnv(render_mode=None)
        task = create_task("reach", seed=42)
        task.bind(env)

        policies = {"random": RandomPolicy(), "home": HomePolicy()}
        results = bench.run_all([task], policies, num_episodes=3, env=env)

        assert len(results) == 2  # 1 task × 2 algorithms
        env.close()

    def test_save_load_results(self, bench, tmp_path):
        """结果保存和加载."""
        from electronbot_benchmark.suite import BenchmarkResult

        bench.results = [
            BenchmarkResult("EB-Reach", "bc", 0.85, 2.0, 0.1),
            BenchmarkResult("EB-Push", "ppo", 0.90, 3.0, 0.2),
        ]
        path = bench._save_results()
        assert path.exists()

        loaded = bench.load_results(str(path))
        assert len(loaded) == 2
        assert loaded[0].task_name == "EB-Reach"
        assert loaded[1].success_rate == 0.90

    def test_print_table(self, bench, capsys):
        """表格打印不崩溃."""
        from electronbot_benchmark.suite import BenchmarkResult

        bench.results = [
            BenchmarkResult("EB-Reach", "bc", 0.85, 2.0, 0.1),
            BenchmarkResult("EB-Reach", "ppo", 0.90, 3.0, 0.2),
        ]
        bench.print_table()
        captured = capsys.readouterr()
        assert "EB-Reach" in captured.out

    def test_compare(self):
        """结果对比."""
        from electronbot_benchmark.suite import ElectronBotBenchmark, BenchmarkResult

        baseline = BenchmarkResult("EB-Reach", "bc", 0.80, 3.0, 0.15)
        candidate = BenchmarkResult("EB-Reach", "ppo", 0.95, 2.0, 0.10)
        diff = ElectronBotBenchmark.compare(baseline, candidate)

        assert diff["success_rate_diff"] == pytest.approx(0.15)
        assert diff["baseline_algo"] == "bc"
        assert diff["candidate_algo"] == "ppo"


class TestReport:
    """报告生成测试."""

    def test_markdown_report(self):
        from electronbot_benchmark.report import generate_report
        from electronbot_benchmark.suite import BenchmarkResult

        results = [
            BenchmarkResult("EB-Reach", "bc", 0.87, 2.3, 0.15),
            BenchmarkResult("EB-Reach", "ppo", 0.98, 1.5, 0.12),
        ]
        report = generate_report(results, fmt="markdown")
        assert "Benchmark" in report
        assert "EB-Reach" in report
        assert "bc" in report
        assert "ppo" in report

    def test_html_report(self):
        from electronbot_benchmark.report import generate_report
        from electronbot_benchmark.suite import BenchmarkResult

        results = [
            BenchmarkResult("EB-Reach", "bc", 0.87, 2.3, 0.15),
        ]
        report = generate_report(results, fmt="html")
        assert "<html" in report
        assert "EB-Reach" in report

    def test_empty_report(self):
        """空结果不应崩溃."""
        from electronbot_benchmark.report import generate_report
        report = generate_report([], fmt="markdown")
        assert "无结果" in report or "无" in report
