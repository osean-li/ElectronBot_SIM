"""ElectronBot Benchmark 评估系统.

对齐 docs/tasks/07-Benchmark 详细设计说明书.

标准化评估体系, 量化不同 AI 策略在 7 个标准任务上的表现.
支持自动化批量评估、结果可视化、排行榜生成.

核心组件:
  - suite.BenchmarkResult:       单次评估结果 dataclass
  - suite.ElectronBotBenchmark:  Benchmark 核心类
  - run:                          CLI 运行脚本
  - report:                       报告生成 (Markdown/HTML)

7 个标准任务:
  EB-Reach / EB-Push / EB-PickPlace / EB-Stack
  EB-Follow / EB-Gesture / EB-VoiceCmd

评估指标:
  - success_rate:           成功率
  - mean_completion_time:   平均完成时间
  - trajectory_smoothness:  轨迹平滑度
  - generalization_gap:     泛化差距 (ID vs OOD)
  - sim2real_gap:           仿真→真机差距 (可选)
"""
from __future__ import annotations

from .suite import BenchmarkResult, ElectronBotBenchmark

__all__ = ["BenchmarkResult", "ElectronBotBenchmark"]
__version__ = "0.2.0"
