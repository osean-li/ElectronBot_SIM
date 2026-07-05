"""Benchmark 报告生成器.

对齐 docs/tasks/07-Benchmark §3.

生成 Markdown / HTML 格式的评估报告,
包含: 结果矩阵、指标明细、排名、可视化建议.
"""
from __future__ import annotations

import logging
from typing import List, Optional

from .suite import BenchmarkResult

logger = logging.getLogger("electronbot_benchmark.report")


def generate_report(results: List[BenchmarkResult], fmt: str = "markdown") -> str:
    """生成评估报告.

    参数:
        results: BenchmarkResult 列表
        fmt:     "markdown" 或 "html"

    返回:
        str: 报告内容
    """
    if fmt == "markdown":
        return _generate_markdown(results)
    elif fmt == "html":
        return _generate_html(results)
    else:
        raise ValueError(f"不支持的格式: {fmt}. 可选: markdown / html")


def _generate_markdown(results: List[BenchmarkResult]) -> str:
    """生成 Markdown 报告."""
    if not results:
        return "# Benchmark 报告\n\n无结果.\n"

    tasks = sorted(set(r.task_name for r in results))
    algos = sorted(set(r.algorithm for r in results))

    lines = [
        "# ElectronBot Benchmark 评估报告",
        "",
        f"**评估时间**: {__import__('time').strftime('%Y-%m-%d %H:%M:%S')}",
        f"**评估组合数**: {len(results)}",
        f"**任务数**: {len(tasks)} | **算法数**: {len(algos)}",
        "",
        "## 1. 成功率矩阵",
        "",
        "| Task | " + " | ".join(algos) + " |",
        "|------|" + "|".join(["------"] * len(algos)) + "|",
    ]

    for task in tasks:
        row = f"| {task} "
        for algo in algos:
            matched = [r for r in results if r.task_name == task and r.algorithm == algo]
            if matched:
                sr = matched[0].success_rate
                row += f"| {sr*100:.0f}% "
            else:
                row += "| — "
        lines.append(row + "|")

    lines.extend([
        "",
        "## 2. 指标明细",
        "",
        "| Task | Algorithm | 成功率 | 平均时间(s) | 平滑度 | 泛化差距 |",
        "|------|-----------|:------:|:---------:|:------:|:--------:|",
    ])

    for r in sorted(results, key=lambda x: (x.task_name, x.algorithm)):
        lines.append(
            f"| {r.task_name} | {r.algorithm} | {r.success_rate*100:.1f}% | "
            f"{r.mean_completion_time:.2f} | {r.trajectory_smoothness:.4f} | "
            f"{r.generalization_gap:.2f} |"
        )

    # 排行榜 (按成功率排序)
    lines.extend([
        "",
        "## 3. 排行榜 (按成功率降序)",
        "",
        "| 排名 | Task | Algorithm | 成功率 |",
        "|:----:|------|-----------|:------:|",
    ])
    ranked = sorted(results, key=lambda x: x.success_rate, reverse=True)
    for i, r in enumerate(ranked, 1):
        lines.append(f"| {i} | {r.task_name} | {r.algorithm} | {r.success_rate*100:.1f}% |")

    # 验收标准检查
    lines.extend([
        "",
        "## 4. 验收标准检查",
        "",
    ])
    bc_reach = [r for r in results if r.algorithm == "bc" and "reach" in r.task_name.lower()]
    if bc_reach:
        sr = bc_reach[0].success_rate
        status = "✅ 合格" if sr >= 0.70 else "❌ 不合格"
        lines.append(f"- BC @ EB-Reach 成功率 > 70%: **{sr*100:.1f}%** {status}")

    ppo_pp = [r for r in results if r.algorithm == "ppo" and "pickplace" in r.task_name.lower()]
    if ppo_pp:
        sr = ppo_pp[0].success_rate
        status = "✅ 合格" if sr >= 0.75 else "❌ 不合格"
        lines.append(f"- PPO @ EB-PickPlace 成功率 > 75%: **{sr*100:.1f}%** {status}")

    lines.append("")
    return "\n".join(lines)


def _generate_html(results: List[BenchmarkResult]) -> str:
    """生成 HTML 报告."""
    if not results:
        return "<html><body><h1>Benchmark 报告</h1><p>无结果.</p></body></html>"

    tasks = sorted(set(r.task_name for r in results))
    algos = sorted(set(r.algorithm for r in results))

    html = [
        "<!DOCTYPE html>",
        "<html lang='zh-CN'>",
        "<head><meta charset='UTF-8'><title>ElectronBot Benchmark 报告</title>",
        "<style>",
        "body { font-family: -apple-system, sans-serif; margin: 40px; }",
        "table { border-collapse: collapse; width: 100%; margin: 20px 0; }",
        "th, td { border: 1px solid #ddd; padding: 8px 12px; text-align: center; }",
        "th { background-color: #f5f5f5; font-weight: 600; }",
        "tr:hover { background-color: #f9f9f9; }",
        ".success-high { color: #2e7d32; font-weight: bold; }",
        ".success-mid { color: #f57c00; }",
        ".success-low { color: #c62828; }",
        "</style></head>",
        "<body>",
        "<h1>ElectronBot Benchmark 评估报告</h1>",
        f"<p>评估组合数: {len(results)} | 任务数: {len(tasks)} | 算法数: {len(algos)}</p>",
        "<h2>1. 成功率矩阵</h2>",
        "<table><thead><tr><th>Task</th>",
    ]
    for a in algos:
        html.append(f"<th>{a}</th>")
    html.append("</tr></thead><tbody>")

    for task in tasks:
        html.append(f"<tr><td><strong>{task}</strong></td>")
        for algo in algos:
            matched = [r for r in results if r.task_name == task and r.algorithm == algo]
            if matched:
                sr = matched[0].success_rate
                cls = "success-high" if sr >= 0.8 else ("success-mid" if sr >= 0.5 else "success-low")
                html.append(f"<td class='{cls}'>{sr*100:.0f}%</td>")
            else:
                html.append("<td>—</td>")
        html.append("</tr>")
    html.append("</tbody></table>")

    # 指标明细
    html.append("<h2>2. 指标明细</h2>")
    html.append("<table><thead><tr><th>Task</th><th>Algorithm</th><th>成功率</th>"
                "<th>平均时间(s)</th><th>平滑度</th></tr></thead><tbody>")
    for r in sorted(results, key=lambda x: (x.task_name, x.algorithm)):
        html.append(
            f"<tr><td>{r.task_name}</td><td>{r.algorithm}</td>"
            f"<td>{r.success_rate*100:.1f}%</td>"
            f"<td>{r.mean_completion_time:.2f}</td>"
            f"<td>{r.trajectory_smoothness:.4f}</td></tr>"
        )
    html.append("</tbody></table>")

    html.append("</body></html>")
    return "\n".join(html)


def main():
    """CLI 入口: 从 JSON 文件生成报告."""
    import argparse
    parser = argparse.ArgumentParser(description="生成 Benchmark 报告")
    parser.add_argument("input", help="结果 JSON 文件路径")
    parser.add_argument("--format", choices=["markdown", "html"], default="markdown")
    parser.add_argument("--output", default=None, help="报告输出路径")
    args = parser.parse_args()

    bench = ElectronBotBenchmark()
    results = bench.load_results(args.input)
    report = generate_report(results, fmt=args.format)

    if args.output:
        from pathlib import Path
        Path(args.output).write_text(report, encoding="utf-8")
        print(f"报告已保存: {args.output}")
    else:
        print(report)


if __name__ == "__main__":
    main()
