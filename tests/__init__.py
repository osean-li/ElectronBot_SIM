"""ElectronBot_SIM 测试套件.

测试覆盖:
  - test_env:         环境基础功能 (reset/step/action_bounds/physics_stable)
  - test_actions:     动作系统 (线性插值/安全裁剪/预设动作/序列)
  - test_mcp_bridge:  MCP 工具 (12 个工具/tools/call 格式/转换)
  - test_sensors:     传感器 (摄像头/关节/接触)
  - test_tasks:       AI 训练任务 (7 个标准任务/奖励/成功判定)
  - test_benchmark:   Benchmark 系统 (评估/结果/报告)

运行:
  pytest tests/ -v
  pytest tests/test_env.py -v
"""
