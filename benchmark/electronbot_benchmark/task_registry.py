"""ElectronBot Benchmark 任务注册中心"""
from electronbot_mujoco.tasks import TASKS as ALL_TASKS

BENCHMARK_CONFIG = {
    "reach": {"arm": "right", "episodes": 50},
    "push": {"arm": "right", "episodes": 50},
    "wave": {"arm": "right", "episodes": 30},
    "pointat": {"arm": "right", "episodes": 50},
    "stack": {"arm": "right", "episodes": 50},
}

def get_task_list():
    return list(ALL_TASKS.keys())

def get_task_config(task_name: str):
    return BENCHMARK_CONFIG.get(task_name, {})

def build_task_env(task_name: str, **kwargs):
    """构建指定任务的 Gym 环境"""
    if task_name not in ALL_TASKS:
        raise ValueError(f"未知任务: {task_name}")
    config = BENCHMARK_CONFIG.get(task_name, {})
    merged = {**config, **kwargs}
    return ALL_TASKS[task_name](**merged)
