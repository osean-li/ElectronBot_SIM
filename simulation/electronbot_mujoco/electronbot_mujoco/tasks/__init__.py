"""ElectronBot Benchmark 任务定义."""
from .base_task import BaseTask
from .reach_task import ReachTask
from .push_task import PushTask
from .wave_task import WaveTask
from .pointat_task import PointAtTask
from .stack_task import StackTask

TASKS = {
    "reach": ReachTask,
    "push": PushTask,
    "wave": WaveTask,
    "pointat": PointAtTask,
    "stack": StackTask,
}
