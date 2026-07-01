#!/usr/bin/env python3
"""
py_trees 行为树编排模块

将感知、决策、执行模块封装为 Behavior Tree 节点，
编排复杂行为序列。

节点类型:
  - PerceiveNode: 调用感知管线 (RGB → 目标检测/物体识别)
  - VLANode: 调用 VLA 推理 (image + prompt → joint angles)
  - ExecuteNode: 发布关节指令到 MuJoCo/实体机器人
  - ConditionNode: 条件判断 (has_target, at_target, etc.)
  - Selector: 任务优先级选择
  - Sequence: 顺序执行

使用方式:
  tree = build_find_and_touch_tree(robot, vla_backend)
  tree.tick()
"""

import py_trees
from typing import Dict, Any, Optional
import numpy as np


class PerceiveNode(py_trees.behaviour.Behaviour):
    """感知节点: 检测目标物体"""

    def __init__(self, name: str, camera, pipeline, target_color: str = "red"):
        super().__init__(name)
        self.camera = camera
        self.pipeline = pipeline
        self.target_color = target_color

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.detected_objects = []
        self.blackboard.target_found = False

    def update(self):
        rgb = self.camera.get_rgb_from_robot(self.robot)
        objects = self.pipeline.detect_red_objects(rgb)
        self.blackboard.detected_objects = objects
        self.blackboard.target_found = len(objects) > 0
        return py_trees.common.Status.SUCCESS


class VLANode(py_trees.behaviour.Behaviour):
    """VLA 决策节点: 图像 + 语言 → 关节动作"""

    def __init__(self, name: str, vla_model, task_prompt: str):
        super().__init__(name)
        self.vla_model = vla_model
        self.task_prompt = task_prompt

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        rgb = self.blackboard.get("current_rgb", None)
        if rgb is None:
            return py_trees.common.Status.FAILURE

        angles = self.vla_model.predict(rgb, self.task_prompt)
        self.blackboard.target_angles = angles
        return py_trees.common.Status.SUCCESS


class ExecuteNode(py_trees.behaviour.Behaviour):
    """执行节点: 发送关节指令到机器人"""

    def __init__(self, name: str, robot, duration: float = 3.0):
        super().__init__(name)
        self.robot = robot
        self.duration = duration
        self._elapsed = 0.0

    def initialise(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        self._elapsed = 0.0

    def update(self):
        target = self.blackboard.get("target_angles", np.zeros(6))
        self.robot.send_position_command(target)
        self.robot.step()
        self._elapsed += 0.02

        if self._elapsed >= self.duration:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class HasTargetCondition(py_trees.behaviour.Behaviour):
    """条件节点: 是否检测到目标"""

    def update(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        return (
            py_trees.common.Status.SUCCESS
            if self.blackboard.get("target_found", False)
            else py_trees.common.Status.FAILURE
        )


class AtTargetCondition(py_trees.behaviour.Behaviour):
    """条件节点: 是否到达目标位置"""

    def __init__(self, name: str, threshold: float = 0.03):
        super().__init__(name)
        self.threshold = threshold

    def update(self):
        self.blackboard = py_trees.blackboard.Blackboard()
        current_pos = self.blackboard.get("ee_position", np.zeros(3))
        target_pos = self.blackboard.get("target_position", np.array([100, 100, 100]))
        dist = np.linalg.norm(current_pos - target_pos)
        return (
            py_trees.common.Status.SUCCESS
            if dist < self.threshold
            else py_trees.common.Status.FAILURE
        )


# ---- 预定义行为树 ----

def build_find_and_touch_tree(
    robot,
    camera,
    pipeline,
    vla_model,
) -> py_trees.trees.BehaviourTree:
    """
    构建"寻找红色球并触碰"行为树

    序列:
      Perceive → VLANode("approach") → Execute → VLANode("touch") → Execute
    """
    root = py_trees.composites.Sequence("FindAndTouch", memory=True)

    perceive = PerceiveNode("Perceive", camera, pipeline, "red")
    vla_approach = VLANode("VLA_Approach", vla_model, "朝红色球移动")
    execute_approach = ExecuteNode("Execute_Approach", robot, duration=2.0)
    vla_touch = VLANode("VLA_Touch", vla_model, "触碰红色球")
    execute_touch = ExecuteNode("Execute_Touch", robot, duration=1.0)

    root.add_children([
        perceive,
        vla_approach,
        execute_approach,
        vla_touch,
        execute_touch,
    ])

    return py_trees.trees.BehaviourTree(root)


def build_emotion_sequence_tree(
    robot,
    camera,
    pipeline,
    vla_model,
) -> py_trees.trees.BehaviourTree:
    """
    构建情绪化行为序列

    Selector: 根据用户指令选择不同情绪
      - "开心" → wave + 跳动
      - "好奇" → 四处张望
      - "疲倦" → 缓慢点头
    """
    root = py_trees.composites.Selector("EmotionSelector", memory=True)

    happy_seq = py_trees.composites.Sequence("Happy")
    happy_seq.add_children([
        VLANode("VLA_Happy", vla_model, "开心地挥手"),
        ExecuteNode("Execute_Happy", robot, duration=3.0),
    ])

    curious_seq = py_trees.composites.Sequence("Curious")
    curious_seq.add_children([
        VLANode("VLA_Curious", vla_model, "好奇地四处张望"),
        ExecuteNode("Execute_Curious", robot, duration=3.0),
    ])

    tired_seq = py_trees.composites.Sequence("Tired")
    tired_seq.add_children([
        VLANode("VLA_Tired", vla_model, "疲倦地慢慢点头"),
        ExecuteNode("Execute_Tired", robot, duration=3.0),
    ])

    root.add_children([happy_seq, curious_seq, tired_seq])
    return py_trees.trees.BehaviourTree(root)
