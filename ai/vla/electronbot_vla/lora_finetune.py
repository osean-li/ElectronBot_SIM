#!/usr/bin/env python3
"""
LoRA 微调脚本

用 ElectronBot 特定指令-动作数据集微调小模型 (Qwen2-1.5B VL)，
提升指令遵循精度。

数据集格式 (JSONL):
  {"image_path": "images/001.jpg", "instruction": "挥手", "joint_angles": [0,0,0,0,80,15]}
"""

import os
import json
import argparse
from typing import Dict, List
import numpy as np

import torch
from transformers import (
    AutoProcessor,
    Qwen2VLForConditionalGeneration,
    TrainingArguments,
    Trainer,
)
from peft import LoraConfig, get_peft_model, TaskType
from datasets import Dataset


def load_dataset(data_path: str) -> Dataset:
    """加载指令-动作数据集"""
    samples = []
    with open(data_path, "r") as f:
        for line in f:
            samples.append(json.loads(line.strip()))

    return Dataset.from_list(samples)


def format_prompt(instruction: str, joint_angles: List[float]) -> str:
    """格式化训练样本"""
    return (
        f"指令: {instruction}\n"
        f"关节角度: {json.dumps(joint_angles)}\n"
        f"输出: {json.dumps({'joint_angles_deg': joint_angles}, ensure_ascii=False)}"
    )


def main():
    parser = argparse.ArgumentParser(description="ElectronBot LoRA 微调")
    parser.add_argument("--model", type=str,
                        default="Qwen/Qwen2-VL-2B-Instruct",
                        help="基础模型")
    parser.add_argument("--data", type=str, required=True,
                        help="训练数据 JSONL 文件")
    parser.add_argument("--output", type=str, default="./lora_output",
                        help="LoRA 权重输出目录")
    parser.add_argument("--epochs", type=int, default=3)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--lr", type=float, default=2e-4)
    parser.add_argument("--lora-r", type=int, default=16)
    parser.add_argument("--lora-alpha", type=int, default=32)
    args = parser.parse_args()

    print(f"[INFO] 加载数据集: {args.data}")
    dataset = load_dataset(args.data)
    print(f"[INFO] 数据集大小: {len(dataset)}")

    print(f"[INFO] 加载基础模型: {args.model}")
    model = Qwen2VLForConditionalGeneration.from_pretrained(
        args.model,
        torch_dtype=torch.float16,
        device_map="auto",
        trust_remote_code=True,
    )

    # LoRA 配置
    lora_config = LoraConfig(
        task_type=TaskType.CAUSAL_LM,
        r=args.lora_r,
        lora_alpha=args.lora_alpha,
        lora_dropout=0.1,
        target_modules=["q_proj", "v_proj", "k_proj", "o_proj"],
    )
    model = get_peft_model(model, lora_config)
    model.print_trainable_parameters()

    # 训练参数
    training_args = TrainingArguments(
        output_dir=args.output,
        num_train_epochs=args.epochs,
        per_device_train_batch_size=args.batch_size,
        gradient_accumulation_steps=2,
        learning_rate=args.lr,
        fp16=True,
        save_strategy="epoch",
        logging_steps=10,
        report_to="none",
    )

    # 训练 (简化: 纯文本 SFT，非多模态)
    trainer = Trainer(
        model=model,
        args=training_args,
        train_dataset=dataset,
        tokenizer=None,  # 需要 processor
    )

    print(f"[INFO] 开始训练...")
    trainer.train()

    # 保存
    model.save_pretrained(args.output)
    print(f"[SUCCESS] LoRA 权重已保存: {args.output}")


if __name__ == "__main__":
    main()
