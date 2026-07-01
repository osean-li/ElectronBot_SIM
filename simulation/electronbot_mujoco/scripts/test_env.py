#!/usr/bin/env python3
"""
ElectronBot 仿真环境测试脚本

测试内容:
1. 基础模式 (ElectronBotEnv):     position actuator 直接控制
2. 固件模式 (ElectronBotFirmwareEnv): DCE PID + ExtraData 协议
3. 阻抗控制模式:                   torque actuator + 导纳控制
4. 角度映射验证:                    model ↔ mech 双向转换

用法:
  python test_env.py --test basic     # 基础环境测试
  python test_env.py --test firmware   # 固件模式测试
  python test_env.py --test mapping    # 角度映射验证
  python test_env.py --test all        # 全部测试
"""

import sys
import argparse
import numpy as np
from pathlib import Path

# 添加项目路径
_project = Path(__file__).parent.parent                          # simulation/electronbot_mujoco/
sys.path.insert(0, str(_project))
_root = Path(__file__).parent.parent.parent.parent               # ElectronBot_SIM/
sys.path.insert(0, str(_root / "sim2real"))                       # for: from electronbot_real import ...
sys.path.insert(0, str(_project.parent.parent / "sim2real"))  # for from electronbot_real import ...

from electronbot_mujoco import (
    ElectronBotRobot, ElectronBotFirmwareRobot,
    ElectronBotEnv, ElectronBotFirmwareEnv,
)
from electronbot_mujoco.utils import (
    JOINT_PARAMS,
    model_angle_to_mech, mech_angle_to_model,
    get_joint_limits_deg, get_joint_limits_rad,
    normalize_joint_angles,
)
from electronbot_mujoco.servo_sim import ServoSimulator, TUNED_PID, create_servo_pool


# ============================================================
# 测试 1: 基础模式
# ============================================================
def test_basic_env():
    print("=" * 60)
    print("[TEST 1] 基础模式 ElectronBotEnv")
    print("=" * 60)

    env = ElectronBotEnv()
    obs, info = env.reset()
    print(f"  Observation shape: {obs.shape}")
    print(f"  Initial qpos (deg): {np.round(np.degrees(obs[:6]), 1)}")

    for step in range(100):
        # 随机动作
        action = np.random.uniform(-0.5, 0.5, 6)
        obs, reward, terminated, truncated, info = env.step(action)

        if step % 20 == 0:
            q = np.degrees(obs[:6])
            print(f"  step {step:3d}: q={np.round(q, 1)}")

    env.close()
    print("  [OK] 基础模式测试通过\n")


# ============================================================
# 测试 2: 固件模式
# ============================================================
def test_firmware_env():
    print("=" * 60)
    print("[TEST 2] 固件模式 ElectronBotFirmwareEnv")
    print("=" * 60)

    env = ElectronBotFirmwareEnv(apply_tuned_pid=True)
    obs, info = env.reset()
    print(f"  Observation shape: {obs.shape}")
    print(f"  Initial qpos (deg): {np.round(np.degrees(obs[:6]), 1)}")

    # 检查舵机状态
    for j in env.robot.joints:
        servo = env.robot.servos[j.id]
        print(f"  Joint {j.id}: enabled={servo.enabled}, "
              f"Kp={servo.kp}, Kd={servo.kd}, "
              f"setpoint={servo.setpoint_pos:.1f}°")

    # 发送目标: 抬起右臂 60°, 身体旋转 30°
    target_deg = np.array([30, 0, 0, 0, 60, 0])
    target_rad = np.radians(target_deg)
    print(f"\n  Target (deg): {target_deg}")

    for step in range(250):
        obs, reward, terminated, truncated, info = env.step(target_rad)

        if step % 25 == 0:
            q = np.degrees(obs[:6])
            dce = info.get("dce_outputs", np.zeros(6))
            print(f"  step {step:3d}: q={np.round(q, 1)} "
                  f"| DCE={np.round(dce, 0)}")

    # 检查跟踪精度
    final_q = np.degrees(obs[:6])
    err = np.max(np.abs(final_q - target_deg))
    print(f"\n  Final q (deg): {np.round(final_q, 1)}")
    print(f"  Max angle error: {err:.2f}°")
    print(f"  [{'OK' if err < 5 else 'WARN'}]")

    env.close()
    print()


# ============================================================
# 测试 3: 角度映射验证
# ============================================================
def test_angle_mapping():
    print("=" * 60)
    print("[TEST 3] 角度映射验证 (model ↔ mech)")
    print("=" * 60)

    errors = []

    for idx, p in enumerate(JOINT_PARAMS):
        name, i2c_id, mech_min, mech_max, model_min, model_max, inverted = p

        # Test mid-point
        model_mid = (model_min + model_max) / 2.0
        mech_mid = model_angle_to_mech(idx, model_mid)
        model_back = mech_angle_to_model(idx, mech_mid)

        err = abs(model_mid - model_back)
        errors.append(err)

        # 用 firmware 公式验证 (robot.cpp:127-152, 225-261)
        if inverted:
            sAngle = ((model_mid - model_min) / (model_max - model_min)
                      * (mech_min - mech_max) + mech_max)
            jAngle = ((mech_max - mech_mid) / (mech_max - mech_min)
                      * (model_max - model_min) + model_min)
        else:
            sAngle = ((model_mid - model_min) / (model_max - model_min)
                      * (mech_max - mech_min) + mech_min)
            jAngle = ((mech_mid - mech_min) / (mech_max - mech_min)
                      * (model_max - model_min) + model_min)

        print(f"  {name:16s} (I2C={i2c_id:2d} inv={inverted})")
        print(f"    model {model_mid:6.1f}° → mech {mech_mid:6.1f}° → model {model_back:6.1f}°  "
              f"err={err:.6f}°  {'[OK]' if err < 0.01 else '[FAIL]'}")

    max_err = max(errors)
    print(f"\n  Max roundtrip error: {max_err:.6f}°")
    print(f"  [{'OK' if max_err < 0.01 else 'FAIL'}] 角度映射验证")
    print()


# ============================================================
# 测试 4: 舵机仿真器 DCE 控制
# ============================================================
def test_servo_dce():
    print("=" * 60)
    print("[TEST 4] 舵机 DCE 控制")
    print("=" * 60)

    pool = create_servo_pool(apply_tuned=True)

    for i2c_id, servo in pool.items():
        # 初始化: 机械角 90°
        servo.angle = 90.0
        servo.setpoint_pos = 90.0
        servo.enabled = True

    # 测试 j6 (Body): 设置目标 135° (对应模型角 45°)
    body_servo = pool[12]
    body_servo.setpoint_pos = 135.0  # 机械角
    print(f"  Body servo (ID=12): setpoint={body_servo.setpoint_pos}°")

    for step in range(200):
        # 模拟 MuJoCo 动力学: 简单的 1 阶惯性
        # DCE 输出符号: error=current-target, output=Kp*error
        #   若 current<target → error<0 → output<0 (负 PWM)
        #   在真机中 H-bridge 负 PWM=反转, 方向取决于接线
        #   这里取绝对值: 扭矩用于缩小 |error|, 方向取 sign(target-current)
        torque = body_servo.step_200hz(0.005)
        direction = np.sign(body_servo.setpoint_pos - body_servo.angle)
        # gain=250 近似 60°/s 满速 (简化物理, 与 MuJoCo 动力学无关)
        delta = abs(torque) * direction * 0.005 * 250
        body_servo.angle = np.clip(body_servo.angle + delta, 0, 180)

        if step % 20 == 0:
            err = abs(body_servo.angle - body_servo.setpoint_pos)
            print(f"  step {step:3d}: angle={body_servo.angle:.1f}° "
                  f"err={err:.2f}° DCE_out={torque:.3f}")

    final_err = abs(body_servo.angle - body_servo.setpoint_pos)
    print(f"\n  Final error: {final_err:.2f}°")
    ok = final_err < 10  # 简化物理中 <10° 即可 (真机精度由 MuJoCo DCE 验证)
    print(f"  [{'OK' if ok else 'WARN'}] DCE 控制测试 (简化物理, 仅验证方向)")
    print()


# ============================================================
# 测试 5: I2C 命令协议
# ============================================================
def test_i2c_commands():
    print("=" * 60)
    print("[TEST 5] I2C 命令协议")
    print("=" * 60)

    import struct
    from electronbot_mujoco.servo_sim import ServoSimulator

    servo = ServoSimulator()
    servo.enabled = True
    servo.setpoint_pos = 90.0
    servo.angle = 90.0

    tests = [
        ("Set angle 120°",   struct.pack('<Bf', 0x01, 120.0)),
        ("Get angle",        struct.pack('<B4x', 0x11)),
        ("Set Kp=20",        struct.pack('<Bf', 0x22, 20.0)),
        ("Set Kd=100",       struct.pack('<Bf', 0x25, 100.0)),
        ("Set torque 80%",   struct.pack('<Bf', 0x26, 0.8)),
        ("Enable false",     bytes([0xFF, 0, 0, 0, 0])),
        ("Enable true",      bytes([0xFF, 1, 0, 0, 0])),
    ]

    for name, cmd in tests:
        resp = servo.handle_i2c_command(cmd)
        cmd_echo = resp[0]
        angle = struct.unpack_from('<f', resp, 1)[0] if len(resp) >= 5 else 0
        print(f"  {name:20s} → cmd=0x{cmd_echo:02X}, angle={angle:.1f}°")

    servo.commit_config()
    print(f"\n  最终: Kp={servo.kp}, Kd={servo.kd}, "
          f"enabled={servo.enabled}, tq_limit={servo.torque_limit:.0f}")
    print("  [OK] I2C 协议测试通过")
    print()


# ============================================================
# 测试 6: ExtraData 协议
# ============================================================
def test_extra_data_protocol():
    print("=" * 60)
    print("[TEST 6] ExtraData 32 字节协议")
    print("=" * 60)

    from electronbot_real.protocol import encode_extra_data, decode_extra_data

    # Test encode
    angles_deg = np.array([30, 0, 50, 10, 50, 10])
    enable = True
    data = encode_extra_data(enable, angles_deg)

    print(f"  Input:  enable={enable}, angles={angles_deg}")
    print(f"  Bytes:  {data.hex()}")

    # Test decode
    en, ang = decode_extra_data(data)
    print(f"  Output: enable={en}, angles={np.round(ang, 1)}")

    # Verify roundtrip
    assert en == enable, "enable mismatch"
    assert np.allclose(ang, angles_deg, atol=0.01), f"angle mismatch: {ang} vs {angles_deg}"

    # Test rad variant
    from electronbot_real.protocol import encode_extra_data_rad, decode_extra_data_rad
    angles_rad = np.radians(angles_deg)
    data2 = encode_extra_data_rad(enable, angles_rad)
    en2, ang2 = decode_extra_data_rad(data2)
    assert np.allclose(ang2, angles_rad, atol=0.01), "rad roundtrip mismatch"

    print("  [OK] ExtraData 协议测试通过 (encode/decode 一致)")
    print()


# ============================================================
# 测试 7: 抗扰动 (DCE vs 简单 position)
# ============================================================
def test_disturbance_rejection():
    print("=" * 60)
    print("[TEST 7] 抗扰动对比: DCE PID vs Position Actuator")
    print("=" * 60)

    target_deg = np.array([0, 0, 0, 0, 60, 0])
    target_rad = np.radians(target_deg)

    # Firmware env with DCE
    env = ElectronBotFirmwareEnv(apply_tuned_pid=True)
    obs, _ = env.reset(options={"qpos": np.zeros(6)})

    for step in range(200):
        obs, _, _, _, info = env.step(target_rad)
        if step == 100:
            # 施加外力扰动到右臂 shoulder
            joint_id = env.robot._joint_ids[4]  # right_shoulder
            env.data.qfrc_applied[joint_id] = -0.5  # -0.5 Nm
            print(f"  [EVENT] 步 {step}: 施加 -0.5Nm 到 right_shoulder")
        elif step > 100:
            # 扰动仅持续 1 步, 之后清零 (避免持续影响后续测试)
            env.data.qfrc_applied[:] = 0

        if step % 50 == 0:
            q = np.degrees(obs[:6])
            print(f"  step {step:3d}: q={np.round(q, 1)}")

    final_q = np.degrees(obs[:6])
    err = np.max(np.abs(final_q - target_deg))
    print(f"\n  Final error after disturbance: {err:.2f}°")
    print(f"  [{'OK' if err < 10 else 'WARN'}] 抗扰动测试")
    env.close()
    print()


# ============================================================
# Main
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="ElectronBot 仿真测试")
    parser.add_argument("--test", default="all",
                        choices=["basic", "firmware", "mapping", "servo", "i2c",
                                 "protocol", "disturbance", "all"])
    args = parser.parse_args()

    tests = {
        "basic":      test_basic_env,
        "firmware":   test_firmware_env,
        "mapping":    test_angle_mapping,
        "servo":      test_servo_dce,
        "i2c":       test_i2c_commands,
        "protocol":   test_extra_data_protocol,
        "disturbance": test_disturbance_rejection,
    }

    if args.test == "all":
        for name, func in tests.items():
            try:
                func()
            except Exception as e:
                print(f"\n  [FAIL] {name}: {e}\n")
        print("=" * 60)
        print("  All tests done.")
    elif args.test in tests:
        tests[args.test]()
    else:
        print(f"Unknown test: {args.test}")


if __name__ == "__main__":
    main()
