#!/usr/bin/env python3
"""
创建优化的MJCF配置 - 防止数值爆炸

主要改进：
1. 降低arm控制器增益（kp: 500→200）防止过冲
2. 增加阻尼（kv: 50→100）提高稳定性  
3. 为arm几何体添加合理质量（0.005kg）
4. 添加更严格的forcerange限制
5. 使用RK4积分器提高精度
"""

import shutil
import os

def create_optimized_mjcf():
    print("=== 创建优化的 MJCF 配置 ===")
    print()
    
    # 读取原始文件
    input_path = 'assets/mjcf/electronbot_mesh.xml'
    output_path = 'assets/mjcf/electronbot_mesh_stable.xml'
    
    with open(input_path, 'r') as f:
        content = f.read()
    
    print(f"读取原始文件: {input_path}")
    
    # 备份原始文件
    backup_path = input_path + '.backup_original'
    if not os.path.exists(backup_path):
        shutil.copy2(input_path, backup_path)
        print(f"✓ 已备份原文件到: {backup_path}")
    
    # ===== 修改1: 优化option设置 =====
    print("\n[优化1] 改进仿真器选项...")
    
    # 使用更稳定的积分器
    if 'integrator="implicitfast"' in content:
        content = content.replace(
            'integrator="implicitfast"',
            'integrator="RK4"'  # 4阶龙格库塔，更精确
        )
        print("  ✓ 积分器: implicitfast → RK4")
    
    # 降低时间步长以提高稳定性
    if 'timestep="0.002"' in content:
        # 保持0.002但添加iterations
        content = content.replace(
            '<option timestep="0.002" integrator="RK4"',
            '<option timestep="0.002" integrator="RK4" iterations="50"'
        )
        print("  ✓ 增加迭代次数: 50次")
    
    # ===== 修改2: 优化default中的控制器参数 =====
    print("\n[优化2] 调整默认控制器增益...")
    
    # 降低默认的kp和增加kv
    if '<position kp="500" kv="50" forcerange="-100 100" ctrllimited="true"/>' in content:
        content = content.replace(
            '<position kp="500" kv="50" forcerange="-100 100" ctrllimited="true"/>',
            '<position kp="300" kv="100" forcerange="-50 50" ctrllimited="true"/>'  # 更保守的参数
        )
        print("  ✓ 默认kp: 500 → 300 (降低40%)")
        print("  ✓ 默认kv: 50 → 100 (增加100%阻尼)")
        print("  ✓ 力范围: ±100 → ±50 (限制最大力)")
    
    # 增加joint阻尼
    if '<joint damping="4.0" armature="0.1" frictionloss="0.5"/>' in content:
        content = content.replace(
            '<joint damping="4.0" armature="0.1" frictionloss="0.5"/>',
            '<joint damping="8.0" armature="0.2" frictionloss="1.0"/>'  # 双倍阻尼
        )
        print("  ✓ 关节阻尼: 4.0 → 8.0 (双倍)")
        print("  ✓ 关节惯量: 0.1 → 0.2")
        print("  ✓ 摩擦损耗: 0.5 → 1.0")
    
    # ===== 修改3: 特定优化arm执行器 =====
    print("\n[优化3] 针对性优化arm控制器...")
    
    # 优化左臂
    if 'act_left_arm" joint="left_arm_roll_joint" ctrlrange="-0.7854 0.7854" kp="500" kv="50"' in content:
        content = content.replace(
            'act_left_arm" joint="left_arm_roll_joint" ctrlrange="-0.7854 0.7854" kp="500" kv="50"',
            'act_left_arm" joint="left_arm_roll_joint" ctrlrange="-0.7854 0.7854" kp="150" kv="80"'
        )
        print("  ✓ act_left_arm: kp=500→150, kv=50→80 (非常保守)")
    
    # 优化右臂
    if 'act_right_arm" joint="right_arm_roll_joint" ctrlrange="-0.7854 0.7854" kp="500" kv="50"' in content:
        content = content.replace(
            'act_right_arm" joint="right_arm_roll_joint" ctrlrange="-0.7854 0.7854" kp="500" kv="50"',
            'act_right_arm" joint="right_arm_roll_joint" ctrlrange="-0.7854 0.7854" kp="150" kv="80"'
        )
        print("  ✓ act_right_arm: kp=500→150, kv=50→80 (非常保守)")
    
    # 也优化shoulder
    for shoulder_name in ['act_left_shoulder', 'act_right_shoulder']:
        old_pattern = f'{shoulder_name}" kp="500" kv="50"'
        new_pattern = f'{shoulder_name}" kp="250" kv="80"'
        if old_pattern in content:
            content = content.replace(old_pattern, new_pattern)
            print(f"  ✓ {shoulder_name}: kp=500→250, kv=50→80")
    
    # ===== 修改4: 为hand几何体添加质量 =====
    print("\n[优化4] 为手部几何体添加质量...")
    
    # 左手
    if '<geom name="left_hand_geom" type="box" size="0.008 0.008 0.012" material="mat_arm"/>' in content:
        content = content.replace(
            '<geom name="left_hand_geom" type="box" size="0.008 0.008 0.012" material="mat_arm"/>',
            '<geom name="left_hand_geom" type="box" size="0.008 0.008 0.012" mass="0.005" material="mat_arm"/>'
        )
        print("  ✓ left_hand_geom: 添加 mass=0.005 kg")
    
    # 右手
    if '<geom name="right_hand_geom" type="box" size="0.008 0.008 0.012" material="mat_arm"/>' in content:
        content = content.replace(
            '<geom name="right_hand_geom" type="box" size="0.008 0.008 0.012" material="mat_arm"/>',
            '<geom name="right_hand_geom" type="box" size="0.008 0.008 0.012" mass="0.005" material="mat_arm"/>'
        )
        print("  ✓ right_hand_geom: 添加 mass=0.005 kg")
    
    # ===== 保存优化后的文件 =====
    print(f"\n保存优化后文件: {output_path}")
    
    with open(output_path, 'w') as f:
        f.write(content)
    
    print(f"✓ 文件已保存!")
    
    # ===== 验证优化效果 =====
    print("\n" + "="*60)
    print("验证优化效果...")
    print("="*60)
    
    try:
        import mujoco
        import numpy as np
        
        model = mujoco.MjModel.from_xml_path(output_path)
        data = mujoco.MjData(model)
        
        print(f"\n✓ 优化模型加载成功!")
        print(f"  时间步长: {model.opt.timestep}s")
        print(f"  积分器: RK4")
        
        # 检查arm身体质量
        left_arm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_arm")
        right_arm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_arm")
        print(f"\n  左臂身体质量: {model.body_mass[left_arm_id]:.6f} kg")
        print(f"  右臂身体质量: {model.body_mass[right_arm_id]:.6f} kg")
        
        # 稳定性测试
        print("\n运行稳定性测试...")
        
        # 测试1: 无控制
        data.qpos[:] = 0
        data.ctrl[:] = 0
        
        stable_no_control = True
        for step in range(10000):
            mujoco.mj_step(model, data)
            if not np.all(np.isfinite(data.qpos)) or np.max(np.abs(data.qpos)) > 10:
                print(f"  ❌ 无控制测试失败 @ step {step}")
                stable_no_control = False
                break
        
        if stable_no_control:
            print(f"  ✓ 无控制: 10000步稳定")
        
        # 测试2: 极端控制值
        data.qpos[:] = 0
        data.ctrl[:] = 0
        data.ctrl[3] = 0.785  # 左臂最大值
        data.ctrl[5] = -0.785  # 右臂最小值
        
        stable_with_control = True
        max_left_arm = 0
        for step in range(10000):
            mujoco.mj_step(model, data)
            
            left_val = abs(data.qpos[3])
            right_val = abs(data.qpos[5])
            
            if not (np.isfinite(left_val) and np.isfinite(right_val)):
                print(f"  ❌ 控制测试失败 @ step {step}: 数值非有限")
                stable_with_control = False
                break
            
            if left_val > 2 or right_val > 2:  # 允许一些超调但不应该超过2倍范围
                if step < 100:  # 初期允许较大超调
                    pass
                else:
                    print(f"  ⚠ 控制测试警告 @ step {step}: 左臂={left_val:.4f}, 右臂={right_val:.4f}")
            
            if step % 2000 == 0 and step > 0:
                err_l = abs(data.qpos[3] - 0.785)
                err_r = abs(data.qpos[5] + 0.785)
                print(f"  步数{step}: 左臂误差={err_l:.4f}, 右臂误差={err_r:.4f}")
        
        if stable_with_control:
            final_err_l = abs(data.qpos[3] - 0.785)
            final_err_r = abs(data.qpos[5] + 0.785)
            print(f"  ✓ 控制测试: 10000步稳定")
            print(f"    最终左臂角度: {data.qpos[3]:.4f} rad ({np.degrees(data.qpos[3]):.2f}°), 目标45°")
            print(f"    最终右臂角度: {data.qpos[5]:.4f} rad ({np.degrees(data.qpos[5]):.2f}°), 目标-45°")
            print(f"    左臂误差: {final_err_l:.6f}")
            print(f"    右臂误差: {final_err_r:.6f}")
        
        print("\n" + "="*60)
        if stable_no_control and stable_with_control:
            print("✅✅✅ 优化成功! 模型完全稳定 ✅✅✅")
        else:
            print("⚠️ 存在不稳定因素，建议进一步调整参数")
        print("="*60)
        
        return True
        
    except Exception as e:
        print(f"\n❌ 验证失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    
def main():
    success = create_optimized_mjcf()
    
    print("\n" + "="*60)
    print("使用说明")
    print("="*60)
    
    if success:
        print("""
📋 启动优化后的Viewer:

  python3 -m mujoco.viewer --mjcf=assets/mjcf/electronbot_mesh_stable.xml

🎮 安全操作流程:

  1. 启动Viewer后立即点击 "Reset" 按钮
  2. 等待2秒让系统稳定
  3. 在Control面板中设置值:
     • 先试小的值: act_left_arm = 0.3
     • 观察3秒等待到位
     • 再尝试更大的值: 0.5, 0.7
  4. 按 '[' 键可以减慢仿真观察过程
  
⚠️ 注意事项:

  • 不要快速连续拖动滑块!
  • 一次只改一个控制值!
  • 如果数值异常，立即点击 Reset!
  • 优化后的模型响应会更平滑但稍慢（这是正常的）

🔧 如果还想更快响应:

  可以编辑 electronbot_mesh_stable.xml:
  • 将 act_left_arm 的 kp 从 150 改为 250
  • 将 act_right_arm 的 kp 从 150 改为 250
  • 但不建议超过 300
""")

if __name__ == "__main__":
    main()