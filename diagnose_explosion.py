#!/usr/bin/env python3
"""
诊断arm关节数值爆炸问题 (9.54e+21)
"""

import mujoco
import numpy as np

def diagnose():
    print("=== 诊断 arm 关节数值爆炸 ===")
    print("观察到的问题:")
    print("  • left_arm_roll_joint = 9.54e+21 (应该是±0.785)")
    print("  • right_arm_roll_joint = 9.54e+21 (应该是±0.785)")
    print()
    
    # 测试原始模型和修复后的模型
    models_to_test = [
        ('原始模型', 'assets/mjcf/electronbot_mesh.xml'),
        ('修复后的模型', 'assets/mjcf/electronbot_mesh_fixed2.xml'),
    ]
    
    for model_name, xml_path in models_to_test:
        print(f"\n{'='*60}")
        print(f"测试 {model_name}: {xml_path}")
        print('='*60)
        
        try:
            model = mujoco.MjModel.from_xml_path(xml_path)
            data = mujoco.MjData(model)
            
            print(f"\n模型加载成功")
            print(f"  时间步长 dt: {model.opt.timestep:.6f} 秒")
            print(f"  左臂身体质量: {model.body_mass[5]:.6f}")
            print(f"  右臂身体质量: {model.body_mass[7]:.6f}")
            
            # 检查执行器参数
            print(f"\n执行器配置:")
            for i in range(model.nu):
                act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                if act_name and 'arm' in act_name:
                    kp = model.actuator_gainprm[i][0]
                    kv = model.actuator_gainprm[i][1]
                    ctrl_range = f"[{model.actuator_ctrlrange[i*2]:.4f}, {model.actuator_ctrlrange[i*2+1]:.4f}]"
                    print(f"  {act_name}: kp={kp:.0f}, kv={kv:.0f}, range={ctrl_range}")
            
            # 运行稳定性测试
            print(f"\n运行稳定性测试...")
            
            # 测试1: 无控制输入，看是否会发散
            data.qpos[:] = 0
            data.ctrl[:] = 0
            mujoco.mj_forward(model, data)
            
            stable = True
            max_qpos = np.max(np.abs(data.qpos))
            
            print(f"  初始状态: max|qpos| = {max_qpos:.10f}")
            
            for step in range(10000):
                mujoco.mj_step(model, data)
                
                # 检查是否爆炸
                current_max = np.max(np.abs(data.qpos))
                if current_max > 100 or not np.isfinite(current_max):
                    print(f"  ❌ 步数 {step}: 数值爆炸! qpos[{np.argmax(data.qpos)}] = {data.qpos[np.argmax(data.qpos)]}")
                    
                    # 打印所有关节值
                    for j in range(model.nq):
                        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                        print(f"     {jname}: {data.qpos[j]:.6e}")
                    stable = False
                    break
                    
                # 每1000步打印一次
                if step % 1000 == 0 and step > 0:
                    print(f"  步数 {step}: max|qpos| = {current_max:.10f}")
            
            if stable:
                final_max = np.max(np.abs(data.qpos))
                print(f"  ✅ 10000步后稳定, max|qpos| = {final_max:.10f}")
                
                # 测试2: 施加控制信号
                print(f"\n  测试2: 施加控制信号 (act_left_arm=0.7)...")
                data.qpos[:] = 0
                data.ctrl[:] = 0
                data.ctrl[3] = 0.7  # 左臂目标角度
                
                control_stable = True
                for step in range(5000):
                    mujoco.mj_step(model, data)
                    
                    left_arm_angle = data.qpos[3]
                    if abs(left_arm_angle) > 100 or not np.isfinite(left_arm_angle):
                        print(f"  ❌ 步数 {step}: 左臂数值爆炸! angle = {left_arm_angle}")
                        control_stable = False
                        break
                        
                    if step % 500 == 0 and step > 0:
                        error = abs(left_arm_angle - 0.7)
                        print(f"  步数 {step}: left_arm = {left_arm_angle:.6f}, 误差 = {error:.6f}")
                
                if control_stable:
                    final_left_arm = data.qpos[3]
                    final_error = abs(final_left_arm - 0.7)
                    print(f"  ✅ 控制测试通过, 最终左臂角度 = {final_left_arm:.6f}, 误差 = {final_error:.6f}")
                    
        except Exception as e:
            print(f"❌ 加载失败: {e}")
    
    print("\n" + "="*60)
    print("诊断结论和建议")
    print("="*60)
    
if __name__ == "__main__":
    diagnose()