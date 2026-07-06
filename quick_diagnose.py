#!/usr/bin/env python3
"""
快速诊断arm关节数值爆炸问题
"""

import mujoco
import numpy as np

def test_stability(xml_path, model_name, control_value=0.7):
    """测试模型稳定性"""
    print(f"\n{'='*60}")
    print(f"测试: {model_name}")
    print(f"路径: {xml_path}")
    print('='*60)
    
    try:
        model = mujoco.MjModel.from_xml_path(xml_path)
        data = mujoco.MjData(model)
        
        print(f"✓ 模型加载成功")
        print(f"  时间步长: {model.opt.timestep:.6f}s")
        print(f"  左臂身体质量: {model.body_mass[5]:.6f}")
        print(f"  右臂身体质量: {model.body_mass[7]:.6f}")
        
        # 测试1: 无控制，检查是否稳定
        print(f"\n[测试1] 无控制输入 - 稳定性测试...")
        data.qpos[:] = 0
        data.ctrl[:] = 0
        mujoco.mj_forward(model, data)
        
        for step in range(10000):
            mujoco.mj_step(model, data)
            if not np.all(np.isfinite(data.qpos)):
                print(f"  ❌ 步数{step}: 数值爆炸!")
                for j in range(model.nq):
                    jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
                    val = data.qpos[j]
                    status = "❌" if (abs(val) > 1 or not np.isfinite(val)) else "✓"
                    print(f"     [{status}] {jname}: {val:.6e}")
                return False
        
        print(f"  ✓ 10000步后稳定")
        
        # 测试2: 施加控制
        print(f"\n[测试2] 控制信号={control_value}...")
        data.qpos[:] = 0
        data.ctrl[:] = 0
        data.ctrl[3] = control_value
        
        for step in range(5000):
            mujoco.mj_step(model, data)
            
            left_arm_val = data.qpos[3]
            right_arm_val = data.qpos[5]
            
            # 检查数值爆炸
            if abs(left_arm_val) > 10 or not np.isfinite(left_arm_val):
                print(f"  ❌ 步数{step}: 左臂数值爆炸! left_arm={left_arm_val:.6e}")
                return False
            
            if abs(right_arm_val) > 10 or not np.isfinite(right_arm_val):
                print(f"  ❌ 步数{step}: 右臂数值爆炸! right_arm={right_arm_val:.6e}")
                return False
            
            if step % 1000 == 0:
                err_left = abs(left_arm_val - control_value)
                err_right = abs(right_arm_val)
                print(f"  步数{step}: left_arm={left_arm_val:.4f}(误差{err_left:.4f}), right_arm={right_arm_val:.4f}")
        
        final_err_left = abs(data.qpos[3] - control_value)
        final_err_right = abs(data.qpos[5])
        print(f"  ✓ 测试通过! 最终误差: left={final_err_left:.4f}, right={final_err_right:.4f}")
        return True
        
    except Exception as e:
        print(f"✗ 加载失败: {e}")
        import traceback
        traceback.print_exc()
        return None

def main():
    print("=== 快速诊断 arm 关节数值爆炸问题 ===")
    print("用户观察到:")
    print("  • left_arm_roll_joint = 9.54e+21 (异常!)")
    print("  • right_arm_roll_joint = 9.54e+21 (异常!)")
    print()
    
    # 测试原始模型
    result1 = test_stability(
        'assets/mjcf/electronbot_mesh.xml',
        '原始模型 (electronbot_mesh.xml)',
        control_value=0.7
    )
    
    # 测试修复后的模型
    result2 = test_stability(
        'assets/mjcf/electronbot_mesh_fixed2.xml',
        '修复后模型 (electronbot_mesh_fixed2.xml)', 
        control_value=0.7
    )
    
    print("\n" + "="*60)
    print("总结")
    print("="*60)
    
    if result1 is True and result2 is True:
        print("✓ 两个模型都稳定，问题可能是Viewer中的特定操作导致")
        print("\n建议:")
        print("  1. 使用原始模型 (更安全)")
        print("  2. 在Control面板中设置较小的值 (如0.3而不是0.7)")
        print("  3. 点击'Reset'按钮重置后再设置控制值")
    elif result1 is False or result2 is False:
        print("⚠ 发现不稳定的模型!")
        print("需要调整物理参数")

if __name__ == "__main__":
    main()