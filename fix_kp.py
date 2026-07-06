"""用 setter 方式可靠更新 kp"""
import mujoco

xml_path = '/mnt/data2/projects/xiaozhi/ElectronBot_SIM/assets/mjcf/electronbot_full_arm.xml'
spec = mujoco.MjSpec.from_file(xml_path)

# 更新 actuator gains
targets = {
    'act_body':        (80, 20),
    'act_head':        (40, 10),
    'act_left_pitch':  (60, 15),
    'act_left_roll':   (30, 8),
    'act_right_pitch': (60, 15),
    'act_right_roll':  (30, 8),
}

for i in range(len(spec.actuator)):
    a = spec.actuator[i]
    if a.name in targets:
        kp, kv = targets[a.name]
        a.kp = kp
        a.kv = kv

# 也更新 default joint damping
for d in spec.default():
    d.joint[0].damping = 4.0

xml = spec.to_xml()
with open(xml_path, 'w') as f:
    f.write(xml)
print('Done: kp restored for radian mode')
