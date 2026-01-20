import mujoco
import mujoco.viewer
import time
import threading
import numpy as np

# ==========================================
# 全局变量控制中心
# ==========================================
blackboard_content = "SYSTEM READY"
current_fan_speed = 0.0
current_light_brightness = 1.0

# 档位映射表
LIGHT_MAP = {
    "0": 0.0,  # 关灯
    "1": 0.3, 
    "2": 0.6, 
    "3": 1.0, 
    "4": 1.5, 
    "5": 2.5   # 极亮
}

FAN_MAP = {
    "6": 0.0,  # 关风扇
    "7": 5.0,  # 一档
    "8": 15.0, # 二档
    "9": 30.0  # 三档
}

def control_console_thread():
    """ 统一控制台监听线程 """
    global blackboard_content, current_fan_speed, current_light_brightness
    
    print("\n" + "="*40)
    print("      教室设备综合控制系统 (按键版)")
    print("="*40)
    print("指令说明：")
    print("1. 输入 0 - 5：调节灯光亮度 (0关, 5最亮)")
    print("2. 输入 6 - 9：调节风扇转速 (6关, 9最快)")
    print("3. 输入 任何英文：同步更新黑板文字")
    print("="*40)

    while True:
        user_in = input("请输入指令 >> ").strip()
        if not user_in: continue

        if user_in in LIGHT_MAP:
            current_light_brightness = LIGHT_MAP[user_in]
            print(f">>> 灯光已调至 {user_in} 档 (亮度: {current_light_brightness})")
        
        elif user_in in FAN_MAP:
            current_fan_speed = FAN_MAP[user_in]
            print(f">>> 风扇已调至 {user_in} 档 (转速: {current_fan_speed})")
        
        else:
            blackboard_content = user_in
            print(f">>> 黑板文字更新为: '{user_in}'")

def main():
    # 1. 加载模型 (确保 xml 文件名正确)
    model = mujoco.MjModel.from_xml_path("classroom_final_v5.xml")
    data = mujoco.MjData(model)

    # 2. 开启后台控制线程
    threading.Thread(target=control_console_thread, daemon=True).start()

    # 3. 启动查看器
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # 设置初始视角
        viewer.cam.distance = 10.0
        viewer.cam.lookat = [0, 3, 1.5]

        while viewer.is_running():
            step_start = time.time()

            # --- A. 灯光实时控制 ---
            # 修改所有光源的漫反射强度
            for i in range(model.nlight):
                model.light_diffuse[i] = [current_light_brightness] * 3

            # --- B. 风扇平滑转速控制 ---
            # 模拟电机物理惯性，缓慢趋近目标速度
            fan_joint_vel = data.joint('fan_joint').qvel[0]
            data.joint('fan_joint').qvel = fan_joint_vel + (current_fan_speed - fan_joint_vel) * 0.05

            # --- C. 物理步进 ---
            mujoco.mj_step(model, data)

            # --- D. 黑板文字实时显示 (Overlay) ---
            with viewer.lock():
                # 使用 User Geometry 渲染文字
                if viewer.user_scn.maxgeom < 1:
                    viewer.user_scn.maxgeom = 1
                
                g = viewer.user_scn.geoms[0]
                g.type = mujoco.mjtGeom.mjGEOM_SPHERE
                g.size[:] = [0.001, 0.001, 0.001]
                g.pos[:] = [0, 7.75, 1.8] # 黑板前方
                g.rgba[:] = [0, 0, 0, 0]   # 透明锚点
                g.label = blackboard_content

            viewer.sync()

            # 维持仿真频率
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()