import mujoco
import mujoco.viewer
import time
import threading
import numpy as np

# 全局变量：存储黑板文字
blackboard_content = "READY"

def terminal_input_thread():
    global blackboard_content
    print("\n[控制台已就绪] 请输入英文并按回车同步到黑板：")
    while True:
        user_in = input(">> ")
        if user_in.lower() == 'clear':
            blackboard_content = ""
        else:
            blackboard_content = user_in

def main():
    global blackboard_content
    
    # 1. 加载模型
    try:
        model = mujoco.MjModel.from_xml_path("classroom_final_v5.xml")
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"加载XML失败: {e}")
        return

    # 2. 启动输入监听线程
    threading.Thread(target=terminal_input_thread, daemon=True).start()

    # 3. 启动查看器
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("\n--- 仿真窗口已启动 ---")
        
        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)

            # 4. 修正形状匹配问题
            with viewer.lock():
                # 获取用户场景中的第一个几何体对象
                g = viewer.user_scn.geoms[0]
                
                # 设置属性
                g.type = mujoco.mjtGeom.mjGEOM_SPHERE
                # 显式使用 numpy 数组确保形状对齐 [3,]
                g.size[:] = np.array([0.001, 0.001, 0.001], dtype=np.float32)
                # 位置：黑板前方中心 [3,]
                g.pos[:] = np.array([0, 7.85, 1.8], dtype=np.float32)
                # 颜色：透明 [4,]
                g.rgba[:] = np.array([0, 0, 0, 0], dtype=np.float32)
                
                # 修复报错：直接赋值 3x3 单位矩阵，不使用 flatten()
                g.mat[:] = np.eye(3, dtype=np.float32)
                
                # 设置文字标签
                g.label = f"{blackboard_content}"
                
                # 激活渲染
                viewer.user_scn.ngeom = 1

            viewer.sync()

            # 控制循环频率
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()