import xml.etree.ElementTree as ET
import xml.dom.minidom as minidom

def generate_ultimate_classroom():
    mujoco = ET.Element('mujoco', model="high_brightness_classroom")
    ET.SubElement(mujoco, 'compiler', angle="degree", coordinate="local", inertiafromgeom="true")
    
    # 1. 资产定义
    asset = ET.SubElement(mujoco, 'asset')
    ET.SubElement(asset, 'texture', name="tex_floor", type="2d", builtin="flat", rgb1="0.3 0.25 0.2", width="512", height="512")
    ET.SubElement(asset, 'material', name="mat_floor", texture="tex_floor", texrepeat="10 10")
    ET.SubElement(asset, 'material', name="mat_wood", rgba="0.5 0.35 0.2 1")
    ET.SubElement(asset, 'material', name="mat_metal", rgba="0.8 0.8 0.8 1")
    ET.SubElement(asset, 'material', name="mat_chair", rgba="0.2 0.2 0.2 1")
    ET.SubElement(asset, 'material', name="mat_wall", rgba="0.98 0.98 0.95 1") # 调高墙面反射率
    ET.SubElement(asset, 'material', name="mat_door", rgba="0.4 0.2 0.1 1")
    ET.SubElement(asset, 'material', name="mat_blackboard", rgba="0.1 0.2 0.1 1") # 深绿色黑板
    ET.SubElement(asset, 'material', name="mat_tech", rgba="0.05 0.05 0.05 1")

    worldbody = ET.SubElement(mujoco, 'worldbody')
    
    # --- 2. 亮度增强系统 ---
    # 添加双重高强度光源，模拟日光灯组
    ET.SubElement(worldbody, 'light', pos="0 3 7", dir="0 0 -1", diffuse="1.2 1.2 1.2", specular="0.3 0.3 0.3")
    ET.SubElement(worldbody, 'light', pos="0 -3 7", dir="0 0 -1", diffuse="1.2 1.2 1.2", specular="0.3 0.3 0.3")
    
    # 地面
    ET.SubElement(worldbody, 'geom', name="floor", type="plane", size="6 8 0.1", material="mat_floor")

    # --- 3. 严丝合缝墙面（右侧） ---
    # 墙面切片逻辑：确保 (墙长度 + 门洞宽度) 总和等于 16m (-8到8)
    ET.SubElement(worldbody, 'geom', name="r_wall_front", type="box", size="0.1 1.75 1.5", pos="6 6.25 1.5", material="mat_wall")
    ET.SubElement(worldbody, 'geom', name="r_wall_mid", type="box", size="0.1 3.5 1.5", pos="6 0 1.5", material="mat_wall")
    ET.SubElement(worldbody, 'geom', name="r_wall_back", type="box", size="0.1 1.75 1.5", pos="6 -6.25 1.5", material="mat_wall")
    # 过梁部分
    ET.SubElement(worldbody, 'geom', name="r_wall_top1", type="box", size="0.1 0.5 0.4", pos="6 4.0 2.6", material="mat_wall")
    ET.SubElement(worldbody, 'geom', name="r_wall_top2", type="box", size="0.1 0.5 0.4", pos="6 -4.0 2.6", material="mat_wall")

    # 门板 (嵌套位置微调)
    def add_door_with_handle(name, y_pos):
        door_b = ET.SubElement(worldbody, 'body', name=name, pos=f"5.98 {y_pos} 1.1")
        ET.SubElement(door_b, 'geom', type="box", size="0.03 0.495 1.1", material="mat_door") # 稍微缩小0.005防穿模
        # 金属把手
        ET.SubElement(door_b, 'geom', type="cylinder", size="0.01 0.04", pos="-0.04 -0.4 0", quat="0.707 0 0.707 0", material="mat_metal")
    
    add_door_with_handle("door_1", 4.0)
    add_door_with_handle("door_2", -4.0)

    # 其余墙面
    ET.SubElement(worldbody, 'geom', name="wall_left", type="box", size="0.1 8 1.5", pos="-6 0 1.5", material="mat_wall")
    ET.SubElement(worldbody, 'geom', name="wall_back", type="box", size="6 0.1 1.5", pos="0 -8 1.5", material="mat_wall")
    # 前墙 (黑板安装位置)
    ET.SubElement(worldbody, 'geom', name="wall_front", type="box", size="6 0.1 1.5", pos="0 8 1.5", material="mat_wall")
    # 新增黑板
    ET.SubElement(worldbody, 'geom', name="blackboard", type="box", size="3 0.02 0.8", pos="0 7.89 1.8", material="mat_blackboard")

    # --- 4. 讲台区域 (电脑、麦克风) ---
    ET.SubElement(worldbody, 'geom', name="podium", type="box", size="2.5 1.2 0.1", pos="0 6.5 0.05", material="mat_wood")
    t_desk = ET.SubElement(worldbody, 'body', pos="0 6.2 0.1")
    ET.SubElement(t_desk, 'geom', type="box", size="0.8 0.4 0.4", pos="0 0 0.4", material="mat_wood")
    # 电脑显示器
    ET.SubElement(t_desk, 'geom', type="box", size="0.25 0.01 0.18", pos="0 0.1 0.98", material="mat_tech")
    # 麦克风
    ET.SubElement(t_desk, 'geom', type="capsule", size="0.005 0.1", pos="0.3 0 0.9", quat="0.92 0.38 0 0", material="mat_metal")
    ET.SubElement(t_desk, 'geom', type="sphere", size="0.02", pos="0.3 -0.05 1.0", material="mat_tech")

    # --- 5. 30套学生桌椅 (桌腿/椅腿补全) ---
    for r in range(5):
        for c in range(6):
            x, y = -3.5 + c * 1.4, 4.0 - r * 1.8
            set_b = ET.SubElement(worldbody, 'body', name=f"student_{r}_{c}", pos=f"{x} {y} 0")
            
            # 桌子面 & 桌斗
            ET.SubElement(set_b, 'geom', type="box", size="0.45 0.3 0.02", pos="0 0 0.82", material="mat_wood")
            ET.SubElement(set_b, 'geom', type="box", size="0.4 0.28 0.01", pos="0 0 0.68", material="mat_wood")
            # 桌腿补全 (4根)
            for lx, ly in [(-0.4,-0.25), (0.4,-0.25), (-0.4,0.25), (0.4,0.25)]:
                ET.SubElement(set_b, 'geom', type="cylinder", size="0.015 0.4", pos=f"{lx} {ly} 0.4", material="mat_metal")

            # 椅子
            chair = ET.SubElement(set_b, 'body', pos="0 -0.7 0")
            ET.SubElement(chair, 'geom', type="box", size="0.22 0.22 0.02", pos="0 0 0.52", material="mat_chair")
            ET.SubElement(chair, 'geom', type="box", size="0.22 0.01 0.22", pos="0 -0.21 0.74", material="mat_chair")
            # 椅腿补全 (4根)
            for cx, cy in [(-0.18,-0.18), (0.18,-0.18), (-0.18,0.18), (0.18,0.18)]:
                ET.SubElement(chair, 'geom', type="cylinder", size="0.012 0.25", pos=f"{cx} {cy} 0.25", material="mat_metal")

    # 导出
    xml_data = ET.tostring(mujoco, encoding='utf-8')
    pretty_xml = minidom.parseString(xml_data).toprettyxml(indent="  ")
    with open("classroom_final_v5.xml", "w", encoding='utf-8') as f:
        f.write(pretty_xml)
    print("生成成功：classroom_final_v5.xml (高亮度、严丝合缝、含黑板与桌椅腿)")

if __name__ == "__main__":
    generate_ultimate_classroom()