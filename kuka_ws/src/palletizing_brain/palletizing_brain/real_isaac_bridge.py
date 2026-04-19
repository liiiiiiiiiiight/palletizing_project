from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False}) 

import numpy as np
import socket
import json
import random  
import math  # 🔥 新增数学库用于计算距离
import omni.usd
from omni.isaac.core.utils.stage import open_stage, is_stage_loading
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import XFormPrim

def main():
    scene_path = "/home/one/palletizing_project/isaac_sim_assets/assembly_isaac_sim.usd"
    print(f"⏳ 加载场景: {scene_path}")
    open_stage(usd_path=scene_path)
    
    simulation_app.update()
    while is_stage_loading():
        simulation_app.update()
        
    world = World()
    world.reset()
    world.play() 
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 9999))
    sock.setblocking(False) 
    
    stage = omni.usd.get_context().get_stage()
    print("✅ 3D 渲染引擎就绪，采用【智能空间测距清场】技术！")
    
    while simulation_app.is_running():
        world.step(render=True)
        
        while True:
            try:
                data, _ = sock.recvfrom(65536)
                payload = json.loads(data.decode('utf-8'))
                
                if payload.get("cmd") == "BATCH_SPAWN":
                    st = payload["station"]
                    batch_id = payload["batch_id"] 
                    l, w, h = payload["length"], payload["width"], payload["height"]
                    
                    # 🔥 核心修改：接收 Python 大脑传来的工位中心坐标
                    wx = payload.get("center_x", 0.0)
                    wy = payload.get("center_y", 0.0)
                    
                    # ==========================================
                    # 智能距离甄别散落法
                    # ==========================================
                    cleared_count = 0
                    kept_count = 0
                    for prim in stage.Traverse():
                        name = prim.GetName()
                        # 找到对应托盘的旧箱子
                        if name.startswith("box_") and f"_{st}_" in name:
                            path = prim.GetPath().pathString
                            try:
                                xf = XFormPrim(prim_path=path)
                                pos, _ = xf.get_world_pose()
                                current_x, current_y = pos[0], pos[1]
                                
                                # 🔥 计算箱子当前位置到原工位中心的水平距离
                                dist = math.hypot(current_x - wx, current_y - wy)
                                
                                if dist < 1.5:  
                                    # 距离 < 1.5 米，说明还是废料，扔掉！
                                    rx = random.uniform(-100, 100)
                                    ry = random.uniform(-100, 100)
                                    rz = random.uniform(-500, -200)
                                    xf.set_world_pose(position=np.array([rx, ry, rz]))
                                    cleared_count += 1
                                else:
                                    # 距离 > 1.5 米，说明已被搬运到 AGV 或目的托盘，保留！
                                    kept_count += 1
                            except:
                                pass
                    
                    if cleared_count > 0 or kept_count > 0:
                        print(f"🧹 智能清场完毕：清理了 {cleared_count} 个残余废料，保留了 {kept_count} 个已完成产品！")

                    # ==========================================
                    # 📦 空间干净了，开始安全空投新订单！
                    # ==========================================
                    print(f"📦 瞬间召唤新订单！共 {len(payload['boxes'])} 个箱子...")
                    
                    for i, box in enumerate(payload["boxes"]):
                        name = f"box_{batch_id}_{st}_{i}" 
                        path = f"/{name}"
                        px, py, pz = box["pos"]
                        
                        DynamicCuboid(
                            prim_path=path,
                            name=name,
                            position=np.array([px, py, pz + 0.02]),
                            orientation=np.array(box["quat"]), 
                            scale=np.array([l, w, h]),
                            color=np.array([0.8, 0.6, 0.4]),
                            mass=2.0 
                        )
                    
                    print(f"✅ {st} 整垛瞬间生成完毕！")
                    
            except (BlockingIOError, json.JSONDecodeError): 
                break
            except Exception as e:
                print(f"❌ 警告: 捕获到异常 {e}")
                
    simulation_app.close()

if __name__ == '__main__':
    main()