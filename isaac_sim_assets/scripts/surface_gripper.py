# ==============================================================
# 第一步：必须最先导入并启动 SimulationApp
# ==============================================================
from isaacsim import SimulationApp

# 启动仿真窗口。headless=False 会打开 GUI 界面
simulation_app = SimulationApp({"headless": False}) 

# ==============================================================
# 第二步：在 App 启动后，再导入 Isaac Sim 的核心功能库
# ==============================================================
import omni.isaac.core.utils.prims as prim_utils
from isaacsim.robot.surface_gripper import _surface_gripper
from omni.isaac.core.utils.stage import is_stage_loading
import omni.timeline

def main():
    # 1. 获取接口
    sg_interface = _surface_gripper.acquire_surface_gripper_interface()

    # 2. 定义抓取器属性
    prop = _surface_gripper.SurfaceGripperProperties()
    prop.parentPrim = "/World/assembly_moveit/ee"  # 确保你的 USD 中存在此路径且为 Rigid Body
    prop.offset.p = (0, 0, 0.05)                  # 相对于末端的偏移
    prop.offset.r = (0.707, 0, -0.707, 0)          # 旋转偏移 (w, x, y, z)
    prop.gripThreshold = 0.8                      # 触发阈值
    prop.forceLimit = 1.0e6                       
    prop.torqueLimit = 1.0e6                      
    prop.bendAngle = 0.785                        

    # 3. 初始化（这会在 USD 中创建必要的 Joint 和相关节点）
    # 注意：初始化前确保 Stage 已经加载完成
    if not is_stage_loading():
        success = sg_interface.initialize(prop)
        if success:
            print("Surface Gripper 初始化成功！")
        else:
            print("Surface Gripper 初始化失败，请检查 parentPrim 路径是否正确。")

    # 获取时间线以便播放仿真
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    # 计数器示例，模拟定时开关
    frame_count = 0

    # ==============================================================
    # 第三步：进入仿真主循环
    # ==============================================================
    while simulation_app.is_running():
        # 更新物理和渲染
        simulation_app.update()
        
        frame_count += 1
        
        # 简单逻辑演示：第 100 帧尝试抓取，第 500 帧释放
        gripper_path = "/World/assembly_moveit/ee/SurfaceGripper"
        
        if frame_count == 100:
            print("尝试开启抓取...")
            sg_interface.close_gripper(gripper_path)
            
        if frame_count == 500:
            print("释放抓取...")
            sg_interface.open_gripper(gripper_path)

    # 停止并关闭
    timeline.stop()
    simulation_app.close()

if __name__ == "__main__":
    main()