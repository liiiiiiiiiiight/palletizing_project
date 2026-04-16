import rclpy
from std_msgs.msg import Bool
import omni.usd
import omni.kit.app

# 引入你提供的 Isaac Sim 5.1.0 Surface Gripper 官方接口
from isaacsim.robot.surface_gripper import _surface_gripper

# ==========================================
# CONFIGURATION
# ==========================================
# 使用你刚才成功抓取的绝对路径
GRIPPER_PRIM_PATH = "/World/assembly_moveit/ee/SurfaceGripper" 
# ==========================================

print("\n--- Initializing ROS 2 Suction Bridge (API v5.1.0) ---")

# 1. 获取 Surface Gripper 接口 (来自你的代码)
sg_interface = _surface_gripper.acquire_surface_gripper_interface()

# 2. 安全初始化 ROS 2
if not rclpy.ok():
    rclpy.init()

# 3. 清理旧的节点（防止多次点击Run导致内存泄漏）
if "my_suction_node" in globals():
    try:
        globals()["my_suction_node"].destroy_node()
    except:
        pass

if "suction_update_sub" in globals():
    try:
        globals()["suction_update_sub"].unsubscribe()
    except:
        pass

# 4. 创建 ROS 2 节点
globals()["my_suction_node"] = rclpy.create_node('isaac_python_suction_bridge')

def suction_cmd_callback(msg):
    print(f"[ROS 2] Received suction command: {msg.data}")
    
    # 核心修改：使用你验证过的 API 来控制吸盘
    if msg.data is True:
        # 收到 True，执行吸取
        sg_interface.close_gripper(GRIPPER_PRIM_PATH)
        print(f"[SUCCESS] Commanded to CLOSE (Grab)")
        
        # 可选：打印当前状态以供调试
        status = sg_interface.get_gripper_status(GRIPPER_PRIM_PATH)
        gripped_objects = sg_interface.get_gripped_objects(GRIPPER_PRIM_PATH)
        if gripped_objects:
            print(f"          -> Actually gripped: {gripped_objects}")
    else:
        # 收到 False，执行释放
        sg_interface.open_gripper(GRIPPER_PRIM_PATH)
        print(f"[SUCCESS] Commanded to OPEN (Release)")

# 5. 订阅你 C++ 发出的主题
sub = globals()["my_suction_node"].create_subscription(
    Bool, '/suction_cmd', suction_cmd_callback, 10
)

# 6. 将 rclpy 挂载到 Isaac Sim 的原生物理帧更新事件上 (绝不卡顿)
def on_update_event(e):
    if rclpy.ok() and "my_suction_node" in globals():
        rclpy.spin_once(globals()["my_suction_node"], timeout_sec=0.001)

update_stream = omni.kit.app.get_app().get_update_event_stream()
globals()["suction_update_sub"] = update_stream.create_subscription_to_pop(
    on_update_event, name="ros2_suction_spin_task"
)

print("[READY] Bridge successfully attached to Isaac Sim Update Loop.")
print("[READY] Waiting for messages on topic: /suction_cmd")
print("-----------------------------------------\n")
