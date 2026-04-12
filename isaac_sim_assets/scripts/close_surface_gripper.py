from isaacsim.robot.surface_gripper import _surface_gripper

# 1. 获取 Surface Gripper 接口
sg_interface = _surface_gripper.acquire_surface_gripper_interface()

# 2. 指定你的 Surface Gripper 节点在 USD 树中的绝对路径

gripper_path = "/World/assembly_moveit/ee/SurfaceGripper"

# 1. Send close command
sg_interface.close_gripper(gripper_path)

# 2. Get and print status (5.1.0 API)
status = sg_interface.get_gripper_status(gripper_path)
print(f"Current Status: {status}")

# 3. Check if any physical objects are actually gripped
gripped_objects = sg_interface.get_gripped_objects(gripper_path)

if gripped_objects:
    print(f"Success! Gripped objects: {gripped_objects}")
else:
    print("Warning: No objects grabbed (check distance, mass, or collision)")

# ==========================================
# To release the object, uncomment the line below:
# sg_interface.open_gripper(gripper_path)

