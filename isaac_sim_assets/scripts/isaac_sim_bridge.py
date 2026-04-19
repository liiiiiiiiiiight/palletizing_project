"""
Isaac Sim Script Editor script
Features: UDP box spawn/cleanup + suction control + ROS2 sync
Usage: Open scene in Isaac Sim, paste into Script Editor and run
"""

import numpy as np
import socket
import json
import random
import math

import omni.usd
import omni.kit.app
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import XFormPrim

from isaacsim.robot.surface_gripper import _surface_gripper

import rclpy
from std_msgs.msg import Bool

# ==========================================
# CONFIGURATION
# ==========================================
GRIPPER_PRIM_PATH = "/World/assembly_moveit/ee/SurfaceGripper"
UDP_PORT = 9999
# ==========================================

# --- 1. Surface Gripper interface ---
sg_interface = _surface_gripper.acquire_surface_gripper_interface()

# --- 2. UDP Socket ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", UDP_PORT))
sock.setblocking(False)

# --- 3. ROS2 init ---
if not rclpy.ok():
    rclpy.init()

# Cleanup old nodes (prevent memory leak on repeated Run)
if "isaac_bridge_node" in globals():
    try:
        globals()["isaac_bridge_node"].destroy_node()
    except:
        pass

if "isaac_bridge_sub" in globals():
    try:
        globals()["isaac_bridge_sub"].unsubscribe()
    except:
        pass

node = rclpy.create_node('isaac_bridge_node')
globals()["isaac_bridge_node"] = node

# --- 4. Suction control callback ---
def suction_cmd_callback(msg):
    if msg.data is True:
        sg_interface.close_gripper(GRIPPER_PRIM_PATH)
        print("[Suction] CLOSE (Grab)")
    else:
        sg_interface.open_gripper(GRIPPER_PRIM_PATH)
        print("[Suction] OPEN (Release)")

node.create_subscription(Bool, '/suction_cmd', suction_cmd_callback, 10)

# --- 5. Main update loop (hooked into Isaac Sim render frame) ---
stage = omni.usd.get_context().get_stage()

def on_update_event(e):
    # ROS2 callback
    if rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.001)

    # UDP box spawn/cleanup
    while True:
        try:
            data, _ = sock.recvfrom(65536)
            payload = json.loads(data.decode('utf-8'))

            if payload.get("cmd") == "BATCH_SPAWN":
                st = payload["station"]
                batch_id = payload["batch_id"]
                l, w, h = payload["length"], payload["width"], payload["height"]
                wx = payload.get("center_x", 0.0)
                wy = payload.get("center_y", 0.0)

                # Distance-based smart cleanup
                cleared_count = 0
                kept_count = 0
                for prim in stage.Traverse():
                    name = prim.GetName()
                    if name.startswith("box_") and f"_{st}_" in name:
                        path = prim.GetPath().pathString
                        try:
                            xf = XFormPrim(prim_path=path)
                            pos, _ = xf.get_world_pose()
                            dist = math.hypot(pos[0] - wx, pos[1] - wy)
                            if dist < 1.5:
                                xf.set_world_pose(position=np.array([
                                    random.uniform(-100, 100),
                                    random.uniform(-100, 100),
                                    random.uniform(-500, -200)
                                ]))
                                cleared_count += 1
                            else:
                                kept_count += 1
                        except:
                            pass

                if cleared_count > 0 or kept_count > 0:
                    print(f"[Clear] {cleared_count} removed, {kept_count} kept")

                # Spawn new boxes
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

                print(f"[Spawn] {st}: {len(payload['boxes'])} boxes generated")

        except (BlockingIOError, json.JSONDecodeError):
            break
        except Exception as e:
            print(f"[UDP] Warning: {e}")
            break

update_stream = omni.kit.app.get_app().get_update_event_stream()
globals()["isaac_bridge_sub"] = update_stream.create_subscription_to_pop(
    on_update_event, name="isaac_bridge_update"
)

print("[READY] Isaac Sim Bridge active")
print(f"  - UDP listening on port {UDP_PORT}")
print(f"  - ROS2 subscribed to /suction_cmd")
print(f"  - Gripper path: {GRIPPER_PRIM_PATH}")

