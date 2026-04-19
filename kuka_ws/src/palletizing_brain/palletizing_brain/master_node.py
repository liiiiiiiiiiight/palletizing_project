import rclpy
from rclpy.node import Node
import os
import json
import re
import socket
import time
from palletizing_interfaces.srv import GetTask

# MoveIt 和几何相关的 ROS 2 消息类型
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class PalletGenerator:
    def __init__(self, pallet_length=1.16, pallet_width=0.96, max_height=1.80):
        self.P_L = pallet_length
        self.P_W = pallet_width
        self.P_H_MAX = max_height

    def generate_full_pallet(self, box_length, box_width, box_height, case_in_stock, offset_x=0.0, offset_y=0.0, offset_z=0.0):
        nx_1 = int(self.P_L // box_length)
        ny_1 = int(self.P_W // box_width)
        count_1 = nx_1 * ny_1

        nx_2 = int(self.P_L // box_width)
        ny_2 = int(self.P_W // box_length)
        count_2 = nx_2 * ny_2

        if count_1 >= count_2:
            best_nx, best_ny = nx_1, ny_1
            b_len, b_wid = box_length, box_width
            quat = [1.0, 0.0, 0.0, 0.0] 
        else:
            best_nx, best_ny = nx_2, ny_2
            b_len, b_wid = box_width, box_length
            quat = [0.7071, 0.0, 0.0, 0.7071] 

        gap = 0.002 
        x_spacing = b_len + gap
        y_spacing = b_wid + gap
        z_spacing = box_height + gap

        max_layers = int(self.P_H_MAX // box_height)
        start_x = - (best_nx * x_spacing) / 2 + (x_spacing / 2)
        start_y = - (best_ny * y_spacing) / 2 + (y_spacing / 2)

        pallet_matrix = []
        generated_count = 0 

        for z in range(max_layers):
            center_z = offset_z + (z * z_spacing) + (box_height / 2.0)
            
            for y in range(best_ny):
                center_y = offset_y + start_y + (y * y_spacing)
                for x in range(best_nx):
                    center_x = offset_x + start_x + (x * x_spacing)
                    
                    box_data = {
                        'position': [round(center_x, 4), round(center_y, 4), round(center_z, 4)],
                        'quaternion': quat
                    }
                    pallet_matrix.append(box_data)
                    generated_count += 1
                    
                    if generated_count >= case_in_stock:
                        return pallet_matrix
                        
        return pallet_matrix

class OrderManager:
    def __init__(self):
        self.generator = PalletGenerator()
        self.active_pallets = {"station_1": [], "station_2": []}
        self.station_offsets = {
            "station_1": (1.45, -0.80, 0.41681),  
            "station_2": (1.45, 0.80, 0.41681)    
        }
        self.data_path = os.path.expanduser('~/palletizing_project/data/箱型主数据.txt')
        self.box_database = self._parse_master_data()

    def _parse_master_data(self):
        database = {}
        if not os.path.exists(self.data_path): return database
        with open(self.data_path, 'r', encoding='utf-8') as f:
            content = f.read()
        matches = re.finditer(r'(\d+)：\s*({.*})', content)
        for match in matches:
            order_id = match.group(1)
            try:
                data = json.loads(match.group(2))
                sku = data['task_item'][0]['sku_info'][0]
                database[order_id] = {
                    'length': sku['length'] / 1000.0,
                    'width': sku['width'] / 1000.0,
                    'height': sku['height'] / 1000.0,
                    'case_in_stock': sku['case_in_stock'],
                    'target_qty': data['task_item'][0]['target_qty']
                }
            except: pass
        return database

    def spawn_pallet_by_order(self, station_name, order_id):
        order_id = str(order_id)
        if order_id not in self.box_database: return None
        info = self.box_database[order_id]
        wx, wy, wz = self.station_offsets[station_name]
        matrix = self.generator.generate_full_pallet(
            info['length'], info['width'], info['height'], info['case_in_stock'],
            offset_x=wx, offset_y=wy, offset_z=wz
        )
        self.active_pallets[station_name] = matrix
        return info['target_qty']

class PalletizingBrainNode(Node):
    def __init__(self):
        super().__init__('state_manager_node')
        self.manager = OrderManager()
        self.orders_root = os.path.expanduser('~/palletizing_project/data/测试订单/')
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.srv = self.create_service(GetTask, 'get_pallet_task', self.handle_get_task)
        
        self.co_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.rviz_spawned_boxes = {"station_1": [], "station_2": []}
        
        # 记忆系统：防止 C++ 预加载时重复触发环境生成
        self.spawned_orders = set()
        
        self.get_logger().info("✅ 算力大脑已就绪，RViz 同步模块启动...")

    def handle_get_task(self, request, response):
        order_id = request.order_id
        box_idx = request.box_index
        order_str = str(order_id)
        
        if order_str not in self.manager.box_database:
            response.success = False
            return response

        info = self.manager.box_database[order_str]
        station_name = "station_2" if order_id % 2 != 0 else "station_1"
        station_id = 2 if order_id % 2 != 0 else 1

        # 只要订单没生成过，就触发双端生成
        if order_str not in self.spawned_orders:
            self.spawned_orders.add(order_str) 
            self.manager.spawn_pallet_by_order(station_name, order_id)
            pallet_matrix = self.manager.active_pallets[station_name]
            batch_id = int(time.time() * 1000)
            
            # 1. 清理 RViz 旧箱子
            for old_box_id in self.rviz_spawned_boxes[station_name]:
                co_remove = CollisionObject()
                co_remove.id = old_box_id
                co_remove.operation = CollisionObject.REMOVE
                self.co_pub.publish(co_remove)
            self.rviz_spawned_boxes[station_name].clear()
            
            # 2. 发送给 Isaac Sim (包含工位中心坐标供智能清场使用)
            wx, wy, wz = self.manager.station_offsets[station_name]
            batch_data = {
                "cmd": "BATCH_SPAWN",
                "batch_id": batch_id, 
                "station": station_name,
                "center_x": wx,  
                "center_y": wy,
                "length": info['length'], "width": info['width'], "height": info['height'],
                "boxes": [{"pos": b['position'], "quat": b['quaternion']} for b in pallet_matrix]
            }
            self.udp_sock.sendto(json.dumps(batch_data).encode('utf-8'), ("127.0.0.1", 9999))
            
            # 3. 🟢 生成 RViz 碰撞盒 (退回最稳健的世界绝对坐标系)
            for i, box in enumerate(pallet_matrix):
                box_id = f"rviz_box_{batch_id}_{station_name}_{i}"
                self.rviz_spawned_boxes[station_name].append(box_id)
                
                co = CollisionObject()
                
                # 🚀 彻底抛弃带旋转的 zone 坐标系，采用绝对世界坐标系
                co.header.frame_id = "world" 
                
                co.id = box_id
                co.operation = CollisionObject.ADD
                
                prim = SolidPrimitive()
                prim.type = SolidPrimitive.BOX
                prim.dimensions = [info['length'], info['width'], info['height']]
                co.primitives.append(prim)
                
                pose = Pose()
                # 🚀 直接使用大脑计算出的、绝对不会错的绝对坐标
                pose.position.x = box['position'][0]
                pose.position.y = box['position'][1]
                pose.position.z = box['position'][2]
                
                pose.orientation.w = box['quaternion'][0]
                pose.orientation.x = box['quaternion'][1]
                pose.orientation.y = box['quaternion'][2]
                pose.orientation.z = box['quaternion'][3]
                co.primitive_poses.append(pose)
                
                self.co_pub.publish(co)
                time.sleep(0.005)
                
            self.get_logger().info(f"🚀 订单 {order_id} 双端同步完成！Isaac Sim & RViz 均已生成 {len(pallet_matrix)} 箱。")

        # 接收到预加载指令，只生成环境，不消耗箱子坐标
        if box_idx == 0:
            response.success = True
            return response

        # 正常抓取取坐标逻辑
        if not self.manager.active_pallets[station_name]:
            response.success = False
            return response
            
        pick_pose = self.manager.active_pallets[station_name].pop()
        place_pose = self._read_place_pose_from_json(order_id, box_idx)

        response.success = True
        response.station_id = station_id
        response.pick_x, response.pick_y, response.pick_z = pick_pose['position']
        response.pick_qw, response.pick_qx, response.pick_qy, response.pick_qz = pick_pose['quaternion']
        response.place_x, response.place_y, response.place_z = place_pose['position']
        response.place_qw, response.place_qx, response.place_qy, response.place_qz = place_pose['quaternion']
        response.target_qty = info['target_qty']
        response.case_in_stock = info['case_in_stock']
        response.box_len = info['length']
        response.box_wid = info['width']
        response.box_hei = info['height']
        return response

    def _read_place_pose_from_json(self, order_id, box_idx):
        path = os.path.join(self.orders_root, f"订单{order_id}", f"box_{box_idx}_execution_data.json")
        with open(path, 'r') as f:
            data = json.load(f)
            tf = data["grasp_info"][0]["tf_map_placed"]
            return {
                "position": [tf["position"]['x'], tf["position"]['y'], tf["position"]['z']],
                "quaternion": [tf["quaternion"]['w'], tf["quaternion"]['x'], tf["quaternion"]['y'], tf["quaternion"]['z']]
            }

def main(args=None):
    rclpy.init(args=args)
    node = PalletizingBrainNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()