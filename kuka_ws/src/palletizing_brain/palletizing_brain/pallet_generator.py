import os
import json
import re
import math

class PalletGenerator:
    def __init__(self, pallet_length=1.16, pallet_width=0.96, max_height=1.80):
        """初始化托盘物理参数 (单位: 米)"""
        self.P_L = pallet_length
        self.P_W = pallet_width
        self.P_H_MAX = max_height

    def generate_full_pallet(self, box_length, box_width, box_height, case_in_stock, offset_x=0.0, offset_y=0.0, offset_z=0.0):
        """核心装箱算法：生成符合真实库存数量的垛型矩阵，并直接转换为世界坐标"""
        # 方案 1：0度排布
        nx_1 = int(self.P_L // box_length)
        ny_1 = int(self.P_W // box_width)
        count_1 = nx_1 * ny_1

        # 方案 2：90度排布
        nx_2 = int(self.P_L // box_width)
        ny_2 = int(self.P_W // box_length)
        count_2 = nx_2 * ny_2

        if count_1 >= count_2:
            best_nx, best_ny = nx_1, ny_1
            b_len, b_wid = box_length, box_width
            quat = {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
        else:
            best_nx, best_ny = nx_2, ny_2
            b_len, b_wid = box_width, box_length
            quat = {'w': 0.7071, 'x': 0.0, 'y': 0.0, 'z': 0.7071}

        max_layers = int(self.P_H_MAX // box_height)
        
        # 相对托盘中心的局部起始偏移量
        start_x = - (best_nx * b_len) / 2 + (b_len / 2)
        start_y = - (best_ny * b_wid) / 2 + (b_wid / 2)

        pallet_matrix = []
        generated_count = 0 

        for z in range(max_layers):
            # 修改 1：加上世界坐标 offset_z + 加上一半高度(中心点) + 每层累加 2 毫米防爆缝隙
            center_z = offset_z + (z * box_height) + (box_height / 2.0) + (0.002 * (z + 1))
            
            for y in range(best_ny):
                # 修改 2：X 和 Y 坐标直接加上托盘在 3D 世界里的绝对坐标
                center_y = offset_y + start_y + (y * b_wid)
                for x in range(best_nx):
                    center_x = offset_x + start_x + (x * b_len)
                    
                    box_data = {
                        'position': [round(center_x, 4), round(center_y, 4), round(center_z, 4)],
                        'quaternion': [quat['w'], quat['x'], quat['y'], quat['z']]
                    }
                    pallet_matrix.append(box_data)
                    generated_count += 1
                    
                    if generated_count >= case_in_stock:
                        return pallet_matrix
                        
        print(f"⚠️ 警告：库存要求 {case_in_stock} 箱，超出 1.8m 限高！")
        return pallet_matrix


class OrderManager:
    def __init__(self):
        self.generator = PalletGenerator()
        self.active_pallets = {"station_1": [], "station_2": []}
        self.station_offsets = {
            "station_1": (1.45, -0.80, 0.41681),  # 对应场景中的 zone1 真实中心点
            "station_2": (1.45, 0.80, 0.41681)    # 对应场景中的 zone2 真实中心点
        }
        # 自动向上寻找 5 层，精准定位到 data 文件夹里的 箱型主数据.txt
        self.data_path = os.path.expanduser('~/palletizing_brain_workspace/data/箱型主数据.txt')
        
        # 一启动就自动把所有箱型数据读进大脑
        self.box_database = self._parse_master_data()

    def _parse_master_data(self):
        """内部方法：解析 txt 文件，提取 11 种订单的全部参数"""
        database = {}
        if not os.path.exists(self.data_path):
            print(f"❌ 致命错误: 找不到数据文件 -> {self.data_path}")
            return database

        with open(self.data_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # 用正则表达式抓取 "1：" 到 "11：" 里面的 JSON 数据
        matches = re.finditer(r'(\d+)：\s*({.*})', content)
        for match in matches:
            order_id = match.group(1)
            try:
                data = json.loads(match.group(2))
                sku = data['task_item'][0]['sku_info'][0]
                database[order_id] = {
                    'length': sku['length'] / 1000.0,   # mm 转 m
                    'width': sku['width'] / 1000.0,     # mm 转 m
                    'height': sku['height'] / 1000.0,   # mm 转 m
                    'case_in_stock': sku['case_in_stock'], # 初始生成总数
                    'target_qty': data['task_item'][0]['target_qty'] # 要搬运的数量
                }
            except Exception as e:
                print(f"⚠️ 解析订单 {order_id} 时出错: {e}")
        
        print(f"📚 成功加载 {len(database)} 种箱型主数据！")
        return database

    def spawn_pallet_by_order(self, station_name, order_id):
        """【终极一键生成】只要传入订单号，全自动查参数并生成垛型"""
        order_id = str(order_id)
        if order_id not in self.box_database:
            print(f"❌ 找不到订单 {order_id} 的数据！")
            return None

        # 从字典中调取该订单的专属数据
        info = self.box_database[order_id]
        # 🔥 获取对应工位的真实世界坐标
        world_x, world_y, world_z = self.station_offsets[station_name]

        print(f"\n🧱 准备在 [{station_name}] 生成 [订单 {order_id}] 的垛型...")
        print(f"   -> 尺寸: {info['length']} x {info['width']} x {info['height']} 米")
        print(f"   -> 初始物理库存: {info['case_in_stock']} 箱")
        print(f"   -> 托盘绝对锚点: X={world_x}, Y={world_y}, Z={world_z}")

        # 调用核心算法生成矩阵 (将真实世界坐标传进去)
        matrix = self.generator.generate_full_pallet(
            info['length'], info['width'], info['height'], info['case_in_stock'],
            offset_x=world_x, offset_y=world_y, offset_z=world_z
        )
        self.active_pallets[station_name] = matrix
        
        return info['target_qty'] # 返回需要搬运的数量，方便后续调度

    def pop_top_box(self, station_name):
        """从指定工位的最顶层拿走一个箱子"""
        if len(self.active_pallets[station_name]) == 0:
            return None
        return self.active_pallets[station_name].pop()


# ==========================================
# 🚀 真实数据试运行区 (验证自动读取)
# ==========================================
if __name__ == "__main__":
    # 初始化管理者 (它会自动去读那个 txt 文件)
    manager = OrderManager()
    
    if manager.box_database:
        # 模拟工作流：一键在 2 处生成【订单 1】的垛型
        target_qty_1 = manager.spawn_pallet_by_order("station_2", order_id=1)
        
        print("\n--- 🤖 机械臂开始执行 订单 1 ---")
        for i in range(1, target_qty_1 + 1):
            box = manager.pop_top_box("station_2")
            if box:
                print(f"   抓走第 {i} 个箱子，坐标 Z={box['position'][2]} 米")
            
        print(f"✅ 订单 1 完成！station_2 残留垃圾箱子: {len(manager.active_pallets['station_2'])} 个。")
        
        # 模拟工作流：一键在 1 处生成【订单 2】的垛型
        target_qty_2 = manager.spawn_pallet_by_order("station_1", order_id=2)