import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from palletizing_interfaces.action import BoxTask

class MockIsaacBridge(Node):
    def __init__(self):
        super().__init__('isaac_sim_bridge')
        
        self._action_server = ActionServer(
            self, BoxTask, 'box_task_channel', self.execute_callback)
        
        self.get_logger().info('✅ 纯净版中继站已上线！')

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        cmd_type = req.command_type
        
        if cmd_type == 1:
            self.get_logger().info('收到生成请求，已放行')
            
        elif cmd_type == 0:
            self.get_logger().info(f'收到抓取指令：去 ({req.pick_x:.2f}, {req.pick_y:.2f}) 抓取！')

        # 无论如何，直接给 C++ 盖章放行，防止 C++ 卡死
        goal_handle.succeed()
        result = BoxTask.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = MockIsaacBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()