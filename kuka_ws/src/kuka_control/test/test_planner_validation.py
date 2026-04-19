#!/usr/bin/env python3
"""
路径规划器验证测试脚本
用于测试改进后的碰撞检测和轨迹规划算法
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
import time
import sys

from kuka_interfaces.action import PalletTask


class PlannerValidator(Node):
    def __init__(self):
        super().__init__('planner_validator')

        # 发布碰撞物体
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)

        # Action客户端
        self._action_client = ActionClient(self, PalletTask, 'pallet_task')

        self.get_logger().info('PlannerValidator 已启动')

    def wait_for_server(self, timeout=10.0):
        self.get_logger().info('等待 Action Server...')
        return self._action_client.wait_for_server(timeout_sec=timeout)

    def send_test_goal(self, box_id, start_pose, end_pose, box_size=[0.25, 0.2, 0.2], mass=10.0, height_limit=1.8):
        goal_msg = PalletTask.Goal()
        goal_msg.box_id = box_id
        goal_msg.start_pose = start_pose
        goal_msg.end_pose = end_pose
        goal_msg.box_size.x = box_size[0]
        goal_msg.box_size.y = box_size[1]
        goal_msg.box_size.z = box_size[2]
        goal_msg.box_mass = mass
        goal_msg.height_limit = height_limit

        self.get_logger().info(f'发送任务: {box_id}')
        self.get_logger().info(f'  起点: x={start_pose.position.x:.3f}, y={start_pose.position.y:.3f}, z={start_pose.position.z:.3f}')
        self.get_logger().info(f'  终点: x={end_pose.position.x:.3f}, y={end_pose.position.y:.3f}, z={end_pose.position.z:.3f}')

        self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
            goal_response_callback=self.goal_response_callback,
            result_callback=self.result_callback
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('任务被拒绝!')
            return
        self.get_logger().info('任务已接受')

    def feedback_callback(self, feedback):
        self.get_logger().info(f'  反馈: {feedback.feedback.phase_description}')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'任务结果: success={result.success}, message={result.message}')

    def create_box(self, name, size, position, frame_id='world'):
        box = CollisionObject()
        box.header.frame_id = frame_id
        box.id = name
        box.operation = CollisionObject.ADD

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size
        box.primitives = [primitive]

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.w = 1.0
        box.primitive_poses = [pose]

        return box

    def publish_scene(self):
        scene = PlanningScene()
        scene.is_diff = True

        # 添加测试障碍物
        # 高处的障碍物，防止机械臂在转移时碰撞
        obstacle_high = self.create_box(
            'test_obstacle_high',
            [0.5, 0.5, 0.3],
            [1.0, 0.5, 1.5]
        )
        scene.world.collision_objects.append(obstacle_high)

        # 侧面的障碍物
        obstacle_side = self.create_box(
            'test_obstacle_side',
            [0.2, 1.0, 1.0],
            [-0.5, 1.5, 0.5]
        )
        scene.world.collision_objects.append(obstacle_side)

        self.scene_pub.publish(scene)
        self.get_logger().info('测试场景已发布')


def main(args=None):
    rclpy.init(args=args)

    validator = PlannerValidator()

    # 等待 server
    if not validator.wait_for_server(timeout=15.0):
        validator.get_logger().error('Action Server 未就绪，退出')
        rclpy.shutdown()
        return

    # 发布测试场景
    validator.publish_scene()
    time.sleep(1)  # 等待场景更新

    # 测试用例：正常码垛任务
    # 起点：抓取区域
    start_pose = Pose()
    start_pose.position.x = 1.5
    start_pose.position.y = -0.3
    start_pose.position.z = 0.6
    start_pose.orientation.w = 1.0

    # 终点：码垛区域
    end_pose = Pose()
    end_pose.position.x = 0.2
    end_pose.position.y = 1.8
    end_pose.position.z = 0.6
    end_pose.orientation.w = 1.0

    validator.send_test_goal(
        box_id='test_box_1',
        start_pose=start_pose,
        end_pose=end_pose,
        box_size=[0.25, 0.2, 0.2],
        mass=10.0
    )

    # 等待任务完成
    time.sleep(30)

    # 额外测试：多个连续任务
    validator.get_logger().info('=== 开始连续任务测试 ===')
    for i in range(3):
        start_pose.position.x = 1.5 - i * 0.3
        end_pose.position.x = 0.2 - i * 0.3

        validator.send_test_goal(
            box_id=f'test_box_{i+2}',
            start_pose=start_pose,
            end_pose=end_pose
        )

        time.sleep(2)

    # 运行足够时间以完成所有任务
    validator.get_logger().info('=== 运行测试，等待任务完成... ===')
    time.sleep(60)

    validator.get_logger().info('=== 测试完成 ===')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
