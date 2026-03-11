#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
@author FTX
@date 2025 / 03 / 03
'''

import time
import threading
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dobot_msgs_v4.srv import EnableRobot, ServoJ
import os

class FollowJointTrajectoryServer(Node):

    def __init__(self):
        super().__init__('dobot_group_controller')
        name = os.getenv("DOBOT_TYPE")
        self.cb_group = ReentrantCallbackGroup()
        # 创建FollowJointTrajectory动作服务器
        self._action_server = ActionServer(self,FollowJointTrajectory,f'/{name}_group_controller/follow_joint_trajectory',self.execute_callback)
        self.get_logger().info("FollowJointTrajectory Action Server is ready...")
        self.EnableRobot_l = self.create_client(EnableRobot, '/dobot_bringup_ros2/srv/EnableRobot', callback_group=self.cb_group)
        self.ServoJ_l = self.create_client(ServoJ, '/dobot_bringup_ros2/srv/ServoJ', callback_group=self.cb_group)
        
        # Esperar servicio con timeout máximo de 30 segundos
        max_retries = 30
        retry_count = 0
        while not self.EnableRobot_l.wait_for_service(timeout_sec=1.0):
            retry_count += 1
            if retry_count >= max_retries:
                self.get_logger().error('Service not available after 30 seconds, continuing anyway...')
                break
            self.get_logger().info('service not available, waiting again...')

    def _ensure_robot_ready(self):
        """Re-enable robot to reset ServoJ mode before each trajectory."""
        if not self.EnableRobot_l.service_is_ready():
            self.get_logger().warn('EnableRobot service not ready — continuing anyway')
            return
        req = EnableRobot.Request()
        future = self.EnableRobot_l.call_async(req)
        _event = threading.Event()
        future.add_done_callback(lambda f: _event.set())
        if not _event.wait(timeout=3.0):
            self.get_logger().warn('EnableRobot timeout — continuing anyway')
        else:
            self.get_logger().info('EnableRobot OK — servo mode reset')

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Received a new trajectory goal!")
        self._ensure_robot_ready()
        # 获取目标轨迹
        trajectory = goal_handle.request.trajectory
        self.execution_trajectory(trajectory)
        goal_handle.succeed()
        # 返回结果
        result = FollowJointTrajectory.Result()
        result.error_code = 0
        return result

    def execution_trajectory(self, trajectory: JointTrajectory):
        self.get_logger().info("Joint Names: {}".format(trajectory.joint_names))
        self.get_logger().info(f"Ejecutando trayectoria: {len(trajectory.points)} puntos")
        Positions = []
        for i, point in enumerate(trajectory.points):
            joint= []
            for ii in point.positions:
                joint.append(180 * ii / 3.14159)
            Positions.append(joint)
            self.get_logger().info(
                "Point {}: Positions: {}, Velocities: {}, Accelerations: {}, TimeFromStart: {}".format(
                    i, joint, point.velocities, point.accelerations, point.time_from_start.sec
                )
            )
        for ii in Positions:
            self.ServoJ_C(ii[0],ii[1],ii[2],ii[3],ii[4],ii[5])
            time.sleep(0.18)

    def ServoJ_C(self, j1, j2, j3, j4, j5, j6):  # 运动指令
        P1 = ServoJ.Request()
        P1.a = float(j1)
        P1.b = float(j2)
        P1.c = float(j3)
        P1.d = float(j4)
        P1.e = float(j5)
        P1.f = float(j6)
        P1.param_value = ["t=0.2"]
        response = self.ServoJ_l.call_async(P1)

def main(args=None):
    rclpy.init(args=args)
    follow_joint_trajectory_server = FollowJointTrajectoryServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(follow_joint_trajectory_server)
    executor.spin()
    follow_joint_trajectory_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
