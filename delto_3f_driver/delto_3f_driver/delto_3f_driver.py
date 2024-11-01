#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# from delto_utility import delto_modbus_TCP as delto_TCP
import math
import time
import threading
import sys
import os
import time

import rclpy

from rclpy.action import ActionServer, CancelResponse, GoalResponse
#Duration rcl
from rclpy.duration import Duration 

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String, Int8, Int32, Bool, Float32MultiArray, Int16MultiArray
from ros_gz_interfaces.msg import Float32Array

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

import rclpy

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from delto_utility import delto_modbus_TCP as delto_TCP



class DeltoROSDriver(Node):

    def __init__(self):

        # ROS2 Node Initialize
        super().__init__('delto_3f_driver')
        qos_profile = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.RELIABLE
        )

        fast_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1,
            lifespan=Duration(seconds=1),
            history=QoSHistoryPolicy.KEEP_LAST)
        
        self.declare_parameter('ip', "169.254.186.62")
        self.declare_parameter('port', 10000)
        self.declare_parameter('slaveID', 1)
        self.declare_parameter('dummy', False)

        self.joint_state_list = [0.0]*12
        self.current_joint_state = [0.0]*12
        self.target_joint_state = [0.0]*12
        self.fixed_joint_state = [0.0]*12

        self.joint_state_feedback = JointTrajectoryPoint()
        self.vel = []
        self.delto_client = delto_TCP.Communication()
        self.stop_thread = False
        self.lock = threading.Lock()

        # Too high frequency will cause blocking sub/pub
        self.publish_rate = 100
        print('publish late : '+str(self.publish_rate))
        
        
        self.is_dummy = False  # bool(self.get_parameter('dummy').value)

        # Action Server
        self.jcm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'delto_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.joint_state_pub = self.create_publisher(
            JointState, 'gripper/joint_states', fast_qos)
        self.grasp_sub = self.create_subscription(
            Int32, 'gripper/cmd', callback=self.grasp_callback, qos_profile=qos_profile)

        self.grasp_mode_sub = self.create_subscription(
            Float32MultiArray, 'gripper/target_joint', callback=self.target_joint_callback, qos_profile=qos_profile)
        # self.grasp_mode_sub2 = self.create_subscription(
        #     Float32Array, 'gripper/target_joint', callback=self.target_joint_callback, qos_profile=qos_profile)
        self.joint_state_timer = self.create_timer(
            1/self.publish_rate, self.timer_callback)
        self.read_joint_timer = self.create_timer(
            1/self.publish_rate, self.read_joint_callback)
        self.fixed_joint_sub = self.create_subscription(
            Int16MultiArray, 'gripper/fixed_joint', self.grasp_callback, qos_profile=qos_profile)

    # Connect to the delto gripper
    def connect(self) -> bool:

        if self.is_dummy:
            print("Dummy mode")
            return True

        print("Connecting to the delto gripper...")
        return self.delto_client.connect(self.get_parameter('ip').value,
                                         self.get_parameter('port').value,
                                         self.get_parameter('slaveID').value)

    # Publish joint state
    def read_joint_callback(self):
        # self.current_joint_stat
        position_tmp = self.get_position()
        # print("position_tmp: ", position_tmp)
        self.current_joint_state = [float(self._deg2rad(x)) for x in position_tmp]
        # print("current_joint_state: ", self.current_joint_state)
        
    def joint_state_publisher(self):

        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()

        joint_state_msg.name = ['F1M1', 'F1M2', 'F1M3', 'F1M4',
                                'F2M1', 'F2M2', 'F2M3', 'F2M4',
                                'F3M1', 'F3M2', 'F3M3', 'F3M4']

        joint_state_msg.position = self.current_joint_state
        self.joint_state_feedback.positions = joint_state_msg.position
        self.joint_state_pub.publish(joint_state_msg)
    # Get current position
    def get_position(self):

        if self.is_dummy:
            return self.current_joint_state

        status = self.delto_client.get_position()
        return status

    def set_position(self, position):

        if self.is_dummy:
            self.current_joint_state = position
            return

        self.delto_client.set_position(position)

    def set_motion_step(self, step):

        if self.is_dummy:
            return

        self.delto_client.set_step(step)

    def grasp_mode_callback(self, mode: Int32):
        self.delto_client.grasp_mode(mode.data)

    def grasp_callback(self, grasp: Bool):
        with self.lock:
            print('check {}'.format(grasp.data))
            self.get_logger().info('check {}'.format(grasp.data))
            self.delto_client.grasp_mode(grasp.data)

    def timer_callback(self):
        self.joint_state_publisher()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel')
        return CancelResponse.ACCEPT

    # FollowJointTrajectory action server callback
    async def execute_callback(self, goal_handle):

        print('FollowJointTrajectory callback...')
        print(goal_handle.request.trajectory)
        goal = goal_handle.request.trajectory.points.copy()
        # download planned path from ros moveit
        self.joint_state_list = []

        if goal:
            self.joint_state_list = [p.positions for p in goal]
        else:
            self.stop_motion()
            return

        if self.joint_state_list:

            # print("joint_state_list: ", self.joint_state_list)

            # add first and last point to the trajectory
            new_array = [self.joint_state_list[0]]

            middle_point = self.joint_state_list[int(
                len(self.joint_state_list)/2)]
            new_array.append(middle_point)
            new_array.append(self.joint_state_list[-1])

            # set motion step by trajectory points
            self.set_motion_step(1)

            new_array = [[self._rad2deg(joints) for joints in subset]
                         for subset in new_array]
            # array = self.joint_state_list[-1]
            # array = [joint for joint in array]
            # print(new_array[-1])
            self._waypointMove(new_array, 0.5)
            # self.set_position(new_array[-1])

            # print("_waypointMoveSuccess")

            result = FollowJointTrajectory.Result()
            feedback_msg = FollowJointTrajectory.Feedback()

            feedback_msg.desired.positions = self.joint_state_feedback.positions
            feedback_msg.actual.positions = self.joint_state_feedback.positions

            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.01)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL

        print("success")
        return result

    def target_joint_callback(self, msg):

        if len(msg.data) != 12:
            self.get_logger().error("Invalid target joint state")
            return

        msg.data = [self._rad2deg(x) for x in msg.data]

        self.target_joint_state = msg.data
        print("target_joint_state: ", self.target_joint_state)
        self.delto_client.set_position(self.target_joint_state)

    def fixed_joint_callback(self, msg):

        if len(msg.data) != 12:
            self.get_logger().error("Invalid fixed joint state")
            return
        self.fixed_joint_state = msg.data
        
        # print("fixed_joint_state: ", self.fixed_joint_state)
        self.delto_client.fix_position(self.fixed_joint_state)
            
    def waypointMove(self, waypointList, threshold):
        self.stop_thread = False
        self.waypoint_thread = threading.Thread(
            target=self._waypointMove, args=(waypointList, threshold))
        self.waypoint_thread.start()

    def stop_motion(self):
        self.delto_client.set_position(self.delto_client.get_position())
        self.stop_thread = True

    def _waypointMove(self, waypointList, threshold=0.3):

        self.stop_thread = False
        i = 0
        move_flag = False
        print("_waypointMove1")
        print("self.stop_thread: " + str(self.stop_thread))
        first_input_flag = True

        first_current_position = [0.0]*12  # float
        current_position = [0.0] * 12

        while not self.stop_thread:

            if (move_flag == False):
                # move to the next waypoint
                self.delto_client.set_position(waypointList[i])
                move_flag = True
                first_input_flag = True

            current_position = self.delto_client.get_position()

            if (first_input_flag):
                first_current_position = current_position
                first_input_flag = False

            error = []
            for a, b, c in zip(current_position, waypointList[i], first_current_position):

                if (b-c) < 0.0001:
                    error.append(0.0)
                else:
                    error.append(abs(((a-b)/(b-c))))

            if all(e < threshold for e in error):
                # arrived at the waypoint
                move_flag = False
                i += 1

            if (i >= len(waypointList)):
                self.stop_thread = True
                return

    def _deg2rad(self, deg):
        return deg * math.pi / 180.0

    def _rad2deg(self, rad):
        return rad * 180.0 / math.pi


def main(args=None):
    rclpy.init(args=args)

    delto_driver = DeltoROSDriver()
    connect = delto_driver.connect()

    if connect == False:
        delto_driver.get_logger().error("network connection failed.")
        return

    if not (delto_driver.is_dummy):
        delto_driver.delto_client.set_free(False)

    time.sleep(0.1)
    delto_driver.get_logger().info("delto_driver initialized")

    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(delto_driver)
    executor.spin()

    executor.shutdown()


if __name__ == '__main__':
    main()
