"""
A simple python interface with MoveIt2 services (and actions). This is an alternative to
moveit_commander, which is not yet ported to ROS 2 (as of Oct 2020).
Note: This module is currently configured for Franka Emika Panda robot.
Note: There is no Ignition-specific code in this module (this repo is just a convinient place).

the code is from https://github.com/AndrejOrsula/ign_moveit2 ,and is modified.
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# from pymoveit2 import MoveIt2Interface
from std_msgs.msg import Float32, Header
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, Point
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import PositionIKRequest, RobotTrajectory, CollisionObject, AttachedCollisionObject

from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetMotionPlan, GetCartesianPath
from moveit_msgs.action import MoveGroup
from action_msgs.msg import GoalStatus
#followtrajectory
from control_msgs.action import FollowJointTrajectory

from typing import List, Optional, Tuple, Union

# from moveit_commander import PlanningSceneInterface

class MoveIt2Interface(Node):

    def __init__(self):
        super().__init__("ign_moveit2_py")
        self.init_robot()
        self.init_compute_fk()
        self.init_compute_ik()
        self.init_plan_kinematic_path()
        self.init_plan_cartesian_path()
        self.init_collision_objects()
        self.get_logger().info("ign_moveit2_py initialised successfuly")

        self.add_collision_primitive( id="table",
            primitive_type=SolidPrimitive.BOX,
            dimensions=(5, 5 , 1.8),
            pose=Pose(position=Point(x=0.5, y=0.5, z=-1.4)),
            frame_id="world",
        )


    def init_robot(self):
        
        self.robot_group_name = "ur3"
        # Arm
        self.arm_group_name = "manipulator"
        self.arm_joints = ["shoulder_pan_joint",
                           "shoulder_lift_joint",
                           "elbow_joint",
                           "wrist_1_joint",
                           "wrist_2_joint",
                           "wrist_3_joint"]
        
        self.arm_links = ["base_link",
                          "shoulder_link",
                          "upper_arm_link",
                          "forearm_link",
                          "wrist_1_link",
                          "wrist_2_link",
                          "wrist_3_link",
                          "ee_link"]
        
        self.arm_base_link = self.arm_links[0]
        self.arm_end_effector = self.arm_links[-1]

        # Publisher of trajectories
        self.joint_trajectory_pub = self.create_publisher(JointTrajectory,
                                                          "set_joint_trajectory", 1)

        # Subscriber of current joint states
        self.joint_state = JointState()
        self.joint_state_mutex = threading.Lock()
        self.joint_state_sub = self.create_subscription(JointState,
                                                        "ign_joint_states",
                                                        self.joint_state_callback, 1)

        # Subscriber of joint trajectory progress
        self.joint_progress = 1.0
        self.joint_progress_cond = threading.Condition()
        self.joint_progress_sub = self.create_subscription(Float32,
                                                           "joint_trajectory_progress",
                                                           self.joint_progress_callback, 1)

        self.robot_trajectory_client = ActionClient(self, FollowJointTrajectory, "scaled_joint_trajectory_controller/follow_joint_trajectory")

    def robot_move(self):

        plan = self.motion_plan_.joint_trajectory
        goal_msg = FollowJointTrajectory.Goal()
        self.goals = {}

        goal_msg.trajectory = plan
        # goal_msg.goal_id.stamp = self.get_clock().now().to_msg()
        # goal_msg.goal_id.id = "goal_1"

        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal_msg.trajectory.header.frame_id = "world"
        goal_msg.trajectory.joint_names = self.arm_joints

        #Duration 1sec
        for i,point in enumerate(goal_msg.trajectory.points):
            point.time_from_start = Duration(sec=1 + i, nanosec=0)

        self.robot_trajectory_client.send_goal_async(goal_msg)

    def joint_state_callback(self, msg):
        """
        Callback for getting current joint states.
        """
        self.joint_state_mutex.acquire()
        self.joint_state = msg
        self.joint_state_mutex.release()

    def get_joint_state(self) -> JointState:
        """
        Get current joint states.
        """
        self.joint_state_mutex.acquire()
        joint_state = self.joint_state
        self.joint_state_mutex.release()
        return joint_state

    def joint_progress_callback(self, msg):
        """
        Callback for getting joint trajectory progress.
        """
        with self.joint_progress_cond:
            self.joint_progress = msg.data
            self.joint_progress_cond.notify_all()

    def wait_until_executed(self):
        """
        Function that halts execution on the current thread until trajectory is executed.
        """
        with self.joint_progress_cond:
            while not self.joint_progress == 1.0:
                self.joint_progress_cond.wait(timeout=0.5)

    def pub_trajectory(self, trajectory):
        """
        Publish trajectory such that it can be executed, e.g. by `JointTrajectoryController`
        Ignition plugin.
        """
        if isinstance(trajectory, JointTrajectory):
            self.joint_trajectory_pub.publish(trajectory)
            self.get_logger().info("pub trajectory msg!")
        elif isinstance(trajectory, RobotTrajectory):
            self.joint_trajectory_pub.publish(trajectory.joint_trajectory)
        else:
            raise Exception("Invalid type passed into pub_trajectory()")

    def execute(self, joint_trajectory=None) -> bool:
        """
        Execute last planned motion plan, or the `joint_trajectory` specified as argument.
        """

        if joint_trajectory == None:
            plan = self.motion_plan_.joint_trajectory
        else:
            plan = joint_trajectory

        # Make sure there is a plan to follow
        if not plan.points:
            self.get_logger().warn(
                "Cannot execute motion plan because it does not contain any trajectory points")
            return False

        # Reset joint progress
        self.joint_progress = 0.0

        self.pub_trajectory(plan)

        # # follow joint trajectory action client for ur3e 
        # self.action_client = ActionClient(self, FollowJointTrajectory, 'follow_joint_trajectory')
        # self.action_client.wait_for_server()
        # goal_msg = MoveGroup.Goal()
        # goal_msg.trajectory = plan
        # self.action_client.send_goal(goal_msg)

        return True

    def move_to_joint_state(self, joint_state,
                            set_position=True,
                            set_velocity=True,
                            set_effort=True):
        """
        Set joint target on all joints defined in `joint_state`. This function does NOT plan a
        smooth trajectory and only publishes joint_state as the next goal that should be reached
        immediately.
        """
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = joint_state.name

        point = JointTrajectoryPoint()
        if set_position:
            point.positions = joint_state.position
        if set_velocity:
            point.velocities = joint_state.velocity
        if set_effort:
            point.effort = joint_state.effort


        joint_trajectory.points.append(point)

        self.pub_trajectory(joint_trajectory)

    # compute_fk
    def init_compute_fk(self):
        """
        Initialise `compute_fk` service.
        """
        self.compute_fk_client = self.create_client(GetPositionFK,
                                                    "compute_fk")
        
        while not self.compute_fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Service [compute_fk] not currently available, waiting...")

        self.fk_request = GetPositionFK.Request()
        self.fk_request.header.frame_id = self.arm_base_link
        # self.fk_request.header.stamp = "Set during request"
        # self.fk_request.fk_link_names = "Set during request"
        # self.fk_request.robot_state.joint_state = "Set during request"
        # self.fk_request.robot_state.multi_dof_joint_state = "Ignored"
        # self.fk_request.robot_state.attached_collision_objects = "Ignored"
        self.fk_request.robot_state.is_diff = False

    def compute_fk(self, fk_link_names=None, joint_state=None) -> GetPositionFK.Response:
        """
        Call `compute_fk` service.
        """
        if fk_link_names == None:
            self.fk_request.fk_link_names = [self.arm_end_effector]
        else:
            self.fk_request.fk_link_names = ['ee_link']

        if joint_state == None:
            self.fk_request.robot_state.joint_state = self.get_joint_state()
        else:
            self.fk_request.robot_state.joint_state = joint_state

        self.fk_request.header.stamp = self._clock.now().to_msg()

        self.compute_fk_client.wait_for_service()
        return self.compute_fk_client.call(self.fk_request)

    # compute_ik
    def init_compute_ik(self):
        """
        Initialise `compute_ik` service.
        """
        # Service client for IK
        self.compute_ik_client = self.create_client(GetPositionIK,
                                                    "compute_ik")
        self.compute_fk_client.service_is_ready()
        while not self.compute_ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Service [compute_ik] not currently available, waiting...")

        self.ik_request = GetPositionIK.Request()
        self.ik_request.ik_request.group_name = self.arm_group_name
        # self.ik_request.ik_request.robot_state.joint_state = "Set during request"
        # self.ik_request.ik_request.robot_state.multi_dof_joint_state = "Ignored"
        # self.ik_request.ik_request.robot_state.attached_collision_objects = "Ignored"
        self.ik_request.ik_request.robot_state.is_diff = False
        # self.ik_request.ik_request.constraints = "Set during request OR Ignored"
        self.ik_request.ik_request.avoid_collisions = True
        # self.ik_request.ik_request.ik_link_name = "Ignored"
        self.ik_request.ik_request.pose_stamped.header.frame_id = self.arm_base_link
        # self.ik_request.ik_request.pose_stamped.header.stamp = "Set during request"
        # self.ik_request.ik_request.pose_stamped.pose = "Set during request"
        # self.ik_request.ik_request.ik_link_names = "Ignored"
        # self.ik_request.ik_request.pose_stamped_vector = "Ignored"
        # self.ik_request.ik_request.timeout.sec = "Ignored"
        # self.ik_request.ik_request.timeout.nanosec = "Ignored"

    def compute_ik(self, pose, start_joint_state=None, constrains=None) -> GetPositionIK.Response:
        """ 
        Call `compute_ik` service.
        """
        if start_joint_state == None:
            self.ik_request.ik_request.robot_state.joint_state = self.get_joint_state()
        else:
            self.ik_request.ik_request.robot_state.joint_state = start_joint_state

        if constrains != None:
            self.ik_request.ik_request.constraints = constrains

        self.ik_request.ik_request.pose_stamped.pose = pose

        self.ik_request.ik_request.pose_stamped.header.stamp = self._clock.now().to_msg()

        self.compute_ik_client.wait_for_service()
        return self.compute_ik_client.call(self.ik_request)

    # plan_kinematic_path
    def init_plan_kinematic_path(self):
        """
        Initialise `plan_kinematic_path` service.
        """
        # Service client for IK
        self.plan_kinematic_path_client = self.create_client(GetMotionPlan,
                                                             "plan_kinematic_path")
        while not self.plan_kinematic_path_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Service [plan_kinematic_path] not currently available, waiting...")

        self.kinematic_path_request = GetMotionPlan.Request()
        self.kinematic_path_request.motion_plan_request.workspace_parameters.header.frame_id = \
            self.arm_base_link
        # self.kinematic_path_request.motion_plan_request.workspace_parameters.header.stamp = \
        # "Set during request"
        self.kinematic_path_request.motion_plan_request.workspace_parameters.min_corner.x = -1.5
        self.kinematic_path_request.motion_plan_request.workspace_parameters.min_corner.y = -1.5
        self.kinematic_path_request.motion_plan_request.workspace_parameters.min_corner.z = -1.5
        self.kinematic_path_request.motion_plan_request.workspace_parameters.max_corner.x = 1.5
        self.kinematic_path_request.motion_plan_request.workspace_parameters.max_corner.y = 1.5
        self.kinematic_path_request.motion_plan_request.workspace_parameters.max_corner.z = 1.5

        self.kinematic_path_request.motion_plan_request.goal_constraints = \
            [Constraints()]

        self.kinematic_path_request.motion_plan_request.group_name = self.arm_group_name

        self.kinematic_path_request.motion_plan_request.max_velocity_scaling_factor = 0.1
        self.kinematic_path_request.motion_plan_request.max_acceleration_scaling_factor = 0.1
        self.kinematic_path_request.motion_plan_request.cartesian_speed_end_effector_link = \
            self.arm_end_effector
        self.kinematic_path_request.motion_plan_request.max_cartesian_speed = 0.1

    def set_max_velocity(self, scaling_factor):
        """
        Set maximum velocity of joints as a factor of joint limits.
        """
        self.kinematic_path_request.motion_plan_request.max_velocity_scaling_factor = scaling_factor

    def set_max_acceleration(self, scaling_factor):
        """
        Set maximum acceleration of joints as a factor of joint limits.
        """
        self.kinematic_path_request.motion_plan_request.max_acceleration_scaling_factor = \
            scaling_factor

    def set_max_cartesian_speed(self, speed):
        """
        Set maximum cartesian speed of end effector.
        """
        self.kinematic_path_request.motion_plan_request.max_cartesian_speed = speed

    def plan_kinematic_path(self,
                            allowed_planning_time=5.0,
                            num_planning_attempts=10) -> GetMotionPlan.Response:
        """
        Call `plan_kinematic_path` service, with goal set using either `set_joint_goal()`,
        `set_position_goal()`, `set_orientation_goal()` or `set_pose_goal()`.
        """

        self.kinematic_path_request.motion_plan_request.num_planning_attempts = \
            num_planning_attempts
        self.kinematic_path_request.motion_plan_request.allowed_planning_time = \
            allowed_planning_time

        self.kinematic_path_request.motion_plan_request.workspace_parameters.header.stamp = \
            self._clock.now().to_msg()

        # Stamp message with current time
        clock_time_now_msg = self._clock.now().to_msg()
        self.kinematic_path_request.motion_plan_request.workspace_parameters.header.stamp = \
            clock_time_now_msg
        for contraints in self.kinematic_path_request.motion_plan_request.goal_constraints:
            for position_constraint in contraints.position_constraints:
                position_constraint.header.stamp = clock_time_now_msg
            for orientation_constraint in contraints.orientation_constraints:
                orientation_constraint.header.stamp = clock_time_now_msg

        self.plan_kinematic_path_client.wait_for_service()
        response = self.plan_kinematic_path_client.call(
            self.kinematic_path_request)

        self.clear_goal_constraints()
        self.motion_plan_ = response.motion_plan_response.trajectory
        return response

    def clear_goal_constraints(self):
        """
        Clear all goal constraints that were previously set.
        Note that this function is called automatically after each `plan_kinematic_path()`.
        """
        self.kinematic_path_request.motion_plan_request.goal_constraints = [Constraints()]

    def create_new_goal_constraint(self):
        """
        Create a new set of goal contraints that will be set together with the request. Each
        subsequent setting of goals with `set_joint_goal()`, `set_pose_goal()` and others will be
        added under this newly created set of contraints.
        """
        (self.kinematic_path_request.motion_plan_request.goal_constraints
         .append(Constraints()))

    def set_joint_goal(self, joint_positions, tolerance=0.001, weight=1.0, joint_names=None):
        """
        Set goal position in joint space. With `joint_names` specified, `joint_positions` can be
        defined for specific joints. Otherwise, first `n` joints defined in `init_robot()` will be
        used, where `n` is the length of `joint_positions`.
        """
        if joint_names == None:
            joint_names = self.arm_joints

        for i in range(len(joint_positions)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_names[i]
            joint_constraint.position = joint_positions[i]
            joint_constraint.tolerance_above = tolerance
            joint_constraint.tolerance_below = tolerance
            joint_constraint.weight = weight

            (self.kinematic_path_request.motion_plan_request.goal_constraints[-1].
             joint_constraints.append(joint_constraint))

    def set_position_goal(self, position, tolerance=0.001, weight=1.0, frame=None):
        """
        Set goal position of `frame` in Cartesian space. Defaults to the end-effector `frame`.
        """
        if frame == None:
            frame = self.arm_base_link

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = frame
        position_constraint.link_name = self.arm_end_effector
        position_constraint.constraint_region.primitive_poses.append(Pose())
        position_constraint.constraint_region.primitive_poses[0].position.x = position[0]
        position_constraint.constraint_region.primitive_poses[0].position.y = position[1]
        position_constraint.constraint_region.primitive_poses[0].position.z = position[2]

        # Goal is defined as a sphere with radius equal to tolerance
        position_constraint.constraint_region.primitives.append(
            SolidPrimitive())
        position_constraint.constraint_region.primitives[0].type = 2
        position_constraint.constraint_region.primitives[0].dimensions = [
            tolerance]
        position_constraint.weight = weight

        (self.kinematic_path_request.motion_plan_request.goal_constraints[-1].position_constraints
         .append(position_constraint))

    def set_orientation_goal(self, quaternion, tolerance=0.001, weight=1.0, frame=None):
        """
        Set goal orientation of `frame`. Defaults to the end-effector `frame`.
        """
        if frame == None:
            frame = self.arm_end_effector

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.arm_base_link
        orientation_constraint.link_name = frame
        orientation_constraint.orientation = quaternion
        orientation_constraint.absolute_x_axis_tolerance = tolerance
        orientation_constraint.absolute_y_axis_tolerance = tolerance
        orientation_constraint.absolute_z_axis_tolerance = tolerance
        orientation_constraint.weight = weight

        (self.kinematic_path_request.motion_plan_request.goal_constraints[-1]
         .orientation_constraints.append(orientation_constraint))

    def set_pose_goal(self, position, quaternion,
                      tolerance_position=0.001, tolerance_orientation=0.001,
                      weight_position=1.0, weight_orientation=1.0,
                      frame=None):
        """
        Set goal pose. This is direct combination of `set_position_goal()` and
        `set_orientation_goal()`.   
        """
        self.set_position_goal(
            position, tolerance_position, weight_position, frame)
        self.set_orientation_goal(
            quaternion, tolerance_orientation, weight_orientation, frame)

    # plan_cartesian_path
    def init_plan_cartesian_path(self):

        self.plan_cartesian_path_service = self.create_client(
        srv_type=GetCartesianPath,
        srv_name="compute_cartesian_path")
        while not self.plan_cartesian_path_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Service [compute_cartesian_path] not currently available, waiting...")
            
        self.cartesian_path_request = GetCartesianPath.Request()

        self.cartesian_path_request.header.frame_id = self.arm_base_link
        self.cartesian_path_request.header.stamp = self._clock.now().to_msg()
        self.cartesian_path_request.waypoints = []
        self.cartesian_path_request.max_step = 0.1
        self.cartesian_path_request.jump_threshold = 0.0
        self.cartesian_path_request.avoid_collisions = True
        self.cartesian_path_request.group_name = self.arm_group_name
        self.cartesian_path_request.link_name = self.arm_end_effector
        self.cartesian_path_request.path_constraints = Constraints()

        self.cartesian_path_request.start_state.joint_state = JointState()
        self.cartesian_path_request.path_constraints.name = ""
        self.cartesian_path_request.path_constraints.joint_constraints = []
        self.cartesian_path_request.path_constraints.position_constraints = []
       
        self.cartesian_path_request.path_constraints.orientation_constraints =[] 
        # self.cartesian_path_request.path_constraints.orientation_constraints.append(tt)
        

        


    def plan_cartesian_path(self, waypoints,
                            start_joint_state=None,
                            constrains=None) -> GetCartesianPath.Response:
       
        self.cartesian_path_request.waypoints = waypoints

        print(self.cartesian_path_request.waypoints)
        self.cartesian_path_request.header.stamp = self._clock.now().to_msg()
        self.cartesian_path_request.group_name = self.arm_group_name
        self.cartesian_path_request.link_name = self.arm_end_effector
        self.cartesian_path_request.start_state.joint_state = self.joint_state
        for constraint in self.cartesian_path_request.path_constraints.position_constraints:
            constraint.header.stamp = self._clock.now().to_msg()

        for constraint in self.cartesian_path_request.path_constraints.orientation_constraints:
            constraint.header.stamp = self._clock.now().to_msg()


        self.plan_cartesian_path_service.wait_for_service()
        response = self.plan_cartesian_path_service.call(self.cartesian_path_request)
        
        self.clear_goal_constraints()
        
        print(response.solution.joint_trajectory)

        # self.motion_plan_.trajectory = response.solution.joint_trajectory
        self.motion_plan_.joint_trajectory = response.solution.joint_trajectory
        
        return response
    
    def init_collision_objects(self):
        self.collision_object_publisher = self.create_publisher(
            CollisionObject, "/collision_object", 10
        )
        self.attached_collision_object_publisher = self.create_publisher(
            AttachedCollisionObject, "/attached_collision_object", 10
        )

    def add_collision_primitive(
        self,
        id: str,
        primitive_type: int,
        dimensions: Tuple[float, float, float],
        pose: Optional[Union[PoseStamped, Pose]] = None,
        position: Optional[Union[Point, Tuple[float, float, float]]] = None,
        quat_xyzw: Optional[
            Union[Quaternion, Tuple[float, float, float, float]]
        ] = None,
        frame_id: Optional[str] = None,
        operation: int = CollisionObject.ADD,
    ):
        """
        Add collision object with a primitive geometry specified by its dimensions.

        `primitive_type` can be one of the following:
            - `SolidPrimitive.BOX`
            - `SolidPrimitive.SPHERE`
            - `SolidPrimitive.CYLINDER`
            - `SolidPrimitive.CONE`
        """

        if (pose is None) and (position is None or quat_xyzw is None):
            raise ValueError(
                "Either `pose` or `position` and `quat_xyzw` must be specified!"
            )

        if isinstance(pose, PoseStamped):
            pose_stamped = pose
        elif isinstance(pose, Pose):
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=frame_id
                    if frame_id is not None
                    else self.arm_base_link,
                ),
                pose=pose,
            )
        else:
            if not isinstance(position, Point):
                position = Point(
                    x=float(position[0]), y=float(position[1]), z=float(position[2])
                )
            if not isinstance(quat_xyzw, Quaternion):
                quat_xyzw = Quaternion(
                    x=float(quat_xyzw[0]),
                    y=float(quat_xyzw[1]),
                    z=float(quat_xyzw[2]),
                    w=float(quat_xyzw[3]),
                )
            pose_stamped = PoseStamped(
                header=Header(
                    stamp=self._node.get_clock().now().to_msg(),
                    frame_id=frame_id
                    if frame_id is not None
                    else self.arm_base_link,
                ),
                pose=Pose(position=position, orientation=quat_xyzw),
            )

        msg = CollisionObject(
            header=pose_stamped.header,
            id=id,
            operation=operation,
            pose=pose_stamped.pose,
        )

        msg.primitives.append(
            SolidPrimitive(type=primitive_type, dimensions=dimensions)
        )

        self.collision_object_publisher.publish(msg)
    