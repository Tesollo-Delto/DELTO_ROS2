import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        timer_period = 5.0  # 1초마다 메시지 발행
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joint_names = ["j_1_1", "j_1_2", "j_1_3", "j_1_4",
                            "j_2_1",  "j_2_2", "j_2_3", "j_2_4",
                            "j_3_1", "j_3_2", "j_3_3", "j_3_4",
                            "j_4_1", "j_4_2", "j_4_3", "j_4_4",
                            "j_5_1", "j_5_2", "j_5_3", "j_5_4"]
        
        self.angles = [[0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0],
                       [0.5, 0.5, 0.5, 0.5,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.5, 0.7, 0.7,
                        0.0, 0.5, 0.7, 0.7],
                       [0.5, 0.5, 0.5, 0.5,
                        -0.3, 0.0, 0.0, 0.0,
                        0.0, 0.5, 0.5, 0.5,
                        0.0, 0.5, 0.7, 0.7,
                        0.0, 0.5, 0.7, 0.7],
                       [0.5, 0.5, 0.5, 0.5,
                        0.0, 0.0, 0.0, 0.0,
                        0.0, 0.5, 0.5, 0.5,
                        0.0, 0.5, 0.7, 0.7,
                        0.0, 0.5, 0.7, 0.7]]
        self.index = 0

    def timer_callback(self):
        message = JointTrajectory()
        message.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.angles[self.index]
        point.time_from_start = Duration(sec=0, nanosec=100000000)

        message.points.append(point)
        self.publisher_.publish(message)
        # self.get_logger().info(f'Published angles: {self.angles[self.index]}')

        self.index += 1
        if self.index >= len(self.angles):
            self.index = 0

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_publisher = JointTrajectoryPublisher()
    rclpy.spin(joint_trajectory_publisher)
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()