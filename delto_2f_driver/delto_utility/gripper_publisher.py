import sys
from PyQt5.QtWidgets import QApplication, QSlider, QVBoxLayout, QWidget, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt, QThread
import rclpy
from std_msgs.msg import Float32MultiArray

class SliderPublisher(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.target_joint_state = [0.0]*12
        self.initUI()

    def initUI(self):
        self.sliders = []
        self.labels = []
        layout = QVBoxLayout()

        for i in range(12):
            slider = QSlider(Qt.Horizontal, self)
            slider.setMinimum(-180)  # Set the minimum value
            slider.setMaximum(180)  # Set the maximum value
            slider.setValue(0) 
            label = QLabel('0', self)
            slider.valueChanged[int].connect(lambda value, i=i, label=label: self.make_pub(value, i+1, label))
            
            h_layout = QHBoxLayout()
            h_layout.addWidget(slider)
            h_layout.addWidget(label)
            layout.addLayout(h_layout)
            self.sliders.append(slider)
            self.labels.append(label)

        self.setLayout(layout)
        self.setWindowTitle('Delto Joint Controller')
        self.show()

    def make_pub(self, value, i, label):

        self.target_joint_state[i-1] = float(value* 3.141592 / 180.0)

        print(self.target_joint_state)
        msg = Float32MultiArray()
        msg.data = self.target_joint_state
        label.setText(str(value))
        self.node.get_logger().info('Publishing: "%s"' % msg.data)
        self.node.create_publisher(Float32MultiArray, 'gripper/target_joint', 10).publish(msg)

class ROSThread(QThread):
    def __init__(self, node):
        QThread.__init__(self)
        self.node = node

    def run(self):
        rclpy.spin(self.node)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('slider_publisher')
    ros_thread = ROSThread(node)
    ros_thread.start()

    app = QApplication(sys.argv)
    ex = SliderPublisher(node)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()