# ROS2 Delto 3F Driver Package

This package provides a ROS2 driver for the DELTO 3F Gripper.


### Prerequisites

- ROS2 (humble)
- DELTO 3F Gripper

### Usage

Delto_Gripper Subscribes to the following topics:
- gripper/target_joint @ std_msgs.msg.Float32MultiArray

target joint position (in radian) for each joint (12 joints)

```python
from std_msgs.msg import Float32MultiArray

# example
target_joint = [0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0]

send_msg = Float32MultiArray()
send_msg.data = target_joint

```


- gripper/gripper_cmd @ std_msgs.msg.Int32 (internal control)

```python
from std_msgs.msg import Int32

# example
gripper_cmd = 1 # 0: open, 1: 3f_close, 2: 2f_close, 3: parallel_close

send_msg = Int32()
send_msg.data = gripper_cmd

```

## internal control

### visualize (rviz2)

```bash
ros2 launch delto_3f_driver  delto_3f_bringup.launch.py delto_ip:=192.168.0.112 delto_port:=502 delto_id:=255

```

or 
### just driver 

```bash
ros2 run delto_3f_driver delto_3f_driver.py --ros-args -p ip:=192.168.0.112 -p port:=502 -p slaveID:=255

```

## external control

```bash
ros2 run delto_3f_driver delto_3f_external_driver_node
```
