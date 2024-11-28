================
Delto 3F Gripper
================


Introduction
------------

.. figure:: /resource/dg3f.png
   :width: 600px
   :align: center

The Delto 3F gripper is an advanced three-fingered robotic gripper designed for versatile manipulation tasks. With its anthropomorphic design and sophisticated control system, it provides exceptional dexterity and precision in grasping various objects.

Features
--------

* **Multi-Joint Control**
    - 12 joints in total (4 joints per finger)
    - Individual joint angle Control
    - position feedback
* **High Precision**
    - Advanced PD control system
    - Adjustable gains for each joint
    - Position accuracy of ±0.1 degrees
* **Monitoring**
    - Joint angle visualization
    - Current vs target position comparison
    - Integrated RViz support

Technical Specifications
------------------------

Joint Configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
::

    Finger 1 & 2 & 3:
        - Joint 1
        - Joint 2
        - Joint 3
        - Joint 4

Control System
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- **Communication**: ROS 2 interface
- **Input Voltage**: 24V DC
- **Control Method**: Position control with PD/PID gains 

Software Interface
-------------------------------------

The gripper can be controlled through:

1. RQT Plugin Interface
    - joint control
    - Visual feedback
    - Gain tuning interface

2. ROS 2 Topics
    - Direct command interface
    - ros2_control interface

Applications
---------------------------------------

The Delto 3F gripper is suitable for:

- Research and Development
    * Robot manipulation studies
    * Grasping algorithm development
    * Human-robot interaction research
- Industrial Applications
    * Precision assembly
    * Object manipulation
    * Quality control inspection
- Educational Purposes
    * Robotics education
    * Control system demonstrations
    * Programming exercises

Performance Metrics
------------------------------------
::

    Maximum Payload: 10kg
    Position Resolution: 0.1 degrees

Installation & Setup
------------------------------------

Requirements
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
- ROS 2 (Humble)
- Ubuntu 22.04 
- Python 3.8+

Quick Start
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
::

    # Install the package
    cd ~/ros2_ws/src
    git clone <repository_url>
    colcon build
    # Launch the gripper control
    ros2 launch delto_3f_gripper bringup.launch.py

Support & Documentation
--------------------------------

For more information:

* `User Manual <https://en.tesollo.com/DG-3F>`_
* Coming soon: API Documentation

Contact Information
------------------------------

For technical support and inquiries:

:Email: khc@tesollo.com
:Website: https://www.en.tesollo.com/

.. note::
   This documentation is maintained by Tesollo.
   Last updated: November 2024

|

.. figure:: /resource/tesollo_logo.png
   :width: 600px
   :align: center

|
