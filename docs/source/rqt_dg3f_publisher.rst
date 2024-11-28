======================
RQT DG3F Publisher
======================

Overview
--------
The RQT DG3F Publisher is a graphical interface plugin for controlling and monitoring the DELTO 3F gripper system. It provides real-time visualization of joint states, control interfaces, and integrated RViz support.

Features
--------
* Real-time Joint Visualization
    - Three separate plots for each finger
    - Live updates of joint angles
    - Target vs. actual position comparison
    - Adjustable plotting ranges

* Joint Control Interface
    - Individual joint control via sliders (-90° to 90°)
    - Direct value input through text fields
    - Current position feedback display
    - Target position setting

* Grasp Control
    - Grasp/Ungrasp functionality
    - Status monitoring
    - Emergency stop capability

ROS Interface
--------------

Publishers
~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Topic
     - Message Type
     - Description
   * - ``gripper/target_joint``
     - Float32MultiArray
     - Target joint angles
   * - ``gripper/grasp``
     - Bool
     - Grasp command signal
   * - ``gripper/idle_target_joint``
     - Float32MultiArray
     - Idle position joints
   * - ``gripper/write_register``
     - Int16MultiArray
     - Register write commands
   * - ``gripper/request/gain``
     - Int16MultiArray
     - PD gain settings
   * - ``gripper/fixed_joint``
     - Int16MultiArray
     - Fixed joint configuration

Subscribers
~~~~~~~~~~
.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Topic
     - Message Type
     - Description
   * - ``/gripper/joint_states``
     - JointState
     - Current joint states

Plugin Components
----------------

Joint Plot Class
~~~~~~~~~~~~~~
.. code-block:: python

    class JointPlot(FigureCanvas):
        """Real-time plotting widget for joint angles visualization."""

Main features:
    - data plotting
    - Customizable display ranges
    - Multiple joint tracking
    - Target position overlay

RQT Plugin Class
~~~~~~~~~~~~~~
.. code-block:: python

    class RqtDelto3FPublisher(Plugin):
        """Main RQT plugin class for the DG3F gripper interface."""

Key functionalities:
    - Joint state monitoring
    - Control command publishing
    - GUI event handling
    - RViz integration

Installation
------------
Dependencies
~~~~~~~~~~~
- ROS 2 (Humble or newer)
- Python 3.8+
- Required Python packages:
    * python_qt_binding
    * matplotlib
    * rqt_gui
    * rqt_gui_py

    For a complete list of dependencies, refer to the `requirements.txt` file located in the root of the repository. To install the required packages, run:

    .. code-block:: bash

      pip install -r requirements.txt

Build Instructions
~~~~~~~~~~~~~~~~
.. code-block:: bash

    # Clone the repository to your ROS 2 workspace
    cd ~/ros2_ws/src
    git clone <repository_url>

    # Build the package
    cd ~/ros2_ws
    colcon build --packages-select rqt_dg3f_publisher

    # Source the workspace
    source install/setup.bash

Usage
-----
Launch Methods
~~~~~~~~~~~~
1. Via RQT:
   
   .. code-block:: bash
   
       rqt --force-discover

   Then select: Plugins > Robot Tools > DG3F Publisher

  .. image:: resource/rqt_info_1.png
      :alt: RQT DG3F Publisher Interface
      :align: center

2. Direct launch:
   
   .. code-block:: bash
   
       ros2 run rqt_dg3f_publisher rqt_dg3f_publisher

Plugin Functions
~~~~~~~~~~~~~~

Visualization
************
- Real-time joint angle plots
- Target vs actual position comparison
- RViz integration for 3D visualization

Control Features
**************
- Individual joint control
- Grasp/ungrasp commands
- PD gain adjustment
- Fixed joint configuration

Configuration
-------------
RViz Configuration
~~~~~~~~~~~~~~~~
The default RViz configuration is located at:
``config/default.rviz``

UI Configuration
~~~~~~~~~~~~~~
The Qt UI file is located at:
``resource/delto_rqt.ui``

Error Handling
-------------
The plugin includes error handling for:
- Invalid joint angles
- Communication failures
- Thread synchronization
- RViz process management

Contributing
-----------
1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Submit a pull request

Contact
-------
Maintainer: khc@tesollo.com
