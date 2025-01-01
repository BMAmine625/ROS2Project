import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspaces/ROS2Project/ros_ws/install/hvac_actuators'
