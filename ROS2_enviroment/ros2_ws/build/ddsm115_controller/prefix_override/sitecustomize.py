import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/feri/Projekt_1/ROS2_enviroment/ros2_ws/install/ddsm115_controller'
