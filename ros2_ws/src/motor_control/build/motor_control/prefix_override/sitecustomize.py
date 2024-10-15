import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ndev/luna/ros2_ws/src/motor_control/install/motor_control'
