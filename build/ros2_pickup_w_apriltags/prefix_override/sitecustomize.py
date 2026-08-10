import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ri-one/ros2_apriltags_to_pickup/install/ros2_pickup_w_apriltags'
