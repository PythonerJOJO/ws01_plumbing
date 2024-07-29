import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dai/ros2_study/ws01_plumbing/install/py01_topic'
