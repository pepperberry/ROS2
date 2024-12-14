import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/endr/Seattle University/Innovation-Lab/ROS2/jazzy/src/install/pkg2'
