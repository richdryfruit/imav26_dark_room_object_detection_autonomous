import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ark-jetson-orin-2/offboard_imav26_test/install/drone_testing'
