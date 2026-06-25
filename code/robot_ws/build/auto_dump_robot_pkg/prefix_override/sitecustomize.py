import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chaesong/auto-dump-bot/code/robot_ws/install/auto_dump_robot_pkg'
