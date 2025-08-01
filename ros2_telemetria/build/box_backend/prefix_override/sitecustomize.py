import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pedroromero/telemetria_eracing/ros2_telemetria_backend/install/box_backend'
