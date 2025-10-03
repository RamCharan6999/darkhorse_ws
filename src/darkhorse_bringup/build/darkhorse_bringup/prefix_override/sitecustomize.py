import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/darkhorse/DH_ws/src/darkhorse_bringup/install/darkhorse_bringup'
