import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/root123/桌面/AIC/源码（国赛）/install/package_detector'
