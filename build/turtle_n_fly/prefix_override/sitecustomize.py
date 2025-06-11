import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bigmist/Documents/GitHub/turtle_n_fly/install/turtle_n_fly'
