#! /usr/bin/env python3

import rospkg
from yaml import load, dump
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

rospack = rospkg.RosPack()
stream = open(rospack.get_path('soundcam_ros') + '/config/config.yaml')
cfgContext = load(stream, Loader=Loader)

if __name__ == '__main__':
    print(cfgContext)


