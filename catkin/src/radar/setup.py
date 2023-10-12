#!/usr/bin/env python
import rospy
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['radar.radariq'],
    package_dir={
        'radar':'./usr/local/lib/python3.6/dist-packages'
        }
)
setup(**d)
