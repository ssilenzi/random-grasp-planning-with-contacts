#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# ROS PACKAGING
# using distutils : https://docs.python.org/2/distutils
# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        'edge_detection',
    ],
    package_dir={
        'edge_detection': 'src/edge_detection',
    }
)
setup(**setup_args)
