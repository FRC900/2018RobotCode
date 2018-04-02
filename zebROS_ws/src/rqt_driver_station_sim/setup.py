#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rqt_driver_station_sim'],
    package_dir={'': 'src'},
    scripts=['scripts/rqt_driver_station_sim']
)

setup(**setup_args)
