#!/usr/bin/python 
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['omniveyor_gui'],
    package_dir={'': 'src'},
    scripts=['./scripts/omniveyor_gui']
)

setup(**d)
