#!/usr/bin/env python

from setuptools import find_packages, setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # list of packages to setup
    packages=find_packages("python/src"),
    # specify location of root ("") package dir
    package_dir={"": "python/src"},
)
setup(**d)
