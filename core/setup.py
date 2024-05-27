#!/usr/bin/env python

from setuptools import find_packages, setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # list of packages to setup
    packages=find_packages("python/src"),
    # specify location of root ("") package dir
    package_dir={"": "python/src"},
)
dist = setup(**d)

# Remove moveit/__init__.py when building .deb packages
# Otherwise, the installation procedure will complain about conflicting files (with moveit_core)
try:
    # installation dir (.../lib/python3/dist-packages)
    libdir = dist.command_obj["install_lib"].install_dir
    if "/debian/ros-" in libdir and "moveit-task-constructor-core/opt/ros/" in libdir:
        import os
        import shutil

        os.remove(os.path.join(libdir, "moveit", "__init__.py"))
        shutil.rmtree(os.path.join(libdir, "moveit", "__pycache__"))
except AttributeError as e:
    pass
