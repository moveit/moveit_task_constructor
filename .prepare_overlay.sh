#!/bin/sh
# we need an unreleased version of moveit_core
mkdir moveit
pushd moveit
git init
git remote add -f origin https://github.com/ros-planning/moveit.git
git config core.sparseCheckout true
echo "moveit_core" >> .git/info/sparse-checkout
git pull origin ${ROS_DISTRO}-devel
popd
