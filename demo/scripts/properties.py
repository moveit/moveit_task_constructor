#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped
import time

from moveit.python_tools import roscpp_init

roscpp_init("mtc_tutorial")

# Create a task container
task = core.Task()

# Create a current state to capture the current planning scene state
currentState = stages.CurrentState("Current State")

# Create a property
p = core.Property()

# Set a descriptive string to describe the properties function
p.setDescription("Foo Property")

# Set the current and the default value
p.setValue("Bar")

# Check if the property is defined
assert p.defined()

# Retrieve the stored value
print(p.value())

# Retrieve the default value
print(p.defaultValue())

# Retrieve the description
print(p.description())

# Create a property map
pm = core.PropertyMap()
props = {"prop1": "test", "prop2": 21, "prop3": PoseStamped(), "prop4": 5.4}
pm.update(props)

# Add a property to the property map using the pythonic way
pm["prop5"] = 2

# Return the value of a property
print(pm["prop5"])

# Return the underlying property object
p2 = pm.property("prop5")

# Iterate through all the values in the property map
print("\n")
for i in pm:
    print(i, "\t\t", pm[i])
print("\n")

# A new property map can also be configured using an existing one
# You can also only use a subset of the properties that should be configured.
pm2 = core.PropertyMap()
pm.exposeTo(pm2, ["prop2", "prop4"])

# Lets test that by printing out our properties
for i in pm2:
    print(i, "\t\t", pm2[i])
print("\n")

# Access the property map of the stage
props = currentState.properties

# Add the stage to the task hierarchy
task.add(currentState)

if task.plan():
    task.publish(task.solutions[0])

time.sleep(100)
