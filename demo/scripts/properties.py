#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped
import time

from moveit_commander.roscpp_initializer import roscpp_initialize

roscpp_initialize("mtc_tutorial")

# Create a task container
task = core.Task()

# [propertyTut10]
# Create a current state to capture the current planning scene state
currentState = stages.CurrentState("Current State")
# [propertyTut10]

# [propertyTut1]
# Create a property
p = core.Property()

# Set a descriptive string to describe the properties function
p.setDescription("Foo Property")
# [propertyTut1]

# Set the current and the default value
p.setValue("Bar")

# [propertyTut2]
# Check if the property is defined
assert p.defined()
# [propertyTut2]

# [propertyTut3]
# Retrieve the stored value
print(p.value())

# Retrieve the default value
print(p.defaultValue())

# Retrieve the description
print(p.description())
# [propertyTut3]

# [propertyTut4]
# Create a property map
pm = core.PropertyMap()
props = {"prop1": "test", "prop2": 21, "prop3": PoseStamped(), "prop4": 5.4}
pm.update(props)
# [propertyTut4]

# [propertyTut5]
# Add a property to the property map using the pythonic way
pm["prop5"] = 2
# [propertyTut5]

# [propertyTut6]
# Return the value of a property
print(pm["prop5"])
# [propertyTut6]

# [propertyTut7]
# Return the underlying property object
p2 = pm.property("prop5")
# [propertyTut7]

# [propertyTut8]
# Iterate through all the values in the property map
print("\n")
for i in pm:
    print(i, "\t\t", pm[i])
print("\n")
# [propertyTut8]

# [propertyTut9]
# A new property map can also be configured using an existing one
# You can also only use a subset of the properties that should be configured.
pm2 = core.PropertyMap()
pm.exposeTo(pm2, ["prop2", "prop4"])
# [propertyTut9]

# Lets test that by printing out our properties
for i in pm2:
    print(i, "\t\t", pm2[i])
print("\n")

# [propertyTut11]
# Access the property map of the stage
props = currentState.properties
# [propertyTut11]

# Add the stage to the task hierarchy
task.add(currentState)

if task.plan():
    task.publish(task.solutions[0])

time.sleep(100)
