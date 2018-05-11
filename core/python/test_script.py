from _core import *
from geometry_msgs.msg import Pose

roscpp_init()
task = Task()
print task.name

props = task.properties
props["float"] = 3.14
props["int"] = 42
props["bool"] = True
props["pose"] = Pose()

print props["float"]
print props["int"]
print props["bool"]
print props["pose"], type(props["pose"])

roscpp_shutdown()
