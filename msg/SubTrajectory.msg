# unique id within task
uint32 id

# associated names
string name

# associated cost
float32 cost

# name of task stage
string stage

# IDs of start and end state?
# Or have an array of them in Solution.msg?
#uint32 start_id
#uint32 end_id

# trajectory
moveit_msgs/RobotTrajectory trajectory
visualization_msgs/MarkerArray markers

# TODO: add PlanningScene diff w.r.t. previous end state?
