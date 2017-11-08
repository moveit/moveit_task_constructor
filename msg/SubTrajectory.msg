# unique id within task
uint32 id

# associated cost
float32 cost

# optional associated name of sub trajectory
string name

# trajectory
moveit_msgs/RobotTrajectory trajectory
# markers, e.g. providing additional hints or illustrating failure
visualization_msgs/MarkerArray markers

# ID of end state
uint32 end_id
# planning scene of end state as diff w.r.t. start state
moveit_msgs/PlanningScene scene_diff
