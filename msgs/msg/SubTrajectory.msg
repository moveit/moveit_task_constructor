# unique id within task
uint32 id

# associated cost
float32 cost

# associated comment, usually providing failure hint
string comment

# id of stage that created this trajectory
uint32 stage_id

# trajectory
moveit_msgs/RobotTrajectory trajectory
# markers, e.g. providing additional hints or illustrating failure
visualization_msgs/Marker[] markers

# planning scene of end state as diff w.r.t. start state
moveit_msgs/PlanningScene scene_diff
