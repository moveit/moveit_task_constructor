# id of generating task
string task_id

# planning scene of start state
moveit_msgs/PlanningScene start_scene

# set of all sub solutions involved
SubSolution[] sub_solution

# (ordered) sequence of actual trajectories
SubTrajectory[] sub_trajectory
