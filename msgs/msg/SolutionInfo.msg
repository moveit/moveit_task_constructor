# unique id within task
uint32 id

# associated cost
float32 cost

# associated comment, usually providing failure hint
string comment

# id of stage that created this trajectory
uint32 stage_id

# name of the planner that created this solution
string planner_id

# markers, e.g. providing additional hints or illustrating failure
visualization_msgs/Marker[] markers
