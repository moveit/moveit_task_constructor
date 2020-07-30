# unique id within task
uint32 id

# associated cost
float32 cost

# associated comment, usually providing failure hint
string comment

# id of stage that created this trajectory
uint32 stage_id

# markers, e.g. providing additional hints or illustrating failure
visualization_msgs/Marker[] markers

# Solution-specific values that provide additional information.
# These are either
# - Stage properties configured to initialize from INTERFACE
# - Properties that are relevant for trajectory execution,
#   e.g., to engage/disengage a suction gripper or trigger a camera
# - Stage-internal variables useful for introspection
Property[] properties
