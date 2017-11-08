# dynamically changing information for a stage

# unique id within task
uint32 id

# parent id, parent_id == id means root
uint32 parent_id

# The order of the following IDs will change depending on their associated cost.
# However, the IDs themselves will always refer to the same underlying entity.

# IDs of received start / end states
uint32[] received_starts
uint32[] received_ends
# IDs of generated start / end states
uint32[] generated_starts
uint32[] generated_ends

# successful and failed solution IDs of this stage
uint32[] solved
uint32[] failed
