# dynamically changing information for a stage

# unique id within task
uint32 id

# parent id, parent_id == id means root
uint32 parent_id

# successful and failed solution IDs of this stage
uint32[] solved
uint32[] failed
