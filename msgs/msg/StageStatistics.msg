# dynamically changing information for a stage

# unique id within task
uint32 id

# successful solution IDs of this stage, sorted by increasing cost
uint32[] solved

# (optional) failed solution IDs of this stage
uint32[] failed
# number of failed solutions (if failed is empty)
uint32   num_failed
# total computation time in seconds
float64 total_compute_time
