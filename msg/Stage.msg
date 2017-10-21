# unique id within task
uint32 id

# name of this stage
string name

# flags: interface, ...
uint32 flags

# parent id, parent_id == id means root
uint32 parent_id

# ids of received start / end states
uint32[] received_starts
uint32[] received_ends
# ids of generated start / end states
uint32[] generated_starts
uint32[] generated_ends

# successful and failed solutions of this stage
Solution[] solutions
Solution[] failures
