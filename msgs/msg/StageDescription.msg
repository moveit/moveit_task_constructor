# static description of a stage

# unique id within task
uint32 id

# parent id, parent_id == id means root
uint32 parent_id

# name of this stage
string name

# flags: interface, ...
uint32 flags

# properties
Property[] properties
