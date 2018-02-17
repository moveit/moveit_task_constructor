#include <ostream>
#include <moveit/task_constructor/stage_p.h>

::std::ostream& operator<<(::std::ostream& os, const moveit::task_constructor::InterfaceFlag& flag);
::std::ostream& operator<<(::std::ostream& os, const moveit::task_constructor::InterfaceFlags& flags);
