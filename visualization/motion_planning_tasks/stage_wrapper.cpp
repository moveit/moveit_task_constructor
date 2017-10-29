#include <stdio.h>

#include "stage_wrapper.h"
#include <moveit_task_constructor/Task.h>

namespace moveit_rviz_plugin {

size_t RemoteStage::numChildren() const
{
	return children_.size();
}

RemoteStage *RemoteStage::child(size_t index)
{
	return children_.at(index).get();
}

bool RemoteStage::setName(const std::string &name)
{
	if (name == name_) return false;
	name_ = name;
	return true; // indicate change
}

size_t RemoteStage::numSolved() const
{
	return 0;
}

size_t RemoteStage::numFailed() const
{
	return 0;
}

}
