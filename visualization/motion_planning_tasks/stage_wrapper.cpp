#include <stdio.h>

#include "stage_wrapper.h"
#include <moveit_task_constructor/container.h>

namespace moveit_rviz_plugin {

size_t RemoteStage::numChildren() const {
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


size_t LocalStage::numChildren() const {
	return children_.size();
}

StageWrapperInterface *LocalStage::child(size_t index)
{
	return children_.at(index).get();
}

bool LocalStage::setName(const std::string &name)
{
	if (stage_->name() == name)
		return false;

	stage_->setName(name);
	return stage_->name() == name;
}

size_t LocalStage::numSolved() const
{
	return stage_->numSolutions();
}

size_t LocalStage::numFailed() const
{
	return 0;
}

}
