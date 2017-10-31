#pragma once

#include <moveit_task_constructor/stage.h>
#include <moveit_task_constructor/task.h>
#include <vector>
#include <map>
#include <memory>
#include <qflags.h>

namespace moveit_rviz_plugin {

enum StageFlag {
	READS_START        = 0x01,
	READS_END          = 0x02,
	WRITES_NEXT_START  = 0x04,
	WRITES_PREV_END    = 0x08,

	IS_EDITABLE        = 0x10,
	IS_DESTROYED       = 0x20, // indicate that creator of this stage was destroyed
	WAS_VISITED        = 0x40, // indicate that model should emit change notifications
};
typedef QFlags<StageFlag> StageFlags;

/** Wrapper class used in QAbstractItemModel of rviz::Panel to show the task tree.
 *  The abstract base class allows transparent access to both local and remote tasks/stages.
 */
class StageWrapperInterface {
public:
	virtual ~StageWrapperInterface() {}

	virtual StageWrapperInterface* parent() = 0;
	virtual size_t numChildren() const = 0;
	virtual StageWrapperInterface* child(size_t index) = 0;

	virtual const std::string& name() const = 0;
	virtual bool setName(const std::string& name) = 0;
	virtual size_t numSolved() const = 0;
	virtual size_t numFailed() const = 0;

	mutable StageFlags flags;
};


class LocalStage : public StageWrapperInterface {
private:
	LocalStage *parent_;
	std::vector<std::unique_ptr<LocalStage>> children_;
protected:
	moveit::task_constructor::Stage* stage_;

public:
	LocalStage(LocalStage *parent, moveit::task_constructor::Stage* stage)
	   : parent_(parent), stage_(stage)
	{}

	virtual StageWrapperInterface* parent() { return parent_; }
	virtual size_t numChildren() const;
	virtual StageWrapperInterface* child(size_t index);

	virtual const std::string& name() const { return stage_->name(); }
	virtual bool setName(const std::string& name);
	virtual size_t numSolved() const;
	virtual size_t numFailed() const;
};

// root stage of a tree of local stages
class LocalTask : public LocalStage {
	std::unique_ptr<moveit::task_constructor::Task> task_;

public:
	LocalTask()
	   : LocalStage(nullptr, nullptr)
	   , task_(std::make_unique<moveit::task_constructor::Task>())
	{
		stage_ = task_->stages();
	}
};


class RemoteStage : public StageWrapperInterface {
	std::string name_;
	RemoteStage *parent_;
	std::vector<std::unique_ptr<RemoteStage>> children_;

public:
	RemoteStage(RemoteStage *parent) : parent_(parent) {}

	RemoteStage* parent() override { return parent_; }
	size_t numChildren() const override;
	RemoteStage* child(size_t index) override;
	void push_back(std::unique_ptr<RemoteStage>&& child) {
		children_.push_back(std::move(child));
	}

	const std::string& name() const override { return name_; }
	bool setName(const std::string& name) override;

	size_t numSolved() const override;
	size_t numFailed() const override;
};

// root stage of a tree of remote stages
class RemoteTask : public RemoteStage {
	friend class TaskModel;
	std::map<uint32_t, RemoteStage*> id_to_stage;

public:
	RemoteTask() : RemoteStage(nullptr) {}
};

}
