#pragma once

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

/** Wrapper class used in QAbstractItemModel of rviz::Panel to show the task tree */
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
