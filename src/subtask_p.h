// copyright Robert Haschke @ 2017

#pragma once

#include <moveit_task_constructor/subtask.h>
#include <moveit_task_constructor/storage.h>

namespace moveit { namespace task_constructor {

class ContainerBasePrivate;
class SubTaskPrivate {
	friend class SubTask;
	friend class BaseTest; // allow access for unit tests
	friend class ContainerBasePrivate; // allow to set parent_ and it_
	friend std::ostream& operator<<(std::ostream &os, const SubTaskPrivate& stage);

public:
	typedef std::list<SubTask::pointer> container_type;

	SubTaskPrivate(SubTask* me, const std::string& name);

	enum InterfaceFlag {
		READS_START        = 0x01,
		READS_END          = 0x02,
		WRITES_NEXT_START  = 0x04,
		WRITES_PREV_END    = 0x08,

		OWN_IF_MASK        = READS_START | READS_END,
		EXT_IF_MASK        = WRITES_NEXT_START | WRITES_PREV_END,
		INPUT_IF_MASK      = READS_START | WRITES_PREV_END,
		OUTPUT_IF_MASK     = READS_END | WRITES_NEXT_START,
	};
	typedef Flags<InterfaceFlag> InterfaceFlags;

	InterfaceFlags interfaceFlags() const;
	InterfaceFlags deducedFlags() const;
	virtual InterfaceFlags announcedFlags() const = 0;
	std::list<SubTrajectory>& trajectories() { return trajectories_; }

	virtual bool canCompute() const = 0;
	virtual bool compute() = 0;

public:
	SubTask* const me_; // associated/owning SubTask instance
	const std::string name_;

	InterfacePtr starts_;
	InterfacePtr ends_;

	inline ContainerBasePrivate* parent() const { return parent_; }
	inline bool isConnected() const { return prev_ends_ || next_starts_; }
	inline Interface* prevEnds() const { return prev_ends_; }
	inline Interface* nextStarts() const { return next_starts_; }
	SubTrajectory& addTrajectory(const robot_trajectory::RobotTrajectoryPtr &, double cost);

protected:
	std::list<SubTrajectory> trajectories_;

private:
	// !! items accessed only by ContainerBasePrivate to maintain hierarchy !!
	ContainerBasePrivate* parent_; // owning parent
	container_type::iterator it_; // iterator into parent's children_ list referring to this
	// caching the pointers to the ends_ / starts_ interface of previous / next stage
	mutable Interface *prev_ends_; // interface to be used for sendBackward()
	mutable Interface *next_starts_;  // interface to be use for sendForward()
};
std::ostream& operator<<(std::ostream &os, const SubTaskPrivate& stage);


class PropagatingEitherWayPrivate : public SubTaskPrivate {
	friend class PropagatingEitherWay;

public:
	PropagatingEitherWay::Direction dir;

	inline PropagatingEitherWayPrivate(PropagatingEitherWay *me, PropagatingEitherWay::Direction dir,
	                                const std::string &name);
	InterfaceFlags announcedFlags() const override;

	bool canCompute() const override;
	bool compute() override;

	bool hasStartState() const;
	const InterfaceState &fetchStartState();

	bool hasEndState() const;
	const InterfaceState &fetchEndState();

protected:
	// get informed when new start or end state becomes available
	void newStartState(const std::list<InterfaceState>::iterator& it);
	void newEndState(const std::list<InterfaceState>::iterator& it);

	Interface::const_iterator next_start_state_;
	Interface::const_iterator next_end_state_;
};


class PropagatingForwardPrivate : public PropagatingEitherWayPrivate {
public:
	inline PropagatingForwardPrivate(PropagatingForward *me, const std::string &name);
};


class PropagatingBackwardPrivate : public PropagatingEitherWayPrivate {
public:
	inline PropagatingBackwardPrivate(PropagatingBackward *me, const std::string &name);
};


class GeneratorPrivate : public SubTaskPrivate {
public:
	inline GeneratorPrivate(Generator *me, const std::string &name);
	InterfaceFlags announcedFlags() const override;

	bool canCompute() const override;
	bool compute() override;
};


class ConnectingPrivate : public SubTaskPrivate {
	friend class Connecting;

public:
	inline ConnectingPrivate(Connecting *me, const std::string &name);
	InterfaceFlags announcedFlags() const override;

	bool canCompute() const override;
	bool compute() override;

	void connect(const robot_trajectory::RobotTrajectoryPtr& t,
	             const InterfaceStatePair& state_pair, double cost);

private:
	// get informed when new start or end state becomes available
	void newStartState(const std::list<InterfaceState>::iterator& it);
	void newEndState(const std::list<InterfaceState>::iterator& it);

	std::pair<Interface::const_iterator, Interface::const_iterator> it_pairs_;
};

} }


// get correctly casted private impl pointer
#define IMPL(Class) Class##Private * const impl = static_cast<Class##Private*>(pimpl_func());
