// copyright Robert Haschke @ 2017

#pragma once

#include <moveit_task_constructor/stage.h>
#include <moveit_task_constructor/storage.h>

// define pimpl() functions accessing correctly casted pimpl_ pointer
#define PIMPL_FUNCTIONS(Class) \
	const Class##Private* Class::pimpl() const { return static_cast<const Class##Private*>(pimpl_); } \
	Class##Private* Class::pimpl() { return static_cast<Class##Private*>(pimpl_); } \

namespace moveit { namespace task_constructor {

class ContainerBase;
class StagePrivate {
	friend class Stage;
	friend std::ostream& operator<<(std::ostream &os, const StagePrivate& stage);

public:
	typedef std::list<Stage::pointer> container_type;
	StagePrivate(Stage* me, const std::string& name);
	virtual ~StagePrivate() = default;

	InterfaceFlags interfaceFlags() const;

	virtual bool canCompute() const = 0;
	virtual bool compute() = 0;

	inline const Stage* me() const { return me_; }
	inline Stage* me() { return me_; }
	inline const std::string& name() const { return name_; }
	inline const ContainerBase* parent() const { return parent_; }
	inline ContainerBase* parent() { return parent_; }
	inline container_type::const_iterator it() const { return it_; }

	inline InterfacePtr& starts() { return starts_; }
	inline InterfacePtr& ends() { return ends_; }
	inline InterfacePtr prevEnds() { return prev_ends_.lock(); }
	inline InterfacePtr nextStarts() { return next_starts_.lock(); }
	inline InterfaceConstPtr starts() const { return starts_; }
	inline InterfaceConstPtr ends() const { return ends_; }
	inline InterfaceConstPtr prevEnds() const { return prev_ends_.lock(); }
	inline InterfaceConstPtr nextStarts() const { return next_starts_.lock(); }

	/// validate that sendForward() and sendBackward() will succeed
	/// should be only called by containers' init() method
	void validate() const;

	/// the following methods should be called only by a container
	/// to setup the connection structure of their children
	inline void setHierarchy(ContainerBase* parent, container_type::iterator it) {
		parent_ = parent;
		it_ = it;
	}
	inline void setPrevEnds(const InterfacePtr& prev_ends) { prev_ends_ = prev_ends; }
	inline void setNextStarts(const InterfacePtr& next_starts) { next_starts_ = next_starts; }

protected:
	Stage* const me_; // associated/owning Stage instance
	std::string name_;

	InterfacePtr starts_;
	InterfacePtr ends_;

private:
	// !! items write-accessed only by ContainerBasePrivate to maintain hierarchy !!
	ContainerBase* parent_;       // owning parent
	container_type::iterator it_; // iterator into parent's children_ list referring to this

	InterfaceWeakPtr prev_ends_;    // interface to be used for sendBackward()
	InterfaceWeakPtr next_starts_;  // interface to be used for sendForward()
};
PIMPL_FUNCTIONS(Stage)
std::ostream& operator<<(std::ostream &os, const StagePrivate& stage);


// ComputeBasePrivate is the base class for all computing stages, i.e. non-containers.
// It adds the trajectories_ variable.
class ComputeBasePrivate : public StagePrivate {
	friend class ComputeBase;

public:
	ComputeBasePrivate(Stage* me, const std::string& name)
	   : StagePrivate(me, name)
	{}

private:
	std::list<SubTrajectory> trajectories_;
};
PIMPL_FUNCTIONS(ComputeBase)


class PropagatingEitherWayPrivate : public ComputeBasePrivate {
	friend class PropagatingEitherWay;

public:
	PropagatingEitherWay::Direction dir;

	inline PropagatingEitherWayPrivate(PropagatingEitherWay *me, PropagatingEitherWay::Direction dir,
	                                   const std::string &name);

	// returns true if prevEnds() or nextStarts() are accessible
	inline bool isConnected() const { return prevEnds() || nextStarts(); }

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
PIMPL_FUNCTIONS(PropagatingEitherWay)


class PropagatingForwardPrivate : public PropagatingEitherWayPrivate {
public:
	inline PropagatingForwardPrivate(PropagatingForward *me, const std::string &name);
};
PIMPL_FUNCTIONS(PropagatingForward)


class PropagatingBackwardPrivate : public PropagatingEitherWayPrivate {
public:
	inline PropagatingBackwardPrivate(PropagatingBackward *me, const std::string &name);
};
PIMPL_FUNCTIONS(PropagatingBackward)


class GeneratorPrivate : public ComputeBasePrivate {
public:
	inline GeneratorPrivate(Generator *me, const std::string &name);

	bool canCompute() const override;
	bool compute() override;
};
PIMPL_FUNCTIONS(Generator)


class ConnectingPrivate : public ComputeBasePrivate {
	friend class Connecting;

public:
	inline ConnectingPrivate(Connecting *me, const std::string &name);

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
PIMPL_FUNCTIONS(Connecting)

} }
