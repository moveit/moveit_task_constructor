// copyright Robert Haschke @ 2017

#pragma once

#include <moveit_task_constructor/stage.h>
#include <moveit_task_constructor/storage.h>

// define pimpl() functions accessing correctly casted pimpl_ pointer
#define PIMPL_FUNCTIONS(Class) \
	const Class##Private* Class::pimpl() const { return static_cast<const Class##Private*>(pimpl_); } \
	Class##Private* Class::pimpl() { return static_cast<Class##Private*>(pimpl_); } \

namespace moveit { namespace task_constructor {

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


class ContainerBasePrivate;
class StagePrivate {
	friend class Stage;
	friend std::ostream& operator<<(std::ostream &os, const StagePrivate& stage);

public:
	typedef std::list<Stage::pointer> container_type;
	StagePrivate(Stage* me, const std::string& name);

	InterfaceFlags interfaceFlags() const;
	InterfaceFlags deducedFlags() const;
	virtual InterfaceFlags announcedFlags() const = 0;

	virtual bool canCompute() const = 0;
	virtual bool compute() = 0;

	inline const std::string& name() const { return name_; }
	inline ContainerBasePrivate* parent() const { return parent_; }
	inline container_type::iterator it() const { return it_; }
	inline Interface* starts() const { return starts_.get(); }
	inline Interface* ends() const { return ends_.get(); }
	inline Interface* prevEnds() const { return prev_ends_; }
	inline Interface* nextStarts() const { return next_starts_; }

	inline bool isConnected() const { return prev_ends_ || next_starts_; }

	inline void setHierarchy(ContainerBasePrivate* parent, container_type::iterator it) {
		parent_ = parent;
		it_ = it;
	}
	inline void setPrevEnds(Interface * prev_ends) { prev_ends_ = prev_ends; }
	inline void setNextStarts(Interface * next_starts) { next_starts_ = next_starts; }

	virtual void append(const SolutionBase& s, std::vector<const SubTrajectory*>& solution) const = 0;

protected:
	Stage* const me_; // associated/owning Stage instance
	const std::string name_;

	InterfacePtr starts_;
	InterfacePtr ends_;

private:
	// !! items write-accessed only by ContainerBasePrivate to maintain hierarchy !!
	ContainerBasePrivate* parent_; // owning parent
	container_type::iterator it_; // iterator into parent's children_ list referring to this

	Interface *prev_ends_;    // interface to be used for sendBackward()
	Interface *next_starts_;  // interface to be used for sendForward()
};
std::ostream& operator<<(std::ostream &os, const StagePrivate& stage);


// ComputeBasePrivate is the base class for all computing stages, i.e. non-containers.
// It adds the trajectories_ variable.
class ComputeBasePrivate : public StagePrivate {
	friend class ComputeBase;

public:
	ComputeBasePrivate(Stage* me, const std::string& name)
	   : StagePrivate(me, name)
	{}
	void append(const SolutionBase& s, std::vector<const SubTrajectory*>& solution) const override {
		assert(s.creator() == this);
		solution.push_back(static_cast<const SubTrajectory*>(&s));
	}

private:
	std::list<SubTrajectory> trajectories_;
};


class PropagatingEitherWayPrivate : public ComputeBasePrivate {
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


class GeneratorPrivate : public ComputeBasePrivate {
public:
	inline GeneratorPrivate(Generator *me, const std::string &name);
	InterfaceFlags announcedFlags() const override;

	bool canCompute() const override;
	bool compute() override;
};


class ConnectingPrivate : public ComputeBasePrivate {
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

PIMPL_FUNCTIONS(Stage)

} }
