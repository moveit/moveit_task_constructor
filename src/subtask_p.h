// copyright Robert Haschke @ 2017

#pragma once

#include <moveit_task_constructor/subtask.h>
#include <moveit_task_constructor/storage.h>

namespace moveit { namespace task_constructor {

class SubTaskPrivate {
	friend class SubTask;
	friend class SubTaskTest; // allow unit tests
	// ContainerBase will maintain the double-linked list of interfaces
	friend class ContainerBasePrivate;
	friend std::ostream& operator<<(std::ostream &os, const SubTaskPrivate& stage);

public:
	inline SubTaskPrivate(SubTask* me, const std::string& name)
	   : me_(me), name_(name), parent_(nullptr), predeccessor_(nullptr), successor_(nullptr)
	{}

	SubTrajectory& addTrajectory(const robot_trajectory::RobotTrajectoryPtr &, double cost);

	virtual SubTask::InterfaceFlags interfaceFlags() const;
	void sendForward(SubTrajectory& traj, const planning_scene::PlanningSceneConstPtr& ps);
	void sendBackward(SubTrajectory& traj, const planning_scene::PlanningSceneConstPtr& ps);

public:
	SubTask* const me_; // associated/owning SubTask instance
	const std::string name_;

	planning_scene::PlanningSceneConstPtr scene_;
	planning_pipeline::PlanningPipelinePtr planner_;

	InterfacePtr input_;
	InterfacePtr output_;
	std::list<SubTrajectory> trajectories_;

	const InterfacePtr prevOutput() const { return predeccessor_ ? predeccessor_->output_ : InterfacePtr(); }
	const InterfacePtr nextInput() const { return successor_ ? successor_->input_ : InterfacePtr(); }

private:
	// items accessed by ContainerBasePrivate only to maintain hierarchy
	SubTaskPrivate* parent_;
	SubTaskPrivate* predeccessor_;
	SubTaskPrivate* successor_;
};
std::ostream& operator<<(std::ostream &os, const SubTaskPrivate& stage);


class PropagatingAnyWayPrivate : public SubTaskPrivate {
	friend class PropagatingAnyWay;

public:
	inline PropagatingAnyWayPrivate(PropagatingAnyWay *me, const std::string &name)
	   : SubTaskPrivate(me, name)
	{
		input_.reset(new Interface([me](const Interface::iterator& it) { me->newInputState(it); }));
		output_.reset(new Interface(std::bind(&PropagatingAnyWay::newOutputState, me, std::placeholders::_1)));
		next_input_ = input_->begin();
		next_output_ = output_->end();
	}

	SubTask::InterfaceFlags interfaceFlags() const override {
		return SubTask::InterfaceFlags({SubTask::READS_INPUT, SubTask::WRITES_NEXT_INPUT,
		                               SubTask::READS_OUTPUT, SubTask::WRITES_PREV_OUTPUT,
		                               SubTask::WRITES_UNKNOWN});
	}

protected:
	Interface::const_iterator next_input_;
	Interface::const_iterator next_output_;
};


class PropagatingForwardPrivate : public PropagatingAnyWayPrivate {
public:
	inline PropagatingForwardPrivate(PropagatingForward *me, const std::string &name)
	   : PropagatingAnyWayPrivate(me, name)
	{
		// indicate, that we don't accept new states from output interface
		output_.reset();
		next_output_ = Interface::iterator();
	}

	SubTask::InterfaceFlags interfaceFlags() const override {
		return SubTask::InterfaceFlags({SubTask::READS_INPUT, SubTask::WRITES_NEXT_INPUT});
	}
};


class PropagatingBackwardPrivate : public PropagatingAnyWayPrivate {
public:
	inline PropagatingBackwardPrivate(PropagatingBackward *me, const std::string &name)
	   : PropagatingAnyWayPrivate(me, name)
	{
		// indicate, that we don't accept new states from input interface
		input_.reset();
		next_input_ = Interface::iterator();
	}

	SubTask::InterfaceFlags interfaceFlags() const override {
		return SubTask::InterfaceFlags({SubTask::READS_OUTPUT, SubTask::WRITES_PREV_OUTPUT});
	}
};


class GeneratorPrivate : public SubTaskPrivate {
public:
	inline GeneratorPrivate(Generator *me, const std::string &name)
	   : SubTaskPrivate(me, name)
	{}

	SubTask::InterfaceFlags interfaceFlags() const override {
		return SubTask::InterfaceFlags({SubTask::WRITES_NEXT_INPUT, SubTask::WRITES_PREV_OUTPUT});
	}
};


class ConnectingPrivate : public SubTaskPrivate {
	friend class Connecting;

public:
	inline ConnectingPrivate(Connecting *me, const std::string &name)
	   : SubTaskPrivate(me, name)
	{
		input_.reset(new Interface(std::bind(&Connecting::newInputState, me, std::placeholders::_1)));
		output_.reset(new Interface(std::bind(&Connecting::newOutputState, me, std::placeholders::_1)));
		it_pairs_ = std::make_pair(input_->begin(), output_->begin());
	}

	SubTask::InterfaceFlags interfaceFlags() const override {
		return SubTask::InterfaceFlags({SubTask::READS_INPUT, SubTask::READS_OUTPUT});
	}

private:
	std::pair<Interface::const_iterator, Interface::const_iterator> it_pairs_;
};

} }


// implement pimpl_func()
#define PRIVATE_CLASS_IMPL(Class) \
	inline Class##Private* Class::pimpl_func() { return static_cast<Class##Private*>(pimpl_); } \
	inline const Class##Private* Class::pimpl_func() const { return static_cast<const Class##Private*>(pimpl_); } \

// get correctly casted private impl pointer
#define IMPL(Class) Class##Private * const impl = pimpl_func()
