// copyright Robert Haschke @ 2017

#pragma once

#include <moveit_task_constructor/subtask.h>
#include <moveit_task_constructor/storage.h>

namespace moveit { namespace task_constructor {

class ContainerBasePrivate;
class SubTaskPrivate {
	friend class SubTask;
	friend class SubTaskTest; // allow unit tests
	friend class ContainerBasePrivate; // allow to set parent_ and it_
	friend std::ostream& operator<<(std::ostream &os, const SubTaskPrivate& stage);

public:
	typedef std::list<SubTask::pointer> array_type;

	inline SubTaskPrivate(SubTask* me, const std::string& name)
	   : me_(me), name_(name), parent_(nullptr)
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

	const SubTaskPrivate* prev() const;
	const SubTaskPrivate* next() const;
	const InterfacePtr prevOutput() const { const SubTaskPrivate* other = prev(); return other ? other->output_ : InterfacePtr(); }
	const InterfacePtr nextInput() const { const SubTaskPrivate* other = next(); return other ? other->input_ : InterfacePtr(); }

private:
	// items accessed by ContainerBasePrivate only to maintain hierarchy
	ContainerBasePrivate* parent_;
	array_type::iterator it_;
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


// get correctly casted private impl pointer
#define IMPL(Class) Class##Private * const impl = static_cast<Class##Private*>(pimpl_func());
