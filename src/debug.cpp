#include <moveit_task_constructor/debug.h>
#include <moveit_task_constructor/task.h>

#include <ros/ros.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>

namespace moveit { namespace task_constructor {

bool publishSolution(ros::Publisher& pub, moveit_msgs::DisplayTrajectory& dt,
                     const std::vector<SubTrajectory*>& solution, bool wait){
		dt.trajectory.clear();
		for(const SubTrajectory* t : solution){
			if(t->trajectory){
				dt.trajectory.emplace_back();
				t->trajectory->getRobotTrajectoryMsg(dt.trajectory.back());
			}
		}

		std::cout << "publishing solution" << std::endl;
		pub.publish(dt);
		if (wait) {
			std::cout << "Press <Enter> to continue ..." << std::endl;
			getchar();
		}
		return true;
}

void publishAllPlans(const Task &task, const std::string &topic, bool wait) {
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<moveit_msgs::DisplayTrajectory>(topic, 1, true);
	moveit_msgs::DisplayTrajectory dt;
	robot_state::robotStateToRobotStateMsg(task.getCurrentRobotState(), dt.trajectory_start);
	dt.model_id= task.getCurrentRobotState().getRobotModel()->getName();

	Task::SolutionCallback processor = std::bind(
		&publishSolution, pub, dt, std::placeholders::_1, wait);

	task.processSolutions(processor);
}


NewSolutionPublisher::NewSolutionPublisher(const Task &task, const std::string &topic)
   : task_(task)
{
	ros::NodeHandle n;
	pub_ = n.advertise<moveit_msgs::DisplayTrajectory>(topic, 1, true);
}

void NewSolutionPublisher::publish()
{
	moveit_msgs::DisplayTrajectory dt;
	robot_state::robotStateToRobotStateMsg(task_.getCurrentRobotState(), dt.trajectory_start);
	dt.model_id = task_.getCurrentRobotState().getRobotModel()->getName();

	Task::SolutionCallback processor = [this,&dt](const std::vector<SubTrajectory*>& solution) {
		bool all_published = true;
		for(const SubTrajectory* t : solution){
			auto result = published_.insert(t);
			// if t was not yet published, the insertion yields result.second == true
			all_published &= !result.second;
		}

		if(all_published)
			return true;

		return publishSolution(pub_, dt, solution, false);
	};
	task_.processSolutions(processor);
}

} }
