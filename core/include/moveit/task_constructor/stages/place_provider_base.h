//
// Created by jafar_abdi on 3/27/21.
//

#ifndef MOVEIT_TASK_CONSTRUCTOR_CORE_PLACE_PROVIDER_BASE_H
#define MOVEIT_TASK_CONSTRUCTOR_CORE_PLACE_PROVIDER_BASE_H
namespace moveit {
namespace task_constructor {
namespace stages {
class PlaceProviderBase : public GeneratePose
{
public:
	PlaceProviderBase(const std::string& name = "place pose");

	void setObject(const std::string& object) { setProperty("object", object); }

protected:
	void onNewSolution(const SolutionBase& s) override;
};
}}}
#include <moveit/task_constructor/stages/generate_pose.h>
#endif  // MOVEIT_TASK_CONSTRUCTOR_CORE_PLACE_PROVIDER_BASE_H
