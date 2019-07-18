#pragma once

#include <rviz/properties/bool_property.h>
#include <moveit/macros/class_forward.h>
#include <visualization_msgs/Marker.h>
#include <deque>
#include <list>
#include <memory>

namespace Ogre {
class SceneNode;
}

namespace rviz {
class DisplayContext;
class MarkerBase;
}

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene)
}
namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotState)
}
}

namespace moveit_rviz_plugin {

MOVEIT_CLASS_FORWARD(MarkerVisualization)

/** Container for all markers created from a vector of Marker messages
 *
 *  Markers within a specific namespace are created as children of a
 *  corresponding scene node, which allows for fast toggling of visibility.
 *  Placement of markers always refers to the frames of a (fixed) planning scene
 *  and is transformed once w.r.t. its planning frame during construction.
 */
class MarkerVisualization
{
	// list of all markers, attached to scene nodes in namespaces_
	struct MarkerData
	{
		visualization_msgs::MarkerPtr msg_;
		std::shared_ptr<rviz::MarkerBase> marker_;

		MarkerData(const visualization_msgs::Marker& marker);
	};
	struct NamespaceData
	{
		Ogre::SceneNode* ns_node_ = nullptr;
		// markers grouped by frame
		std::map<std::string, Ogre::SceneNode*> frames_;
	};

	// list of all markers
	std::deque<MarkerData> markers_;
	// markers grouped by their namespace
	std::map<std::string, NamespaceData> namespaces_;

	// planning_frame_ of scene
	std::string planning_frame_;
	// flag indicating that markers were created
	bool markers_created_ = false;

public:
	MarkerVisualization(const std::vector<visualization_msgs::Marker>& markers,
	                    const planning_scene::PlanningScene& end_scene);
	~MarkerVisualization();

	/// did we successfully created all markers (and scene nodes)?
	bool created() const { return markers_created_; }
	/// create markers (placed at planning frame of scene)
	bool createMarkers(rviz::DisplayContext* context, Ogre::SceneNode* scene_node);
	/// update marker position/orientation based on frames of given scene + robot_state
	void update(const planning_scene::PlanningScene& end_scene, const moveit::core::RobotState& robot_state);

	const std::map<std::string, NamespaceData>& namespaces() const { return namespaces_; }
	void setVisible(const QString& ns, Ogre::SceneNode* parent_scene_node, bool visible);

private:
	void update(MarkerData& data, const planning_scene::PlanningScene& end_scene,
	            const moveit::core::RobotState& robot_state) const;
};

/** rviz property allowing to group markers by their namespace
 *
 *  The class remembers which MarkerVisualization instances are currently hosted
 *  and provides the user interaction to toggle marker visibility by namespace.
 */
class MarkerVisualizationProperty : public rviz::BoolProperty
{
	Q_OBJECT

	rviz::DisplayContext* context_ = nullptr;
	Ogre::SceneNode* parent_scene_node_ = nullptr;  // scene node provided externally
	Ogre::SceneNode* marker_scene_node_ = nullptr;  // scene node all markers are attached to
	std::map<QString, rviz::BoolProperty*> namespaces_;  // rviz properties for encountered namespaces
	std::list<MarkerVisualizationPtr> hosted_markers_;  // list of hosted MarkerVisualization instances
	rviz::BoolProperty* all_markers_at_once_;

public:
	MarkerVisualizationProperty(const QString& name, Property* parent = nullptr);
	~MarkerVisualizationProperty();

	void onInitialize(Ogre::SceneNode* scene_node, rviz::DisplayContext* context);

	/// remove all hosted markers from display
	void clearMarkers();
	/// add markers in MarkerVisualization for display
	void addMarkers(MarkerVisualizationPtr markers);
	/// update pose of all markers
	void update(const planning_scene::PlanningScene& scene, const moveit::core::RobotState& robot_state);

	bool allAtOnce() const;

public Q_SLOTS:
	void onEnableChanged();
	void onNSEnableChanged();
	void onAllAtOnceChanged();

Q_SIGNALS:
	void allAtOnceChanged(bool);
};

}  // namespace moveit_rviz_plugin
