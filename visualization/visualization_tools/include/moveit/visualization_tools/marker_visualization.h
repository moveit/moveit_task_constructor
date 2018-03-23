#pragma once

#include <rviz/properties/bool_property.h>
#include <moveit/macros/class_forward.h>
#include <visualization_msgs/Marker.h>
#include <deque>
#include <list>
#include <memory>

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class DisplayContext;
class MarkerBase;
}

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene)
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
	typedef std::pair<visualization_msgs::MarkerConstPtr, std::shared_ptr<rviz::MarkerBase>> MarkerData;
	std::deque<MarkerData> markers_;
	// markers grouped by their namespace
	std::map<QString, Ogre::SceneNode*> namespaces_;

public:
	MarkerVisualization(const std::vector<visualization_msgs::Marker>& markers,
	                    const planning_scene::PlanningScene& end_scene);
	~MarkerVisualization();

	void createMarkers(rviz::DisplayContext* context, Ogre::SceneNode* scene_node);
	const std::map<QString, Ogre::SceneNode*>& namespaces() const { return namespaces_; }

	void setVisible(const QString &ns, Ogre::SceneNode* parent_scene_node, bool visible);
};


/** rviz property allowing to group markers by their namespace
 *
 *  The class remembers which MarkerVisualization instances are currently hosted
 *  and provides the user interaction to toggle marker visibility by namespace.
 */
class MarkerVisualizationProperty: public rviz::BoolProperty
{
	Q_OBJECT

	rviz::DisplayContext* context_ = nullptr;
	Ogre::SceneNode* parent_scene_node_ = nullptr; // scene node provided externally
	Ogre::SceneNode* marker_scene_node_ = nullptr; // scene node all markers are attached to
	std::map<QString, rviz::BoolProperty*> namespaces_; // rviz properties for encountered namespaces
	std::list<MarkerVisualizationPtr> hosted_markers_; // list of hosted MarkerVisualization instances

public:
	MarkerVisualizationProperty(const QString& name, Property* parent = nullptr);
	~MarkerVisualizationProperty();

	void onInitialize(Ogre::SceneNode* scene_node, rviz::DisplayContext* context);

	/// remove all hosted markers from display
	void clearMarkers();
	/// add all markers in MarkerVisualization for display
	void addMarkers(MarkerVisualizationPtr markers);

public Q_SLOTS:
	void onEnableChanged();
	void onNSEnableChanged();
};

} // namespace moveit_rviz_plugin
