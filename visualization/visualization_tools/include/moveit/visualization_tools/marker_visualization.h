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

namespace moveit_rviz_plugin {

MOVEIT_CLASS_FORWARD(MarkerVisualization)

class MarkerVisualization
{
	// list of all markers, attached to scene nodes in namespaces_
	typedef std::pair<visualization_msgs::MarkerConstPtr, std::shared_ptr<rviz::MarkerBase>> MarkerData;
	std::deque<MarkerData> markers_;
	// markers grouped by their namespace
	std::map<QString, Ogre::SceneNode*> namespaces_;

public:
	MarkerVisualization(const std::vector<visualization_msgs::Marker>& markers);
	~MarkerVisualization();

	void createMarkers(rviz::DisplayContext* context, Ogre::SceneNode* scene_node);
	const std::map<QString, Ogre::SceneNode*>& namespaces() const { return namespaces_; }

	void setVisible(const QString &ns, Ogre::SceneNode* parent_scene_node, bool visible);
};


class MarkerVisualizationProperty: public rviz::BoolProperty
{
	Q_OBJECT

	rviz::DisplayContext* context_ = nullptr;
	Ogre::SceneNode* parent_scene_node_ = nullptr; // scene node provided externally
	Ogre::SceneNode* marker_scene_node_ = nullptr; // scene node all markers are attached to
	std::map<QString, rviz::BoolProperty*> namespaces_;
	std::list<MarkerVisualizationPtr> visible_markers_;

public:
	MarkerVisualizationProperty(const QString& name, Property* parent = nullptr);
	~MarkerVisualizationProperty();

	void onInitialize(Ogre::SceneNode* scene_node, rviz::DisplayContext* context);

	void clearMarkers();
	void showMarkers(MarkerVisualizationPtr markers);

public Q_SLOTS:
	void onEnableChanged();
	void onNSEnableChanged();
};

} // namespace moveit_rviz_plugin
