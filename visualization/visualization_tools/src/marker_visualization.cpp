#include <moveit/visualization_tools/marker_visualization.h>
#include <moveit/planning_scene/planning_scene.h>

#include <rviz/default_plugin/markers/marker_base.h>
#include <rviz/default_plugin/marker_utils.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <tf2_msgs/TF2Error.h>
#include <ros/console.h>

namespace moveit_rviz_plugin {

// create MarkerData with nil marker_ pointer, just with a copy of message
MarkerVisualization::MarkerData::MarkerData(const visualization_msgs::Marker& marker) {
	msg_.reset(new visualization_msgs::Marker(marker));
	msg_->header.stamp = ros::Time();
	msg_->frame_locked = false;
}

MarkerVisualization::MarkerVisualization(const std::vector<visualization_msgs::Marker>& markers,
                                         const planning_scene::PlanningScene& end_scene) {
	planning_frame_ = end_scene.getPlanningFrame();
	// remember marker message, postpone rviz::MarkerBase creation until later
	for (const auto& marker : markers) {
		if (!end_scene.knowsFrameTransform(marker.header.frame_id)) {
			ROS_WARN_ONCE("unknown frame '%s' for solution marker in namespace '%s'", marker.header.frame_id.c_str(),
			              marker.ns.c_str());
			continue;  // ignore markers with unknown frame
		}

		// remember marker message
		markers_.emplace_back(marker);
		// remember namespace name
		namespaces_.insert(std::make_pair(marker.ns, NamespaceData()));
	}
}

MarkerVisualization::~MarkerVisualization() {
	for (const auto& pair : namespaces_) {
		if (pair.second.ns_node_)
			pair.second.ns_node_->getCreator()->destroySceneNode(pair.second.ns_node_);
	}
}

void setVisibility(Ogre::SceneNode* node, Ogre::SceneNode* parent, bool visible) {
	if (visible && node->getParent() != parent)
		parent->addChild(node);
	else if (!visible && node->getParent())
		node->getParent()->removeChild(node);
}

void MarkerVisualization::setVisible(const QString& ns, Ogre::SceneNode* parent_scene_node, bool visible) {
	auto it = namespaces_.find(ns.toStdString());
	if (it == namespaces_.end())
		return;
	setVisibility(it->second.ns_node_, parent_scene_node, visible);
}

bool MarkerVisualization::createMarkers(rviz::DisplayContext* context, Ogre::SceneNode* parent_scene_node) {
	if (markers_created_)
		return true;  // already called before

	// fetch transform from planning_frame_ to rviz' fixed frame
	const std::string& fixed_frame = context->getFrameManager()->getFixedFrame();
	Ogre::Quaternion quat;
	Ogre::Vector3 pos = Ogre::Vector3::ZERO;

	try {
		std::shared_ptr<tf2_ros::Buffer> tf = context->getFrameManager()->getTF2BufferPtr();
		geometry_msgs::TransformStamped tm;
		tm = tf->lookupTransform(planning_frame_, fixed_frame, ros::Time());
		auto q = tm.transform.rotation;
		auto p = tm.transform.translation;
		quat = Ogre::Quaternion(q.w, -q.x, -q.y, -q.z);
		pos = Ogre::Vector3(p.x, p.y, p.z);
	} catch (const tf2::TransformException& e) {
		ROS_WARN_STREAM_NAMED("MarkerVisualization", e.what());
		return false;
	}

	for (MarkerData& data : markers_) {
		if (data.marker_)
			continue;

		auto ns_it = namespaces_.find(data.msg_->ns);
		Q_ASSERT(ns_it != namespaces_.end());  // we have added all namespaces before!
		if (ns_it->second.ns_node_ == nullptr)  // create scene node for this namespace
			ns_it->second.ns_node_ = parent_scene_node->getCreator()->createSceneNode();
		Ogre::SceneNode* node = ns_it->second.ns_node_;

		// create a scene node for all markers with given frame name
		auto frame_it = ns_it->second.frames_.insert(std::make_pair(data.msg_->header.frame_id, nullptr)).first;
		if (frame_it->second == nullptr)
			frame_it->second = node->createChildSceneNode();
		node = frame_it->second;

		data.marker_.reset(rviz::createMarker(data.msg_->type, nullptr, context, node));
		if (!data.marker_)
			continue;  // failed to create marker

		// setMessage() initializes the marker, placing it at the message-specified frame
		// w.r.t. rviz' current fixed frame. However, we want to place the marker w.r.t.
		// the planning frame of the planning scene!

		// Hence, temporarily modify the message-specified frame to planning_frame_
		const std::string msg_frame = data.msg_->header.frame_id;
		data.msg_->header.frame_id = planning_frame_;
		data.marker_->setMessage(data.msg_);
		data.msg_->header.frame_id = msg_frame;

		// ... and subsequently revert any transform between rviz' fixed frame and planning_frame_
		data.marker_->setOrientation(quat * data.marker_->getOrientation());
		data.marker_->setPosition(quat * data.marker_->getPosition() + pos);
	}
	markers_created_ = true;
	return true;
}

void MarkerVisualization::update(MarkerData& data, const planning_scene::PlanningScene& scene,
                                 const moveit::core::RobotState& robot_state) const {
	Q_ASSERT(scene.getPlanningFrame() == planning_frame_);

	const visualization_msgs::Marker& marker = *data.msg_;
	if (marker.header.frame_id == scene.getPlanningFrame())
		return;  // no need to transform nodes placed at planning frame

	// fetch base pose from robot_state / scene
	Eigen::Affine3d pose;
	if (robot_state.knowsFrameTransform(marker.header.frame_id))
		pose = robot_state.getFrameTransform(marker.header.frame_id);
	else if (scene.knowsFrameTransform(marker.header.frame_id))
		pose = scene.getFrameTransform(marker.header.frame_id);
	else {
		ROS_WARN_ONCE_NAMED("MarkerVisualization", "unknown frame '%s' for solution marker in namespace '%s'",
		                    marker.header.frame_id.c_str(), marker.ns.c_str());
		return;  // ignore markers with unknown frame
	}

	auto ns_it = namespaces_.find(marker.ns);
	Q_ASSERT(ns_it != namespaces_.end());  // we have added all namespaces before
	auto frame_it = ns_it->second.frames_.find(marker.header.frame_id);
	Q_ASSERT(frame_it != ns_it->second.frames_.end());  // we have created all of them

	const Eigen::Quaterniond q{ pose.linear() };
	const Eigen::Vector3d& p = pose.translation();
	frame_it->second->setOrientation(Ogre::Quaternion(q.w(), q.x(), q.y(), q.z()));
	frame_it->second->setPosition(Ogre::Vector3(p.x(), p.y(), p.z()));
}

void MarkerVisualization::update(const planning_scene::PlanningScene& end_scene,
                                 const moveit::core::RobotState& robot_state) {
	for (MarkerData& data : markers_)
		update(data, end_scene, robot_state);
}

MarkerVisualizationProperty::MarkerVisualizationProperty(const QString& name, rviz::Property* parent)
  : rviz::BoolProperty(name, true, "Enable/disable markers", parent) {
	all_markers_at_once_ =
	    new rviz::BoolProperty("All at once?", false, "Show all markers of multiple subsolutions at once?", this,
	                           SLOT(onAllAtOnceChanged()), this);

	connect(this, SIGNAL(changed()), this, SLOT(onEnableChanged()));
}

MarkerVisualizationProperty::~MarkerVisualizationProperty() {
	if (marker_scene_node_)
		marker_scene_node_->getCreator()->destroySceneNode(marker_scene_node_);
}

void MarkerVisualizationProperty::onInitialize(Ogre::SceneNode* scene_node, rviz::DisplayContext* context) {
	context_ = context;
	parent_scene_node_ = scene_node;
	marker_scene_node_ = parent_scene_node_->createChildSceneNode();
}

void MarkerVisualizationProperty::clearMarkers() {
	// detach all existing scene nodes
	marker_scene_node_->removeAllChildren();
	// clear list of hosted markers
	hosted_markers_.clear();
}

void MarkerVisualizationProperty::addMarkers(const MarkerVisualizationPtr& markers) {
	if (!markers)
		return;

	// remember that those markers are hosted
	hosted_markers_.push_back(markers);
	// create markers if not yet done
	if (!markers->created() && !markers->createMarkers(context_, marker_scene_node_))
		return;  // if markers not created, nothing to do here

	// attach all scene nodes from markers
	for (const auto& pair : markers->namespaces()) {
		QString ns = QString::fromStdString(pair.first);
		// create sub property for newly encountered namespace, enabling visibility by default
		auto ns_it = namespaces_.insert(std::make_pair(ns, nullptr)).first;
		if (ns_it->second == nullptr) {
			ns_it->second = new rviz::BoolProperty(ns, true, "Show/hide markers of this namespace", this,
			                                       SLOT(onNSEnableChanged()), this);
		}
		Q_ASSERT(pair.second.ns_node_);  // nodes should have been created in createMarkers()

		if (ns_it->second->getBool())
			marker_scene_node_->addChild(pair.second.ns_node_);
	}
}

void MarkerVisualizationProperty::update(const planning_scene::PlanningScene& scene,
                                         const moveit::core::RobotState& robot_state) {
	for (const auto& markers : hosted_markers_) {
		if (!markers->created())
			if (!markers->createMarkers(context_, marker_scene_node_))
				continue;
		markers->update(scene, robot_state);
	}
}

bool MarkerVisualizationProperty::allAtOnce() const {
	return all_markers_at_once_->getBool();
}

void MarkerVisualizationProperty::onEnableChanged() {
	setVisibility(marker_scene_node_, parent_scene_node_, getBool());
}

void MarkerVisualizationProperty::onNSEnableChanged() {
	rviz::BoolProperty* ns_property = static_cast<rviz::BoolProperty*>(sender());
	const QString& ns = ns_property->getName();
	bool visible = ns_property->getBool();
	// for all hosted markers, set visibility of given namespace
	for (const auto& markers : hosted_markers_)
		markers->setVisible(ns, marker_scene_node_, visible);
}

void MarkerVisualizationProperty::onAllAtOnceChanged() {
	Q_EMIT allAtOnceChanged(all_markers_at_once_->getBool());
}

}  // namespace moveit_rviz_plugin
