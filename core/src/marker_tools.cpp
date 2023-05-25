#include <moveit/task_constructor/marker_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>

namespace vm = visualization_msgs;

namespace moveit {
namespace task_constructor {

/** generate marker msgs to visualize the planning scene, calling the given callback for each of them
 *  object_names: set of links to include (or all if empty) */
void generateMarkersForObjects(const planning_scene::PlanningSceneConstPtr& scene, const MarkerCallback& /*callback*/,
                               const std::vector<std::string>& /*object_names*/) {
	scene->printKnownObjects(std::cout);
	/*
	   const std::vector<std::string>* names = object_names.empty() ? &scene->getCollisionObjectMsg()
	                                                              : &link_names;
	   for (const auto &name : *names) {
	      visualization_msgs::msg::MarkerArray markers;
	      robot_state.getRobotMarkers(markers, {name}, false);
	      for (auto &marker : markers.markers)
	         callback(marker, name);
	   }
	*/
}

visualization_msgs::msg::Marker& createGeometryMarker(visualization_msgs::msg::Marker& marker,
                                                      const urdf::Geometry& geom, const urdf::Pose& pose,
                                                      const urdf::Color& color) {
	rviz_marker_tools::makeFromGeometry(marker, geom);
	marker.pose.position.x = pose.position.x;
	marker.pose.position.y = pose.position.y;
	marker.pose.position.z = pose.position.z;
	marker.pose.orientation.w = pose.rotation.w;
	marker.pose.orientation.x = pose.rotation.x;
	marker.pose.orientation.y = pose.rotation.y;
	marker.pose.orientation.z = pose.rotation.z;
	marker.color.r = color.r;
	marker.color.g = color.g;
	marker.color.b = color.b;
	marker.color.a = color.a;
	return marker;
}

const urdf::Color& materialColor(const urdf::ModelInterface& model, const std::string& material_name) {
	static urdf::Color default_color;
	if (default_color.r == 0.0f) {
		default_color.r = 0.8f;
		default_color.g = 0.0f;
		default_color.b = 0.0f;
		default_color.a = 1.0f;
	};
	urdf::MaterialSharedPtr material;
	if (!material_name.empty())
		material = model.getMaterial(material_name);
	return material ? material->color : default_color;
}

// type traits to access collision/visual array or single element
template <class T>
const std::vector<T>& elements_vector(const urdf::Link& link);
template <>
const std::vector<urdf::CollisionSharedPtr>& elements_vector(const urdf::Link& link) {
	return link.collision_array;
}
template <>
const std::vector<urdf::VisualSharedPtr>& elements_vector(const urdf::Link& link) {
	return link.visual_array;
}

template <class T>
const T& element(const urdf::Link& link);
template <>
const urdf::CollisionSharedPtr& element(const urdf::Link& link) {
	return link.collision;
}
template <>
const urdf::VisualSharedPtr& element(const urdf::Link& link) {
	return link.visual;
}

template <class T>
const std::string& materialName(const T& element);
template <>
const std::string& materialName(const urdf::Visual& element) {
	return element.material_name;
}
template <>
const std::string& materialName(const urdf::Collision& /*element*/) {
	static std::string empty;
	return empty;
}

std::vector<std::string> linkNames(const std::vector<const moveit::core::LinkModel*>& link_models) {
	std::vector<std::string> names;
	names.reserve(link_models.size());
	for (const moveit::core::LinkModel* link : link_models)
		names.push_back(link->getName());
	return names;
}

/** generate marker msgs to visualize the robot state, calling the given callback for each of them
 *  link_names: set of links to include (or all if empty) */
template <class T>  // with T = urdf::Visual or urdf::Collision
void generateMarkers(const moveit::core::RobotState& robot_state, const MarkerCallback& callback,
                     const std::vector<std::string>& link_names = {}) {
	const std::vector<std::string>* names =
	    link_names.empty() ? &robot_state.getRobotModel()->getLinkModelNames() : &link_names;
	const urdf::ModelInterfaceSharedPtr& model = robot_state.getRobotModel()->getURDF();
	if (!model)
		return;

	visualization_msgs::msg::Marker m;
	m.header.frame_id = robot_state.getRobotModel()->getModelFrame();

	// code adapted from rviz::RobotLink::createVisual() / createCollision()
	for (const auto& name : *names) {
		const urdf::LinkConstSharedPtr& link = model->getLink(name);
		if (!link)
			return;

		bool valid_found = false;
		auto element_handler = [&](const T& element) {
			if (element && element->geometry) {
				createGeometryMarker(m, *element->geometry, element->origin, materialColor(*model, materialName(*element)));
				if (m.scale.x == 0 && m.scale.y == 0 && m.scale.z == 0)
					return;  // skip zero-size marker
				m.pose = rviz_marker_tools::composePoses(robot_state.getGlobalLinkTransform(name), m.pose);
				callback(m, name);
				valid_found = true;
			}
		};

		// either we have an array of collision/visual elements
		for (const auto& element : elements_vector<T>(*link))
			element_handler(element);

		// or there is a single such element
		if (!valid_found)
			element_handler(element<T>(*link));
	}
}

void generateCollisionMarkers(const moveit::core::RobotState& robot_state, const MarkerCallback& callback,
                              const std::vector<std::string>& link_names) {
	generateMarkers<urdf::CollisionSharedPtr>(robot_state, callback, link_names);
}
void generateCollisionMarkers(const moveit::core::RobotState& robot_state, const MarkerCallback& callback,
                              const std::vector<const moveit::core::LinkModel*>& link_models) {
	generateMarkers<urdf::CollisionSharedPtr>(robot_state, callback, linkNames(link_models));
}

void generateVisualMarkers(const moveit::core::RobotState& robot_state, const MarkerCallback& callback,
                           const std::vector<std::string>& link_names) {
	generateMarkers<urdf::VisualSharedPtr>(robot_state, callback, link_names);
}
void generateVisualMarkers(const moveit::core::RobotState& robot_state, const MarkerCallback& callback,
                           const std::vector<const moveit::core::LinkModel*>& link_models) {
	generateMarkers<urdf::VisualSharedPtr>(robot_state, callback, linkNames(link_models));
}

/** generate marker msgs to visualize the planning scene, calling the given callback for each of them
 *  calls generateMarkersForRobot() and generateMarkersForObjects() */
void generateMarkersForScene(const planning_scene::PlanningSceneConstPtr& scene, const MarkerCallback& callback) {
	generateMarkers<urdf::VisualSharedPtr>(scene->getCurrentState(), callback);
	generateMarkersForObjects(scene, callback);
}
}  // namespace task_constructor
}  // namespace moveit
