#pragma once

#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Geometry>

namespace urdf {
class Geometry;
}

namespace rviz_marker_tools {

enum Color
{
	BLACK = 0,
	BROWN = 1,
	BLUE = 2,
	CYAN = 3,
	GREY = 4,
	DARK_GREY = 5,
	GREEN = 6,
	LIME_GREEN = 7,
	MAGENTA = 8,
	ORANGE = 9,
	PURPLE = 10,
	RED = 11,
	PINK = 12,
	WHITE = 13,
	YELLOW = 14,
};
std_msgs::msg::ColorRGBA getColor(Color color, double alpha = 1.0);
std_msgs::msg::ColorRGBA& setColor(std_msgs::msg::ColorRGBA& color, Color color_id, double alpha = 1.0);

std_msgs::msg::ColorRGBA& interpolate(std_msgs::msg::ColorRGBA& color, const std_msgs::msg::ColorRGBA& other,
                                      double fraction);
std_msgs::msg::ColorRGBA& brighten(std_msgs::msg::ColorRGBA& color, double fraction);
std_msgs::msg::ColorRGBA& darken(std_msgs::msg::ColorRGBA& color, double fraction);

geometry_msgs::msg::Pose composePoses(const geometry_msgs::msg::Pose& first, const Eigen::Isometry3d& second);
geometry_msgs::msg::Pose composePoses(const Eigen::Isometry3d& first, const geometry_msgs::msg::Pose& second);

/** All routines only touch the geometry part of the marker
 *  pose, color, namespace, id, etc need to be set externally
 */

/// create planes with corners (-1,-1) - (+1,+1)
visualization_msgs::msg::Marker& makeXYPlane(visualization_msgs::msg::Marker& m);
visualization_msgs::msg::Marker& makeXZPlane(visualization_msgs::msg::Marker& m);
visualization_msgs::msg::Marker& makeYZPlane(visualization_msgs::msg::Marker& m);

/// create a cone of given angle along the x-axis
visualization_msgs::msg::Marker& makeCone(visualization_msgs::msg::Marker& m, double angle);

visualization_msgs::msg::Marker& makeSphere(visualization_msgs::msg::Marker& m, double radius = 1.0);

/// create a cylinder along z-axis
visualization_msgs::msg::Marker& makeCylinder(visualization_msgs::msg::Marker& m, double diameter, double height);

/// create a box with given dimensions along x, y, z axes
visualization_msgs::msg::Marker& makeBox(visualization_msgs::msg::Marker& m, double x, double y, double z);

/// create a mesh marker
visualization_msgs::msg::Marker& makeMesh(visualization_msgs::msg::Marker& m, const std::string& filename,
                                          double sx = 1.0, double sy = 1.0, double sz = 1.0);
inline visualization_msgs::msg::Marker& makeMesh(visualization_msgs::msg::Marker& m, const std::string& filename,
                                                 double scale) {
	return makeMesh(m, filename, scale, scale, scale);
}

/// create an arrow with a start and end point
visualization_msgs::msg::Marker& makeArrow(visualization_msgs::msg::Marker& m, const Eigen::Vector3d& start_point,
                                           const Eigen::Vector3d& end_point, double diameter, double head_length = 0.0);

/// create an arrow along x-axis
visualization_msgs::msg::Marker& makeArrow(visualization_msgs::msg::Marker& m, double scale = 1.0,
                                           bool tip_at_origin = false);

/// create text marker
visualization_msgs::msg::Marker& makeText(visualization_msgs::msg::Marker& m, const std::string& text);

/// create marker from urdf::Geom
visualization_msgs::msg::Marker& makeFromGeometry(visualization_msgs::msg::Marker& m, const urdf::Geometry& geom);

template <typename T>
void appendFrame(T& container, const geometry_msgs::msg::PoseStamped& pose, double scale = 1.0,
                 const std::string& ns = "frame", double diameter_fraction = 0.1) {
	visualization_msgs::msg::Marker m;
	makeCylinder(m, scale * diameter_fraction, scale);
	m.ns = ns;
	m.header = pose.header;

	// x-axis
	m.pose = composePoses(pose.pose, Eigen::Translation3d(scale / 2.0, 0, 0) *
	                                     Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()));
	setColor(m.color, RED);
	container.push_back(m);

	// y-axis
	m.pose = composePoses(pose.pose, Eigen::Translation3d(0, scale / 2.0, 0) *
	                                     Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()));
	setColor(m.color, GREEN);
	container.push_back(m);

	// z-axis
	m.pose = composePoses(pose.pose, Eigen::Translation3d(0, 0, scale / 2.0) * Eigen::Isometry3d::Identity());
	setColor(m.color, BLUE);
	container.push_back(m);
}

}  // namespace rviz_marker_tools
