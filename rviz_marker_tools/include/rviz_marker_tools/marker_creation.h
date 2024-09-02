#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

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
std_msgs::ColorRGBA getColor(Color color, double alpha = 1.0);
std_msgs::ColorRGBA& setColor(std_msgs::ColorRGBA& color, Color color_id, double alpha = 1.0);

std_msgs::ColorRGBA& interpolate(std_msgs::ColorRGBA& color, const std_msgs::ColorRGBA& other, double fraction);
std_msgs::ColorRGBA& brighten(std_msgs::ColorRGBA& color, double fraction);
std_msgs::ColorRGBA& darken(std_msgs::ColorRGBA& color, double fraction);

geometry_msgs::Pose composePoses(const geometry_msgs::Pose& first, const Eigen::Isometry3d& second);
geometry_msgs::Pose composePoses(const Eigen::Isometry3d& first, const geometry_msgs::Pose& second);

/** All routines only touch the geometry part of the marker
 *  pose, color, namespace, id, etc need to be set externally
 */

/// create planes with corners (-1,-1) - (+1,+1)
visualization_msgs::Marker& makeXYPlane(visualization_msgs::Marker& m);
visualization_msgs::Marker& makeXZPlane(visualization_msgs::Marker& m);
visualization_msgs::Marker& makeYZPlane(visualization_msgs::Marker& m);

/// create a cone of given angle along the x-axis
visualization_msgs::Marker& makeCone(visualization_msgs::Marker& m, double angle);

visualization_msgs::Marker& makeSphere(visualization_msgs::Marker& m, double radius = 1.0);

/// create a cylinder along z-axis
visualization_msgs::Marker& makeCylinder(visualization_msgs::Marker& m, double diameter, double height);

/// create a box with given dimensions along x, y, z axes
visualization_msgs::Marker& makeBox(visualization_msgs::Marker& m, double x, double y, double z);

/// create a mesh marker
visualization_msgs::Marker& makeMesh(visualization_msgs::Marker& m, const std::string& filename, double sx = 1.0,
                                     double sy = 1.0, double sz = 1.0);
inline visualization_msgs::Marker& makeMesh(visualization_msgs::Marker& m, const std::string& filename, double scale) {
	return makeMesh(m, filename, scale, scale, scale);
}

/// create an arrow with a start and end point
visualization_msgs::Marker& makeArrow(visualization_msgs::Marker& m, const Eigen::Vector3d& start_point,
                                      const Eigen::Vector3d& end_point, double diameter, double head_length = 0.0);

/// create an arrow along x-axis
visualization_msgs::Marker& makeArrow(visualization_msgs::Marker& m, double scale = 1.0, bool tip_at_origin = false);

/// create text marker
visualization_msgs::Marker& makeText(visualization_msgs::Marker& m, const std::string& text);

/// create marker from urdf::Geom
visualization_msgs::Marker& makeFromGeometry(visualization_msgs::Marker& m, const urdf::Geometry& geom);

template <typename T>
void appendFrame(T& container, const geometry_msgs::PoseStamped& pose, double scale = 1.0,
                 const std::string& ns = "frame", double diameter_fraction = 0.1) {
	visualization_msgs::Marker m;
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
