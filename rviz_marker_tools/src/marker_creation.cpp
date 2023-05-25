#include <rviz_marker_tools/marker_creation.h>
#include <urdf_model/link.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <rclcpp/logging.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("rviz_marker_tools");

namespace vm = visualization_msgs;

namespace rviz_marker_tools {

std_msgs::msg::ColorRGBA& setColor(std_msgs::msg::ColorRGBA& color, Color color_id, double alpha) {
	switch (color_id) {
		case RED:
			color.r = 0.8;
			color.g = 0.1;
			color.b = 0.1;
			color.a = alpha;
			break;
		case GREEN:
			color.r = 0.1;
			color.g = 0.8;
			color.b = 0.1;
			color.a = alpha;
			break;
		case BLUE:
			color.r = 0.1;
			color.g = 0.1;
			color.b = 0.8;
			color.a = alpha;
			break;
		case WHITE:
			color.r = 1.0;
			color.g = 1.0;
			color.b = 1.0;
			color.a = alpha;
			break;
		case GREY:
			color.r = 0.9;
			color.g = 0.9;
			color.b = 0.9;
			color.a = alpha;
			break;
		case DARK_GREY:
			color.r = 0.6;
			color.g = 0.6;
			color.b = 0.6;
			color.a = alpha;
			break;
		case BLACK:
			color.r = 0.0;
			color.g = 0.0;
			color.b = 0.0;
			color.a = alpha;
			break;
		case YELLOW:
			color.r = 1.0;
			color.g = 1.0;
			color.b = 0.0;
			color.a = alpha;
			break;
		case ORANGE:
			color.r = 1.0;
			color.g = 0.5;
			color.b = 0.0;
			color.a = alpha;
			break;
		case BROWN:
			color.r = 0.597;
			color.g = 0.296;
			color.b = 0.0;
			color.a = alpha;
			break;
		case PINK:
			color.r = 1.0;
			color.g = 0.4;
			color.b = 1;
			color.a = alpha;
			break;
		case LIME_GREEN:
			color.r = 0.6;
			color.g = 1.0;
			color.b = 0.2;
			color.a = alpha;
			break;
		case PURPLE:
			color.r = 0.597;
			color.g = 0.0;
			color.b = 0.597;
			color.a = alpha;
			break;
		case CYAN:
			color.r = 0.0;
			color.g = 1.0;
			color.b = 1.0;
			color.a = alpha;
			break;
		case MAGENTA:
			color.r = 1.0;
			color.g = 0.0;
			color.b = 1.0;
			color.a = alpha;
			break;
	}
	return color;
}

// interpolate between start and end with fraction in range from 0..1
double interpolate(double start, double end, double fraction) {
	return start * (1.0 - fraction) + end * fraction;
}

std_msgs::msg::ColorRGBA& interpolate(std_msgs::msg::ColorRGBA& color, const std_msgs::msg::ColorRGBA& other,
                                      double fraction) {
	if (fraction < 0.0)
		fraction = 0.0;
	if (fraction > 1.0)
		fraction = 1.0;
	color.r = interpolate(color.r, other.r, fraction);
	color.g = interpolate(color.g, other.g, fraction);
	color.b = interpolate(color.b, other.b, fraction);
	color.a = interpolate(color.a, other.a, fraction);
	return color;
}

std_msgs::msg::ColorRGBA& brighten(std_msgs::msg::ColorRGBA& color, double fraction) {
	static std_msgs::msg::ColorRGBA white;
	if (white.r == 0.0)
		setColor(white, WHITE);
	return interpolate(color, white, fraction);
}

std_msgs::msg::ColorRGBA& darken(std_msgs::msg::ColorRGBA& color, double fraction) {
	static std_msgs::msg::ColorRGBA black;
	return interpolate(color, black, fraction);
}

std_msgs::msg::ColorRGBA getColor(Color color, double alpha) {
	std_msgs::msg::ColorRGBA result;
	setColor(result, color, alpha);
	return result;
}

geometry_msgs::msg::Pose composePoses(const geometry_msgs::msg::Pose& first, const Eigen::Isometry3d& second) {
	Eigen::Isometry3d result_eigen;
	tf2::fromMsg(first, result_eigen);
	result_eigen = result_eigen * second;
	return tf2::toMsg(result_eigen);
}

geometry_msgs::msg::Pose composePoses(const Eigen::Isometry3d& first, const geometry_msgs::msg::Pose& second) {
	Eigen::Isometry3d result_eigen;
	tf2::fromMsg(second, result_eigen);
	result_eigen = first * result_eigen;
	return tf2::toMsg(result_eigen);
}

void prepareMarker(vm::msg::Marker& m, int marker_type) {
	m.action = vm::msg::Marker::ADD;
	m.type = marker_type;
	m.points.clear();
	m.colors.clear();

	// ensure non-null orientation
	if (m.pose.orientation.w == 0 && m.pose.orientation.x == 0 && m.pose.orientation.y == 0 && m.pose.orientation.z == 0)
		m.pose.orientation.w = 1.0;
}

vm::msg::Marker& makeXYPlane(vm::msg::Marker& m) {
	geometry_msgs::msg::Point p[4];

	p[0].x = 1.0;
	p[0].y = 1.0;
	p[0].z = 0.0;

	p[1].x = -1.0;
	p[1].y = 1.0;
	p[1].z = 0.0;

	p[2].x = -1.0;
	p[2].y = -1.0;
	p[2].z = 0.0;

	p[3].x = 1.0;
	p[3].y = -1.0;
	p[3].z = 0.0;

	m.scale.x = m.scale.y = m.scale.z = 1.0;
	prepareMarker(m, vm::msg::Marker::TRIANGLE_LIST);
	m.points.push_back(p[0]);
	m.points.push_back(p[1]);
	m.points.push_back(p[2]);

	m.points.push_back(p[2]);
	m.points.push_back(p[3]);
	m.points.push_back(p[0]);
	return m;
}

vm::msg::Marker& makeXZPlane(vm::msg::Marker& m) {
	makeXYPlane(m);
	// swap y and z components of points
	for (auto& p : m.points)
		std::swap(p.y, p.z);
	return m;
}

vm::msg::Marker& makeYZPlane(vm::msg::Marker& m) {
	makeXZPlane(m);
	// (additionally) swap x and y components of points
	for (auto& p : m.points)
		std::swap(p.x, p.y);
	return m;
}

/// create a cone of given angle along the x-axis
vm::msg::Marker makeCone(double angle, vm::msg::Marker& m) {
	m.scale.x = m.scale.y = m.scale.z = 1.0;
	prepareMarker(m, vm::msg::Marker::TRIANGLE_LIST);
	geometry_msgs::msg::Point p[3];
	p[0].x = p[0].y = p[0].z = 0.0;
	p[1].x = p[2].x = 1.0;

	const double delta_theta = M_PI / 16.0;
	double theta = 0;

	for (std::size_t i = 0; i < 32; i++) {
		p[1].y = cos(theta) / angle;
		p[1].z = sin(theta) / angle;

		p[2].y = cos(theta + delta_theta) / angle;
		p[2].z = sin(theta + delta_theta) / angle;

		m.points.push_back(p[0]);
		m.points.push_back(p[1]);
		m.points.push_back(p[2]);

		theta += delta_theta;
	}
	return m;
}

vm::msg::Marker& makeSphere(vm::msg::Marker& m, double radius) {
	m.scale.x = m.scale.y = m.scale.z = radius;
	prepareMarker(m, vm::msg::Marker::SPHERE);
	return m;
}

vm::msg::Marker& makeBox(vm::msg::Marker& m, double x, double y, double z) {
	m.scale.x = x;
	m.scale.y = y;
	m.scale.z = z;
	prepareMarker(m, vm::msg::Marker::CUBE);
	return m;
}

vm::msg::Marker& makeCylinder(vm::msg::Marker& m, double diameter, double height) {
	m.scale.x = m.scale.y = diameter;
	m.scale.z = height;
	prepareMarker(m, vm::msg::Marker::CYLINDER);
	return m;
}

vm::msg::Marker& makeMesh(vm::msg::Marker& m, const std::string& filename, double sx, double sy, double sz) {
	m.scale.x = sx;
	m.scale.y = sy;
	m.scale.z = sz;
	prepareMarker(m, vm::msg::Marker::MESH_RESOURCE);
	m.mesh_resource = filename;
	m.mesh_use_embedded_materials = 1u;
	return m;
}

vm::msg::Marker& makeArrow(vm::msg::Marker& m, const Eigen::Vector3d& start_point, const Eigen::Vector3d& end_point,
                           double diameter, double head_length) {
	// scale.y is set according to default proportions in rviz/default_plugin/markers/arrow_marker.cpp#L61
	// for the default head_length=0, the head length will keep the default proportion defined in arrow_marker.cpp#L106
	m.scale.x = diameter;
	m.scale.y = 2 * diameter;
	m.scale.z = head_length;
	prepareMarker(m, vm::msg::Marker::ARROW);
	m.points.resize(2);
	m.points[0] = tf2::toMsg(start_point);
	m.points[1] = tf2::toMsg(end_point);
	return m;
}

vm::msg::Marker& makeArrow(vm::msg::Marker& m, double scale, bool tip_at_origin) {
	m.scale.y = m.scale.z = 0.1 * scale;
	m.scale.x = scale;
	prepareMarker(m, vm::msg::Marker::ARROW);
	if (tip_at_origin)
		m.pose = composePoses(m.pose, Eigen::Translation3d(-scale, 0, 0) * Eigen::Isometry3d::Identity());
	return m;
}

vm::msg::Marker& makeText(vm::msg::Marker& m, const std::string& text) {
	m.scale.x = m.scale.y = m.scale.z = 1.0;
	prepareMarker(m, vm::msg::Marker::TEXT_VIEW_FACING);
	m.text = text;
	return m;
}

vm::msg::Marker& makeFromGeometry(vm::msg::Marker& m, const urdf::Geometry& geom) {
	switch (geom.type) {
		case urdf::Geometry::SPHERE: {
			const urdf::Sphere& sphere = static_cast<const urdf::Sphere&>(geom);
			makeSphere(m, sphere.radius);
			break;
		}
		case urdf::Geometry::BOX: {
			const urdf::Box& box = static_cast<const urdf::Box&>(geom);
			makeBox(m, box.dim.x, box.dim.y, box.dim.z);
			break;
		}
		case urdf::Geometry::CYLINDER: {
			const urdf::Cylinder& cylinder = static_cast<const urdf::Cylinder&>(geom);
			makeCylinder(m, 2.0 * cylinder.radius, cylinder.length);
			break;
		}
		case urdf::Geometry::MESH: {
			const urdf::Mesh& mesh = static_cast<const urdf::Mesh&>(geom);
			makeMesh(m, mesh.filename, mesh.scale.x, mesh.scale.y, mesh.scale.z);
			break;
		}
		default:
			RCLCPP_WARN(LOGGER, "Unsupported geometry type: %d", geom.type);
			break;
	}

	return m;
}

}  // namespace rviz_marker_tools
