#include <moveit/task_constructor/properties.h>

#include <gtest/gtest.h>
#include <initializer_list>

using namespace moveit::task_constructor;

TEST(Property, serialContainer) {
	PropertyMap props;
	props.declare<double>("double1");
	props.declare<double>("double2", 1);

	// avoid second declaration with different type
	props.declare<double>("double1");
	ASSERT_THROW(props.declare<long double>("double1"), std::runtime_error);

	// types not matching?
	ASSERT_THROW(props.set("double1", 1), std::runtime_error);

	props.set("double1", 3.14);
	ASSERT_EQ(props.get<double>("double1"), 3.14);
}
