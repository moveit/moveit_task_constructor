#include <moveit/task_constructor/properties.h>

#include <moveit/task_constructor/stages/connect.h>
#include <geometry_msgs/PoseStamped.h>

#include <gtest/gtest.h>
#include <initializer_list>

using namespace moveit::task_constructor;

TEST(Property, standard) {
	PropertyMap props;
	props.declare<double>("double1", 1, "first");
	props.declare<double>("double2", 2);
	props.declare<double>("double4");

	EXPECT_EQ(props.get<double>("double1"), 1.0);
	EXPECT_EQ(props.get<double>("double2"), 2.0);
	EXPECT_THROW(props.get<double>("double3"), Property::undeclared);

	EXPECT_THROW(props.get<double>("double4"), Property::undefined);
	EXPECT_FALSE(props.property("double4").defined());

	props.set("double3", 3.0);
	EXPECT_EQ(props.get<double>("double3"), 3.0);
}

TEST(Property, directset) {
	PropertyMap props;
	props.set("int1", 1);
	EXPECT_EQ(props.get<int>("int1"), 1);
	EXPECT_EQ(props.property("int1").serialize(), "1");

	props.set("int2", boost::any(2));
	EXPECT_EQ(props.get<int>("int2"), 2);
	EXPECT_EQ(props.property("int2").serialize(), "2");
}

TEST(Property, redeclare) {
	PropertyMap props;
	props.declare<double>("double1");

	// avoid second declaration with different type
	props.declare<double>("double1");
	EXPECT_THROW(props.declare<long double>("double1"), Property::type_error);

	// types not matching?
	EXPECT_THROW(props.set("double1", 1), Property::type_error);

	props.set("double1", 3.14);
	EXPECT_EQ(props.get<double>("double1"), 3.14);
}

TEST(Property, reset) {
	PropertyMap props;
	props.declare<double>("double1");

	// setCurrent() only assigns temporary values
	props.setCurrent("double1", 1.0);
	ASSERT_EQ(props.get<double>("double1"), 1.0);
	EXPECT_TRUE(props.property("double1").defaultValue().empty());

	// they can be reset to their defaults
	props.reset();
	EXPECT_FALSE(props.property("double1").defined());

	// set() also updates the default
	props.set("double1", 1.0);
	EXPECT_EQ(props.get<double>("double1"), 1.0);
	EXPECT_EQ(boost::any_cast<double>(props.property("double1").defaultValue()), 1.0);

	props.setCurrent("double1", 2.0);
	EXPECT_EQ(props.get<double>("double1"), 2.0);
	EXPECT_EQ(boost::any_cast<double>(props.property("double1").defaultValue()), 1.0);

	// back to default
	props.reset();
	EXPECT_EQ(props.get<double>("double1"), 1.0);
}

TEST(Property, anytype) {
	PropertyMap props;
	props.declare<boost::any>("any", "store any type");
	props.set("any", 1);
	EXPECT_EQ(props.get<int>("any"), 1);

	props.set("any", std::string("foo"));
	EXPECT_EQ(props.get<std::string>("any"), "foo");
}

TEST(Property, serialize_basic) {
	EXPECT_TRUE(IsStreamSerializable<int>::value);
	EXPECT_TRUE(IsStreamSerializable<double>::value);
	EXPECT_TRUE(IsStreamSerializable<std::string>::value);

	EXPECT_TRUE(IsStreamDeserializable<int>::value);
	EXPECT_TRUE(IsStreamDeserializable<double>::value);
	EXPECT_TRUE(IsStreamDeserializable<std::string>::value);
}

TEST(Property, serialize) {
	PropertyMap props;
	props.declare<int>("int");
	EXPECT_EQ(props.property("int").serialize(), "");
	props.set("int", 42);
	EXPECT_EQ(props.property("int").serialize(), "42");

	// std::map doesn't provide operator<< serialization
	props.declare<std::map<int, int>>("map", std::map<int, int>());
	EXPECT_EQ(props.property("map").serialize(), "");
}

#define STORABLE_PROPERTY(T, V)                                                \
	{                                                                           \
		T v{ V };                                                                \
		PropertyMap example_props;                                               \
		example_props.declare<T>("property", "description");                     \
		example_props.set("property", v);                                        \
		moveit_task_constructor_msgs::Property props_msg;                        \
		example_props.property("property").fillMsg(props_msg);                   \
		std::cout << "serialized property message:\n" << props_msg << std::endl; \
		T deserialized_value;                                                    \
		try {                                                                    \
			deserialized_value = Property::deserialize<T>(props_msg);             \
		} catch (boost::bad_any_cast&) {                                         \
			ADD_FAILURE() << "deserialization failed";                            \
		}                                                                        \
		EXPECT_EQ(deserialized_value, v);                                        \
	}

TEST(Property, storeInMsg) {
	STORABLE_PROPERTY(std::string, "custom text");
	STORABLE_PROPERTY(int, 42);
	STORABLE_PROPERTY(double, 42);
	STORABLE_PROPERTY(stages::Connect::MergeMode, stages::Connect::WAYPOINTS);
	//	STORABLE_PROPERTY(double, M_PI);

	STORABLE_PROPERTY(geometry_msgs::PoseStamped, []() {
		geometry_msgs::PoseStamped msg;
		msg.header.frame_id = "world";
		msg.pose.orientation.x = 0.5;
		msg.pose.orientation.y = 0.5;
		msg.pose.orientation.z = -0.5;
		msg.pose.orientation.w = -0.5;
		return msg;
	}());

	std::map<std::string, double> joint_targets{ { "joint0", -0.5 }, { "joint1", 1.0 } };
	STORABLE_PROPERTY(decltype(joint_targets), joint_targets);
}

class InitFromTest : public ::testing::Test
{
protected:
	void SetUp() override {
		master.declare<double>("double1", 1);
		master.declare<double>("double2", 2);
		master.declare<double>("double4", 4);

		slave.declare<double>("double1");
		slave.declare<double>("double2");
		slave.declare<double>("double3");
	}
	PropertyMap master;
	PropertyMap slave;
};

TEST_F(InitFromTest, standard) {
	slave.configureInitFrom(1);  // init all matching vars
	ASSERT_FALSE(slave.property("double1").defined());

	slave.performInitFrom(1, master);
	EXPECT_EQ(slave.get<double>("double1"), 1.0);
	EXPECT_EQ(slave.get<double>("double2"), 2.0);
	EXPECT_FALSE(slave.property("double3").defined());
	EXPECT_THROW(slave.property("double4"), Property::undeclared);
}

TEST_F(InitFromTest, limited) {
	slave.configureInitFrom(1, { "double1" });  // limit init to listed props
	slave.performInitFrom(1, master);
	EXPECT_EQ(slave.get<double>("double1"), 1.0);
	EXPECT_FALSE(slave.property("double2").defined());
	EXPECT_FALSE(slave.property("double3").defined());
	EXPECT_THROW(slave.property("double4"), Property::undeclared);
}

TEST_F(InitFromTest, sourceId) {
	slave.configureInitFrom(1);  // init all matching vars
	slave.performInitFrom(2, master);  // init with wrong sourceId -> no effect
	EXPECT_FALSE(slave.property("double1").defined());
	EXPECT_FALSE(slave.property("double2").defined());
	EXPECT_FALSE(slave.property("double3").defined());
	EXPECT_THROW(slave.property("double4"), Property::undeclared);
}

TEST_F(InitFromTest, multipleSourceIds) {
	slave.configureInitFrom(1);
	slave.configureInitFrom(1);  // init is allowed second time with same id
	EXPECT_THROW(slave.configureInitFrom(2), std::runtime_error);  // but not with other id
}

TEST_F(InitFromTest, otherName) {
	slave.property("double1").configureInitFrom(1, "double2");  // init double1 from double2
	slave.performInitFrom(1, master);
	EXPECT_EQ(slave.get<double>("double1"), 2.0);
	EXPECT_FALSE(slave.property("double2").defined());
	EXPECT_FALSE(slave.property("double3").defined());
	EXPECT_THROW(slave.property("double4"), Property::undeclared);
}

TEST_F(InitFromTest, function) {
	slave.property("double3").configureInitFrom(1, [](const PropertyMap& other) -> boost::any {
		return other.get<double>("double1") + other.get<double>("double2");
	});
	slave.performInitFrom(1, master);
	EXPECT_EQ(slave.get<double>("double3"), 3.0);
}
