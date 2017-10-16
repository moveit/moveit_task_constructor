#pragma once

#include <moveit/task_constructor/stage.h>
#include <deque>
#include <map>

namespace moveit { namespace planning_interface {
MOVEIT_CLASS_FORWARD(MoveGroupInterface)
} }

namespace moveit { namespace task_constructor { namespace stages {

class ModifyPlanningScene : public PropagatingEitherWay {
public:
	typedef std::vector<std::string> Names;
	ModifyPlanningScene(std::string name);

	bool computeForward(const InterfaceState& from) override;
	bool computeBackward(const InterfaceState& to) override;

	/// methods to attach objects to a robot link
	template <class C>
	void attachObjects(const C& objects, const std::string& attach_link);
	template <class C>
	void detachObjects(const C& objects, const std::string& attach_link);
	/// attachObjects() and detachObjects() forward to this method, setting attach = true resp. false
	void attachObjects(const Names& objects, const std::string& attach_link, bool attach);

	/// method to enable / disable collision between object pairs
	template <class T1, class T2>
	void enableCollisions(const T1& first, const T2& second, bool enable_collision);
	/// enable / disable all collisions for given object
	void enableCollisions(const std::string& object, bool enable_collision);

protected:
	// list of objects to attach (true) / detach (false) to a given link
	std::map<std::string, std::pair<Names, bool> > attach_objects_;

	// list of objects to mutually
	struct CollisionMatrixPairs {
		Names first;
		Names second;
		bool enable;
	};
	std::list<CollisionMatrixPairs> collision_matrix_edits_;

protected:
	// apply stored modifications to scene
	void attachObjects(planning_scene::PlanningScene &scene, const std::pair<std::string, std::pair<Names, bool> >& pair, bool invert);
	void setCollisions(planning_scene::PlanningScene &scene, const CollisionMatrixPairs& pairs, bool invert);
	planning_scene::PlanningScenePtr apply(const planning_scene::PlanningSceneConstPtr &scene, bool invert);
};


// As C++ doesn't allow for partial specialization of functions / methods, we need to use a helper class
namespace detail {

// generic implementation
template <class T> struct AttachHelper {
	static void impl(ModifyPlanningScene* stage, const T& objects, const std::string& link, bool attach) {
		static_assert(std::is_base_of<std::string, typename T::value_type>::value, "T must be a container of std::strings");
		stage->attachObjects(ModifyPlanningScene::Names(objects.cbegin(), objects.cend()), link, attach);
	}
};

// specialization for Names
template<>
struct AttachHelper<ModifyPlanningScene::Names> {
	static void impl(ModifyPlanningScene* stage, const ModifyPlanningScene::Names& objects, const std::string& link, bool attach) {
		stage->attachObjects(objects, link, attach);
	}
};

// specialization for std::string
template<>
struct AttachHelper<std::string> {
	static void impl(ModifyPlanningScene* stage, const std::string& object, const std::string& link, bool attach) {
		stage->attachObjects(ModifyPlanningScene::Names({object}), link, attach);
	}
};
// specialization for string literal
template<int N>
struct AttachHelper<char[N]> {
	static void impl(ModifyPlanningScene* stage, const char object[N], const std::string& link, bool attach) {
		stage->attachObjects(ModifyPlanningScene::Names({object}), link, attach);
	}
};

} // namespace detail

// implementation of methods forwards to AttachHelper::impl()
template <class T>
void ModifyPlanningScene::attachObjects(const T& first, const std::string& attach_link) {
	detail::AttachHelper<T>::impl(this, first, attach_link, true);
}
template <class T>
void ModifyPlanningScene::detachObjects(const T& first, const std::string& attach_link) {
	detail::AttachHelper<T>::impl(this, first, attach_link, false);
}



// specialization for Names, implemented in .cpp
template <>
void ModifyPlanningScene::enableCollisions<ModifyPlanningScene::Names, ModifyPlanningScene::Names>
(const ModifyPlanningScene::Names& first, const ModifyPlanningScene::Names& second, bool enable_collision);

// As C++ doesn't allow for partial specialization of functions / methods, we need to use a helper class
namespace detail {

// generic implementation
template <class T1, class T2> struct CollisionHelper {
	static void impl(ModifyPlanningScene* stage, const T1& first, const T2& second, bool enable) {
		static_assert(std::is_base_of<std::string, typename T1::value_type>::value, "T1 must be a container of std::strings");
		static_assert(std::is_base_of<std::string, typename T2::value_type>::value, "T2 must be a container of std::strings");
		stage->enableCollisions(ModifyPlanningScene::Names(first.cbegin(), first.cend()),
		                        ModifyPlanningScene::Names(second.cbegin(), second.cend()), enable);
	}
};

// specialization for Names
template<>
struct CollisionHelper<ModifyPlanningScene::Names, ModifyPlanningScene::Names> {
	static void impl(ModifyPlanningScene* stage, const ModifyPlanningScene::Names& first, const ModifyPlanningScene::Names& second, bool enable) {
		stage->enableCollisions(first, second, enable);
	}
};

// specialization for pair of std::string
template<>
struct CollisionHelper<std::string, std::string> {
	static void impl(ModifyPlanningScene* stage, const std::string& first, const std::string& second, bool enable) {
		stage->enableCollisions(ModifyPlanningScene::Names({first}), ModifyPlanningScene::Names({second}), enable);
	}
};
// specialization for pair of string literals
template<int N1, int N2>
struct CollisionHelper<char[N1], char[N2]> {
	static void impl(ModifyPlanningScene* stage, const char first[N1], const char second[N2], bool enable) {
		stage->enableCollisions(ModifyPlanningScene::Names({std::string(first)}),
		                        ModifyPlanningScene::Names({std::string(second)}), enable);
	}
};

// specialization for std::string, container
template<class T2>
struct CollisionHelper<std::string, T2> {
	static void impl(ModifyPlanningScene* stage, const std::string& first, const T2& second, bool enable) {
		stage->enableCollisions(ModifyPlanningScene::Names({first}), second, enable);
	}
};
// specialization for string literal, container
template<int N1, class T2>
struct CollisionHelper<char[N1], T2> {
	static void impl(ModifyPlanningScene* stage, const char first[N1], const T2& second, bool enable) {
		stage->enableCollisions(ModifyPlanningScene::Names({std::string(first)}), second, enable);
	}
};

} // namespace detail

// implementation of method forwards to CollisionHelper::impl()
template <class T1, class T2>
inline void ModifyPlanningScene::enableCollisions(const T1& first, const T2& second, bool enable_collision) {
	detail::CollisionHelper<T1, T2>::impl(this, first, second, enable_collision);
}

// single-object variant forwards to empty Names in second arg
inline void ModifyPlanningScene::enableCollisions(const std::string &object, bool enable_collision) {
	enableCollisions(Names({object}), Names(), enable_collision);
}

} } }
