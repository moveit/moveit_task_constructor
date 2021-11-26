/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke */

#pragma once

#include <QObject>
#include <QString>
#include <map>
#include <functional>
#include <typeindex>

#include <moveit/task_constructor/properties.h>

namespace rviz_common {
class DisplayContext;
namespace properties {
class Property;
class PropertyTreeModel;
}  // namespace properties
}  // namespace rviz_common

namespace planning_scene {
class PlanningScene;
}
namespace moveit {
namespace task_constructor {
class Stage;
}
}  // namespace moveit

namespace moveit_rviz_plugin {

/** Registry for rviz_common::properties::Property and rviz_common::properties::PropertyTreeModel creator functions.
 *
 *  To inspect (and edit) properties of stages, our MTC properties are converted to rviz properties,
 *  which are finally shown in an rviz_common::properties::PropertyTree.
 *  To allow customization of property display, one can register creator functions for individual
 *  properties as well as creator functions for a complete stage. The latter allows to fully customize
 *  the display of stage properties, e.g. hiding specific properties, or returning a subclassed
 *  PropertyTreeModel with modified behaviour. By default, defaultPropertyTreeModel() creates an rviz
 *  property for each MTC property.
 */
class PropertyFactory
{
public:
	static PropertyFactory& instance();

	using PropertyFactoryFunction = std::function<rviz_common::properties::Property*(
	    const QString&, moveit::task_constructor::Property&, const planning_scene::PlanningScene*,
	    rviz_common::DisplayContext*)>;
	using TreeFactoryFunction = std::function<rviz_common::properties::PropertyTreeModel*(
	    moveit::task_constructor::PropertyMap&, const planning_scene::PlanningScene*, rviz_common::DisplayContext*)>;

	/// register a factory function for type T
	template <typename T>
	inline void registerType(const PropertyFactoryFunction& f) {
		moveit::task_constructor::PropertySerializer<T>();  // register serializer
		registerType(moveit::task_constructor::PropertySerializer<T>::typeName(), f);
	}

	/// register a factory function for stage T
	template <typename T>
	inline void registerStage(const TreeFactoryFunction& f) {
		registerStage(typeid(T), f);
	}

	/// create rviz_common::properties::Property for given MTC Property
	rviz_common::properties::Property* create(const std::string& prop_name, moveit::task_constructor::Property& prop,
	                                          const planning_scene::PlanningScene* scene,
	                                          rviz_common::DisplayContext* display_context) const;
	/// create rviz_common::properties::Property for property of given name, type, description, and value
	static rviz_common::properties::Property* createDefault(const std::string& name, const std::string& type,
	                                                        const std::string& description, const std::string& value,
	                                                        rviz_common::properties::Property* old = nullptr);

	/// create PropertyTreeModel for given Stage
	rviz_common::properties::PropertyTreeModel* createPropertyTreeModel(moveit::task_constructor::Stage& stage,
	                                                                    const planning_scene::PlanningScene* scene,
	                                                                    rviz_common::DisplayContext* display_context);

	/// turn a PropertyMap into an rviz_common::properties::PropertyTreeModel
	rviz_common::properties::PropertyTreeModel*
	defaultPropertyTreeModel(moveit::task_constructor::PropertyMap& properties,
	                         const planning_scene::PlanningScene* scene, rviz_common::DisplayContext* display_context);

	/// add all properties from map that are not yet in root
	void addRemainingProperties(rviz_common::properties::Property* root,
	                            moveit::task_constructor::PropertyMap& properties,
	                            const planning_scene::PlanningScene* scene,
	                            rviz_common::DisplayContext* display_context);

private:
	std::map<std::string, PropertyFactoryFunction> property_registry_;
	std::map<std::type_index, TreeFactoryFunction> stage_registry_;

	/// class is singleton
	PropertyFactory();
	PropertyFactory(const PropertyFactory&) = delete;
	void operator=(const PropertyFactory&) = delete;

	void registerType(const std::string& type_name, const PropertyFactoryFunction& f);
	void registerStage(const std::type_index& type_index, const TreeFactoryFunction& f);
};
}  // namespace moveit_rviz_plugin
