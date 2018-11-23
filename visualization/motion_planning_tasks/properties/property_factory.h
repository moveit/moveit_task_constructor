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
#include <moveit_task_constructor_msgs/Property.h>

namespace rviz {
class Property;
class PropertyTreeModel;
}

namespace moveit_rviz_plugin {

class PropertyFactory
{
public:
	static PropertyFactory& instance();

	typedef std::function<rviz::Property*(const QString& name, moveit::task_constructor::Property&)> FactoryFunction;

	/// register a new factory function for type T
	template <typename T>
	inline void registerType(const FactoryFunction& f) {
		moveit::task_constructor::PropertySerializer<T>();  // register serializer
		registerType(moveit::task_constructor::Property::typeName(typeid(T)), f);
	}

	/// create rviz::Property for given MTC Property
	rviz::Property* create(const std::string &prop_name, moveit::task_constructor::Property &prop) const;
	/// create rviz::Property for given MTC property message
	rviz::Property* create(const moveit_task_constructor_msgs::Property& p, rviz::Property* old) const;

private:
	std::map<std::string, FactoryFunction> registry_;

	/// class is singleton
	PropertyFactory();
	PropertyFactory(const PropertyFactory&) = delete;
	void operator=(const PropertyFactory&) = delete;

	void registerType(const std::string& type_name, const FactoryFunction& f);
};

/// turn a PropertyMap into an rviz::PropertyTreeModel
rviz::PropertyTreeModel* defaultPropertyTreeModel(moveit::task_constructor::PropertyMap &properties, QObject *parent = nullptr);

}
